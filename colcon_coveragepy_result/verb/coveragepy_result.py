# Copyright 2020 Christophe Bedard
# Licensed under the Apache License, Version 2.0

from collections import OrderedDict
import os
from pathlib import Path

from colcon_core.command import add_log_level_argument
from colcon_core.event_handler import add_event_handler_arguments
from colcon_core.executor import add_executor_arguments
from colcon_core.executor import execute_jobs
from colcon_core.executor import Job
from colcon_core.logging import colcon_logger
from colcon_core.package_selection import add_arguments as add_packages_arguments
from colcon_core.package_selection import get_package_descriptors
from colcon_core.package_selection import select_package_decorators
from colcon_core.plugin_system import satisfies_version
from colcon_core.task import TaskContext
from colcon_core.topological_order import topological_order_packages
from colcon_core.verb import check_and_mark_build_tool
from colcon_core.verb import VerbExtensionPoint

from ..task.coveragepy import coverage_combine
from ..task.coveragepy import coverage_html
from ..task.coveragepy import coverage_report
from ..task.coveragepy import CoveragePyTask

logger = colcon_logger.getChild(__name__)


class CoveragePyResultVerb(VerbExtensionPoint):
    """Collect and display coverage.py results."""

    def __init__(self):  # noqa: D107
        super().__init__()
        satisfies_version(VerbExtensionPoint.EXTENSION_POINT_VERSION, '^1.0')

    def add_arguments(self, *, parser):  # noqa: D102
        parser.add_argument(
            '--build-base',
            default='build',
            help='The base path for all build directories (default: %(default)s)',
        )
        parser.add_argument(
            '--coveragepy-base',
            default='coveragepy',
            help='The path for coveragepy artifacts and outputs (default: %(default)s)',
        )
        parser.add_argument(
            '--coverage-report-args',
            nargs='*', metavar='*', type=str.lstrip,
            help="Pass arguments to 'coverage report'. Arguments matching "
                 'other options must be prefixed by a space, '
                 'e.g. --coverage-report-args " --help"',
        )
        parser.add_argument(
            '--coverage-html-args',
            nargs='*', metavar='*', type=str.lstrip,
            help="Pass arguments to 'coverage html'. Arguments matching "
                 'other options must be prefixed by a space, '
                 'e.g. --coverage-html-args " --help"',
        )
        parser.add_argument(
            '--verbose',
            action='store_true',
            help='Show coverage results for individual packages and overall',
        )
        add_packages_arguments(parser)
        add_log_level_argument(parser)
        add_executor_arguments(parser)
        add_event_handler_arguments(parser)

    def main(self, *, context):  # noqa: D102
        build_base = context.args.build_base
        check_and_mark_build_tool(build_base)

        # Combine each package's .coverage files
        coveragepy_pkgs = self._get_coveragepy_packages(context)
        jobs = OrderedDict()
        for pkg in coveragepy_pkgs:
            task_context = TaskContext(
                pkg=pkg,
                args=context.args,
                dependencies=OrderedDict(),
            )
            task = CoveragePyTask()
            job = Job(
                identifier=pkg.name,
                dependencies=set(),
                task=task,
                task_context=task_context,
            )
            jobs[pkg.name] = job
        rc = execute_jobs(context, jobs)

        # Get all packages' .coverage files
        coverage_files = [
            str(Path(CoveragePyTask.get_package_combine_dir(build_base, pkg.name)) / '.coverage')
            for pkg in coveragepy_pkgs
        ]
        # Filter out non-existing files in case processing failed for some packages
        coverage_files = list(filter(os.path.exists, coverage_files))
        if 0 == len(coverage_files):
            logger.warning('No coverage files found')
            return 0
        logger.info('Coverage files: {coverage_files}'.format_map(locals()))

        # Combine .coverage files
        coveragepy_base_dir = str(os.path.abspath(context.args.coveragepy_base))
        Path(coveragepy_base_dir).mkdir(exist_ok=True)
        rc, stdout, _ = coverage_combine(coverage_files, coveragepy_base_dir)
        if 0 == rc.returncode and context.args.verbose:
            # Print report
            rc, stdout, _ = coverage_report(
                coveragepy_base_dir,
                context.args.coverage_report_args,
            )
            if 0 == rc.returncode:
                print('\n' + stdout.decode())
        # Generate HTML report
        rc, stdout, _ = coverage_html(coveragepy_base_dir, context.args.coverage_html_args)
        return rc.returncode

    @staticmethod
    def _get_coveragepy_packages(context, additional_argument_names=None):
        """Get packages that could have coverage.py results."""
        descriptors = get_package_descriptors(
            context.args,
            additional_argument_names=additional_argument_names,
        )
        decorators = topological_order_packages(descriptors, recursive_categories=('run', ))
        select_package_decorators(context.args, decorators)
        coveragepy_pkgs = []
        for decorator in decorators:
            if not decorator.selected:
                continue
            pkg = decorator.descriptor
            if pkg.type in ['ros.ament_cmake', 'ros.ament_python']:
                coveragepy_pkgs.append(pkg)
            else:
                logger.info(
                    "Specified package '{pkg.name}' is not a coverage.py-compatible "
                    'package. Not collecting coverage information.'.format_map(locals())
                )
        return coveragepy_pkgs
