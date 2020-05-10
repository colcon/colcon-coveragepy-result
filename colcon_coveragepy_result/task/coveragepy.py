# Copyright 2020 Christophe Bedard
# Licensed under the Apache License, Version 2.0

from glob import glob
import os
from pathlib import Path
from shutil import copy2
import subprocess

from colcon_core.logging import colcon_logger
from colcon_core.plugin_system import satisfies_version
from colcon_core.task import TaskExtensionPoint

logger = colcon_logger.getChild(__name__)


class CoveragePyTask(TaskExtensionPoint):
    """Run coverage.py on a package."""

    TASK_NAME = 'coveragepy'

    def __init__(self):  # noqa: D107
        super().__init__()
        satisfies_version(TaskExtensionPoint.EXTENSION_POINT_VERSION, '^1.0')

    async def coveragepy(self, *, additional_hooks=None):  # noqa: D102
        pkg = self.context.pkg
        args = self.context.args

        # Check if the package has been built
        pkg_build_path = Path(os.path.abspath(os.path.join(args.build_base, pkg.name)))
        if not pkg_build_path.exists():
            logger.info(
                "Skipping package '{pkg.name}' since it has not been built".format_map(locals())
            )
            return 0

        logger.info("Running coveragepy task on package '{pkg.name}'".format_map(locals()))

        # Get list of .coverage files, depending on package type
        coverage_files = []
        if 'ros.ament_cmake' == pkg.type:
            coverage_files.extend(glob(str(pkg_build_path / 'pytest_cov/*/.coverage')))
        elif 'ros.ament_python' == pkg.type:
            coverage_files.append(str(pkg_build_path / '.coverage'))

        # Filter out non-existing files in case they have not been generated
        coverage_files = list(filter(os.path.exists, coverage_files))
        if 0 == len(coverage_files):
            logger.warning(
                "No .coverage files found for package '{pkg.name}' of type '{pkg.type}'"
                .format_map(locals())
            )
            return 0
        logger.info(
            "Coverage files for package '{pkg.name}': {coverage_files}".format_map(locals())
        )

        # Copy .coverage files to a new directory, because combining files deletes them
        coveragepy_dir = self.get_package_combine_dir(args.build_base, pkg.name)
        coveragepy_path = Path(coveragepy_dir)
        coveragepy_path.mkdir(exist_ok=True)
        logger.info('Copying coverage files to {coveragepy_dir}'.format_map(locals()))
        coverage_files_copies = [
            str(coveragepy_path / ('.coverage.' + str(i))) for i in range(len(coverage_files))
        ]
        for original, copy in zip(coverage_files, coverage_files_copies):
            copy2(original, copy)

        # Combine .coverage files
        rc, stdout, _ = coverage_combine(coverage_files_copies, coveragepy_dir)
        if 0 == rc.returncode and args.verbose:
            # Report
            rc, stdout, _ = coverage_report(coveragepy_dir, args.coverage_report_args)
            if 0 == rc.returncode:
                print('\n' + stdout.decode())
        return rc.returncode

    @staticmethod
    def get_package_combine_dir(build_base, pkg_name):
        """Get the directory in which to combine .coverage files for a given package."""
        pkg_build_dir = os.path.abspath(os.path.join(build_base, pkg_name))
        return str(os.path.abspath(os.path.join(pkg_build_dir, 'coveragepy')))


def coverage_combine(files, cwd):
    """Combine .coverage files."""
    cmd = ['coverage', 'combine'] + files
    logger.debug('Running command {cmd} in {cwd}'.format_map(locals()))
    process = subprocess.Popen(cmd, cwd=cwd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    stdout, stderr = process.communicate()
    if 0 != process.returncode:
        logger.error('coverage_combine() failed: ' + stderr.decode())
    return process, stdout, stderr


def coverage_html(cwd, additional_args):
    """Create an HTML report from a .coverage file."""
    cmd = ['coverage', 'html'] + (additional_args or [])
    logger.debug('Running command {cmd} in {cwd}'.format_map(locals()))
    process = subprocess.Popen(cmd, cwd=cwd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    stdout, stderr = process.communicate()
    if 0 != process.returncode:
        logger.error('coverage_html() failed: ' + stderr.decode())
    return process, stdout, stderr


def coverage_report(cwd, additional_args):
    """Produce a report for a .coverage file."""
    cmd = ['coverage', 'report'] + (additional_args or [])
    logger.debug('Running command {cmd} in {cwd}'.format_map(locals()))
    process = subprocess.Popen(cmd, cwd=cwd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    stdout, stderr = process.communicate()
    if 0 != process.returncode:
        logger.error('coverage_report() failed: ' + stderr.decode())
    return process, stdout, stderr
