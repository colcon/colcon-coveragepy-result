# colcon-coveragepy-result

[![PyPI](https://img.shields.io/pypi/v/colcon-coveragepy-result)](https://pypi.org/project/colcon-coveragepy-result/)
[![GitHub Action Status](https://github.com/colcon/colcon-coveragepy-result/workflows/Test/badge.svg)](https://github.com/colcon/colcon-coveragepy-result/actions)

A [colcon](https://colcon.readthedocs.io/) extension for collecting [coverage.py](https://coverage.readthedocs.io/en/stable/) results.
It provides the `coveragepy-result` command.

## Install

The corresponding package names are:

* `apt`: `python3-colcon-coveragepy-result`
* `pip`: `colcon-coveragepy-result`

Refer to the [colcon installation instructions](https://colcon.readthedocs.io/en/released/user/installation.html) for more information.
## Usage

1. Build packages with coverage, e.g. using a [colcon mixin](https://colcon.readthedocs.io/en/released/reference/verb/mixin.html) for [coverage](https://github.com/colcon/colcon-mixin-repository/blob/master/coverage.mixin) (installation of the mixin is separate from this extension)
   ```shell
   $ colcon build --mixin coverage-pytest
   ```
1. Test packages with coverage, again using a mixin
   ```shell
   $ colcon test --mixin coverage-pytest
   ```
1. Collect coverage results using the `coveragepy-result` command provided by this extension
   ```shell
   $ colcon coveragepy-result
   ```
1. Open HTML report, which by default is under `coveragepy/htmlcov/`

## Options

* Print coverage reports for each package and a combined coverage report of all packages
   ```shell
   $ colcon coveragepy-result --verbose
   ```
* Provide additional arguments for reports generation, e.g. to show lines without coverage
   ```shell
   $ colcon coveragepy-result --coverage-report-args -m
   ```
* Provide additional arguments for HTML report generation, e.g. to skip files with no code
   ```shell
   $ colcon coveragepy-result --coverage-html-args --skip-empty
   ```
* For more options
   ```shell
   $ colcon coveragepy-result --help
   ```

## Contributing

See:

* [*Contributions*](https://colcon.readthedocs.io/en/released/developer/contribution.html) for guidelines
* [*Bootstrap from source*](https://colcon.readthedocs.io/en/released/developer/bootstrap.html) to test the package from source
