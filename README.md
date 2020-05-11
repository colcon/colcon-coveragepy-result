# colcon-coveragepy-result

[![PyPI](https://img.shields.io/pypi/v/colcon-coveragepy-result)](https://pypi.org/project/colcon-coveragepy-result/)
[![GitHub Action Status](https://github.com/colcon/colcon-coveragepy-result/workflows/Test/badge.svg)](https://github.com/colcon/colcon-coveragepy-result/actions)

A [colcon](https://colcon.readthedocs.io/) extension for collecting [coverage.py](https://coverage.readthedocs.io/en/stable/) results.

## Usage

1. Build packages with coverage, e.g. using a [colcon mixin](https://colcon.readthedocs.io/en/released/reference/verb/mixin.html) for [coverage](https://github.com/colcon/colcon-mixin-repository/blob/master/coverage.mixin)
   ```shell
   $ colcon build --mixin coverage-pytest
   ```
1. Test packages with coverage, again using a mixin
   ```shell
   $ colcon test --mixin coverage-pytest
   ```
1. Collect coverage results
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
