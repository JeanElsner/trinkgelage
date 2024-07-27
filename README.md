# trinkgelage

[![Actions Status][actions-badge]][actions-link]
[![Documentation Status][rtd-badge]][rtd-link]
[![PyPI version][pypi-version]][pypi-link]
[![PyPI platforms][pypi-platforms]][pypi-link] [![codecov][cov-badge]][cov-link]

Robotic two-arm demonstrator serving beer from a keg as shown at Geriatronics
Summit 2024. Watch a video of the demo
[here](https://www.youtube.com/shorts/Ao5W2cQ6zYw).

<p align="center">
	<img src="https://raw.githubusercontent.com/tum-robotics/trinkgelage/main/.github/trinkgelage.png">
	<img src="https://raw.githubusercontent.com/tum-robotics/trinkgelage/main/.github/statemachine.png">
</p>

## Install

To install run

```
pip install trinkgelage
```

or if you're working with the code in a local clone of the repository

```
pip install -v -e .[dev]
```

## Run

This Python package installs an executable for convenience. You can start the
demo by running `trinkgelage-demo` from the environment where you installed the
package. This program includes a terminal interface to control the demo
manually. In order to run the program, you need to have two Franka robots
connected with their respective hostnames/IPs available in the environment
variables `PANDA_LEFT` and `PANDA_RIGHT`.

## Requirements

The robots are controlled using
[panda-py](https://github.com/JeanElsner/panda-py), which is automatically
installed from pypi as part of the requirements. However, if you use an older
firmware or the FR3, you will need to manually install the correct version.

<!-- SPHINX-START -->

<!-- prettier-ignore-start -->
[actions-badge]:            https://img.shields.io/github/actions/workflow/status/tum-robotics/trinkgelage/ci.yml
[actions-link]:             https://github.com/tum-robotics/trinkgelage/actions
[pypi-link]:                https://pypi.org/project/trinkgelage/
[pypi-platforms]:           https://img.shields.io/pypi/pyversions/trinkgelage
[pypi-version]:             https://img.shields.io/pypi/v/trinkgelage
[rtd-badge]:                https://readthedocs.org/projects/trinkgelage/badge/?version=latest
[rtd-link]:                 https://trinkgelage.readthedocs.io/en/latest/?badge=latest
[cov-badge]:                https://img.shields.io/codecov/c/gh/tum-robotics/trinkgelage
[cov-link]:                 https://app.codecov.io/gh/tum-robotics/trinkgelage
<!-- prettier-ignore-end -->
