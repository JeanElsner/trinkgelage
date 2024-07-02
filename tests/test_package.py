from __future__ import annotations

import importlib.metadata

import trinkgelage as m


def test_version():
    assert importlib.metadata.version("trinkgelage") == m.__version__
