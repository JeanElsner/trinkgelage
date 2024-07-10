from __future__ import annotations

from unittest import mock

import pytest


@pytest.fixture(autouse=True)
def _mock_environ():
    with mock.patch.dict(
        "os.environ",
        {
            "PANDA_LEFT": "left",
            "PANDA_RIGHT": "right",
        },
    ):
        yield
