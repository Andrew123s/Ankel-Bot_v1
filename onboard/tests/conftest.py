import json
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from ankelbot.config import load_params  # noqa: E402


@pytest.fixture(scope="session")
def p():
    return load_params()


@pytest.fixture(scope="session")
def ref():
    """Input/output vectors produced by the MATLAB reference (matlab/export_reference.m)."""
    with open(ROOT / "tests" / "data" / "matlab_reference.json", "r", encoding="utf-8") as f:
        return json.load(f)
