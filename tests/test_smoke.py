import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from main import app


def test_app_instance_exists() -> None:
    assert app is not None
