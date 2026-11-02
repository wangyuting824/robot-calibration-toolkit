from pathlib import Path
import sys
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from examples.generate_demo import generate


@pytest.fixture
def demo(tmp_path):
    return generate(tmp_path / "synthetic")
