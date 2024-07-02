import pytest
from pathlib import Path

@pytest.fixture
def root_dir():
    return Path(__file__).parents[2]

@pytest.fixture(autouse=True)
def chdir_to_root(root_dir, monkeypatch):
    monkeypatch.chdir(root_dir)

@pytest.mark.parametrize("script_file", ["rlbench_gym.py", "rlbench_gym_vector.py"])
def test_example(script_file):
    import subprocess
    subprocess.run(["python", f"examples/{script_file}"], check=True)