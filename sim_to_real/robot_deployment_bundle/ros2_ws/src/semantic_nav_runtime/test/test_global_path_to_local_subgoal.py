from pathlib import Path
import runpy
globals().update(runpy.run_path(str(Path(__file__).resolve().parents[4] / 'tests/test_global_path_to_local_subgoal.py')))
