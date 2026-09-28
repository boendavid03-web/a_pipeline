from pathlib import Path
import runpy
globals().update(runpy.run_path(str(Path(__file__).resolve().parents[4] / 'tests/test_fixed_dual_scan_adapter.py')))
