import importlib.util
import json
import os
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import numpy as np


MODEL_PATH = (
    Path(__file__).resolve().parents[3]
    / "methods"
    / "baselines"
    / "semantic_cnn"
    / "training"
    / "scripts"
    / "model.py"
)
SPEC = importlib.util.spec_from_file_location("semantic_cnn_model_for_stats_test", MODEL_PATH)
MODEL = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODEL)


class SemanticCnnNormalizationTests(unittest.TestCase):
    def test_loads_two_dimensional_train_stats(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            stats_path = Path(temp_dir) / "stats.json"
            stats_path.write_text(
                json.dumps(
                    {
                        "sub_goal_local_xy": {
                            "mean": [0.8, -0.05],
                            "std_population": [0.55, 0.32],
                        }
                    }
                ),
                encoding="utf-8",
            )
            stats = MODEL.load_sub_goal_normalization(stats_path)
            self.assertEqual(stats["source"], "stats_json")
            np.testing.assert_allclose(stats["mean"], [0.8, -0.05])
            np.testing.assert_allclose(stats["std"], [0.55, 0.32])

    def test_legacy_fallback_is_two_dimensional(self):
        with mock.patch.dict(os.environ, {}, clear=True):
            stats = MODEL.load_sub_goal_normalization()
        self.assertEqual(stats["source"], "legacy_constants")
        self.assertEqual(stats["mean"].shape, (2,))
        self.assertEqual(stats["std"].shape, (2,))

    def test_rejects_nonpositive_standard_deviation(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            stats_path = Path(temp_dir) / "stats.json"
            stats_path.write_text(
                json.dumps(
                    {
                        "sub_goal_local_xy": {
                            "mean": [0.0, 0.0],
                            "std_population": [1.0, 0.0],
                        }
                    }
                ),
                encoding="utf-8",
            )
            with self.assertRaisesRegex(ValueError, "std must be positive"):
                MODEL.load_sub_goal_normalization(stats_path)


if __name__ == "__main__":
    unittest.main()
