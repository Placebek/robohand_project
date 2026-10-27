import importlib.util
import unittest
from pathlib import Path

SCRIPT = Path(__file__).resolve().parents[1] / "tools" / "arm" / "teach_repeat.py"
SPEC = importlib.util.spec_from_file_location("teach_repeat", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def sample():
    return {
        "schema_version": 1,
        "points": [
            {"name": "home", "joints": [0, 1, 2, 3, 4, 5],
             "pose": [200, 0, 300, 180, 0, 90], "vacuum": False},
            {"name": "pick", "joints": [1, 2, 3, 4, 5, 6],
             "pose": [210, 0, 200, 180, 0, 90], "vacuum": True},
        ],
    }


class ProgramValidationTests(unittest.TestCase):
    def test_valid_program(self):
        points = MODULE.validate_program(sample())
        self.assertEqual(len(points), 2)
        self.assertTrue(points[1]["vacuum"])

    def test_requires_two_points(self):
        data = sample()
        data["points"] = data["points"][:1]
        with self.assertRaises(ValueError):
            MODULE.validate_program(data)

    def test_rejects_non_finite_joint(self):
        data = sample()
        data["points"][0]["joints"][0] = float("nan")
        with self.assertRaises(ValueError):
            MODULE.validate_program(data)

    def test_rejects_non_boolean_vacuum(self):
        data = sample()
        data["points"][0]["vacuum"] = 1
        with self.assertRaises(ValueError):
            MODULE.validate_program(data)


if __name__ == "__main__":
    unittest.main()
