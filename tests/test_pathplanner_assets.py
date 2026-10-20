"""Deploy-file regression checks; run with python3 -m unittest discover -s tests."""
import json
import math
from pathlib import Path
import re
import unittest

ROOT = Path(__file__).resolve().parents[1]
DEPLOY = ROOT / "src/main/deploy/pathplanner"


class PathPlannerAssetsTest(unittest.TestCase):
    def test_settings_standardized_on_neo_and_physical_geometry(self):
        settings = json.loads((DEPLOY / "settings.json").read_text())
        self.assertTrue(settings["holonomicMode"])
        self.assertGreater(settings["robotMass"], 0)
        self.assertEqual("NEO", settings["driveMotorType"])
        self.assertAlmostEqual(0.0381, settings["driveWheelRadius"], places=4)
        self.assertAlmostEqual(0.673, settings["robotTrackwidth"], places=3)
        neo_free_speed_mps = (5676.0 / 60.0) * (0.0762 * math.pi) / ((45.0 * 22.0) / (13.0 * 15.0))
        self.assertLessEqual(settings["maxDriveSpeed"], neo_free_speed_mps)

    def test_chooser_autos_exist_with_exact_case(self):
        source = (ROOT / "src/main/java/frc/robot/RobotContainer.java").read_text()
        names = re.findall(r'autoChooser.addOption\("[^"]+", "([^"]+)"\)', source)
        self.assertTrue(names)
        files = {p.name for p in (DEPLOY / "autos").iterdir()}
        for name in names:
            self.assertIn(name + ".auto", files)

    def test_teleop_trench_paths_exist(self):
        source = (ROOT / "src/main/java/frc/robot/RobotContainer.java").read_text()
        trench_paths = re.findall(r'loadTrenchPath\("([^"]+)"\)', source)
        self.assertTrue(trench_paths)
        for name in trench_paths:
            self.assertTrue((DEPLOY / "paths" / (name + ".path")).is_file(), name)

    def test_auto_path_references_exist(self):
        def check(node):
            if isinstance(node, dict):
                if node.get("type") == "path":
                    name = node["data"]["pathName"]
                    self.assertTrue((DEPLOY / "paths" / (name + ".path")).is_file(), name)
                for value in node.values():
                    check(value)
            elif isinstance(node, list):
                for value in node:
                    check(value)

        autos = list((DEPLOY / "autos").glob("*.auto"))
        self.assertTrue(autos)
        for auto in autos:
            check(json.loads(auto.read_text()))


if __name__ == "__main__":
    unittest.main()
