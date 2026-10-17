"""Deploy-file regression checks; run with python3 -m unittest discover -s tests."""
import json
from pathlib import Path
import re
import unittest

ROOT = Path(__file__).resolve().parents[1]
DEPLOY = ROOT / "src/main/deploy/pathplanner"


class PathPlannerAssetsTest(unittest.TestCase):
    def test_settings_in_library_expected_directory(self):
        settings = json.loads((DEPLOY / "settings.json").read_text())
        self.assertTrue(settings["holonomicMode"])
        self.assertGreater(settings["robotMass"], 0)

    def test_chooser_autos_exist_with_exact_case(self):
        source = (ROOT / "src/main/java/frc/robot/RobotContainer.java").read_text()
        names = re.findall(r'autoChooser.addOption\("[^"]+", "([^"]+)"\)', source)
        self.assertTrue(names)
        files = {p.name for p in (DEPLOY / "autos").iterdir()}
        for name in names:
            self.assertIn(name + ".auto", files)

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
