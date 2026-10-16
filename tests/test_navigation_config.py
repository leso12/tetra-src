"""ROS-independent configuration regression tests (requires Python3 and PyYAML)."""
import unittest
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml

ROOT = Path(__file__).resolve().parents[1]


class NavigationConfigTests(unittest.TestCase):
    def test_teb_footprint_profiles(self):
        configs = {
            "config/teb_local_planner_params.yaml": "line",
            "odom_nav_config/teb_local_planner_params.yaml": "polygon",
        }
        for relative, kind in configs.items():
            with self.subTest(profile=relative):
                cfg = yaml.safe_load((ROOT / "tetraDS_2dnav" / relative).read_text(encoding="utf-8"))
                shape = cfg["TebLocalPlannerROS"]["footprint_model"]
                self.assertNotIn("types", shape)
                self.assertEqual(shape["type"], kind)
                if kind == "line":
                    self.assertEqual(len(shape["line_start"]), 2)
                    self.assertEqual(len(shape["line_end"]), 2)
                else:
                    self.assertGreaterEqual(len(shape["vertices"]), 3)
                    self.assertTrue(all(len(point) == 2 for point in shape["vertices"]))

    def test_amcl_resample_interval_is_int(self):
        root = ET.parse(ROOT / "tetraDS_2dnav/launch/amcl.launch.xml").getroot()
        nodes = root.findall(".//node[@pkg='amcl']")
        self.assertEqual(len(nodes), 2)
        for node in nodes:
            with self.subTest(node=node.get("name")):
                p = node.find("param[@name='resample_interval']")
                self.assertIsNotNone(p)
                self.assertEqual(p.get("value"), "1")
                self.assertEqual(p.get("type"), "int")

    def test_filtered_odometry_launch_override(self):
        root = ET.parse(ROOT / "tetraDS_2dnav/launch/teb_move_base.launch").getroot()
        args = {x.get("name"): x.get("default") for x in root.findall("arg")}
        self.assertEqual(args["odom_topic"], "/$(arg namespace)/odometry/filtered")
        node = root.find("node[@pkg='move_base']")
        self.assertIsNotNone(node)
        children = list(node)
        loads = [i for i, x in enumerate(children) if x.tag == "rosparam" and "teb_local_planner_params.yaml" in x.get("file", "")]
        overrides = [i for i, x in enumerate(children) if x.tag == "param" and x.get("name") == "TebLocalPlannerROS/odom_topic" and x.get("value") == "$(arg odom_topic)"]
        self.assertEqual(len(loads), 1)
        self.assertEqual(len(overrides), 1)
        self.assertGreater(overrides[0], loads[0])

    def test_all_navigation_launch_xml_well_formed(self):
        for path in (ROOT / "tetraDS_2dnav/launch").glob("*.launch*"):
            with self.subTest(file=path.name):
                self.assertEqual(ET.parse(path).getroot().tag, "launch")


if __name__ == "__main__":
    unittest.main()
