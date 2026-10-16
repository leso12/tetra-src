"""Regression coverage for web-settable ROS navigation settings."""
import json
import os
import sys
import tempfile
import unittest
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tetraDS_2dnav' / 'scripts'))
from navigation_tuning_core import TuningStore, TuningError, PARAMS, snapshot

class NavigationTuningTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.file = os.path.join(self.temp.name, 'overrides.yaml')
        with open(self.file, 'w') as out:
            out.write('{}\n')
        self.store = TuningStore(str(ROOT / 'tetraDS_2dnav' / 'config'), self.file)

    def tearDown(self):
        self.temp.cleanup()

    def test_defaults_come_from_existing_configs(self):
        values, _ = self.store.read()
        self.assertEqual(values, {
            'lidar_mark':20.0,'lidar_clear':21.5,'depth_range':1.5,'cygbot_range':2.0,
            'local_inflation':0.5,'global_inflation':0.5,
            'teb_min_obstacle':0.5,'teb_inflation':0.5})

    def test_staged_save_and_revision(self):
        old, rev = self.store.read()
        updated = dict(old)
        updated.update(lidar_mark=12.0,lidar_clear=13.0,local_inflation=0.75)
        result, newrev = self.store.save(updated, rev)
        self.assertEqual(result, updated)
        self.assertNotEqual(rev, newrev)
        self.assertTrue(os.path.isfile(self.file + '.bak'))
        with self.assertRaises(TuningError):
            self.store.save(old, rev)
        import yaml
        with open(self.file) as inp:
            content = yaml.safe_load(inp)
        for path in PARAMS['lidar_mark']:
            part = content
            for name in path.split('/'):
                part = part[name]
            self.assertEqual(part, 12.0)

    def test_invalid_save_does_not_change_yaml(self):
        old, rev = self.store.read()
        for changes in ({'lidar_clear':10.0},{'depth_range':float('nan')},
                        {'cygbot_range':True},{'global_inflation':0.05},
                        {'teb_min_obstacle':0.9},{'extra':5.0}):
            with self.subTest(changes=changes):
                value = dict(old)
                value.update(changes)
                with self.assertRaises(TuningError):
                    self.store.save(value, rev)
        self.assertEqual(self.store.read(), (old,rev))

    def test_saved_not_equal_live_is_pending(self):
        old, rev = self.store.read()
        self.assertTrue(json.loads(snapshot(old, {}, rev, False))['pending_restart'])
        self.assertFalse(json.loads(snapshot(old, old, rev, True))['pending_restart'])
        changed = dict(old)
        changed['lidar_mark'] = 12.0
        self.assertTrue(json.loads(snapshot(changed, old, rev, True))['pending_restart'])
if __name__ == '__main__':
    unittest.main()
