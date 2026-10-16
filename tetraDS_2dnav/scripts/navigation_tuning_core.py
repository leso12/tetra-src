#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""ROS Melodic-compatible YAML staging; no live navigation modifications."""
from __future__ import division
import hashlib
import json
import math
import os
import shutil
import tempfile
import threading
import yaml

# name => (baseline filename, nested path, min m, max m)
FIELDS = {
    'lidar_mark': ('costmap_common_params.yaml', ('obstacle_layer','laser_scan_sensor','obstacle_range'), 0.5,25.0),
    'lidar_clear': ('costmap_common_params.yaml', ('obstacle_layer','laser_scan_sensor','raytrace_range'), 0.5,25.0),
    'depth_range': ('costmap_common_params.yaml', ('rgbd_obstacle_layer2','obstacle_range'), 0.3,3.0),
    'cygbot_range': ('costmap_common_params.yaml', ('cygbot_obstacle_layer','obstacle_range'), 0.3,4.0),
    'local_inflation': ('local_costmap_params.yaml', ('local_costmap','inflation_layer','inflation_radius'), 0.25,1.5),
    'global_inflation': ('global_costmap_params.yaml', ('global_costmap','inflation_layer','inflation_radius'), 0.25,1.5),
    'teb_min_obstacle': ('teb_local_planner_params.yaml', ('TebLocalPlannerROS','min_obstacle_dist'), 0.25,1.2),
    'teb_inflation': ('teb_local_planner_params.yaml', ('TebLocalPlannerROS','inflation_dist'), 0.25,1.5),
}
# All paths relative to /<ROS_NAMESPACE>/move_base.
PARAMS = {
    'lidar_mark': ['global_costmap/obstacle_layer/laser_scan_sensor/obstacle_range',
                   'local_costmap/obstacle_layer/laser_scan_sensor/obstacle_range'],
    'lidar_clear': ['global_costmap/obstacle_layer/laser_scan_sensor/raytrace_range',
                    'local_costmap/obstacle_layer/laser_scan_sensor/raytrace_range'],
    'depth_range': ['local_costmap/rgbd_obstacle_layer2/obstacle_range'],
    'cygbot_range': ['local_costmap/cygbot_obstacle_layer/obstacle_range'],
    'local_inflation': ['local_costmap/inflation_layer/inflation_radius'],
    'global_inflation': ['global_costmap/inflation_layer/inflation_radius'],
    'teb_min_obstacle': ['TebLocalPlannerROS/min_obstacle_dist'],
    'teb_inflation': ['TebLocalPlannerROS/inflation_dist'],
}

def nested_get(tree, path):
    for part in path:
        tree = tree[part]
    return tree

def nested_set(tree, path, value):
    for part in path[:-1]:
        tree = tree.setdefault(part, {})
    tree[path[-1]] = value

class TuningError(Exception):
    pass

def validate(values):
    if not isinstance(values, dict) or set(values) != set(FIELDS):
        raise TuningError('Expected all 8 approved parameters, and no extras')
    clean = {}
    for key, (_, _, lo, hi) in FIELDS.items():
        val = values[key]
        if isinstance(val, bool) or not isinstance(val, (int, float)):
            raise TuningError('%s must be a number' % key)
        num = float(val)
        if math.isnan(num) or math.isinf(num) or not lo <= num <= hi:
            raise TuningError('%s out of range: %.2f to %.2f metres' % (key, lo, hi))
        clean[key] = num
    if clean['lidar_clear'] < clean['lidar_mark']:
        raise TuningError('LiDAR clearing range must be >= marking range')
    if clean['teb_inflation'] < clean['teb_min_obstacle']:
        raise TuningError('TEB inflation must be >= minimum obstacle distance')
    return clean

class TuningStore(object):
    def __init__(self, config_dir, override_path):
        self.override_path = override_path
        self.lock = threading.RLock()
        self.defaults = {}
        for key, (filename, path, _, _) in FIELDS.items():
            with open(os.path.join(config_dir, filename)) as f:
                self.defaults[key] = float(nested_get(yaml.safe_load(f), path))
        validate(self.defaults)

    def _read(self):
        with open(self.override_path, 'rb') as f:
            raw = f.read()
        overlay = yaml.safe_load(raw) or {}
        if not isinstance(overlay, dict):
            raise TuningError('Invalid YAML overrides')
        values = dict(self.defaults)
        for key, paths in PARAMS.items():
            try:
                values[key] = nested_get(overlay, paths[0].split('/'))
            except (KeyError, TypeError):
                pass
        return validate(values), hashlib.sha256(raw).hexdigest()

    def read(self):
        with self.lock:
            return self._read()

    def save(self, values, revision):
        with self.lock:
            if self._read()[1] != revision:
                raise TuningError('Settings changed: please reload before saving')
            values = validate(values)
            overrides = {}
            for key, paths in PARAMS.items():
                for path in paths:
                    nested_set(overrides, path.split('/'), values[key])
            directory = os.path.dirname(os.path.abspath(self.override_path))
            fd, staged = tempfile.mkstemp(prefix='.tuning_', suffix='.tmp', dir=directory)
            try:
                with os.fdopen(fd, 'w') as f:
                    yaml.safe_dump(overrides, f, default_flow_style=False)
                    f.flush()
                    os.fsync(f.fileno())
                os.chmod(staged, 0o644)
                shutil.copy2(self.override_path, self.override_path + '.bak')
                os.rename(staged, self.override_path)
            finally:
                if os.path.exists(staged):
                    os.unlink(staged)
            return self._read()

def snapshot(values, active, revision, nav_running):
    pending = not nav_running
    if nav_running:
        pending = any(active.get(k) is None or abs(active[k] - v) > 1e-6 for k,v in values.items())
    return json.dumps({'values':values, 'active':active, 'revision':revision,
                       'navigation_running':nav_running, 'pending_restart':pending,
                       'bounds':{k:[spec[2],spec[3]] for k,spec in FIELDS.items()}})
