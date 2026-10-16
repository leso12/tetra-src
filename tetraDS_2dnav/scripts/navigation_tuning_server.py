#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""ROS service for navigation configuration staging; no automatic restart."""
import json
import os
import rospy
import rospkg
from navigation_tuning_core import TuningStore, TuningError, PARAMS, snapshot
from tetraDS_2dnav.srv import (GetNavigationTuning, GetNavigationTuningResponse,
                               SetNavigationTuning, SetNavigationTuningResponse)

class Server(object):
    def __init__(self):
        root = rospkg.RosPack().get_path('tetraDS_2dnav')
        directory = os.path.join(root, 'config')
        self.store = TuningStore(directory, os.path.join(directory, 'nav_operator_overrides.yaml'))
        ns = rospy.get_namespace().strip('/')
        self.move_base = '/' + (ns + '/' if ns else '') + 'move_base'
        rospy.Service('nav_tuning/get', GetNavigationTuning, self.get)
        rospy.Service('nav_tuning/save', SetNavigationTuning, self.save)
        rospy.loginfo('Navigation tuning service ready: staged YAML writes only')

    def state(self):
        values, revision = self.store.read()
        active = {}
        exists = rospy.has_param(self.move_base + '/TebLocalPlannerROS')
        for key, paths in PARAMS.items():
            found = []
            for suffix in paths:
                param = self.move_base + '/' + suffix
                if rospy.has_param(param):
                    found.append(float(rospy.get_param(param)))
            if len(found) == len(paths) and all(abs(n - found[0]) < 1e-6 for n in found):
                active[key] = found[0]
        return snapshot(values, active, revision, exists)

    def get(self, _request):
        try:
            return GetNavigationTuningResponse(True, 'OK', self.state())
        except Exception as ex:
            rospy.logerr('Navigation tuning read: %s', ex)
            return GetNavigationTuningResponse(False, str(ex), '')

    def save(self, request):
        try:
            data = json.loads(request.settings_json)
            if not isinstance(data, dict) or set(data) != set(['values','revision']):
                raise TuningError('Provide values and revision only')
            self.store.save(data['values'], data['revision'])
            rospy.logwarn('Operator settings staged; navigation restart required for effect')
            return SetNavigationTuningResponse(True, '저장 완료. 안전 정지 후 내비게이션을 재시작해야 적용됩니다.', self.state())
        except Exception as ex:
            rospy.logwarn('Navigation tuning rejected: %s', ex)
            return SetNavigationTuningResponse(False, str(ex), '')

if __name__ == '__main__':
    rospy.init_node('navigation_tuning_server')
    Server()
    rospy.spin()
