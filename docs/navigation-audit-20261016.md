# TETRA ROS navigation code audit (2026-10-16)

## Baseline and scope
- Main branch starting commit: 360d960090104d0c684a1bc18d94c3e2fcea673c.
- Reviewed: tetraDS_2dnav launch/config/odom_nav_config, tetraDS/src/tetraDS.cpp, navi_goals/src/waypoints.cpp, tetraDS_TCP/src/tetraDS_TCP.cpp, and sh_files/startup.sh/navigation.sh.
- Launch flow: startup.sh -> tetra_configuration.launch (LiDAR, EKF, motors and sensors); navigation.sh -> move_base_tetra.launch -> teb_move_base.launch + map_server + amcl.launch.xml.
- Changes in this PR: navigation configuration only, tests, and documentation. Existing motor, docking, TCP, waypoint logic, velocity/acceleration limits, maps, and obstacle safety distances remain intact.

## Findings by priority

| Priority | File | Finding | Decision |
| --- | --- | --- | --- |
| P1 | config/teb_local_planner_params.yaml | TEB expects footprint_model/type, but the active profile says types: "line"; the optimizer may instead use the default point footprint. | Correct to type: "line" without changing the line coordinates. |
| P1 | odom_nav_config/teb_local_planner_params.yaml | Alternate profile also uses types: "polygon". This profile is not loaded by the normal navigation launch. | Correct the typo, retain polygon vertices. |
| P1 | launch/teb_move_base.launch | odom_topic arg was declared but never passed to TEB. Its YAML uses raw odom even though filtered odometry is brought up by EKF. | Add explicit TEB private-parameter override after YAML is loaded. |
| P1 | launch/amcl.launch.xml | resample_interval is a float literal ("1.0"), whereas AMCL expects an integer. | Specify type=int value=1 in both launch branches. |
| P0 follow-up | tetraDS/src/tetraDS.cpp | The inspected velocity loop reuses command variables and has no timestamp-based stale cmd_vel watchdog. Joystick, virtual joystick and autonomous commands share control. | Next PR with simulator/stop-distance tests; not changed on live driver without testing. |
| P1 follow-up | navi_goals/src/waypoints.cpp | waitForResult(duration) does not cancel a timed-out goal before the next waypoint. | Next PR for timeout cancellation and explicit stop/skip/retry policy. |
| P1 follow-up | tetraDS_TCP/src/tetraDS_TCP.cpp | Socket parser uses fixed-size arrays and unbounded strcpy on input; commands reach navigation and GPIO services. | Separate protocol-compatible hardening PR. |
| P2 follow-up | costmap_common_params.yaml vs active TEB | Costmap footprint is a full polygon, but TEB intentionally uses a shorter center line. Fixing the erroneous key activates that line and changes planner collision geometry from potential point fallback. | Verify corner clearance in Gazebo before migrating to a polygon and before deploying this PR. |
| P2 follow-up | teb_local_planner_params.yaml | Very low max_vel_theta, backward speed and conservative min_obstacle_dist can slow or restrict navigation. | Leave untuned until logged path/clearance/CPU benchmarks. |
| P2 follow-up | move_base_params.yaml | Recovery behaviors disabled, planner_frequency=0. | Preserve behavior pending recovery/replanning acceptance tests. |

## Automated static validation
Run: python3 -m pip install PyYAML
Then: python3 -m unittest discover -s tests -p 'test_navigation_config.py' -v

Tests verify both TEB footprint types, the two AMCL branches, launch XML syntax, and that the TEB filtered-odom override is applied after YAML loading. CI exercises these checks on affected PRs.

## Acceptance / rollback: simulation first
The corrected TEB footprint type can **change obstacle clearance**; the odom override changes TEB's velocity estimate source. Neither is approved for unattended deployment without tests.

In the TETRA_NS environment, inspect:
- rostopic hz /TETRA_NS/scan
- rostopic hz /TETRA_NS/odometry/filtered
- rosparam get /TETRA_NS/move_base/TebLocalPlannerROS/footprint_model/type (expected line for active profile)
- rosparam get /TETRA_NS/move_base/TebLocalPlannerROS/odom_topic (expected /TETRA_NS/odometry/filtered)
- rosrun tf tf_echo /map /TETRA_NS/base_footprint

On simulator/test stand: initial pose, goal dispatch/cancel, static and moving obstacles, narrow aisle and reverse, loss of scan/odom/TF, docking, joystick takeover, E-stop, controller restart. Record commanded vs actual velocity, clearance, TF discontinuities and stop distance. Check that an odometry outage does **not** produce unsafe motion; this PR does not add a watchdog.

Rollback: leave main unmodified until acceptance. Do not merge if runtime or safety tests fail; revert the three configuration files and one launch file if needed. Deployment should require a reviewer and an operator-controlled test plan.

## Suggested follow-on PR sequence
1. Motion-control safety: explicit command source ownership, stale-command watchdog, validated halt behavior.
2. Navigation mission reliability: cancel timed-out goals, track action status, configurable retry/abort.
3. Localization and TF: rosbag verification of map->odom->base_footprint, scan frame, covariance/time alignment.
4. Planner tuning: simulate line vs polygon footprint; compare obstacle distances, CPU usage, speed, and recovery behavior before adjusting the actual robot.
