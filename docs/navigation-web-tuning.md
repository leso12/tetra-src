# TETRA navigation tuning web panel (prototype, ROS Melodic)

**Scope**: This adds a usable settings screen under \`web_test/html_tetra/navigation_settings.html\`. The existing deployed \`/home/tetra/work/tetra-single-api\` frontend is not present in this repository; this PR does **not** replace it. The original example buttons, map and docking functions remain unchanged.

## Supported staged settings
The UI displays staged and observed ROS parameter values (in metres):
- LiDAR obstacle marking and raytracing ranges (shared global/local Costmap)
- Depth camera STVL obstacle range (local)
- Cygbot STVL obstacle range (local)
- Local and Global Costmap inflation radii
- TEB minimum obstacle distance and TEB inflation distance

**Important:** LiDAR marking distance != physical sensor range != safe braking distance. These user settings do not replace the hardware safety scanner or emergency stop.

The ROS backend provides \`/<ROS_NAMESPACE>/nav_tuning/get\` and \`/<ROS_NAMESPACE>/nav_tuning/save\` (service types \`tetraDS_2dnav/GetNavigationTuning\` and \`tetraDS_2dnav/SetNavigationTuning\`). It validates numeric bounds and cross-field invariants, checks a revision to prevent stale writes, writes **only** \`tetraDS_2dnav/config/nav_operator_overrides.yaml\` atomically, and backs up the prior version as \`.bak\`. Baseline YAMLs are not edited. No dynamic-reconfigure or active ROS parameters are changed while saving.

The default \`{ }\` override has no effect. \`teb_move_base.launch\` loads operator overrides only during move_base startup after all base configs. **A normal navigation restart, with the robot safely stopped, is required.** The server does not cancel missions, restart move_base or send motion commands. If the package config directory is not writable by the ROS runtime user, saves fail safely; provision write access intentionally.

## Development / test
\`\`\`bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
python3 -m unittest discover -s src -p 'test_navigation_tuning.py' -v
\`\`\`
For the repository root, the offline test command is \`python3 -m unittest discover -s tests -p 'test_navigation*.py' -v\`. GitHub Actions runs these Python tests; no ROS1 runtime execution is provided in CI.

ROS services are launched with \`tetra_configuration.launch\` alongside the existing rosbridge node. On an isolated test LAN, serve \`web_test/html_tetra\` over HTTP, open \`navigation_settings.html\`, and enter the robot's accessible ROSBridge URL (for example, \`ws://192.168.33.2:9090\`) and \`TETRA_NS\`. Never expose unauthenticated ROSBridge on the public internet. Provide authentication and network isolation before production use.

## Acceptance
1. Simulator: verify the get/save services exist, default values match existing YAML, and rejection of invalid values works.
2. Save a new LiDAR marking range, load the page again; verify the saved value persists and UI explicitly says \`재시작 필요\`.
3. Verify the **running** ROS Costmap parameter stays unchanged before navigation restart.
4. Safely stop and restart navigation on the simulator, verify both local/global obstacle-layer params reflect the staged values. Check the active values displayed in the UI.
5. Verify paths, collision clearance, obstacle markings, unknown-space behavior, TEB footprint model, localization and docking. Never infer collision safety from passing a configuration test.
6. Test stale-web-session saves, missing ROSBridge, malformed JSON, nonnumeric values and unwritable configuration directory.

## Rollback
Restore \`nav_operator_overrides.yaml\` from its \`.bak\` or replace it with \`{}\`; then restart navigation safely. Do not merge or deploy until qualified test-bench results are reviewed.

## Existing web UI constraints
The example \`html_tetra/robot_control_TETRA.js\` directly publishes \`cmd_vel\`. A separate safety PR should implement server-side command arbitration, motion-command watchdog and reliable release/blur stop semantics before redesigning teleoperation.
