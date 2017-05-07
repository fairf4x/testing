# testing
testing launch files and configurations for experiments with bebop drones

## main launch files

1. `indoor_teleop.launch` - teleoperated drone flight
2. `indoor_teleop_record.launch` - teleoperated flight and record everything into rosbag
3. `indoor_teleop_combo.launch` - teleoperated flight combining DSO and drone odometry with robot_localization
4. `indoor_teleop_gcalib.launch` - geometric calibration (not working properly - possibly due to some timing issue - see code)
5. `view_rosbag.launch` - play previously recorded rosbag (view video)

## prerequisities

TODO: explain in more detail

* bebop_autonomy
* DSO
* robot_localization
* joy_teleop

 
