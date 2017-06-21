# testing
testing launch files and configurations for experiments with bebop drones

## main launch files

1. `teleop.launch` - teleoperated drone flight
2. `teleop_record.launch` - teleoperated flight and record everything into rosbag
3. `teleop_localization.launch` - teleoperated flight combining DSO and drone odometry with robot_localization
4. `geometric_calibration.launch` - geometric calibration (not working properly - possibly due to some timing issue - see code)
5. `view_rosbag.launch` - play previously recorded rosbag (view video)

## prerequisities

TODO: explain in more detail

* bebop_autonomy - [ros package on github](https://github.com/AutonomyLab/bebop_autonomy)
* DSO - [ros package on github](https://github.com/JakobEngel/dso)
* robot_localization - ros package
* joy_teleop - ros package
