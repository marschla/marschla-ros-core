# this repositry contains two packages:

## marschla_lane_following

This contains the controller + the launch file, that launches the necessary dt-core nodes and the controller. The controller can be started using the keyboard_control. The Programm is started like a normal demo.

## marschla_node_launchers

This contains only the launch file, that launch the necessary nodes for lane_following, but does not start the controller. The controller has to be started seperately. This is useful, if one is tuning the controller and does not want to restart all nodes for everytime the controller is rebuildt.
