# Autonomous Yardwork Robot

This project is the code base for an autonomous ywardwork robot. Currently the focus is on snow removal capabilities. The software architecture is based on the turtlebot3 platform, with changes to work with the custom built robot.

## Dependencies

- depthimage_to_laserscan (https://github.com/ros-perception/depthimage_to_laserscan)
- imu_tools (https://github.com/CCNYRoboticsLab/imu_tools/tree/melodic)
- navigation (https://github.com/ros-planning/navigation/tree/melodic-devel)
- path_coverage_ros (https://gitlab.com/Humpelstilzchen/path_coverage_ros)
- realsense-ros (https://github.com/IntelRealSense/realsense-ros)
- robot_state_publisher (https://github.com/ros/robot_state_publisher/tree/melodic-devel)
- rosserial (https://github.com/ros-drivers/rosserial/tree/melodic-devel)
- rtabmap_ros (https://github.com/introlab/rtabmap_ros/tree/melodic-devel)
- teleop_twist_joy (https://github.com/ros-teleop/teleop_twist_joy/tree/melodic-devel)

## Executing program

### Mapping

To run mapping:
- Upload Arduino_Only_OneStick_Speed_Attachment to the Arduino
- Run ywr_mapping.launch

### Snow Removal



## Authors

Hunter Peeters
Matthew Bugeya

## License

This project is open for anyone to reuse for their own purpose. 
To this end, the Authors listed above take no responsibility for any damages caused by the use or misuse of this software

## Acknowledgments

