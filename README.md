# RViz hitl SLAM interaction tool

RViz plugin to publish human interaction for human in the loop SLAM

## Dependencies
Install [ROS](http://wiki.ros.org/ROS/Installation)

## Build
1. Add the project path to `ROS_PACKAGE_PATH`
1. Run `make`

## Run
1. Run rviz as usual:
    ```
    rosrun rviz rviz
    ```
1. Add the plugin to RViz using the "+" button on the toolbar, and select "HitlSlamTool". If you do not see the tool listed, ensure that this repo is in your `ROS_PACKAGE_PATH`.
1. To use the tool, click on the "Hitl Slam Tool" button.
1. Click and drag to select first line.
1. Click and drag to select second line.
1. After the second line is selected, the results are published to the topic `/hitl_slam_input`, of type `rviz_hitl_slam_tool/HitlSlamInputMsg`, which is also defined by this package.
