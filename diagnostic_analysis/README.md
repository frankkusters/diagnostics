Diagnostic Analysis <!-- omit in toc -->
=================================

The `Diagnostic_analysis` package converts the rosbag `/diagnostic` topic messages to csv file.

Building
--------
Run:
colcon build --packages-select diagnostic_analysis


Usage
-----
Run:
ros2 run diagnostic_analysis exporter `rosbag_path` -d `output_path`