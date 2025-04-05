# ROS2 OLED SSD1306 for Waver

## Description

This ROS2 package made in C++ provides a way to drive an SSD1306 128x32 Oled display with I2C interface. Based on the work of Andrew Duncan, this package has been modified to work with ROS2 and includes a simple example of how to use the driver to display text and graphics on the OLED screen.

## Configuration

```
cd <ros2_workspace>/src
colcon build --packages-select waver_oled_ssd1306
```

## Usage

```
ros2 run waver_oled_ssd1306 waver_oled_ssd1306_node
ros2 topic pub /oled_text std_msgs/msg/String "{data: 'Hello, OLED!'}"
```
