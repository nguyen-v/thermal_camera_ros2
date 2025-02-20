# Thermal Camera Node

This ROS2 package provides a node to capture and process thermal camera images. The node performs the following functions:

- **Image Capture & Processing:** Reads video frames from a specified device, applies scaling, contrast adjustment, optional blur, and a colormap.
- **Temperature Calculation:** Extracts and computes a temperature value from thermal data.
- **Overlays:** Draws a crosshair at the image center and overlays a HUD (Heads-Up Display) with camera settings and temperature.
- **Configurable Rotation:** Rotates the thermal image (0, 90, 180, or 270°) while keeping the overlay text in the correct orientation.

## Features

- **HUD & Crosshair:** Overlay with configurable options.
- **Dynamic Parameters:** All settings (HUD, crosshair, device number, scale, contrast, blur, colormap, temperature threshold, orientation) are configurable via ROS parameters.

Tested on a Raspberry Pi 5.

## Prerequisites

- ROS2 (Foxy, Galactic, Humble, or later)
- Python 3
- OpenCV
- `cv_bridge`

## Installation & Build Instructions

1. **Clone the Repository:**

   ```bash
   cd ~/ros2_ws/src

   git clone <repository_url> thermal_camera

2. **Build the package**

    ```bash
    cd ~/ros2_ws

    colcon build --packages-select thermal_camera

3. **Source the workspace**

    ```bash
    source ~/ros2_ws/install/setup.bash

## Running the node with params

```bash
ros2 run thermal_camera thermal_camera_node --ros-args -p hud:=True -p device:=9 -p orientation:=270
```


## Running the node with params (launch file)

```bash
ros2 launch thermal_camera thermal_camera_launch.py hud:=True device:=9 orientation:=270
```

## Credits
Based on [PyThermalCamera](https://github.com/leswright1977/PyThermalCamera).

