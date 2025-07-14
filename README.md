# fleet_adapter_demo

This package aims to test the custom [fleet_adapter](https://github.com/Thales-Silva/fleet_adapter).

## 1. Installation

Install the OpenRMF binaries with
```bash
sudo apt install ros-jazzy-rmf-dev
```

Clone all this repositories to some workspace inside the robot (let it be ```some_ws``` as an example), then build it with the shell commands
```bash
cd ~/some_ws/src
git clone https://github.com/Thales-Silva/fleet_adapter_demo.git
git clone https://github.com/Thales-Silva/fleet_adapter.git
cd ~/some_ws
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
source ~/some_ws/install/setup.bash
``` 

The export compile commands is optional. It generates a ```compile_commands.json``` in the build folder, which can be used by clangd, assisting the user while editing code with vscode.

Optionally, add the source command to your ```.bashrc``` with 
```bash
echo "source ~/some_ws/install/setup.bash" >> ~/.bashrc
```

## 2. Testing

To test it, perform the following shell commands to run the demonstration
```bash
ros2 launch fleet_adapter_demo fleet_adapter_demo.launch.py
``` 

Then, do
```bash
ros2 run rmf_demos_tasks dispatch_patrol -p north_west north_east south_east south_west -n 2 -st 0
```
With this command, the robot is expected to navigate to all of the four goals.