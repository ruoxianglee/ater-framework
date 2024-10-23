## Dependencies
- OS: Ubuntu 22 (tested)
- [bebeltrace2](https://github.com/efficios/babeltrace)
  ```bash
  sudo apt install babeltrace2
  ```
- uuid
    ```bash
    sudo apt-get install uuid-dev
    ```
- [jsonbuilder](https://github.com/microsoft/jsonbuilder/tree/master/include/jsonbuilder)
- [lttng-consume](https://github.com/microsoft/lttng-consume)
- [ROS 2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
- CARET with Live Tracing Mode
- Customized Autoware Reference System

## Set Evaluation Environment
### Isolate CPUs (5)
1. Open GRUB file: 
  ```sudo nano /etc/default/grub```
2. Modify: 
   ```GRUB_CMDLINE_LINUX_DEFAULT="quiet splash isolcpus=3,4,5,6,7”```
3. Update GRUB: 
   ```sudo update-grub```
4. Reboot: 
   ```sudo reboot```
5. Check: 
   ```grep isolcpus /proc/cmdline```
6. Check single CPU: 
   ```ps -eLo psr | grep <core_number> | wc -l```

## Build 
### Build Caret
```
cd ros2_caret_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### Build ROS 2 Application (Autoware-reference System)
```
source /opt/ros/humble/setup.bash
source ~/ros2_caret_ws/install/local_setup.bash
colcon build --symlink-install --packages-up-to autoware_reference_system --cmake-args -DBUILD_TESTING=OFF
```

## Tracing
### Tracing with **non-live mode**
```
source ~/ros2_caret_ws/install/local_setup.bash
export ROS_TRACE_DIR=~/ros2_ws/evaluate

ros2 caret record -s your-session-name
```

### Tracing with **live mode**
```
source ~/ros2_caret_ws/install/local_setup.bash
ros2 caret record -s ATER -m
```

## Launch ATER
```
source ~/ros2_caret_ws/install/setup.bash

python3 ATER.py net://localhost/host/your-host-name/ATER True path-to-save-message-drops/drops.txt path-to-save-cputime/cputime.txt path-to-save-throughput/throughput.txt
```
### Parameter
- True: baseline
- False: ATER

## Launch Target Application (Autoware Reference System)
```
source ~/ros2_caret_ws/install/local_setup.bash
source ~/ros2_ws/install/local_setup.bash
export LD_PRELOAD=$(readlink -f ~/ros2_caret_ws/install/caret_trace/lib/libcaret.so)

ros2 run autoware_reference_system autoware_system_1 > path_to_save_latency_results
```
