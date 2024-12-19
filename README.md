# ros2_lifecycle
ros2 lifecycle example &amp;&amp; lifecycle monitor

### 1. lifecycle demo
ros2 lifecycle demo:
- build && run this demo
```
cd lifecycle_demo
colcon build
source install/setup.bash
ros2 run lifecycle_demo lifecycle_demo
```
- in other terminal
```
ros2 lifecycle set /sensorLifecycleNode configure
ros2 lifecycle set /sensorLifecycleNode activate
ros2 lifecycle set /sensorLifecycleNode deactivate
ros2 lifecycle set /sensorLifecycleNode cleanup
ros2 lifecycle set /sensorLifecycleNode shutdown
```
### 2. lifecycle monitor
ros2 monitor lifecycle node
- build && run this demo
```
cd lifecycle_monitor
colcon build
source install/setup.bash
ros2 run lifecycle_monitor lifecycle_monitor
```
- run step1 and translate status to activate or deactivate