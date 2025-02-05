
# Building

```bash
cd <ros_ws>/src/apollo_2_0
rosdep install --from-paths . --ignore-src -r -y
chmod +x setup.sh
./setup.sh
colcon build
```

# Mapping

```bash
ros2 launch apollo_2_0 cartographer_mapping.launch.py 
# Default {"use_sim_time":True}
```


# Save Map

```bash
ros2 run nav2_map_server map_saver_cli -f ~/apollo_ws/src/apollo_2_0/config/map
```