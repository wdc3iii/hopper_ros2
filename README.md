# ROS2 Local Mapping Package for ARCHER

This pacakge implements a sensing-based local mapping package using a combination of Intel's t265 tracking camera and d435 depth camera. 

## Launching procedure

Source ROS2 and this workspace (activate virtual environment first, if it is used). I added the following line to my `~/.bashrc` to source everythin with `source_hopper`:
```
alias source_hopper='mamba activate trav_seg && source /opt/ros/humble/setup.bash && source /home/noelcs/repos/t265_ws/install/setup.bash && source /home/noelcs/hopper_ws/install/setup.bash'
```

To launch the relevant nodes, run
```
ros2 launch local_mapper launch_local_mapper.py
```

This launches the t265 tracking camera, publishes two static tranforms relating the hopper frame to the camera frames, and launch the node which processes camera data to free space polytopes. If true, ```viz_poly``` publishes the polytopes to RViz. If true, ```publish_occ``` publishes the occupancy grid to RViz. If true, ```local_prompt``` prompts the segmenter from this launch file. Otherwise, on a machine connected to the ROS2 network, run
```
ros2 run local_mapper_client seg_prompt_client_node
```
This node runs an action server which can trigger re-prompting of the segmenter model, and does so by default on startup.

RViz can be launched with
```
ros2 launch local_mapper_client launch_mapper_client.py
```
to visualize the robot frames, occupancy grid, and free-space polytopes if desired.


## Launching Spoof Node
To test downstream processing of free polytopes without running the entire stack, the `local_mapper_client` provides a node to spoof polytope data. Run the node via
```
ros2 run local_mapper_client fake_poly_node
```
The polytopes it publishes can be edited directly in `local_mapper_client/fake_poly_node.py`. 


## Networking??

```
nmcli connection modify 'Wired connection 1' ipv4.addresses 192.168.1.1/24 ipv4.method manual
```