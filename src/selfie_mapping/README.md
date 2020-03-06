# Selfie mapping
`Selfie mapping` package provides Google Cartographer's launch and configuration files.
## Launching
To build a map you can choose between two options. Each of them can use one or two lidars.
#### Fast mapping
Create a map as fast as possible with no visualisation.
* one lidar
```
roslaunch selfie_mapping fast_mapping_1lidar.launch bag_filenames:=${HOME}/path/bag_filename.bag
```
* two lidars
```
roslaunch selfie_mapping fast_mapping_2lidar.launch bag_filenames:=${HOME}/path/bag_filename.bag
```

#### Viz mapping
Launch mapping with real time visualisation in Rviz.
* one lidar
```
roslaunch selfie_mapping viz_mapping_1lidar.launch bag_filename:=${HOME}/path/bag_filename.bag
```
* two lidars
```
roslaunch selfie_mapping viz_mapping_2lidar.launch bag_filename:=${HOME}/path/bag_filename.bag
```

## Map saving
To save map use the following command:
```
rosrun map_server map_saver --occ 58 --free 48 -f map_name
```
The map will be saved in the current path.

## Subscribed topics
`odom` ([nav_msgs/Odometry](http://docs.ros.org/melodic/api/nav_msgs/html/msg/Odometry.html))

`scan` ([sensor_msgs/LaserScan](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/LaserScan.html))

`tf` ([tf/tfMessage](http://docs.ros.org/melodic/api/tf/html/msg/tfMessage.html))

## Published topics

`map` ([nav_msgs/OccupancyGrid](http://docs.ros.org/melodic/api/nav_msgs/html/msg/OccupancyGrid.html))


## Provided tf transforms

`~map` → `~odom`
