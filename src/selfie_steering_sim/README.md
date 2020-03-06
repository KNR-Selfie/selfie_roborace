# Steering Simulator

`selfie_steering_sim` package provides a node with the same name that takes in steering commands and calculates expected vehicle position over time based on [Ackermann geometry](https://en.wikipedia.org/wiki/Ackermann_steering_geometry). The simulator operates under ideal world assumptions, so it doesn't take into account tire slippage or similar error sources. The position is published both in the `odom` topic and as a [tf2](http://wiki.ros.org/tf2) transformation.

To run the simulator, execute the following command:
```
rosrun selfie_steering_sim main.py [params...]
```

## `selfie_steering_sim`

## Subscribed topics

`drive` ([ackermann_msgs/AckermannDriveStamped](http://docs.ros.org/api/ackermann_msgs/html/msg/AckermannDriveStamped.html))
Steering commands to be applied.

## Published topics

`sim/odom` ([nav_msgs/Odometry](http://docs.ros.org/melodic/api/nav_msgs/html/msg/Odometry.html))

## Parameters

`~wheelbase` (`float`, default: 235)
Vehicle wheelbase in millimeters.

`~max_steering_angle` (`float`, default: π/4)
Maximum absolute value of the steering angle, in radians. If there is a larger value present in an incoming command, it will be clamped. Set to zero to omit this constraint.

`~max_steering_angle_velocity` (`float`, default: π)
Maximum absolute value of the steering angle, in radians per second. If there is a larger value present in an incoming command, it will be clamped. Set to zero to omit this constraint.

`~odom_frame` (`string`, default: odom)
The name of the odometry frame.

`~rear_axis_frame` (`string`, default: base_link)
The name of the rear axis frame.

## Provided tf transforms

`~odom_frame` → `~rear_axis_frame`
