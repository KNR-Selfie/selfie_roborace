#!/usr/bin/env python

import rospy
import tf
import math

from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

UPDATE_RATE = 50

class VehicleState:
    # Steering angle
    delta = 0.0

    # Position & orientation
    x = 0.0
    y = 0.0
    th = 0.0

    # Velocity
    vx = 0.0
    vy = 0.0
    vth = 0.0

wheelbase = None
max_steering_angle = None
max_steering_angle_velocity = None

steering_command = None

state = None
last_update = None

# Source: https://www.python.org/dev/peps/pep-0485/#proposed-implementation
def isclose(a, b, rel_tol=1e-09, abs_tol=1e-15):
    return abs(a-b) <= max(rel_tol * max(abs(a), abs(b)), abs_tol)

def handle_command(cmd):
    global steering_command
    steering_command = cmd.drive

def init_state(time):
    global state, last_update
    state = VehicleState()
    last_update = time

def update_state(time):
    global wheelbase
    global steering_command
    global state, last_update

    if steering_command is not None:
        v0 = steering_command.speed
        a0 = steering_command.acceleration
        j0 = steering_command.jerk

        time_diff = time - last_update

        dt = time_diff.secs + time_diff.nsecs * 1e-9
        ds = v0 * dt + a0 * dt**2/2 + j0 * dt**3/6

        v = v0 + a0 * dt + j0 * dt**2/2

        delta = steering_command.steering_angle

        if max_steering_angle > 0:
            delta = max(-max_steering_angle, min(delta, max_steering_angle))

        if max_steering_angle_velocity > 0:
            delta = math.max(state.delta - max_steering_angle_velocity * dt,
                             min(delta,
                             state.delta + max_steering_angle_velocity * dt))

        state.delta = delta
        if isclose(delta, 0.0):
            # Approximate movement as linear motion
            state.x += ds * math.cos(state.th)
            state.y += ds * math.sin(state.th)

        else:
            # Treat movement as circular motion

            # Calculate radius of curvature (positive sign = left turn)
            r = wheelbase / math.tan(delta)

            # Calculate coordinates of the center of curvature
            cx = state.x - r * math.sin(state.th)
            cy = state.y + r * math.cos(state.th)

            # Calculate current yaw and angular velocity
            state.th += ds / r
            state.vth = v / r

            # Calculate new position
            state.x = cx + r * math.sin(state.th)
            state.y = cy - r * math.cos(state.th)

        # Calculate instantaneous velocity
        state.vx = v * math.cos(state.th)
        state.vy = v * math.sin(state.th)

    last_update = time
    return state

def construct_odom_msg(status, time, odom_frame, rear_axis_frame):
    msg = Odometry()

    msg.header.stamp = current_time
    msg.header.frame_id = odom_frame
    msg.child_frame_id = rear_axis_frame

    orientation = tf.transformations.quaternion_from_euler(0, 0, status.th)
    msg.pose.pose = Pose(Point(state.x, state.y, 0),
                              Quaternion(*orientation))

    msg.twist.twist = Twist(Vector3(state.vx, state.vy, 0),
                                 Vector3(0, 0, state.vth))
    return msg

if __name__ == '__main__':
    rospy.init_node('selfie_steering_sim')

    # Read node parameters
    wheelbase = rospy.get_param('~wheelbase', 235) / 1000.0
    max_steering_angle = rospy.get_param('~max_steering_angle', math.pi / 4)
    max_steering_angle = rospy.get_param('~max_steering_angle_velocity', math.pi)
    odom_frame = rospy.get_param('~odom_frame', 'odom')
    rear_axis_frame = rospy.get_param('~rear_axis_frame', 'base_link')

    # Subscribe for steering commands
    drive_sub = rospy.Subscriber('drive', AckermannDriveStamped, handle_command, queue_size=1)

    # Configure topic publishers
    odom_pub = rospy.Publisher('odom', Odometry, queue_size=UPDATE_RATE)
    speed_pub = rospy.Publisher('speed', Float64, queue_size=UPDATE_RATE)

    # Configure transform broadcaster
    tf_br = tf.TransformBroadcaster()

    # Set update/publishing rate
    rate = rospy.Rate(UPDATE_RATE)

    init_state(rospy.Time.now())
    while not rospy.is_shutdown():
        current_time = rospy.Time.now()
        state = update_state(current_time)

        # Broadcast odometry transformation
        orientation = tf.transformations.quaternion_from_euler(0, 0, state.th)
        tf_br.sendTransform((state.x, state.y, 0),
                            orientation,
                            current_time,
                            rear_axis_frame,
                            odom_frame)

        # Publish to odometry topic
        odom_msg = construct_odom_msg(state, current_time, odom_frame, rear_axis_frame)
        odom_pub.publish(odom_msg)

        # Publish to speed topic
        speed = math.sqrt(state.vx**2 + state.vy**2)
        speed_pub.publish(speed)

        rate.sleep()
