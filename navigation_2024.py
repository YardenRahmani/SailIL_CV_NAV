#!/usr/bin/env python

import rospy
from math import atan2, degrees
from cv_from_zed.msg import ObjectDistanceInfo
from std_msgs.msg import String
from collections import defaultdict

# Define global variables to store objects' positions and labels
object_positions = defaultdict(list)

# Constants
DISTANCE_THRESHOLD = 8
TIMEOUT = 5.0
FREQ = 1
ANGLE_LEFT = 315
ANGLE_RIGHT = 45
ANGLE_FORWARD = 0

class TimerManager:
    def __init__(self):
        self.timer = None
        self.timer_running = False

    def start_timer(self, duration):
        if not self.timer_running:
            self.timer = rospy.Timer(rospy.Duration(duration), timer_callback)
            self.timer_running = True

    def stop_timer(self):
        if self.timer_running:
            self.timer.shutdown()
            self.timer_running = False

def position_callback(msg):
    object_positions[msg.label].append((
        msg.distance_x,  # Horizontal
        msg.distance_y,  # Vertical
        msg.distance_z,  # Depth
    ))

def timer_callback(event):
    rospy.signal_shutdown("No objects detected")
    rospy.loginfo("Finish")

def find_closest(obj_list):
    closest_distance = float('inf')
    nearest_obj = None
    for obj in obj_list:
        if obj[2] < DISTANCE_THRESHOLD and obj[2] < closest_distance:
            closest_distance = obj[2]
            nearest_obj = obj
    return nearest_obj

def get_angle(p1, p2=None):
    if p2:
        middle_point = ((p1[0] + p2[0]) / 2, (p1[1] + p2[1]) / 2, (p1[2] + p2[2]) / 2)
    else:
        middle_point = p1
    angle = round(degrees(atan2(middle_point[0], middle_point[2])))
    if angle < 0:
        angle += 360
    return angle

def get_bounds():
    nearest_red = find_closest(object_positions['Red ball'])
    nearest_green = find_closest(object_positions['Green ball'])
    nearest_black = find_closest(object_positions['Black ball'])
    nearest_yellow = find_closest(object_positions['Yellow ball'])

    if not (nearest_yellow or nearest_black):
        return nearest_red, nearest_green
    elif nearest_black and nearest_yellow:
        if nearest_black[2] < nearest_yellow[2]:
            return nearest_black, nearest_green
        else:
            return nearest_red, nearest_yellow
    elif nearest_black:
        return nearest_black, nearest_green
    else:
        return nearest_red, nearest_yellow

def publish_angle(steering_pub, angle):
    steering_pub.publish(f"{angle}")

def navigate():
    rospy.init_node('navigation_node', anonymous=True)
    rospy.Subscriber('object_distance_info', ObjectDistanceInfo, position_callback)
    steering_pub = rospy.Publisher('steering_directions', String, queue_size=10)
    timer = TimerManager()
    rate = rospy.Rate(FREQ)
    last_seen = None

    while not rospy.is_shutdown():
        if not object_positions['end']:
            rate.sleep()
            continue

        left_bound, right_bound = get_bounds()

        if left_bound and right_bound:
            angle = get_angle(left_bound, right_bound)
            publish_angle(steering_pub, angle)
            timer.stop_timer()
        elif left_bound:
            angle = get_angle(left_bound)
            if angle < 90 or angle > 345:
                publish_angle(steering_pub, ANGLE_RIGHT)
            else:
                publish_angle(steering_pub, ANGLE_FORWARD)
            timer.stop_timer()
        elif right_bound:
            angle = get_angle(right_bound)
            if angle > 270 or angle < 15:
                publish_angle(steering_pub, ANGLE_LEFT)
            else:
                publish_angle(steering_pub, ANGLE_FORWARD)
            timer.stop_timer()
        else:
            timer.start_timer(TIMEOUT)
            if last_seen == 'r':
                publish_angle(steering_pub, ANGLE_RIGHT)
            elif last_seen == 'g':
                publish_angle(steering_pub, ANGLE_LEFT)
            else:
                publish_angle(steering_pub, ANGLE_FORWARD)

        if left_bound and right_bound:
            last_seen = 'r&g'
        elif left_bound:
            last_seen = 'r'
        elif right_bound:
            last_seen = 'g'

        object_positions.clear()
        rate.sleep()

if __name__ == '__main__':
    try:
        navigate()
    except rospy.ROSInterruptException:
        pass