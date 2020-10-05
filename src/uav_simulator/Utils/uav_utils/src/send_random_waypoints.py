#!/usr/bin/env python2

import rospy
from geometry_msgs.msg import PoseStamped
import random


class WaypointFeed(object):

    def __init__(self):
        print('[INFO] Initializing object...')
        # COMBAK: temporarily setting queue_size to 1
        self.waypoint_pub = rospy.Publisher('/move_base_simple/goal',
                                            PoseStamped,
                                            queue_size=1)
        # Getting the x and y dimensions of the map to use for the random
        # waypoint
        self.map_x_size = rospy.get_param("/pcl_render_node/map/x_size", 40.0)
        self.map_y_size = rospy.get_param("/pcl_render_node/map/y_size", 40.0)

    def publish_waypoint(self):
        waypoint_msg = PoseStamped()
        waypoint_msg.header.seq = 1
        waypoint_msg.header.stamp = rospy.Time.now()
        waypoint_msg.header.frame_id = "world"
        # Randomly setting a desired x and y position
        waypoint_msg.pose.position.x = random.uniform(self.map_x_size/-2,
                                                      self.map_x_size/2)
        waypoint_msg.pose.position.y = random.uniform(self.map_y_size/-2,
                                                      self.map_y_size/2)
        waypoint_msg.pose.position.z = 0
        # COMBAK: Set w to 1.0 because that's what simply clicking on a point
        # did. Dragging when setting 2D Nav Goal changes z and w but that
        # doesn't seem to change the behavior of the drone
        waypoint_msg.pose.orientation.w = 1.0
        # Printing for debugging purposes
        print(waypoint_msg)
        self.waypoint_pub.publish(waypoint_msg)
        print('[INFO] Waypoint published')


def main():
    print('[INFO] Running file')
    rospy.init_node('waypoint_feed_node', anonymous=True)
    waypoint_feed_object = WaypointFeed()
    # COMBAK: See if there is a way to make it repeat every time EXEC_STATE =
    # WAIT_TARGET instead of a fixed time
    duration = rospy.Duration(30)
    while not rospy.is_shutdown():
        waypoint_feed_object.publish_waypoint()
        rospy.sleep(duration)
    print("Shutting down")
    print('[INFO] Finishing running file')


if __name__ == '__main__':
    main()
