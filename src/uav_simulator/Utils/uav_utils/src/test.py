#!/usr/bin/env python2

import rospy
from geometry_msgs.msg import PoseStamped


class WaypointFeed(object):

    def __init__(self):
        print('[INFO] Initializing object...')
        # COMBAK: temporarily setting queue_size to 1
        self.waypoint_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)

    def publish_waypoint(self):
        waypoint_msg = PoseStamped()
        waypoint_msg.header.seq = 1
        waypoint_msg.header.stamp = rospy.Time.now()
        waypoint_msg.header.frame_id = "world"
        waypoint_msg.pose.position.x = 10.1873664856
        waypoint_msg.pose.position.y = -13.2499380112
        waypoint_msg.pose.position.z = 0
        waypoint_msg.pose.orientation.w = 1.0
        print(waypoint_msg)
        self.waypoint_pub.publish(waypoint_msg)
        print('[INFO] Waypoint published')


def main():
    print('[INFO] Running file')
    rospy.init_node('waypoint_feed_node', anonymous=True)
    waypoint_feed_object = WaypointFeed()
    duration = rospy.Duration(3)
    while not rospy.is_shutdown():
        waypoint_feed_object.publish_waypoint()
        rospy.sleep(duration)
    print("Shutting down")
    print('[INFO] Finishing running file')


if __name__ == '__main__':
    main()

# header: 
#   seq: 0
#   stamp: 
#     secs: 1601399584
#     nsecs: 535222742
#   frame_id: "world"
# poses: 
#   - 
#     header: 
#       seq: 0
#       stamp: 
#         secs: 1601399584
#         nsecs: 530466050
#       frame_id: "world"
#     pose: 
#       position: 
#         x: 10.1873664856
#         y: -13.2499380112
#         z: 0.0
#       orientation: 
#         x: 0.0
#         y: 0.0
#         z: 0.0
#         w: 1.0
# ---
# header: 
#   seq: 1
#   stamp: 
#     secs: 1601399596
#     nsecs: 870259172
#   frame_id: "world"
# poses: 
#   - 
#     header: 
#       seq: 1
#       stamp: 
#         secs: 1601399596
#         nsecs: 867084480
#       frame_id: "world"
#     pose: 
#       position: 
#         x: 7.1756477356
#         y: 8.04176425934
#         z: 0.0
#       orientation: 
#         x: 0.0
#         y: 0.0
#         z: 0.0
#         w: 1.0
# ---
