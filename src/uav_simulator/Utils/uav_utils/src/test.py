#!/usr/bin/env python2

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


class WaypointFeed(object):

    def __init__(self):
        print('[INFO] Initializing object...')
        # COMBAK: temporarily setting queue_size to 1
        self.waypoint_pub = rospy.Publisher('/waypoint_generator/waypoints', Path, queue_size=10)
        waypoint_msg = Path()
        waypoint_msg.header.seq = 0
        waypoint_msg.header.stamp = rospy.Time.now()
        waypoint_msg.header.frame_id = "world"
        pose = PoseStamped()
        pose.header.seq = 0
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "world"
        pose.pose.position.x = 10.1873664856
        pose.pose.position.y = -13.2499380112
        pose.pose.position.z = 0
        pose.pose.orientation.w = 1.0
        waypoint_msg.poses.append(pose)
        print(waypoint_msg)
        self.waypoint_pub.publish(waypoint_msg)
        print('[INFO] Published')


def main():
    print('[INFO] Running file')
    rospy.init_node('waypoint_feed_node', anonymous=True)
    camera_feed_object = WaypointFeed()
    try:
        rospy.spin()
    except KeyboardInterrupt:
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
