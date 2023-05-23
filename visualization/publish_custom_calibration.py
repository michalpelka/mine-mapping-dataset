#!/usr/bin/env python

import rospy
import tf2_ros
import geometry_msgs.msg

def publish_static_transform(parent, child, t, r):
    static_broadcaster = tf2_ros.StaticTransformBroadcaster()

    static_transformStamped = geometry_msgs.msg.TransformStamped()

    static_transformStamped.header.stamp = rospy.Time.now()
    static_transformStamped.header.frame_id = parent
    static_transformStamped.child_frame_id = child

    static_transformStamped.transform.translation.x = t[0]  # Example translation values
    static_transformStamped.transform.translation.y = t[1]
    static_transformStamped.transform.translation.z = t[2]

    static_transformStamped.transform.rotation.x = r[0]  # Example rotation values
    static_transformStamped.transform.rotation.y = r[1]
    static_transformStamped.transform.rotation.z = r[2]
    static_transformStamped.transform.rotation.w = r[3]

    static_transformStamped.header.stamp = rospy.Time.now()
    static_broadcaster.sendTransform(static_transformStamped)
    

if __name__ == '__main__':
    try:
        rospy.init_node('static_transform_publisher', anonymous=True)

        rate = rospy.Rate(10)  # Publishing rate in Hz

        while not rospy.is_shutdown():
            publish_static_transform("base_link", "livox", [0,0,0],[0,0,0,1])
            publish_static_transform("velodyne_rot_angular_offset", "velodyne_rot", [0.0463717952371,-0.0222407989204,0.00731471739709],
                                     [0.503462910652,0.501919329166,0.494878023863,0.499697744846])

            publish_static_transform("base_link", "velodyne", [0.553831,-0.0671387,-0.476559],
                                     [-0.736552,-0.460854,-0.493972,-0.0331143])
            publish_static_transform("base_link", "velodyne_rot_base", [0.29413,-0.275557,-0.359802],
                                     [-0.0571129,0.700139,-0.0524328,0.709785])

            rate.sleep()

    except rospy.ROSInterruptException:
        pass

