#!/usr/bin/env python3
import rclpy

# Because of transformations
# import tf_conversions

import tf2_ros
import geometry_msgs.msg
import nav_msgs.msg
import tf2_msgs.msg
from rclpy.node import Node
from rclpy.duration import Duration
from time import sleep


odom_base = nav_msgs.msg.Odometry()

class tf_listener(Node):
    def __init__(self, name='tf_listener_node'):
        super().__init__(name)
        self.pose = geometry_msgs.msg.TransformStamped()
        self._tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self._tf_buffer, self)
        
    def get_odom_base_tf(self):
        now = rclpy.time.Time()
        try:
            self.get_logger().info("Trying to get odom->base tf")
            print(self._tf_buffer)
            print(now)
            trans = self._tf_buffer.lookup_transform("map","odom_slash",now,timeout=Duration(seconds=3.0))
            print(trans)
        except LookupError:
            self.get_logger().info("Transform odom->base not ready yet!")
    

def map_odom_tf_callback(msg,trans_odom_map):
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    global odom_base

    
    #map -> baselink
    t.header.stamp = rclpy.Time.now()
    t.header.frame_id = "map"
    t.child_frame_id = "odom"
    t.transform.translation.x = msg.pose.pose.position.x
    t.transform.translation.y = msg.pose.pose.position.y
    t.transform.translation.z = msg.pose.pose.position.z
    t.transform.rotation.x = trans_odom_map.transform.rotation.x * msg.pose.pose.orientation.x
    t.transform.rotation.y = trans_odom_map.transform.rotation.y * msg.pose.pose.orientation.y
    t.transform.rotation.z = trans_odom_map.transform.rotation.z * msg.pose.pose.orientation.z
    t.transform.rotation.w = -trans_odom_map.transform.rotation.w  *  msg.pose.pose.orientation.w

    br.sendTransform(t)

# def odom_base_callback(msg):
#     global odom_base
#     # odom -> base_link
#     odom_base.pose.pose.position.x = msg.pose.pose.position.x
#     odom_base.pose.pose.position.y = msg.pose.pose.position.y
#     odom_base.pose.pose.position.z = msg.pose.pose.position.z
#     odom_base.pose.pose.orientation.x = msg.pose.pose.orientation.x
#     odom_base.pose.pose.orientation.y = msg.pose.pose.orientation.y
#     odom_base.pose.pose.orientation.z = msg.pose.pose.orientation.z
#     odom_base.pose.pose.orientation.w = msg.pose.pose.orientation.w

if __name__ == '__main__':
    rclpy.init()
    
    node = tf_listener('map_odom_tf_node')
    sleep(1)
    while not rclpy.shutdown():
        try:

            node.get_odom_base_tf()
            rclpy.spin(node)
        except KeyboardInterrupt:
            rclpy.shutdown()
    
    
    
    
    
    # try:
    #     trans_odom_map = tfBuffer.lookup_transform("odom","map",rclpy.time.Time())
    #     print(trans_odom_map)
    #     #rclpy.Subscriber('/slash/odom_slash',nav_msgs.msg.Odometry,trans_odom_map,map_odom_tf_callback)
    # except:
    #     pass
    
    # #rclpy.Subscriber('/tf',tf2_msgs.msg.msg,odom_base_callback)
    # rclpy.spin()