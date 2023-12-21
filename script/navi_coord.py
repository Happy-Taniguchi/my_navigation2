#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile
from std_srvs.srv import Empty
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
# from happymimi_navigation.srv import NaviCoord, NaviCoordResponse
from my_navigation2.srv import NaviCoord, NaviCoordResponse

class NaviCoordServer(Node):
    def __init__(self):
        super().__init__('navi_coord_server')
        self.srv = self.create_service(NaviCoord, 'navi_coord_server', self.send_goal)
        self.get_logger().info('Ready to navi_coord_server')
        # Action
        self.ac = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.ac.wait_for_server()
        # Publisher
        self.head_pub = self.create_publisher(Float64, '/servo/head', 1)
        # Service
        self.clear_costmap = self.create_client(Empty, 'clear_costmaps')
        while not self.clear_costmap.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

    def send_goal(self, request, response):
        location_list = request.loc_coord
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = location_list[0]
        goal.pose.position.y = location_list[1]
        goal.pose.orientation.z = location_list[2]
        goal.pose.orientation.w = location_list[3]

        self.head_pub.publish(Float64(data=0))
        self.get_logger().info("Clearing costmap...")
        self.clear_costmap.call_async(Empty.Request())
        goal_msg = NavigateToPose.Goal()
        goal_msg.target_pose = goal
        goal_handle = self.ac.send_goal(goal_msg)

        while rclpy.ok():
            rclpy.spin_once(self)
            if goal_handle.accepted:
                self.get_logger().info('Navigation accepted!')
                response.result = True
                return response
            elif goal_handle.canceled:
                self.get_logger().info('Navigation canceled')
                response.result = False
                return response


def main(args=None):
    rclpy.init(args=args)
    try:
        navi_coord_server = NaviCoordServer()
        rclpy.spin(navi_coord_server)
    except KeyboardInterrupt:
        pass
    finally:
        navi_coord_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()