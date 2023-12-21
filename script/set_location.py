#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener
#from happymimi_navigation.srv import SetLocation, SetLocationResponse
from my_navigation2.srv import NaviCoord, NaviCoordResponse

class SetLocationServer(Node):
    def __init__(self):
        super().__init__('set_location_server')
        self.srv = self.create_service(SetLocation, '/set_location_server', self.check_state)
        self.get_logger().info('Ready to set_location_server')
        # Value
        self.tf = TransformListener(self)

        self.location_dict = {}

    def get_map_position(self):
        transform = self.tf.lookup_transform("map", "base_link", rclpy.Time())
        self.location_pose_x = transform.transform.translation.x
        self.location_pose_y = transform.transform.translation.y
        self.location_pose_z = transform.transform.rotation.z
        self.location_pose_w = transform.transform.rotation.w

    def check_state(self, request, response):
        if request.state == 'add':
            self.get_logger().info("Add location")
            response.result = self.add_location(request.name)
        elif request.state == 'save':
            self.get_logger().info("Save location")
            response.result = self.save_location(request.name)
        else:
            self.get_logger().error(f"<{request.state}> state doesn't exist.")
            response.result = False
        return response

    def add_location(self, name):
        if name in self.location_dict:
            self.get_logger().error(f'<{name}> has been registered. Please enter a different name.')
            return False
        elif name == '':
            self.get_logger().error("No location name entered.")
            return False
        else:
            self.get_map_position()
            self.location_dict[name] = [self.location_pose_x, self.location_pose_y, self.location_pose_z, self.location_pose_w]
            self.get_logger().info(f"Registered <{name}>")
            return True

    def save_location(self, file_name):
        try:
            param_path = rclpy.get_package_share_directory('happymimi_params')
            map_path = rclpy.get_package_share_directory('happymimi_navigation')
            self.set_parameters([], [{'name': '/location_dict', 'value': self.location_dict}])
            rclpy.logging.get_logger('rosparam').info("Location dictionary:")
            rclpy.logging.get_logger('rosparam').info(self.location_dict)

            process = sp.Popen(['ros2', 'run', 'nav2_map_server', 'map_saver', '--map', f'{map_path}/maps/{file_name}'])
            process.wait()
            self.get_logger().info(f"Saved as <{file_name}>")
            return True
        except rclpy.exceptions.ROSInterruptException:
            self.get_logger().error("Could not save.")
            return False

def main(args=None):
    rclpy.init(args=args)
    try:
        set_location_server = SetLocationServer()
        rclpy.spin(set_location_server)
    except KeyboardInterrupt:
        pass
    finally:
        set_location_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()