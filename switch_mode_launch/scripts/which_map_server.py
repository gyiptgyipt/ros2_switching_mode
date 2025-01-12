#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rom_interfaces.srv import WhichMaps
import os
import shutil
from ament_index_python.packages import get_package_share_directory

class WhichMapsServer(Node):
    def __init__(self):
        super().__init__('which_maps_server')
        self.srv = self.create_service(WhichMaps, 'which_maps', self.which_map_answer)

        self.map_topic_exists = False
        self.package_name = "which_maps"
        self.package_directory = get_package_share_directory(self.package_name) + "/maps/"

        # Check if the /map topic exists
        topics = self.get_topic_names_and_types()
        for topic in topics:
            if topic[0] == '/map':
                self.map_topic_exists = True
                break

        if self.map_topic_exists:
            self.get_logger().info("The /map topic exists.")
        else:
            self.get_logger().warn("The /map topic does not exist.")

    def which_map_answer(self, request, response):
        if request.request_string == "which_map_do_you_have":
            self.handle_which_map(request, response)
        elif request.request_string == "save_map":
            self.handle_save_map(request, response)
        elif request.request_string == "select_map":
            self.handle_select_map(request, response)
        elif request.request_string == "mapping":
            self.handle_mode_change("mapping", response)
        elif request.request_string == "navi":
            self.handle_mode_change("navi", response)
        elif request.request_string == "remapping":
            self.handle_mode_change("remapping", response)
        else:
            self.handle_invalid_request(response)
    
        def handle_which_map(self, request, response):
            try:
                yaml_file_count = 0
                # Ensure the response map_names list is empty before appending
                response.map_names = []
    
                for entry in os.scandir(self.package_directory):
                    if entry.is_file() and entry.name.endswith('.yaml'):
                        response.map_names.append(entry.name)
                        self.get_logger().info(entry.name)
                        yaml_file_count += 1
    
                # Ensure the types are correct: `total_maps` is an integer and `status` is an integer
                response.total_maps = yaml_file_count
                response.status = 1  # ok
                self.get_logger().info(f"Sending : Response Status OK")
            except FileNotFoundError:
                self.get_logger().error("Error accessing directory.")
                response.total_maps = 0
                response.status = -1  # not ok
                self.get_logger().info("Sending : Response Status not OK")


        def handle_save_map(self, request, response):
            map_name = request.map_name_to_save
            if self.map_topic_exists:
                cmd = f"cd /home/mr_robot/Desktop/Git/rom_dynamics_robots/developer_packages/rom2109/rom2109_nav2/maps && ros2 run nav2_map_server map_saver_cli -f {map_name}"
                ret_code = os.system(cmd)
                if ret_code == 0:
                    response.status = 1  # ok
                    self.get_logger().info("Map saver command executed successfully.")
                else:
                    response.status = -1  # not ok
                    self.get_logger().error(f"Map saver command failed with return code: {ret_code}")
                self.get_logger().info(f"Sending : Response Status {'OK' if ret_code == 0 else 'not OK'}")
            else:
                self.get_logger().warn("The /map topic does not exist.")
                response.status = -1  # not ok
                self.get_logger().info("Sending : Response Status not OK")

    def handle_select_map(self, request, response):
        map_name = request.map_name_to_select
        if map_name == "":
            response.status = -1  # not ok
            self.get_logger().info("Sending : Response Status not OK")
        else:
            response.status = 1  # ok
            self.get_logger().info("Sending : Response Status OK")

    def handle_mode_change(self, mode, response):
        response.status = 1  # ok
        self.get_logger().info(f"Sending : Response Status OK for mode: {mode}")

    def handle_invalid_request(self, response):
        response.total_maps = 0
        response.status = -1  # not ok
        self.get_logger().info("Sending : Response Status not OK")


def main(args=None):
    rclpy.init(args=args)
    node = WhichMapsServer()

    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
