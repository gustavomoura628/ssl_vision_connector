from typing import List
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D, Twist

import socket
import sys
sys.path.append('/home/gus/ufg/SSL/ros2_ws/src/my_package/my_package')

from messages_robocup_ssl_wrapper_pb2 import SSL_WrapperPacket




class SSLVisionProtobufToROS(Node):

    def __init__(self):
        super().__init__('protobuf_to_ros')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('blue_robot_count', 3),
                ('yellow_robot_count', 3),
                ('frequency', 60),
            ]
        )
        self.blue_robot_count = self.get_parameter('blue_robot_count').get_parameter_value().integer_value
        self.yellow_robot_count = self.get_parameter('yellow_robot_count').get_parameter_value().integer_value
        frequency = self.get_parameter('frequency').get_parameter_value().integer_value
        time_step_ms = 1000 // frequency


        # Protobuf connection setup
        self.ssl_vision_ip = '224.5.23.2'
        self.ssl_vision_port = 10006

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 128)
        self.sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_LOOP, 1)
        self.sock.bind((self.ssl_vision_ip, self.ssl_vision_port))



        self.ball_publisher = self.create_publisher(Pose2D, '/simulator/poses/ball', 10)

        # /simulator/robots_poses/blue/0, /simulator/robots_poses/blue/1, ...
        self.pose_publishers = []
        for i in range(self.blue_robot_count):
            self.pose_publishers.append(
                self.create_publisher(Pose2D, f'/simulator/poses/blue/robot{i}', 10)
            )
        for i in range(self.yellow_robot_count):
            self.pose_publishers.append(
                self.create_publisher(Pose2D, f'/simulator/poses/yellow/robot{i}', 10)
            )

        self.yellow_robot_id_presence_counter = [0 for i in range(16)]
        self.blue_robot_id_presence_counter = [0 for i in range(16)]
        
        timer_period = time_step_ms / 1000.0  # seconds
        self.timer = self.create_timer(timer_period, self.get_protobuf_and_publish)

        self.get_logger().info('Protobuf Connector Node Started')

    
    def get_blue_robot_id_converter(self):
        print("Blue presence counter: ", self.blue_robot_id_presence_counter)
        blue_robot_list = [(count, id) for id, count in enumerate(self.blue_robot_id_presence_counter)]
        print("Blue robot list: ", blue_robot_list)
        sorted_by_count = sorted(blue_robot_list, key=lambda x: x[0], reverse=True)
        print("sorted by count: ", sorted_by_count)
        top_k = sorted_by_count[:self.blue_robot_count]
        print("top k: ", top_k)
        sorted_by_id = sorted(top_k, key=lambda x: x[1])
        print("sorted_by_id: ", sorted_by_id)
        converter = [pair[1] for pair in sorted_by_id]
        print("converter: ", converter)
        return converter

    def get_yellow_robot_id_converter(self):
        print("yellow presence counter: ", self.yellow_robot_id_presence_counter)
        yellow_robot_list = [(count, id) for id, count in enumerate(self.yellow_robot_id_presence_counter)]
        print("yellow robot list: ", yellow_robot_list)
        sorted_by_count = sorted(yellow_robot_list, key=lambda x: x[0], reverse=True)
        print("sorted by count: ", sorted_by_count)
        top_k = sorted_by_count[:self.yellow_robot_count]
        print("top k: ", top_k)
        sorted_by_id = sorted(top_k, key=lambda x: x[1])
        print("sorted_by_id: ", sorted_by_id)
        converter = [pair[1] for pair in sorted_by_id]
        print("converter: ", converter)
        return converter

    def decay_presence_counter(self):
        DECAY_PARAMETER = 0.9

        for i in range(len(self.yellow_robot_id_presence_counter)):
            self.yellow_robot_id_presence_counter[i] *= DECAY_PARAMETER

        for i in range(len(self.blue_robot_id_presence_counter)):
            self.blue_robot_id_presence_counter[i] *= DECAY_PARAMETER
        
        #print("Blue presence counter: ", self.blue_robot_id_presence_counter)
        #print("Yellow presence counter: ", self.yellow_robot_id_presence_counter)
        
    def increase_presence_counter(self, ssl_protobuf_packet):
        for yellow_robot in ssl_protobuf_packet.detection.robots_yellow:
            self.yellow_robot_id_presence_counter[yellow_robot.robot_id] += 1
        for blue_robot in ssl_protobuf_packet.detection.robots_blue:
            self.blue_robot_id_presence_counter[blue_robot.robot_id] += 1
        




    def get_protobuf_and_publish(self):
        # Get converter from robot_id to our 0 to robot_count IDs
        blue_robot_id_converter = self.get_blue_robot_id_converter()
        yellow_robot_id_converter = self.get_yellow_robot_id_converter()

        # Units are meters, meters/s, degrees
        # state is [ball_x, ball_y, ball_z, ball_v_x, ball_v_y,
        #           blue_0_x, blue_0_y, blue_0_angle, blue_0_v_x, blue_0_v_y, blue_0_v_angle,
        #           blue_0_infrared, blue_0_desired_wheel0_speed, blue_0_desired_wheel1_speed,
        #           blue_0_desired_wheel2_speed, blue_0_desired_wheel3_speed, ...]

        # Get protobuf datagram
        data = self.sock.recvfrom(1024)
        ssl_protobuf_packet = SSL_WrapperPacket.FromString(data[0])

        #print("Detection = ", ssl_protobuf_packet)

        # Publish ball position
        if len(ssl_protobuf_packet.detection.balls) > 0:
            ball_msg = Pose2D()
            protobuf_ball = ssl_protobuf_packet.detection.balls[0]
            #print("Ball: ", protobuf_ball)
            ball_msg.x = protobuf_ball.x
            ball_msg.y = protobuf_ball.y
            ball_msg.theta = 0.0
            self.ball_publisher.publish(ball_msg)
        
        # Increase presence counter
        self.increase_presence_counter(ssl_protobuf_packet)
        
        # Decay presence counter
        self.decay_presence_counter()
            
        
        for blue_robot in ssl_protobuf_packet.detection.robots_blue:
            print(blue_robot)

            if blue_robot.robot_id not in blue_robot_id_converter:
                # This means the robot that was detected might be a flicker of the vision system
                print("Robot id", blue_robot.robot_id, "is not in the top k robots, classifying as flicker")
                continue
            else:
                index = blue_robot_id_converter.index(blue_robot.robot_id)
                print("Robot ", blue_robot.robot_id, " has a ros id of", index)

            msg = Pose2D()
            msg.x = blue_robot.x
            msg.y = blue_robot.y
            msg.theta = blue_robot.orientation
            self.pose_publishers[index].publish(msg)
        
        for yellow_robot in ssl_protobuf_packet.detection.robots_yellow:
            print(yellow_robot)

            if yellow_robot.robot_id not in yellow_robot_id_converter:
                # This means the robot that was detected might be a flicker of the vision system
                print("Robot id", yellow_robot.robot_id, "is not in the top k robots, classifying as flicker")
                continue
            else:
                index = yellow_robot_id_converter.index(yellow_robot.robot_id)
                print("Robot ", yellow_robot.robot_id, " has a ros id of", index)

            msg = Pose2D()
            msg.x = yellow_robot.x
            msg.y = yellow_robot.y
            msg.theta = yellow_robot.orientation
            self.pose_publishers[self.blue_robot_count + index].publish(msg)

def main(args=None):
    rclpy.init(args=args)

    protobuf_to_ros = SSLVisionProtobufToROS()

    rclpy.spin(protobuf_to_ros)

    protobuf_to_ros.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
