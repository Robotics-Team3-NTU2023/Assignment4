#!/usr/bin/env python
import rclpy
import cv2
from cv_bridge import CvBridge
from rclpy.node import Node

import sys
sys.path.append('/home/robot/colcon_ws/install/tm_msgs/lib/python3.6/site-packages')
from tm_msgs.msg import *
from tm_msgs.srv import *

from sensor_msgs.msg import Image
import argparse
import numpy as np

# parser = argparse.ArgumentParser()
# parser.add_argument("--filename", "-f",
#                     type=str,
#                     help='saved filename',
#                     required=True)
# args = parser.parse_args()

def find_center(image):
    '''
    input:
    img: img read from opencv
    
    output:
    list[
        [id,center,theta],[id,center,theta]...
    ]

    '''
    gray=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
    ret,thresh=cv2.threshold(gray,125,255,cv2.THRESH_BINARY)
    # plt.imshow(thresh)
    # plt.show()
    contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)  

    block=[]
   
    for i,cnt in enumerate(contours):
        area = cv2.contourArea(cnt)
        # print(f'Area{i}:', area)
        if area>1000 and area<(960*1280/2):
            block.append(cnt)
    # print(block)


    img_draw=cv2.drawContours(image,block,-1,(0,255,0),5)
    # plt.imshow(img_draw)
    # plt.show()
    # Determine center of gravity and orientation using Moments
    
    output=[]
    

    for i,cnt in enumerate(block):
        M = cv2.moments(cnt)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        theta = 0.5*np.arctan2(2*M["mu11"],M["mu20"]-M["mu02"]) 

        output.append([i,center,theta])
        
    
    return output

class ImageSub(Node):
    def __init__(self, nodeName):
        super().__init__(nodeName)
        self.subscription = self.create_subscription(Image,
        'techman_image', self.image_callback, 10)
        self.subscription

    def image_callback(self, data):
        self.get_logger().info('Received image')
        # f = input("Filename: ")
        # TODO (write your code here)
        br = CvBridge()
        img = br.imgmsg_to_cv2(data)
        cv2.imwrite(f"/home/robot/workspace2/team3_ws/test.png", img)

def send_script(script):
    arm_node = rclpy.create_node('arm')
    arm_cli = arm_node.create_client(SendScript, 'send_script')

    while not arm_cli.wait_for_service(timeout_sec=1.0):
        arm_node.get_logger().info('service not availabe, waiting again...')

    move_cmd = SendScript.Request()
    move_cmd.script = script
    arm_cli.call_async(move_cmd)
    arm_node.destroy_node()

def set_io(state):
    gripper_node = rclpy.create_node('gripper')
    gripper_cli = gripper_node.create_client(SetIO, 'set_io')

    while not gripper_cli.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('service not availabe, waiting again...')

    io_cmd = SetIO.Request()
    io_cmd.module = 1
    io_cmd.type = 1
    io_cmd.pin = 0
    io_cmd.state = state
    gripper_cli.call_async(io_cmd)
    gripper_node.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ImageSub('image_sub')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
