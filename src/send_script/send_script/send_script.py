#!/usr/bin/env python

import rclpy
import time
import sys
import cv2
sys.path.append('/home/robot/colcon_ws/install/tm_msgs/lib/python3.6/site-packages')
from tm_msgs.msg import *
from tm_msgs.srv import *
import matplotlib.pyplot as plt
import math
import numpy as np
import time


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


# arm client
def send_script(script):
    arm_node = rclpy.create_node('arm')
    arm_cli = arm_node.create_client(SendScript, 'send_script')

    while not arm_cli.wait_for_service(timeout_sec=1.0):
        arm_node.get_logger().info('service not availabe, waiting again...')

    move_cmd = SendScript.Request()
    move_cmd.script = script
    arm_cli.call_async(move_cmd)
    arm_node.destroy_node()

# gripper client
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

def move_to(targetP1):
    # targetP2 = "350.00, 350, 690, -180.00, 0.0, 135.00"
    script1 = "PTP(\"CPP\","+targetP1+",100,200,0,false)"
    # script2 = "PTP(\"CPP\","+targetP2+",100,200,0,false)"
    send_script(script1)
    # send_script(script2)

def ImagetoRobot(x,y):
    img_y_max=1280
    img_x_max=960
    robot_y_min=171.70
    robot_y_max=507.10
    robot_x_min=19.63
    robot_x_max=460.95

    y2=robot_y_max-x*(robot_y_max-robot_y_min)/img_x_max
    x2=y*(robot_x_max-robot_x_min)/img_y_max+robot_x_min

    return (x2,y2)

def main(args=None):

    rclpy.init(args=args)

    
    


    #--- move command by joint angle ---#
    # script = 'PTP(\"JPP\",45,0,90,0,90,0,35,200,0,false)'

    #--- move command by end effector's pose (x,y,z,a,b,c) ---#
    # targetP1 = "398.97, -122.27, 748.26, -179.62, 0.25, 90.12"s

    # Initial camera position for taking image (Please do not change the values)
    # For right arm: targetP1 = "230.00, 230, 730, -180.00, 0.0, 135.00"
    # For left  arm: targetP1 = "350.00, 350, 730, -180.s00, 0.0, 135.00"
    # set_io(0.0)
    # move_to("230.00, 230, 110, -180.00, 0.0, 135.00")
    # set_io(0.0)
    # reset
    # move_to("230.00, 230, 730, -180.00, 0.0, 135.00")
    
    # move_to("250, 250, 500, -180.00, 0.0, 180.00") # position take photo
    move_to("250, 250, 500, -180.00, 0.0, 180.00")
    # move_to("458, 160, 120, -180.00, 0.0, 180.00") # leftest position
    # move_to("24, 509, 120, -180.00, 0.0, 180.00") # rightest position
    send_script("Vision_DoJob(job1)")
    cv2.waitKey(1)


    time.sleep(10)

    # for i in range(130, 331, 10):
    #     move_to(f"{i}, 130, 500, -180.00, 0.0, 180.00")

    
    def image_to_cartiesian(img):
        height,width,_ = img.shape
        cart = img.copy
        for j in range(0,height):
            for k in range(0,width):
                cart[k][height-j-1]=img[j][k]
        return cart

    
    
    img=cv2.imread('/home/robot/workspace2/team3_ws/try/test.png')

    
    # img=cv2.transpose(img)
    # img = cv2.flip(img,1)

    
    # print(img.shape)
    # img = cv2.flip(img,0)
    # img = cv2.flip(img,1)
    # plt.imshow(img)
    # plt.show()

    # move_to("200, 180, 200, -180.00, 0.0, 180.00")



    # f = open("./center.txt", "r")
    # for line in f:
    #     print(line)
    #     print("hi")
    #     if 'str' in line:
    #         break
    # f.close()
    # send_script("Vision_DoJob(job1)")
    # cv2.waitKey(1)

    # image = cv2.imread('/home/robot/workspace2/team3_ws/try/test.png')
    output=find_center(img)


    img_draw=img.copy()

    point_size=1
    point_color=(0,0,255)
    thickness=4

    for data in output:
        
        cv2.circle(img_draw,data[1],point_size,point_color,thickness)
        
        p1=(int(data[1][0]-100*math.cos(data[2])),int(data[1][1]-100*math.sin(data[2])))
        p2=(int(data[1][0]+100*math.cos(data[2])),int(data[1][1]+100*math.sin(data[2])))
        cv2.line(img_draw, p1, p2, (255, 0, 0), 5)
    
    cv2.imwrite("/home/robot/workspace2/team3_ws/try/draw.png",img_draw)

    print(output)
    height = 110
    for data in output:
        x_new,y_new = ImagetoRobot(data[1][1],data[1][0])
        deg=-(data[2]*180/math.pi+90)
        if deg > -90:
            deg -= 180
        if deg < -270:
            deg += 180
        
        # cdeg = deg+90 if deg > -180 else deg-90
        # cax = math.cos(cdeg)*10
        # cay = math.sin(cdeg)*10
        # x_new += cax
        # y_new += cay
        # print(x_new,y_new, deg, cdeg, cax, cay)
        set_io(0.0)
        move_to(f"{(int)(x_new)}, {(int)(y_new)}, 300, -180.00, 0.0, {deg}")
        for i in range(160, 109, -10):
            move_to(f"{(int)(x_new)}, {(int)(y_new)}, {i}, -180.00, 0.0, {deg}")

        set_io(1.0)
        move_to(f"{(int)(x_new)}, {(int)(y_new)}, 300, -180.00, 0.0, {deg}")


        move_to("571, 180, 300, -180.00, 0.0, 180.00")
        move_to(f"571, 180, {height}, -180.00, 0.0, 180.00")
        set_io(0.0)
        move_to("571, 180, 300, -180.00, 0.0, 180.00")
        height += 25


    





    # set_io(1.0) # 1.0: close gripper, 0.0: open gripper
    # # set_io(0.0)

    # move_to("230.00, 230, 300, -180.00, 0.0, 135.00")
    # move_to("300.00, 300, 110, -180.00, 0.0, 135.00")
    # set_io(0.0)
    # move_to("300.00, 300, 300, -180.00, 0.0, 135.00")
    rclpy.shutdown()

if __name__ == '__main__':
    main()




