# %%
import glob
import cv2
import numpy as np
import matplotlib.pyplot as plt
import math
# %%
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
    _,contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)  

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

# %%
image_list = glob.glob("*.png")
for path in image_list:
    image = cv2.imread(path)

    output=find_center(image)

    img_draw=image.copy()

    point_size=1
    point_color=(0,0,255)
    thickness=4

    for data in output:
        
        cv2.circle(img_draw,data[1],point_size,point_color,thickness)
        
        p1=(int(data[1][0]-100*math.cos(data[2])),int(data[1][1]-100*math.sin(data[2])))
        p2=(int(data[1][0]+100*math.cos(data[2])),int(data[1][1]+100*math.sin(data[2])))
        cv2.line(img_draw, p1, p2, (255, 0, 0), 5)
    
    plt.imshow(img_draw)
    plt.show()
    