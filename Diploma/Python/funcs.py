import asyncio
import math
import numpy as np
from concurrent.futures import ThreadPoolExecutor
#from optirtouch_py import optitouch_multi

# BLUETOOTH

async def ainput(prompt: str = "") -> str:
    with ThreadPoolExecutor(1, "AsyncInput") as executor:
        return await asyncio.get_event_loop().run_in_executor(executor, input, prompt)

# NAVIGATION

def guide(loc, tar):
    # the angle we need to reach
    a, b = float(tar[0]), float(tar[1])
    if len(loc)!=3:
        return False
    x, y, theta = float(loc[0]), float(loc[1]), float(loc[2])
    # transform into robot coordinate system
    x_r = (a-x)*math.cos(theta)+(b-y)*math.sin(theta)
    y_r = (b-y)*math.cos(theta)-(a-x)*math.sin(theta)

    target_theta = math.atan2(y_r,x_r)

    # Align with our target and drive to the target in a straight line
    if abs(target_theta)>1:
        # P regulation
        Kp_theta = 2/3.14
        reg_val = target_theta*Kp_theta
        # minimal reg_val
        angle_thr = 1.75
        if reg_val>-angle_thr and reg_val<0:
            reg_val = -angle_thr
        elif reg_val<angle_thr and reg_val>0:
            reg_val = angle_thr
        w = reg_val
        return [0,w]
    elif (math.sqrt((a-x)*(a-x)+(b-y)*(b-y))>0.02):
        # P regulation
        Kp_dist = 0.3
        reg_val = math.sqrt((a-x)*(a-x)+(b-y)*(b-y)) * Kp_dist
        # max speed
        max_spd_thr = 0.4
        if reg_val>max_spd_thr:
            reg_val = max_spd_thr
        # min speed
        min_spd_thr = 0.12
        if reg_val<min_spd_thr:
            reg_val = max_spd_thr
        v = reg_val 

        # adjust orientation
        Kp_adj = 1.5
        adj_w = target_theta*Kp_adj
        # max adj
        max_adj_thr = 1
        if adj_w>max_adj_thr:
            adj_w = max_adj_thr
        return [v,adj_w]
    else:
        return [0,0]
    

def order(newX, newY, oldX, oldY):
    threshold = 500
    for j,value in enumerate(newX):
        if np.amin(abs(oldX-value))<threshold:
            arr1 = abs(oldX-value)<threshold
            for val in newY:
                if np.amin(abs(oldY-val))<threshold:
                    arr2 = abs(oldY-val)<threshold
                    # find index of the most similar element by absolute difference
                    x = arr1 & arr2
                    index1 = int(x.nonzero()[0])
                    index2 = np.argmin(abs(newX+newY-value-val))
                    #index = np.argmin(abs(oldX+oldY-value-val))
            if index1 != index2:
                # switch values
                newX[index1], newX[index2] = newX[index2], newX[index1]
                newY[index1], newY[index2] = newY[index2], newY[index1]
    return newX, newY


def find_triangle(X_3, Y_3):
    x_3, y_3 = tick_to_m(X_3, Y_3)

    # pythagoras's theorem to find the base of triangle
    dist1 = math.sqrt((x_3[0]-x_3[1])**2+(y_3[0]-y_3[1])**2)
    dist2 = math.sqrt((x_3[0]-x_3[2])**2+(y_3[0]-y_3[2])**2)

    print(f'd1:{dist1}')
    print(f'd2:{dist2}')

    # we have the bottom point
    if abs(dist1-dist2)<0.015:
        bot = np.array([X_3[0], Y_3[0]])
        side1 = np.array([X_3[1], Y_3[1]])
        side2 = np.array([X_3[2], Y_3[2]])
        
    # we have one of sides
    else:
        if dist1>dist2:
            side1 = np.array([X_3[0], Y_3[0]])
            side2 = np.array([X_3[1], Y_3[1]])
            bot = np.array([X_3[2], Y_3[2]])
        else:
            side1 = np.array([X_3[0], Y_3[0]])
            bot = np.array([X_3[1], Y_3[1]])
            side2 = np.array([X_3[2], Y_3[2]])

    return side1, side2, bot

def unit_vector(vector):
    # Returns the unit vector of the vector.
    return vector/np.linalg.norm(vector)

def angle_between(v1, v2):
    # Returns the angle in radians between vectors 'v1' and 'v2'
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))


def get_pos_theta(X_3,Y_3):
    side1, side2, bot = find_triangle(X_3, Y_3)
    # orientation vector = vector from bottom to middle of long side
    center = (side1+side2)/2
    orient_v = center - bot

    # frame coord system, measure theta around x axis
    base_orient_v = np.array([1,0])

    #b_side1, b_side2, b_bot = find_triangle(multitouch_client.offset[0], multitouch_client.offset[1])
    #base_orient_v = b_bot - (b_side1+b_side2)/2

    theta = angle_between(base_orient_v, orient_v)

    # code to check if rotation is clockwise or counterclockwise from base vector

    # 90 degree rotated base vector
    base_orient_v_90 = np.array([base_orient_v[1], -base_orient_v[0]])

    base_cross = np.cross(np.append(base_orient_v, [0]), np.append(base_orient_v_90, [0]))
    base_cross = unit_vector(base_cross)
    cross = np.cross(np.append(base_orient_v, [0]), np.append(orient_v, [0]))
    cross = unit_vector(cross)

    if (np.allclose(cross, base_cross)):
        theta = -theta

    return center[0], center[1], theta

HEIGHT = 32115
WIDTH = 32539
x2y = 39.5/67.2


def tick_to_m(x,y):
    # 67.2cm = 32539
    # 39.5cm = 32115 
    x = x * 0.672/WIDTH
    y = y * 0.395/HEIGHT   
    return x, y    


def frame(multitouch_client):
    posX, posY = multitouch_client.get_multitouch()

    posX = posX[:3]
    posY = posY[:3]

    # put coordinates in the same order
    posX, posY = order(posX,posY, multitouch_client.oldX, multitouch_client.oldY)

    # replace -1
    for count,value in enumerate(posX):
        if value == -1:
            posX[count] = multitouch_client.oldX[count]
    for count,value in enumerate(posY):
        if value == -1:
            posY[count] = multitouch_client.oldY[count]


    multitouch_client.oldX = posX
    multitouch_client.oldY = posY

    # set offset of first point
    if not multitouch_client.offset:
        side1, side2, bot = find_triangle(multitouch_client.oldX,multitouch_client.oldY)
        multitouch_client.offset_v = (side1+side2)/2
        multitouch_client.offset = (multitouch_client.oldX,multitouch_client.oldY)

    # offset the points
    posX_t = posX - multitouch_client.offset_v[0]
    posY_t = posY - multitouch_client.offset_v[1]


    print(f'x_t:{posX_t}')
    print(f'y_t:{posY_t}')

    # calculate center of robot position and orientation
    x,y,theta = get_pos_theta(posX_t, posY_t)

    x,y = tick_to_m(x,y)

    print(f"Location: {[x,y,theta]}")
    
    return [x,y,theta]


# PATHFIND

def pos_to_coord(pos, rows, columns, width, height):
    gap_r = height // rows
    gap_c = width // columns 
    y,x = pos[0], pos[1]

    row = y // gap_r
    col = x // gap_c

    return row, col

def coord_to_pos(coord, rows, columns, width, height):
    gap_r = height // rows
    gap_c = width // columns 
    row,col = coord[0], coord[1]

    x = gap_c * col + gap_c // 2
    y = gap_r * row + gap_r // 2

    return x, y


if __name__ == '__main__': 
    # initialise class
    '''
    multitouch_client = optitouch_multi()
    print("init")

    while multitouch_client.end == False:

        loc = frame(multitouch_client)
        if False:
            get_robot_pos(loc, ROWS, COLUMNS, width, height)
        print(loc)
    '''

