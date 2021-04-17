"""
Submission by Alexandre Sajus and Fabien Roger
Coders Strike Back Challenge: https://www.codingame.com/multiplayer/bot-programming/coders-strike-back
More information at:
https://github.com/AlexandreSajus
"""

import sys
import math
import numpy as np
from math import sqrt, atan2

# Auto-generated code below aims at helping you parse
# the standard input according to the problem statement.


def calc_absolute_angle(x1, y1, x2, y2):
    vector_1 = [x1, y1]
    vector_2 = [x2, y2]
    unit_vector_1 = vector_1 / np. linalg. norm(vector_1)
    unit_vector_2 = vector_2 / np. linalg. norm(vector_2)
    dot_product = np.dot(unit_vector_1, unit_vector_2)
    return np.degrees(np.arccos(dot_product))

def calc_angle(x1, y1, x2, y2):
    return np.degrees(np.arctan2(y2-y1,x2-x1))

laps = int(input())
checkpoint_count = int(input())
checkpoints = []
for i in range(checkpoint_count):
    checkpoint_x, checkpoint_y = [int(j) for j in input().split()]
    checkpoints.append((checkpoint_x, checkpoint_y))

last_checkpoint_id1 = -1
loop1 = 0
boost1 = 1
last_checkpoint_id2 = -1
loop2 = 0
boost2 = 1

def behavior1(x,y,vx,vy,angle, next_check_point_id, loop, last_checkpoint_id, boost):
    """
    PARAMETERS
    """
    direction_compensation = 1.4    # The amount to overcompensate the direction to next checkpoint
    dist_slowdown = 2400    # At what distance from checkpoint we slow down
    thrust_slowdown = 20    # What thrust when we slow down
    thrust_amplitude = 140  # Amplitude of the gaussian defining the thrust
    thrust_deviation = 5000  # Deviation of the gaussian defining the thrust
    thrust_min = 50  # Thrust when we are going in the wrong direction
    dist_boost = 2500   # At what distance from checkpoint is it safe to boost
    angle_boost = 10    # At what angle from checkpoint is it safe to boost
    boost = 1   # Do we have a boost
    radius = 500   # The radius of a checkpoint
    dist_turn = 1800     # At what distance from checkpoint we start to turn to the next checkpoint
    max_angle = 120
    map_center = [8000, 4500]
    """
    DEFINING METRICS
    """

    if last_checkpoint_id != next_check_point_id:
        loop += 1

    last_checkpoint_id = next_check_point_id

    next_checkpoint_x, next_checkpoint_y = checkpoints[next_check_point_id]

    # Compute the distance to next checkpoint
    dist_checkpoint = sqrt((x - next_checkpoint_x) **
                           2 + (y - next_checkpoint_y)**2)

    # Compute our current speed
    speed = sqrt(vx**2 + vy**2)

    # Compute the angle between the speed vector and the vector MC
    # where M the the position and C is the next checkpoint
    to_checkpoint_x = next_checkpoint_x - x
    to_checkpoint_y = next_checkpoint_y - y
    speed_checkpoint_angle = calc_absolute_angle(vx, vy, 
                                                 to_checkpoint_x, to_checkpoint_y)

    # Compute the angle between MC and MC'
    # Where C' is the side of the checkpoint
    limit_angle = np.degrees(np.arctan(600/dist_checkpoint))
    switch_checkpoint = False

    # Compute the angle between the direciton and the direction to the next checkpoint
    next_checkpoint_angle_abs = calc_angle(x, y, next_checkpoint_x, next_checkpoint_y)
    next_checkpoint_angle = (next_checkpoint_angle_abs - angle + 180) % 360 - 180

    # Define a target angle by overcompensating
    target_angle = next_checkpoint_angle*direction_compensation
    #print(target_angle, file=sys.stderr, flush=True)
    # Clamp the target angle
    target_angle = min(max_angle, target_angle)
    target_angle = max(-max_angle, target_angle)

    # Compute the difference between checkpoint and target angle
    # We will use this to define a target point as (X,Y)
    diff_angle = target_angle - next_checkpoint_angle
    
    #print("next_checkpoint_angle", next_checkpoint_angle_abs, file=sys.stderr, flush=True)
    #print("next_checkpoint_angle", next_checkpoint_angle, file=sys.stderr, flush=True)

    """
    THRUST BEHAVIOUR
    """
    thrust = 0
    # If we are going in the wrong direction, low thrust
    if next_checkpoint_angle > 90 or next_checkpoint_angle < -90:
        thrust = thrust_min
    # If the next checkpoint is far and we are aiming at it, BOOST
    elif dist_checkpoint > 8000 and abs(next_checkpoint_angle) < angle_boost and boost == 1:
        thrust = "BOOST"
        boost = 0
    elif next_checkpoint_angle < angle_boost and loop == laps and next_check_point_id == 0:
        if boost == -1:
            thurst = 100
        else:
            thrust = "BOOST"
    # If we are close to the next checkpoint and racing toward it at full speed
    # Then start turning toward the checkpoint after that
    elif dist_checkpoint < dist_turn and speed > 400 \
         and speed_checkpoint_angle<limit_angle:

        thrust=0
        switch_checkpoint = True
        next_checkpoint = checkpoints[(next_check_point_id+1)%len(checkpoints)]
        next_checkpoint_x, next_checkpoint_y = next_checkpoint
    # If we are close to the next checkpoint with a high speed, slow down
    elif dist_checkpoint < dist_slowdown and speed > 400:
        thrust = thrust_slowdown
    # Else, the thrust is defined by a gaussian according to the checkpoint angle
    # If we are aiming at the checkpoint go fast, if not go slow
    else:
        thrust = thrust_amplitude * \
            np.exp(-next_checkpoint_angle**2/thrust_deviation)
        thrust = min(100, thrust)

    """
    DIRECTION BEHAVIOUR
    """
    # Defining a target (X,Y)

    # We wont aim for the center of the checkpoint
    # Instead well aim for the part of the checkpoint closest to the center of the map
    checkpoint_to_center = np.asarray([next_checkpoint_x - map_center[0], next_checkpoint_y - map_center[1]])
    checkpoint_to_center_norm = sqrt(checkpoint_to_center[0]**2 + checkpoint_to_center[1]**2)
    # Unit vector from checkpoint to center
    checkpoint_to_center_unit = checkpoint_to_center/checkpoint_to_center_norm
    # Update the target (X,Y)
    next_checkpoint_x -= checkpoint_to_center_unit[0]*radius
    next_checkpoint_y -= checkpoint_to_center_unit[1]*radius

    # This is the vector to the checkpoint
    to_checkpoint = np.asarray(
        [[next_checkpoint_x - x], [next_checkpoint_y - y]])
    #print("to_checkpoint", file=sys.stderr, flush=True)
    #print(to_checkpoint, file=sys.stderr, flush=True)

    # We rotate to_checkpoint using the target_angle
    theta = np.radians(diff_angle)
    c, s = np.cos(theta), np.sin(theta)
    R = np.array(((c, -s), (s, c)))
    #print("R", file=sys.stderr, flush=True)
    #print(R, file=sys.stderr, flush=True)

    # To target is the vector oriented by the target_angle
    to_target = np.dot(R, to_checkpoint)
    #print("to_target", file=sys.stderr, flush=True)
    #print(to_target, file=sys.stderr, flush=True)

    # This is our target (X,Y)
    target_x = to_target[0][0] + x
    target_y = to_target[1][0] + y

    # See the case in which we aim at the checkpoint after the next one
    if switch_checkpoint:
        target_x = next_checkpoint_x
        target_y = next_checkpoint_y

    map_center = [8000, 4500]   # The center of the map

    r = None

    if thrust == "BOOST":
        boost1 = 0
        r = str(int(target_x)) + " " + str(int(target_y)) + " " + thrust
    else:
        r = str(int(target_x)) + " " + str(int(target_y)) + " " + str(int(thrust))

    return loop, last_checkpoint_id, boost, r

# game loop
while True:
    x1, y1, vx1, vy1, angle1, next_check_point_id1 = [int(j) for j in input().split()]
    x2, y2, vx2, vy2, angle2, next_check_point_id2 = [int(j) for j in input().split()]

    x_1, y_1, vx_1, vy_1, angle_1, next_check_point_id_1 = [int(j) for j in input().split()]
    x_2, y_2, vx_2, vy_2, angle_2, next_check_point_id_2 = [int(j) for j in input().split()]
    
    def hunter():
        """
        PARAMETERS
        """
        direction_compensation = 1    # The amount to overcompensate the direction to next checkpoint
        thrust_amplitude = 140  # Amplitude of the gaussian defining the thrust
        thrust_deviation = 5000  # Deviation of the gaussian defining the thrust
        thrust_min = 50  # Thrust when we are going in the wrong direction
        dist_boost = 6000   # At what distance from checkpoint is it safe to boost
        angle_boost = 10    # At what angle from checkpoint is it safe to boost

        """
        DEFINING METRICS
        """
        to_target_1 = np.array([[x_1 - x2], [y_1 - y2]])
        to_target_2 = np.array([[x_2 - x2], [y_2 - y2]])
        dist_target_1 = np.linalg.norm(to_target_1)
        dist_target_2 = np.linalg.norm(to_target_2)

        if dist_target_1 < dist_target_2:
            target = [x_1, y_1]
            to_target = to_target_1
            dist_target = dist_target_1
        else:
            target = [x_2, y_2]
            to_target = to_target_2
            dist_target = dist_target_2

        vect_speed = np.array([[vx2], [vy2]])
        speed = np.linalg.norm(vect_speed)

        def calc_angle(x1, y1, x2, y2):
            return np.degrees(np.arctan2(y2-y1,x2-x1))

        to_target_abs = calc_angle(0, 0, to_target[0][0], to_target[1][0])
        target_angle = (to_target_abs - angle2 + 180) % 360 - 180

        target_angle_before = target_angle

        target_angle = target_angle*direction_compensation
        target_angle = min(target_angle, 179)
        target_angle = max(target_angle, -179)

        print("target_angle", file=sys.stderr, flush=True)
        print(target_angle, file=sys.stderr, flush=True)


        """
        THRUST BEHAVIOUR
        """
        if target_angle > 90 or target_angle < -90:
            thrust = thrust_min
        else:
            thrust = thrust_amplitude * \
                np.exp(-target_angle**2/thrust_deviation)
            thrust = int(min(100, thrust))
        
        """
        DIRECTION BEHAVIOUR
        """

        diff_angle = target_angle - target_angle_before

        theta = np.radians(diff_angle)
        c, s = np.cos(theta), np.sin(theta)
        R = np.array(((c, -s), (s, c)))

        to_target = np.dot(R, to_target)

        target_x = to_target[0][0] + x2
        target_y = to_target[1][0] + y2

        print(str(int(target_x)) + " " + str(int(target_y)) + " " + str(thrust))
    
    # You have to output the target position
    # followed by the power (0 <= thrust <= 100)
    # i.e.: "x y thrust"
    loop1, last_checkpoint_id1, boost1, r1 =\
        behavior1(x1,y1,vx1,vy1,angle1, next_check_point_id1, loop1, last_checkpoint_id1, boost1)
    print(r1)
    '''loop2, last_checkpoint_id2, boost1, r2 =\
        behavior1(x2,y2,vx2,vy2,angle2, next_check_point_id2, loop2, last_checkpoint_id2, boost2)
    print(r2)'''
    hunter()


   
