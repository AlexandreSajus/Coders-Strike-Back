"""
Submission by Alexandre Sajus
Coders Strike Back Challenge: https://www.codingame.com/multiplayer/bot-programming/coders-strike-back
More information at:
https://github.com/AlexandreSajus
"""

import sys
import math
import numpy as np
from math import sqrt

# Auto-generated code below aims at helping you parse
# the standard input according to the problem statement.

# I save the last positions to compute speed
last_x = 0
last_y = 0

# game loop
while True:
    """
    INPUT
    """
    # next_checkpoint_x: x position of the next check point
    # next_checkpoint_y: y position of the next check point
    # next_checkpoint_dist: distance to the next checkpoint
    # next_checkpoint_angle: angle between your pod orientation and the direction of the next checkpoint
    x, y, next_checkpoint_x, next_checkpoint_y, next_checkpoint_dist, next_checkpoint_angle = [
        int(i) for i in input().split()]
    opponent_x, opponent_y = [int(i) for i in input().split()]

    # Write an action using print
    # To debug: print("Debug messages...", file=sys.stderr, flush=True)

    """
    PARAMETERS
    """
    direction_compensation = 1.2    # The amount to overcompensate the direction to next checkpoint
    dist_slowdown = 2400    # At what distance from checkpoint we slow down
    thrust_slowdown = 20    # What thrust when we slow down
    thrust_amplitude = 140  # Amplitude of the gaussian defining the thrust
    thrust_deviation = 5000  # Deviation of the gaussian defining the thrust
    thrust_min = 50  # Thrust when we are going in the wrong direction
    dist_boost = 2500   # At what distance from checkpoint is it safe to boost
    angle_boost = 10    # At what angle from checkpoint is it safe to boost
    boost = 1   # Do we have a boost
    radius = 1000   # The radius of a checkpoint

    """
    DEFINING METRICS
    """
    # Compute the distance to next checkpoint
    dist_checkpoint = sqrt((x - next_checkpoint_x) **
                           2 + (y - next_checkpoint_y)**2)
    #print("dist_checkpoint", file=sys.stderr, flush=True)
    #print(dist_checkpoint, file=sys.stderr, flush=True)

    # Compute our current speed
    speed = sqrt((x - last_x)**2 + (y - last_y)**2)
    #print("speed", file=sys.stderr, flush=True)
    #print(speed, file=sys.stderr, flush=True)

    # Define a target angle by overcompensating
    target_angle = next_checkpoint_angle*direction_compensation
    # Clamp the target angle
    target_angle = min(179, target_angle)
    target_angle = max(-179, target_angle)

    # Compute the difference between checkpoint and target angle
    # We will use this to define a target point as (X,Y)
    diff_angle = target_angle - next_checkpoint_angle
    #print("diff_angle", file=sys.stderr, flush=True)
    #print(diff_angle, file=sys.stderr, flush=True)

    map_center = [8000, 4500]   # The center of the map

    """
    THRUST BEHAVIOUR
    """
    # If we are going in the wrong direction, low thrust
    if next_checkpoint_angle > 90 or next_checkpoint_angle < -90:
        thrust = thrust_min
    # If the next checkpoint is far and we are aiming at it, BOOST
    elif dist_checkpoint > 8000 and abs(next_checkpoint_angle) < angle_boost and boost == 1:
        thrust = "BOOST"
        boost = 0
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

    # We save those to compute speed later
    last_x = x
    last_y = y

    # You have to output the target position
    # followed by the power (0 <= thrust <= 100)
    # i.e.: "x y thrust"

    """
    OUTPUT
    """
    if thrust == "BOOST":
        print(str(int(target_x)) + " " + str(int(target_y)) + " " + thrust)
    else:
        print(str(int(target_x)) + " " +
              str(int(target_y)) + " " + str(int(thrust)))