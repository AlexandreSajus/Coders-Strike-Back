import sys
from math import sqrt, atan2
import numpy as np

laps = int(input())
checkpoint_count = int(input())
for i in range(checkpoint_count):
    checkpoint_x, checkpoint_y = [int(j) for j in input().split()]

while True:
    x1, y1, vx1, vy1, angle1, next_check_point_id1 = [int(j) for j in input().split()]
    x2, y2, vx2, vy2, angle2, next_check_point_id2 = [int(j) for j in input().split()]

    x_1, y_1, vx_1, vy_1, angle_1, next_check_point_id_1 = [int(j) for j in input().split()]
    x_2, y_2, vx_2, vy_2, angle_2, next_check_point_id_2 = [int(j) for j in input().split()]
    print("8000 4500 100")

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

        target_angle = target_angle*direction_compensation

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
        theta = np.radians(target_angle)
        c, s = np.cos(theta), np.sin(theta)
        R = np.array(((c, -s), (s, c)))

        to_target = np.dot(R, to_target)

        target_x = to_target[0][0] + x2
        target_y = to_target[1][0] + y2

        print(str(int(target_x)) + " " + str(int(target_y)) + " " + str(thrust))
    
    hunter()