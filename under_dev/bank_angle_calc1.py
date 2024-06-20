import math
import numpy as np
def get_bank_angle(attitude):
    roll, pitch, yaw = np.radians(attitude)
    
    R = np.array([
        [
            math.cos(yaw) * math.cos(pitch),
            math.cos(yaw) * math.sin(pitch) * math.sin(roll) - math.sin(yaw) * math.cos(roll),
            math.cos(yaw) * math.sin(pitch) * math.cos(roll) + math.sin(yaw) * math.sin(roll)
        ],
        [
            math.sin(yaw) * math.cos(pitch),
            math.sin(yaw) * math.sin(pitch) * math.sin(roll) + math.cos(yaw) * math.cos(roll),
            math.sin(yaw) * math.sin(pitch) * math.cos(roll) - math.cos(yaw) * math.sin(roll)
        ],
        [
            -math.sin(pitch),
            math.cos(pitch) * math.sin(roll),
            math.cos(pitch) * math.cos(roll)
        ]
    ])
    
    bank_angle = np.degrees(math.atan2(R[2, 1], R[2, 2]))
    return bank_angle


# print(get_bank_angle([80,30,10]))