# Custom ros msg for Robotiq Grippers' Finger Status

uint8 finger_id # [Hand-e] not used
                # [3F] 0 = Finger A, 1 = Finger B, 2 = Finger C, 3 = Scissor
uint8 position  # 0 = open, 255 = closed
uint8 speed     # 0 = stopped, 255 = max speed
uint8 force     # 0 = low force, 255 = max force