# Custom ros msg for Robotiq Grippers' Finger Status

uint8 finger_id     # [Hand-e] not used
                    # [3F] 0 = Finger A, 1 = Finger B, 2 = Finger C, 3 = Scissor
uint8 g_pr           # Requested position
uint8 g_po           # Current position
                    # 0 = open, 255 = closed
uint8 g_cu           # Motor current consumption
                    # mA = gCU * 0.1
uint8 g_dt           # Object detection trigger
                    # 0 = Finger is in motion, 1 = Object detected while opening, 2 = Object detected while closing, 3 = Finger at requested position