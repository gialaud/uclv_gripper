# Custom ros msg for Robotiq Grippers status
std_msgs/Header header

uint8 g_act  # Gripper activation status
            # 0 = Gripper reset; 1 = Gripper active
uint8 g_mod  # Gripper mode
            # (for 3f) 0 = Basic mode; 1 = Pinch mode; 2 = Wide mode; 3 = Scissor mode
uint8 g_gto  # Gripper action status
            # 0 = Stop; 1 = Go
uint8 g_sta  # Gripper status
            # 0 = Reset or Auto-Release; 1 = Activation in progress; 2 = Mode change in progress; 3 = Ready
uint8 g_obj  # Gripper object detection status
            # [Hand-e] 0 = Fingers in motion; 1 = Object while opening; 2 = Object while closing; 3 = Fingers at req. pos.
            # [3f] 0 = Fingers in motion; 1 = One or two fingers stopped; 2 = All fingers stopped; 3 = Fingers at req. pos.
uint8 g_flt  # Gripper fault status (see manual for fault codes)

uclv_robotiq_interfaces/FingerStatus[] fingers
            