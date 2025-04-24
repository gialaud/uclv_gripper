#!/usr/bin/env python

import rclpy
import tkinter as tk

from rclpy.action import ActionClient

from uclv_robotiq_interfaces.msg import FingerCommandArray, FingerCommand
from uclv_robotiq_interfaces.action import ChangeMode

publisher_ = None
change_gripper_mode_client = None

def pressed(pos,vel,force):
    global publisher_
    cmd_msg = FingerCommandArray()
    cmd_msg.fingers_command.clear()
    finger = FingerCommand()
    finger.position = pos
    finger.speed = vel
    finger.force = force
    finger.finger_id = 0
    cmd_msg.fingers_command.append(finger)
    publisher_.publish(cmd_msg)

def change_mode(node, mode):
    global change_gripper_mode_client
    goal = ChangeMode.Goal()
    goal.target_mode = mode
    goal.individual_finger_control = False
    goal.individual_scissor_control = False

    while not change_gripper_mode_client.wait_for_server(timeout_sec=1.0):
        print('change gripper mode not available, waiting again...')
    
    send_goal_future = change_gripper_mode_client.send_goal_async(goal)
    
    rclpy.spin_until_future_complete(node, send_goal_future)

    goal_handle = send_goal_future.result()
    if not goal_handle.accepted:
        print('Change Mode Goal rejected')
        return -1

    get_result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, get_result_future)

    result = get_result_future.result()
    if result.result.success is True:
        return 0    
    else:
        print('Change Mode Action failed')
        return -1

def main(args=None):
    global publisher_
    global change_gripper_mode_client
    rclpy.init(args=args)
    node = rclpy.create_node('threef_gui')

    publisher_ = node.create_publisher(FingerCommandArray, 'threef_fingers_command', 10)

    change_gripper_mode_client = ActionClient(node, ChangeMode, '/threef_change_mode')

    window = tk.Tk()
    window.title("3f Gripper")
    window.minsize(10,10)
    
    pos1_lbl = tk.Label(window, text="Position (0-255):", font=('helvetica', 20))
    pos1_lbl.grid(column=0, row=0, padx=5, pady=10) 
    pos1_form = tk.Entry(window, width=5, font=('helvetica', 20))
    pos1_form.insert(0,"80")
    pos1_form.grid(column=1, row=0, padx=5, pady=10)

    vel_lbl = tk.Label(window, text="Velocity (0-255):", font=('helvetica', 20))
    vel_lbl.grid(column=0, row=2, padx=5, pady=10) 
    vel_form = tk.Entry(window, width=5, font=('helvetica', 20))
    vel_form.insert(0,"255")
    vel_form.grid(column=1, row=2, padx=5, pady=10)

    force_lbl = tk.Label(window, text="Force (0-255):", font=('helvetica', 20))
    force_lbl.grid(column=0, row=3, padx=5, pady=10) 
    force_form = tk.Entry(window, width=5, font=('helvetica', 20))
    force_form.insert(0,"100")
    force_form.grid(column=1, row=3, padx=5, pady=10)


    button1 = tk.Button(window, 
                        text="Send Pos",
                        activeforeground = "red",
                        bd = 3,
                        fg="black",
                        font=('helvetica', 20, 'bold'),
                        command=lambda: pressed(int(pos1_form.get()),
                                                int(vel_form.get()),
                                                int(force_form.get())))
    button1.grid(column=1, row=4, padx=10, pady=10)

    button2 = tk.Button(window, 
                        text="Open",
                        activeforeground = "red",
                        bd = 3,
                        fg="black",
                        font=('helvetica', 20, 'bold'),
                        command=lambda: pressed(int(0),
                                                int(vel_form.get()),
                                                int(force_form.get())))
    button2.grid(column=0, row=4, padx=10, pady=10)

    mode_lbl = tk.Label(window, text="Modality (0-3):", font=('helvetica', 20))
    mode_lbl.grid(column=0, row=5, padx=5, pady=10) 
    mode_form = tk.Entry(window, width=5, font=('helvetica', 20))
    mode_form.insert(0,"0")
    mode_form.grid(column=1, row=5, padx=5, pady=10)

    button3 = tk.Button(window, 
                        text="Change Mode",
                        activeforeground = "red",
                        bd = 3,
                        fg="black",
                        font=('helvetica', 20, 'bold'),
                        command=lambda: change_mode(node, int(mode_form.get())))
    button3.grid(column=1, row=6, padx=10, pady=10)

    window.mainloop()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
