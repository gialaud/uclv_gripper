#!/usr/bin/env python

import rclpy
import tkinter as tk

from uclv_robotiq_interfaces.msg import FingerCommandArray, FingerCommand

publisher_ = None

def pressed(pos,vel,force):
    global publisher_
    cmd_msg = FingerCommandArray()
    cmd_msg.fingers_command.clear()
    finger = FingerCommand()
    finger.position = pos
    finger.speed = vel
    finger.force = force
    cmd_msg.fingers_command.append(finger)
    publisher_.publish(cmd_msg)

def main(args=None):
    global publisher_
    rclpy.init(args=args)
    node = rclpy.create_node('hand_e_gui')

    publisher_ = node.create_publisher(FingerCommandArray, 'fingers_command', 10)

    window = tk.Tk()
    window.title("Hand-E Gripper")
    window.minsize(10,10)
    
    pos1_lbl = tk.Label(window, text="Position (0-255):", font=('helvetica', 20))
    pos1_lbl.grid(column=0, row=0, padx=5, pady=10) 
    pos1_form = tk.Entry(window, width=5, font=('helvetica', 20))
    pos1_form.insert(0,"120")
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

    window.mainloop()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
