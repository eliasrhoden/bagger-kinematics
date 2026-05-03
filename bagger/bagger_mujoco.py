import time

import mujoco
import mujoco.viewer
import numpy as np

import bagger_kinematics
import read_xbox

# Window to show ctrl-inputs
import tkinter as tk
root = tk.Tk()
label = tk.Label(root, text="", font=("Courier", 16))
label.pack()


m = mujoco.MjModel.from_xml_path(r"bagger\bagger_kinematics_3.urdf")
d:mujoco.MjData = mujoco.MjData(m)

xbox = read_xbox.Xbox()
#xbox = read_xbox.DummyXbox()

robot_mode = True

def read_3_angs_xbox():

    left_x, left_y, right_x, right_y = xbox.read_value()

    #des_rot = right_x
    #des_pith = right_y 
    #des_tilt = left_x

    des_rot = left_x
    des_pith = right_y 
    des_tilt = right_x*-1

    DZ = 5*1e-2

    if np.abs(des_rot) < DZ:
        des_rot = 0

    if np.abs(des_pith) < DZ:
        des_pith = 0


    if np.abs(des_tilt) < DZ:
        des_tilt = 0

    return -1*des_pith, -1*des_tilt, des_rot*0.2

def deadzone(value, dz):

    if np.abs(value) < dz:
        return 0.0
    else:
        return value - np.sign(value)*dz

with mujoco.viewer.launch_passive(m, d) as viewer:
    # Close the viewer automatically after 30 wall-seconds.
    start = time.time()

    q = np.zeros(m.nq)
    # Starting pos
    q[0] = np.deg2rad(0)   
    q[1] = np.deg2rad(0)    
    q[2] = np.deg2rad(60)    
    q[3] = np.deg2rad(60)    

    while viewer.is_running() and time.time() - start < 120:
        step_start = time.time()

        # Only run the fwd kinematics in mujoco
        d.qpos[:] = q
        mujoco.mj_forward(m, d)


        # Inv kinematics or manual?
        if robot_mode:

            # Xbox input
            #des_pith, des_tilt, des_rot = read_3_angs_xbox()

            # Compare jacobians from sim and screws
            jacp = np.empty((3,m.nv)) 
            jacr = np.empty((3,m.nv)) 
            point = np.array([0,0,0])
            body = np.array([m.nv])
            mujoco.mj_jac(m, d, jacp, jacr, point, body)

            Js_muju = np.r_[jacr,jacp]

            DZ = 5*1e-2
            left_x, left_y, right_x, right_y = xbox.read_value()



            #xdot = -1*deadzone(left_x,DZ)
            #ydot = deadzone(right_y,DZ)
            #zdot = -1*deadzone(left_y,DZ)

            xdot = -1*deadzone(left_x,DZ)
            zdot = deadzone(right_y,DZ)
            ydot = deadzone(left_y,DZ)

            label.config(text=f"X-vel: {xdot:.2f}  Y-vel: {ydot:.2f}  Z-vel: {zdot:.2f}")
            root.update()


            #dot_theta,Js_elias = rotoscrews.inv_kinematics(q[0],q[1],q[2],des_pith*2, des_tilt*2, des_rot*2)

            dot_theta,Js_elias = bagger_kinematics.tcp_linear(q, xdot, ydot, zdot)
            #dot_theta,Js_elias = bagger_kinematics.tcp_linear2(q, xdot, ydot, zdot, Js_muju)

            for i in range(len(dot_theta)):
                q[i] += dot_theta[i]*1e-1*0.7


            
            err = np.abs(Js_muju - Js_elias)
            max_err = np.max(err)

            stop = np.any(max_err > 1e-3)
            
            if stop:
                print(f"*** {max_err} ***")
                print("MUJUCO")
                print(np.round(Js_muju,3))
                print("Elias")
                print(np.round(Js_elias,3))
                print("Err")
                print(err.round(3))
                print("")

                #raise Exception("ERROR BETWEEN ELIAS AND MUJOCO")


        else:
            # Manual mode
            DZ = 5*1e-2
            left_x, left_y, right_x, right_y = xbox.read_value()

            #Swing
            q[0] += -1*deadzone(left_x, DZ)*1e-2*1.5

            # boom
            q[2] += -1*deadzone(right_y, DZ)*1e-2*1.5

            # stick
            q[3] += deadzone(left_y, DZ)*1e-2*1.5

            # bucket pitch
            q[4] += -1*deadzone(right_x, DZ)*1e-2*1.5
            

        #d.qpos[:] = q
        #mujoco.mj_forward(m, d)

        # Pick up changes to the physics state, apply perturbations, update options from GUI.
        viewer.sync()

        # Rudimentary time keeping, will drift relative to wall clock.
        time_until_next_step = m.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)
