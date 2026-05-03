import time

import mujoco
import mujoco.viewer
import numpy as np

import rotoscrews
import read_xbox

# Window to show ctrl-inputs
import tkinter as tk
root = tk.Tk()
label = tk.Label(root, text="", font=("Courier", 16))
label.pack()



m = mujoco.MjModel.from_xml_path(r"rototilt\rototilt.urdf")
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

    return -1*des_pith, des_tilt, des_rot*0.2


with mujoco.viewer.launch_passive(m, d) as viewer:
    # Close the viewer automatically after 30 wall-seconds.
    start = time.time()

    q = np.zeros(m.nq)
    # Starting pos
    q[0] = np.deg2rad(-45)    # Pitch
    q[1] = np.deg2rad(0)    # Tilt
    q[2] = np.deg2rad(0)    # Rotation

    while viewer.is_running() and time.time() - start < 120:
        step_start = time.time()

        # Only run the fwd kinematics in mujoco
        d.qpos[:] = q
        mujoco.mj_forward(m, d)

        # Xbox inputs
        des_pith, des_tilt, des_rot = read_3_angs_xbox()

        label.config(text=f"Pitch: {des_pith:.2f}  Tilt: {des_tilt:.2f}  Rot: {des_rot:.2f}")
        root.update()

        # Inv kinematics or manual?
        if robot_mode:
            dot_theta,Js_elias = rotoscrews.inv_kinematics(q[0],q[1],q[2],des_pith*2, des_tilt*2, des_rot*2)
            for i in range(3):
                q[i] += dot_theta[i]*0.1

            # Compare jacobians from sim and screws
            jacp = np.empty((3,m.nv)) 
            jacr = np.empty((3,m.nv)) 
            point = np.array([0,0,0])
            body = np.array([3])
            mujoco.mj_jac(m, d, jacp, jacr, point, body)

            Js_muju = np.r_[jacr,jacp]
            
            err = np.abs(Js_muju - Js_elias)

            stop = np.any(err > 1e-6)
            
            if stop:

                print("MUJUCO")
                print(Js_muju)
                print("Elias")
                print(Js_elias)
                print("Err")
                print(err.round(3))

                raise Exception("ERROR BETWEEN ELIAS AND MUJOCO")


        else:
            # Manual mode
            q[0] += des_pith*1e-2*2
            q[1] += des_tilt*1e-2*2
            q[2] += des_rot*1e-1*1.5

        d.qpos[:] = q
        mujoco.mj_forward(m, d)

        # Pick up changes to the physics state, apply perturbations, update options from GUI.
        viewer.sync()

        # Rudimentary time keeping, will drift relative to wall clock.
        time_until_next_step = m.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)
