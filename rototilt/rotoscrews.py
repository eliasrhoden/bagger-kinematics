import modern_robotics as mr
import numpy as np

def create_zero_frames():
    # Returns all frames at zero-config
    scale = 1e-3

    # Base (Base frame) end of sticka
    T0 = mr.RpToTrans(np.eye(3),np.zeros(3))

    # Bucket pitch
    S0 = mr.ScrewToAxis(np.array([0,0,0]), np.array([1,0,0]), 0)

    # Offset to tilt joint
    tilt_ax_dist_z = 62*scale
    tilt_ax_dist_y = 25*scale
    tilt_ax_dist_x = 25*scale
    T1 = mr.RpToTrans(np.eye(3),np.array([-tilt_ax_dist_x,tilt_ax_dist_y,-tilt_ax_dist_z]))

    # Tilt joint
    S1 = mr.ScrewToAxis(np.array([0,0,0]), np.array([0,1,0]), 0)

    # Offset tilt to rotation
    offset_z_tilt_to_rot = 57*scale
    offset_y_tilt_to_rot = 50*scale
    T2  = mr.RpToTrans(np.eye(3),np.array([0,offset_y_tilt_to_rot,-offset_z_tilt_to_rot]))

    # Rotation joint
    S2 = mr.ScrewToAxis(np.array([0,0,0]), np.array([0,0,1]), 0)

    # Offset to bucket tcp from rotator
    tcp_heigh = 215*scale
    tcp_depth = 160*scale
    T3 = mr.RpToTrans(np.eye(3),np.array([0,-tcp_depth,-tcp_heigh]))

    return T0, S0, T1,S1, T2, S2, T3


def fwd_kinematics(pitch, tilt, rot):

    T0,S0, T1,S1, T2,S2, T3 = create_zero_frames()
    
    # pitched frame
    T0p = mr.MatrixExp6(mr.VecTose3(S0*pitch)) @ T0 
    
    # tilted frame
    T1p = T0p @ T1 @ mr.MatrixExp6(mr.VecTose3(S1*tilt))

    # rotated frame
    T2p = T1p @ T2 @ mr.MatrixExp6(mr.VecTose3(S2*rot))

    # bucket tcp
    T3p = T2p @ T3

    return T0p, T1p, T2p, T3p



def reduced_screws(pitch, tilt, rot):
    T0,S0, T1,S1, T2, S2, T3 = create_zero_frames()

    M = T0@T1@T2
    
    S1s = mr.Adjoint(T1)@S1
    S2s = mr.Adjoint(T1@T2)@S2

    Se0 = mr.MatrixExp6(mr.VecTose3(S0*pitch))
    Se1 = mr.MatrixExp6(mr.VecTose3(S1s*tilt))
    Se2 = mr.MatrixExp6(mr.VecTose3(S2s*rot))

    return S0, S1s, S2s, M


def jacobian_spatial(pitch, tilt, rot):

    # Reduced screws
    S0, S1s, S2s, M = reduced_screws(pitch, tilt, rot)

    # Exp coordinates
    Se0 = mr.MatrixExp6(mr.VecTose3(S0*pitch))
    Se1 = mr.MatrixExp6(mr.VecTose3(S1s*tilt))
    Se2 = mr.MatrixExp6(mr.VecTose3(S2s*rot))

    # spatial end effector jacobian
    Js0 = S0 
    Js1 = mr.Adjoint(Se0)@S1s 
    Js2 = mr.Adjoint(Se0@Se1)@S2s

    Js = np.c_[Js0, Js1, Js2]
   
    return Js


def inv_kinematics(pitch, tilt, rot , des_pitch, des_tilt, des_rot):

    Js = jacobian_spatial(pitch, tilt, rot)

    # spatial twist Vs = [Js0 Js1 Js2] @ [dot th0 dot th1 dot th2]

    # what is our desired global twist?

    # pitch and tilt in body frame, i.e. rotate from current tcp
    T_tcp = tcp(pitch, tilt, rot)
    R_tcp,_ = mr.TransToRp(T_tcp)

    # Convert body rot to global rot
    omega_b = np.array([des_pitch, des_tilt, 0.0])*0.1
    omega_s = R_tcp@omega_b

    # yaw (rotation) is always around global Z-axis, simply add it
    omega_s[2] += des_rot


    # Resulting desired (global/spatial) twist is around origo, i.e. zero linear vel
    zero_p = np.array([0,0,0])

    V_des = mr.ScrewToAxis(zero_p, omega_s,0)

    sol = np.linalg.lstsq(Js, V_des, rcond=-1)
    theta = sol[0]

    V_res = Js@theta 

    V_err = np.abs(V_des - V_res)
    if np.max(V_err) > 1e-2:
        print("Large error in inverse kinematics")
        print(V_err)

        #raise Exception("Failed to copmute inverse kinematics")

    return theta, Js



def tcp(pitch, tilt, rot):

    S0, S1s, S2s, M = reduced_screws(pitch, tilt, rot)

    Se0 = mr.MatrixExp6(mr.VecTose3(S0*pitch))
    Se1 = mr.MatrixExp6(mr.VecTose3(S1s*tilt))
    Se2 = mr.MatrixExp6(mr.VecTose3(S2s*rot))

    T_tcp = Se0@Se1@Se2@M 

    return T_tcp



def main():


    pitch = np.deg2rad(20)
    tilt = np.deg2rad(11)
    rot = np.deg2rad(90)

    pitch = np.deg2rad(0)
    tilt = np.deg2rad(0)
    rot = np.deg2rad(0)

    _,_,T,_ = fwd_kinematics(pitch, tilt, rot)

    Tr = reduced_screws(pitch, tilt, rot)

    print(T)
    print("")
    print(Tr)
    print("")
    err_mat = (T - Tr).round(2)
    max_err = np.max(np.abs(err_mat))
    print(err_mat)
    print("error ", max_err)

    print("tcp")
    tcp(pitch, tilt, rot) 

if __name__ == '__main__':
    main()

