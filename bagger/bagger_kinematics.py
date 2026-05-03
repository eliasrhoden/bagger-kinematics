import modern_robotics as mr
import numpy as np
from mr_urdf_loader import loadURDF

urdf_name = r"bagger\bagger_kinematics_no_STLs.urdf"
mr_URDF=loadURDF(urdf_name)


def tcp(q):
    global mr_URDF

    S_list = mr_URDF["Slist"]

    m_R = np.array([[1,0,0],
                    [0,0,1],
                    [0,-1,0]])
    m_p = np.array([0,5.310, 3.518])

    M = mr.RpToTrans(m_R, m_p)
    T_tcp = mr.FKinSpace(M,S_list, q)

    return T_tcp

def tcp_linear(q, x_des, y_des, z_des):
    global mr_URDF

    S_list = mr_URDF["Slist"]
    
    Js = mr.JacobianSpace(S_list,q)

    # what is our desired global twist?

    # pitch and tilt in body frame, i.e. rotate from current tcp
    T_tcp = tcp(q)
    R_tcp,p_tcp = mr.TransToRp(T_tcp)

    # we only want linear movement in global frame
    
    vel_vec = np.array([x_des,y_des,z_des])
   

    # Resulting desired (global/spatial) twist is around origo, i.e. zero linear vel
    zero_p = np.array([0,0,0])

    V_des = np.r_[np.zeros(3),vel_vec]

    # Robot is over defined, only use selcted joints
    q_indx = [
        0, # main swing
        #1, # boom swing
        2, # boom
        3, # sticka
        4, # bucket pitch
        5, # tilt
        6, # rotator
    ]

    Js = np.array(Js)
    nj = Js.shape[1]
    col_indx = [i in q_indx for i in range(nj)]
    Js_act = Js[:,col_indx]

    sol = np.linalg.lstsq(Js_act, V_des, rcond=-1)
    theta_act = sol[0]

    theta = np.zeros(nj)

    theta[col_indx] = theta_act 

    V_res = Js@theta 

    V_err = np.abs(V_des - V_res)
    if np.max(V_err) > 1e-2:
        print("Large error in inverse kinematics")
        print(V_err)

        #raise Exception("Failed to copmute inverse kinematics")

    return theta, Js


def tcp_linear2(q, x_des, y_des, z_des,Js):

    # what is our desired global twist?

    # pitch and tilt in body frame, i.e. rotate from current tcp
    T_tcp = tcp(q)
    R_tcp,p_tcp = mr.TransToRp(T_tcp)

    # we only want linear movement in global frame
    
    vel_vec = np.array([x_des,y_des,z_des])

    p_zero = np.zeros(3)

    #V_des = mr.ScrewToAxis(p_zero,vel_vec,0.0)
    V_des = np.r_[np.zeros(3),vel_vec]

    # Robot is over defined, only use selcted joints
    q_indx = [
        0, # main swing
        #1, # boom swing
        2, # boom
        3, # sticka
        4, # bucket pitch
        5, # tilt
        6, # rotator
    ]

    Js = np.array(Js)
    nj = Js.shape[1]
    col_indx = [i in q_indx for i in range(nj)]
    Js_act = Js[:,col_indx]

    sol = np.linalg.lstsq(Js_act, V_des, rcond=-1)
    theta_act = sol[0]

    theta = np.zeros(nj)
    theta[col_indx] = theta_act 

    V_res = Js@theta 

    V_err = np.abs(V_des - V_res)
    if np.max(V_err) > 1e-2:
        print("Large error in inverse kinematics")
        print(V_err)

        #raise Exception("Failed to copmute inverse kinematics")

    return theta, Js

if __name__ == '__main__':

    q = np.array([0,0,0,0,0,0,0])

    theta, Js = tcp_linear(q,-1.0,0.0,0.0)

    print(theta)
    print('****')
    print(Js)






