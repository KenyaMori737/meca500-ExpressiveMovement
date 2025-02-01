# 2025.01.25 Author @ Kenya Mori
# 手先の指令値に追従することの確認

import os
import sys
import math
import csv
import numpy as np
import time

import mecademicpy.robot as mdr

ACTIVATION_TIMEOUT = 40
dt = 0.01
robot = mdr.Robot()

#OK
def rotz(ang):
    """
    Returns a 3x3 rotation matrix representing a rotation
    about the z-axis by 'ang' degrees.
    """
    ang_rad = ang * np.pi / 180.0  # convert degrees to radians
    R = np.eye(3)
    R[0, 0] = np.cos(ang_rad)
    R[0, 1] = -np.sin(ang_rad)
    R[1, 0] = np.sin(ang_rad)
    R[1, 1] = np.cos(ang_rad)
    return R


def cross_x(v):
    """
    Returns the 3x3 skew-symmetric cross-product matrix of a 3D vector v.
    That is:
        cross_x([v1, v2, v3]) = [[ 0,   -v3,  v2 ],
                                [ v3,   0,  -v1 ],
                                [-v2,   v1,  0  ]]
    """
    return np.array([
        [0.0,    -v[2],  v[1]],
        [v[2],   0.0,   -v[0]],
        [-v[1],  v[0],   0.0]
    ])

#OK
def calc_Jacobian_i_j(R_j_i, p_j_ji):
    """
    Python equivalent of:
        function J_i_j = calc_Jacobian_i_j(R_j_i, p_j_ji)
            R_i_j = R_j_i';
            J_i_j = [R_i_j,           -R_i_j * cross_x(p_j_ji);
                     zeros(3, 3),      R_i_j];
        end

    Parameters
    ----------
    R_j_i : np.ndarray, shape (3, 3)
        Rotation matrix from frame j to frame i (in MATLAB, R_j_i).
    p_j_ji : np.ndarray, shape (3,)
        Position vector from joint j to joint i, expressed in frame j.

    Returns
    -------
    J_i_j : np.ndarray, shape (6, 6)
        The 6x6 Jacobian transformation matrix.
    """
    # Transpose in Python is simply R_j_i.T
    R_i_j = R_j_i.T

    # Skew-symmetric matrix of p_j_ji
    cross_p = cross_x(p_j_ji)

    # Construct the 6x6 block matrix
    # top-left:    R_i_j
    # top-right:   - R_i_j * cross_x(p_j_ji)
    # bottom-left: 0
    # bottom-right R_i_j
    top_left = R_i_j
    top_right = -R_i_j @ cross_p
    bottom_left = np.zeros((3, 3))
    bottom_right = R_i_j

    J_i_j = np.block([
        [top_left,     top_right],
        [bottom_left,  bottom_right]
    ])
    return J_i_j


def calc_eff_jacobian2(tmp_q):
    """
    Translated version of the MATLAB function:
    
        function [J_eff] = calc_eff_jacobian2(obj, tmp_q)
    
    It computes an effective Jacobian (J_eff) given the joint angles 'tmp_q'.
    """

    # local_data indexed from 1..7 to match MATLAB style.
    # We insert a dummy entry at index 0 so local_data[1] matches MATLAB's local_data(1).
    local_data = [None] * 8
    
    local_data[1] = {
        'R_base_j_i': np.array([1, 0, 0, 0, 1, 0, 0, 0, 1], dtype=float),
        'p_j_ji':     np.array([0.0,     0.0,     0.0],     dtype=float)
    }
    local_data[2] = {
        'R_base_j_i': np.array([1, 0, 0, 0, 1, 0, 0, 0, 1], dtype=float),
        'p_j_ji':     np.array([0.0,     0.0,     0.0910],  dtype=float)
    }
    local_data[3] = {
        'R_base_j_i': np.array([1.0000,  0,       0,
                                0,       0.0000, -1.0000,
                                0,       1.0000,  0.0000], dtype=float),
        'p_j_ji':     np.array([0.0,     0.0,     0.0445],  dtype=float)
    }
    local_data[4] = {
        'R_base_j_i': np.array([1, 0, 0, 0, 1, 0, 0, 0, 1], dtype=float),
        'p_j_ji':     np.array([0.0,    -0.1350,  0.0],     dtype=float)
    }
    local_data[5] = {
        'R_base_j_i': np.array([0.0000,  0,      -1.0000,
                                0,       1.0000,  0,
                                1.0000,  0,       0.0000], dtype=float),
        'p_j_ji':     np.array([0.0615, -0.0380,  0.0],     dtype=float)
    }
    local_data[6] = {
        'R_base_j_i': np.array([0.0000,  0,       1.0000,
                                0,       1.0000,  0,
                                -1.0000, 0,       0.0000], dtype=float),
        'p_j_ji':     np.array([0.0,     0.000004, 0.0584], dtype=float)
    }
    local_data[7] = {
        'R_base_j_i': np.array([0.0000,  0,      -1.0000,
                                0,       1.0000,  0,
                                1.0000,  0,       0.0000], dtype=float),
        'p_j_ji':     np.array([0.0590,  0.0,     0.0],     dtype=float)
    }

    start_idx = 2
    end_idx   = 7

    # Create a 'link' list so link[k] can hold R_0_i, p_0_0i, etc.
    link = [None] * (end_idx + 1)

    # Compute for "L0" (MATLAB's if start_idx == 2, set link(1) = identity, zero)
    if start_idx == 2:
        link[1] = {
            'R_0_i':  np.eye(3, dtype=float),
            'p_0_0i': np.zeros(3, dtype=float)
        }

    # Compute for L1 - L6 in MATLAB, which corresponds to k in [2..7].
    for k in range(start_idx, end_idx + 1):
        if k == 1:
            # In MATLAB code this branch is never hit since start_idx=2, but we keep it for completeness.
            link[k] = {
                'R_0_i':  np.eye(3, dtype=float),
                'p_0_0i': np.zeros(3, dtype=float)
            }
        else:
            # Joint angle for the k-th link:
            # Note: k-(start_idx-1) in MATLAB => k-1 in Python if start_idx=2
            q_i = tmp_q[k - start_idx]  # expects tmp_q as shape (6,) or (6,1)
            
            # Reshape local_data[k]['R_base_j_i'] into a 3x3, then multiply by rotz(...) in degrees
            R_base = local_data[k]['R_base_j_i'].reshape((3,3)).transpose()
            R_j_i  = R_base @ rotz(q_i * 180.0 / np.pi)
            
            # Build up link[k] from link[k-1] data
            link[k] = {}
            link[k]['q_i']   = q_i
            link[k]['R_j_i'] = R_j_i
            link[k]['R_0_i'] = link[k-1]['R_0_i'] @ R_j_i
            link[k]['p_0_0i'] = (link[k-1]['p_0_0i'] +
                                 link[k-1]['R_0_i'] @ local_data[k]['p_j_ji'])

    # Now compute J_eff
    J_eff = np.zeros((6, 6), dtype=float)

    # final link for end-effector data
    p_0_0eff = link[end_idx]['p_0_0i']
    R_0_eff  = link[end_idx]['R_0_i']

    # Fill columns of J_eff
    for k in range(start_idx, end_idx + 1):
        R_i_0   = link[k]['R_0_i'].T     # Transpose
        R_i_eff = R_i_0 @ R_0_eff
        
        p_0_0i   = link[k]['p_0_0i']
        p_i_ieff = R_i_0 @ (p_0_0eff - p_0_0i)

        # Compute single-link Jacobian
        tmp_J_eff_i = calc_Jacobian_i_j(R_i_eff, p_i_ieff)

        # In MATLAB: J_eff(:,k-1) = tmp_J_eff_i(:,6),
        # meaning fill column (k-1) with the 6th column (1-based) => index 5 in Python.
        # For k from 2..7, (k-1) runs from 1..6 => we can shift to 0..5 in Python by using (k-2).
        J_eff[:, k - 2] = tmp_J_eff_i[:, 5]

    return J_eff

def move_line():
    count = 100
    log_ang = [[0]*6 for _ in range(count)]

    #x軸は正面左
    #y軸は下向き
    #z軸は正面方向
    
    dx = -1 # mm/s 正面右 danger 側 
    dy = -1 # mm/s  up
    dz = -1 # mm/s back

    #dwx = 1
    dwy = 30
    dwz = 30
    
    dq_old = np.zeros(6)
    
    for i in range(count):
        v_ref = np.array([0,0,0]) #mm/s
        w_ref = np.array([0,dwy,0]) #deg/s
        q = np.array(robot.GetJoints())*np.pi/180.0
        J = calc_eff_jacobian2(q)

        invJ = np.linalg.inv(J)
        dq_ref = np.dot(invJ, np.concatenate([v_ref,w_ref],0) )

        robot.MoveJointsVel(dq_ref[0],dq_ref[1],dq_ref[2],dq_ref[3],dq_ref[4],dq_ref[5])
        tmp_ang = robot.GetJoints()
        log_ang[i] = tmp_ang
        #print(dq_ref)
        time.sleep(dt)
    
    return log_ang


def motion_playback_task( x_dot ):

    #x軸は正面左
    #y軸は下向き
    #z軸は正面方向

    print("start play back")
    print(len(x_dot))
    log_ang = [[0]*6 for _ in range(len(x_dot))]

    start_time = time.time()
    for i in range(len(x_dot)-1):
        #set ref vel
        # x,y,zはmujocoの座標系準拠
        # 第一引数 omoto yに動く
        # 第二引数 omoto xに動く
        twist_ref = np.array([x_dot[i][0],x_dot[i][1],x_dot[i][2],x_dot[i][3],x_dot[i][4],x_dot[i][5]])
        
        q = np.dot( np.array(robot.GetJoints()), np.pi/180.0)
        J = calc_eff_jacobian2(q)
        
        invJ = np.linalg.inv(J)
        dq_ref = np.dot( np.dot(invJ, twist_ref ), 180.0/np.pi)

        robot.MoveJointsVel(dq_ref[0],dq_ref[1],dq_ref[2],dq_ref[3],dq_ref[4],dq_ref[5])
        
        tmp_ang = robot.GetJoints()
        cur_time = time.time()
        diff_time = (  cur_time-start_time ) % dt
        log_ang[i] = [cur_time, tmp_ang]
        time.sleep( dt-diff_time )
    
    print("end play back")
    return log_ang

def main():
    #initialize
    robot.Connect(address='192.168.0.100', enable_synchronous_mode=False) #!!!!

    robot.ActivateAndHome()

    activation_status = False
    start_time = time.time()

    while activation_status==False:

        tmp = robot.GetStatusRobot()
        if tmp.activation_state:
            activation_status = True
            print("Robot activated")
        if time.time() - start_time > ACTIVATION_TIMEOUT:
            print("Activation error")
            break

    print("move robot to the initial pose")
    #robot.MoveJoints(90,0,30,0,-30,0)
    #robot.MovePose(0,130,242,180,0,-180)
    robot.MoveJoints(0,0,0,0,0,0)
    time.sleep(5)
    robot.MovePose(0,130,242,180,0,-180)
    #robot.MovePose(0,-130,242,180,0,-90)

    tmp = robot.GetStatusRobot()
    print( tmp.can_move())
    time.sleep(5)
    tmp = robot.GetStatusRobot()
    print( tmp.can_move())

    time.sleep(5)
    print(robot.GetJoints())

    #with open(r'test_dx_task.csv') as f:
    #with open(r'motion1_w_eff_vel.csv') as f:
    
    #with open(r'motion1_non_expressive.csv') as f:
    #with open(r'motion2_non_expressive.csv') as f:

    #with open(r'./motions/motion_offline_ik_1_hw.csv') as f:
    with open(r'./motions/motion_offline_ik_3_hw_debug.csv') as f:
        reader = csv.reader(f)
        x_dot = [[float(v) for v in row] for row in reader]
    log_ang = motion_playback_task( x_dot )

    with open('log.csv', 'w', newline='', encoding='utf-8') as outfile:
        writer = csv.writer(outfile)
        for col in log_ang:
            writer.writerow(col)  # 列を1行ずつ書き込む

    #finalize
    robot.DeactivateRobot()
    start_time = time.time()

    while activation_status==True:

        tmp = robot.GetStatusRobot()
        if not( tmp.activation_state ):
            activation_status = False
            print("Robot deactivated")
        if time.time() - start_time > ACTIVATION_TIMEOUT:
            print("Deactivation error")
            break

    robot.Disconnect()

if __name__ == "__main__":
    main()
