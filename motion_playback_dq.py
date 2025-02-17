#2025.02.17 kenya mori

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

def motion_playback( q_dot ):
    log_ang = [[0]*6 for _ in range(len(q_dot))]
    for i in range(len(q_dot)-1):
        robot.MoveJointsVel(q_dot[i][0],q_dot[i][1],q_dot[i][2],q_dot[i][3],q_dot[i][4],q_dot[i][5])
        tmp_ang = robot.GetJoints()
        log_ang[i] = tmp_ang
        #print(tmp_ang)
        time.sleep(dt)
    
    return log_ang

def main():
    #initialize
    robot.Connect(address='192.168.0.100', enable_synchronous_mode=False) #!!!!

    robot.ActivateAndHome()
    robot.SetVelTimeout(0.2)
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
    #robot.MoveJoints(0,0,0,0,0,0)
    #time.sleep(5)
    robot.MovePose(0,130,242,180,0,-180) # for q_dot_pahigh_up (P1 to P2)
    #robot.MovePose(0,-130,242,180,0,-90) # for q_dot_rehigh_up (P3 to P1)
    tmp = robot.GetStatusRobot()
    print( tmp.can_move())
    time.sleep(5)
    tmp = robot.GetStatusRobot()
    print( tmp.can_move())

    time.sleep(5)

    with open(r'./motions/motion_offline_ik_3_dq.csv') as f:
        reader = csv.reader(f)
        q_dot = [[float(v) for v in row] for row in reader]
    log_ang = motion_playback( np.dot(q_dot, 180.0/np.pi) )

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
