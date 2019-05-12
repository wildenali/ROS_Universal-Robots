#!/usr/bin/python
#
# Send joint values to UR5 using messages
#

from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory

from trajectory_msgs.msg import JointTrajectoryPoint
import rospy

from std_msgs.msg import String
import time

waypoints = [[0.0, -1.44, 1.4, 0.6, 0, -0.33], [0,0,0,0,0,0]]
# global int_x
pos_x = 0

def callback_terima_dari_bb_pub_py(msg):
    # rospy.loginfo("Message received: ")
    # rospy.loginfo(msg)
    # print(msg.data)
    global pos_x
    # int_x = float(msg.data)
    int_x = int(msg.data)
    pos_x = int_x
    # global int_x
    # print(pos_x)


def main():
    global pos_x

    rospy.init_node('send_joints')
    pub = rospy.Publisher('/arm_controller/command',
                          JointTrajectory,
                          queue_size=10)

    # Create the topic message
    traj = JointTrajectory()
    traj.header = Header()
    # Joint names for UR5
    traj.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint',
                        'elbow_joint', 'wrist_1_joint', 'wrist_2_joint',
                        'wrist_3_joint']

    # rate = rospy.Rate(1)
    rate = rospy.Rate(100)
    cnt = 0
    motor1 = 0
    pts = JointTrajectoryPoint()


    traj.header.stamp = rospy.Time.now()

    while not rospy.is_shutdown():

        if motor1 >= 6.3:
            motor1 = 6.3


        # maksimal pos_x = 640
        posisi_x = float(pos_x/100.0)
        print(posisi_x)
        # print(motor1)


        pts.positions = posisi_x, -1.44, 1.4, -3.0, -1.5, -0.33

        motor1 += 0.1

        pts.time_from_start = rospy.Duration(1.0)

        # Set the points to the trajectory
        traj.points = []
        traj.points.append(pts)
        # Publish the message
        pub.publish(traj)
        rate.sleep()

if __name__ == '__main__':
    try:
        hajarbleh = True
        while hajarbleh == True:
            pub = rospy.Subscriber("/bb_pub_py", String, callback_terima_dari_bb_pub_py)
            main()
            time.sleep(1)

    except rospy.ROSInterruptException:
        print ("Program interrupted before completion")




# roslaunch face_tracker_pkg start_tracking.launch
# roslaunch ur_gazebo ur5.launch
# ~/catkin_ws/src/ur5_ROS-Gazebo/old code$
    # python testmotion_ubah_01.py
