#!/usr/bin/python
#
# Send joint values to UR5 using messages
#

from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import *
import rospy

waypoints = [[0.0, -1.44, 1.4, 0.6, 0, -0.33], [1,-0.5,0,0,0,0]]


def main():

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

    rate = rospy.Rate(1)
    cnt = 0
    pts = JointTrajectoryPoint()
    traj.header.stamp = rospy.Time.now()

    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()

    while not rospy.is_shutdown():
        cnt += 1

        if cnt%2 == 1:
            # pts.positions = waypoints[0]
            g.trajectory.points = [
                JointTrajectoryPoint(positions=Q1, velocities=[0]*6, time_from_start=rospy.Duration(2.0)),
                JointTrajectoryPoint(positions=Q2, velocities=[0]*6, time_from_start=rospy.Duration(3.0)),
                JointTrajectoryPoint(positions=Q3, velocities=[0]*6, time_from_start=rospy.Duration(4.0))]
        else:
            # pts.positions = waypoints[1]
            g.trajectory.points = [
                JointTrajectoryPoint(positions=Q1, velocities=[0]*6, time_from_start=rospy.Duration(2.0)),
                JointTrajectoryPoint(positions=Q2, velocities=[0]*6, time_from_start=rospy.Duration(3.0)),
                JointTrajectoryPoint(positions=Q3, velocities=[0]*6, time_from_start=rospy.Duration(4.0))]

        pts.time_from_start = rospy.Duration(1.0)

        # Set the points to the trajectory
        traj.points = []
        traj.points.append(pts)

        # Publish the message
        pub.publish(traj)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        print ("Program interrupted before completion")
