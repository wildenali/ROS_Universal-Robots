#!/usr/bin/python
#
# Send joint values to UR5 using messages
#

from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory

from trajectory_msgs.msg import JointTrajectoryPoint
import rospy

waypoints = [[0.0, -1.44, 1.4, 0.6, 0, -0.33], [1,0,0,0,0,0]]

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

    while not rospy.is_shutdown():
        cnt += 1

        if cnt%2 == 1:
            pts.positions = waypoints[0]
            # pts.position.x = 0.1052
            # pts.position.y = -0.4271
            # pts.position.z = 0.4005
            #
            # pts.orientation.x = 0.4811
            # pts.orientation.y = 0.4994
            # pts.orientation.z = -0.5121
    	    # pts.orientation.w = 0.5069
        else:
            pts.positions = waypoints[1]
            # pts.position.x = 0.0
            # pts.position.y = 0.0
            # pts.position.z = 0.0
            #
            # pts.orientation.x = 0.0
            # pts.orientation.y = 0.0
            # pts.orientation.z = 0.0
    	    # pts.orientation.w = 0.0

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
