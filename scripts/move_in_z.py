#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
This script moves the ROBOTIS Open Manipulator 6DOF robot using topic publishers.
"""

import rospy
import sys
import time
from geometry_msgs.msg import Pose
from open_manipulator_msgs.msg import KinematicsPose, OpenManipulatorState, JointPosition
from std_msgs.msg import Float64, String

class OpenManipulatorTopicControl:
    def __init__(self):
        rospy.init_node('open_manipulator_topic_control', anonymous=True)
        
        # Set the namespace for the 6DOF robot
        self.ns = '/open_manipulator_6dof'
        
        # Create publishers for controlling the robot
        self.joint_position_pub = rospy.Publisher(
            self.ns + '/goal_joint_position',
            JointPosition,
            queue_size=5
        )
        
        self.kinematics_pose_pub = rospy.Publisher(
            self.ns + '/goal_kinematics_pose',
            KinematicsPose,
            queue_size=5
        )
        
        # Subscribe to robot state
        self.robot_state = "STOPPED"
        rospy.Subscriber(self.ns + '/states', OpenManipulatorState, self.robot_state_callback)
        
        # Subscribe to current end-effector pose
        self.current_pose = None
        rospy.Subscriber(self.ns + '/gripper/kinematics_pose', KinematicsPose, self.kinematics_pose_callback)
        
        # Wait for current pose data to be available
        timeout = 5.0
        start_time = time.time()
        while self.current_pose is None and not rospy.is_shutdown():
            if time.time() - start_time > timeout:
                rospy.logerr("Timed out waiting for current pose")
                sys.exit(1)
            rospy.sleep(0.1)
        
        rospy.loginfo("Robot is ready!")
        
    def robot_state_callback(self, msg):
        # Remove quotes if they exist in the state string
        self.robot_state = msg.open_manipulator_moving_state.strip('"')
        
    def kinematics_pose_callback(self, msg):
        self.current_pose = msg
        
    def wait_for_completion(self):
        rospy.loginfo("Waiting for movement to complete...")
        rate = rospy.Rate(10)  # 10 Hz
        while self.robot_state == "MOVING" and not rospy.is_shutdown():
            rate.sleep()
        rospy.loginfo("Movement completed.")
    
    def move_z_direction(self, z_offset, time_seconds=2.0):
        """
        Move the end-effector in Z direction by the specified offset.
        
        Args:
            z_offset: Amount to move in Z direction (meters)
            time_seconds: Time to complete the movement (seconds)
        """
        # First, get and display the current position
        current_x = self.current_pose.pose.position.x
        current_y = self.current_pose.pose.position.y
        current_z = self.current_pose.pose.position.z
        
        rospy.loginfo("Current position: x=%f, y=%f, z=%f" % (current_x, current_y, current_z))
        rospy.loginfo("Moving %.3f meters in Z direction" % z_offset)
        
        # Calculate the target position
        target_z = current_z + z_offset
        
        # Create a KinematicsPose message
        kinematics_pose = KinematicsPose()
        
        # Maintain current XY position, change only Z
        kinematics_pose.pose.position.x = current_x
        kinematics_pose.pose.position.y = current_y
        kinematics_pose.pose.position.z = target_z
        
        # Maintain current orientation
        kinematics_pose.pose.orientation.x = self.current_pose.pose.orientation.x
        kinematics_pose.pose.orientation.y = self.current_pose.pose.orientation.y
        kinematics_pose.pose.orientation.z = self.current_pose.pose.orientation.z
        kinematics_pose.pose.orientation.w = self.current_pose.pose.orientation.w
        
        # Set the movement time
        kinematics_pose.path_time = time_seconds
        
        # Send the message to the topic
        rospy.loginfo("Publishing movement command to z=%.3f" % target_z)
        self.kinematics_pose_pub.publish(kinematics_pose)
        
        # Wait for movement to complete
        self.wait_for_completion()
        
        # Get the new position and display it
        new_z = self.current_pose.pose.position.z
        rospy.loginfo("Movement complete. New z=%.3f (target was %.3f)" % (new_z, target_z))
    
    def move_line(self, target_x, target_y, target_z, time_seconds=2.0):
        """
        Move the end-effector in a straight line to the target position.
        
        Args:
            target_x: Target X position (meters)
            target_y: Target Y position (meters)
            target_z: Target Z position (meters)
            time_seconds: Time to complete the movement (seconds)
        """
        # Display the current and target positions
        current_x = self.current_pose.pose.position.x
        current_y = self.current_pose.pose.position.y
        current_z = self.current_pose.pose.position.z
        
        rospy.loginfo("Current position: x=%.3f, y=%.3f, z=%.3f" % (current_x, current_y, current_z))
        rospy.loginfo("Moving to: x=%.3f, y=%.3f, z=%.3f" % (target_x, target_y, target_z))
        
        # Create a KinematicsPose message
        kinematics_pose = KinematicsPose()
        
        # Set the target position
        kinematics_pose.pose.position.x = target_x
        kinematics_pose.pose.position.y = target_y
        kinematics_pose.pose.position.z = target_z
        
        # Maintain current orientation
        kinematics_pose.pose.orientation.x = self.current_pose.pose.orientation.x
        kinematics_pose.pose.orientation.y = self.current_pose.pose.orientation.y
        kinematics_pose.pose.orientation.z = self.current_pose.pose.orientation.z
        kinematics_pose.pose.orientation.w = self.current_pose.pose.orientation.w
        
        # Set the movement time
        kinematics_pose.path_time = time_seconds
        
        # Send the message to the topic
        rospy.loginfo("Publishing movement command")
        self.kinematics_pose_pub.publish(kinematics_pose)
        
        # Wait for movement to complete
        self.wait_for_completion()
        
        # Get the new position and display it
        new_x = self.current_pose.pose.position.x
        new_y = self.current_pose.pose.position.y
        new_z = self.current_pose.pose.position.z
        rospy.loginfo("Movement complete. New position: x=%.3f, y=%.3f, z=%.3f" % (new_x, new_y, new_z))

    def run_z_demo(self):
        """Run a demonstration of Z direction movement"""
        # Move up 5cm
        self.move_z_direction(0.05)
        rospy.sleep(1.0)  # Pause
        
        # Move down 5cm (back to starting position)
        self.move_z_direction(-0.05)
        rospy.sleep(1.0)  # Pause
    
    def run_line_demo(self):
        """Run a demonstration of line movement"""
        # Get current position
        start_x = self.current_pose.pose.position.x
        start_y = self.current_pose.pose.position.y
        start_z = self.current_pose.pose.position.z
        
        # Move forward 10cm in X direction
        self.move_line(start_x + 0.1, start_y, start_z)
        rospy.sleep(1.0)  # Pause
        
        # Move back to start
        self.move_line(start_x, start_y, start_z)
        rospy.sleep(1.0)  # Pause
        
        # Move 10cm in Y direction
        self.move_line(start_x, start_y + 0.1, start_z)
        rospy.sleep(1.0)  # Pause
        
        # Move back to start
        self.move_line(start_x, start_y, start_z)

if __name__ == '__main__':
    try:
        robot = OpenManipulatorTopicControl()
        
        # Choose which demo to run
        # robot.run_z_demo()  # Up and down movement
        robot.run_line_demo()  # Line movement in X and Y directions
        
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr("Error in topic control: %s" % str(e))
        sys.exit(1)