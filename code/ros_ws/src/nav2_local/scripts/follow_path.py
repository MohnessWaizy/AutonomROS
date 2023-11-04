#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Modified by AutomaticAddison.com
 
import time # Time library
 
from geometry_msgs.msg import PoseStamped # Pose with ref frame and timestamp
from rclpy.duration import Duration # Handles time for ROS 2
import rclpy # Python client library for ROS 2
 
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult # Helper module 
 
'''
Navigates a robot through goal poses.
'''

 

def main():
 

  # Set the robot's initial pose if necessary
  initial_pose = PoseStamped()
  initial_pose.header.frame_id = 'map'
  initial_pose.header.stamp = navigator.get_clock().now().to_msg()
  initial_pose.pose.position.x = -2.935292790366883
  initial_pose.pose.position.y = -4.508613183463758
  initial_pose.pose.position.z = 0.0
  initial_pose.pose.orientation.x = 0.0
  initial_pose.pose.orientation.y = 0.00016057199885296652
  initial_pose.pose.orientation.z = 0.01000638106784498
  initial_pose.pose.orientation.w = 0.9999494881283788
  navigator.setInitialPose(initial_pose)
 
  # Activate navigation, if not autostarted. This should be called after setInitialPose()
  # or this will initialize at the origin of the map and update the costmap with bogus readings.
  # If autostart, you should `waitUntilNav2Active()` instead.
  # navigator.lifecycleStartup()
 
  # Wait for navigation to fully activate. Use this line if autostart is set to true.
  # navigator.waitUntilNav2Active(localizer=None)
  time.sleep(5)
  # If desired, you can change or load the map as well
  navigator.changeMap('./src/nav2_local/maps/simplestreet3.yaml')
 
  # You may use the navigator to clear or obtain costmaps
  navigator.clearAllCostmaps()  # also have clearLocalCostmap() and clearGlobalCostmap()
  global_costmap = navigator.getGlobalCostmap()
  local_costmap = navigator.getLocalCostmap()
 
  # Set the robot's goal poses
  goal_poses = []
   
   
  goal_pose = PoseStamped()
  goal_pose.header.frame_id = 'map'
  goal_pose.header.stamp = navigator.get_clock().now().to_msg()
  goal_pose.pose.position.x = 0.4409140658886807
  goal_pose.pose.position.y = -1.5860494973772004
  goal_pose.pose.position.z = 0.0
  goal_pose.pose.orientation.x = -0.00011284051734596846
  goal_pose.pose.orientation.y = 0.00011431662877236697
  goal_pose.pose.orientation.z = 0.711830795468663
  goal_pose.pose.orientation.w = 0.702319817956686
  goal_poses.append(goal_pose)
   
  goal_pose1 = PoseStamped()
  goal_pose1.header.frame_id = 'map'
  goal_pose1.header.stamp = navigator.get_clock().now().to_msg()
  goal_pose1.pose.position.x = -2.7524988606411136
  goal_pose1.pose.position.y = 0.3711580252843079
  goal_pose1.pose.position.z = 0.0
  goal_pose1.pose.orientation.x = -0.0001470617389479547
  goal_pose1.pose.orientation.y = 0.0
  goal_pose1.pose.orientation.z = 0.9998809787654733
  goal_pose1.pose.orientation.w = -0.015427464935684605
  goal_poses.append(goal_pose1)

  goal_pose6 = PoseStamped()
  goal_pose6.header.frame_id = 'map'
  goal_pose6.header.stamp = navigator.get_clock().now().to_msg()
  goal_pose6.pose.position.x = -3.8046968885309482
  goal_pose6.pose.position.y = 2.9090082939744266
  goal_pose6.pose.position.z = 0.0
  goal_pose6.pose.orientation.x = -0.00011431183048623226
  goal_pose6.pose.orientation.y = 0.00011766081958371351
  goal_pose6.pose.orientation.z = 0.6962265329920684
  goal_pose6.pose.orientation.w = 0.7178221143476852
  goal_poses.append(goal_pose6)


  
  goal_pose2 = PoseStamped()
  goal_pose2.header.frame_id = 'map'
  goal_pose2.header.stamp = navigator.get_clock().now().to_msg()
  goal_pose2.pose.position.x = 1.4884651277322913
  goal_pose2.pose.position.y = 3.826961965477066
  goal_pose2.pose.position.z = 0.0
  goal_pose2.pose.orientation.x = 0.0
  goal_pose2.pose.orientation.y = 0.00014669269831041787
  goal_pose2.pose.orientation.z = -0.01702023319031068
  goal_pose2.pose.orientation.w = 0.9998551345710741
  goal_poses.append(goal_pose2)
  
    
  goal_pose3 = PoseStamped()
  goal_pose3.header.frame_id = 'map'
  goal_pose3.header.stamp = navigator.get_clock().now().to_msg()
  goal_pose3.pose.position.x = 0.374867291951793
  goal_pose3.pose.position.y = 2.3742998302282925
  goal_pose3.pose.position.z = 0.0
  goal_pose3.pose.orientation.x = 0.0
  goal_pose3.pose.orientation.y = 0.00011120789820033709
  goal_pose3.pose.orientation.z = 0.683384130322041
  goal_pose3.pose.orientation.w = 0.7300589764282748
  goal_poses.append(goal_pose3)


  goal_pose4 = PoseStamped()
  goal_pose4.header.frame_id = 'map'
  goal_pose4.header.stamp = navigator.get_clock().now().to_msg()
  goal_pose4.pose.position.x = -3.161049131476712
  goal_pose4.pose.position.y = 4.410652816192905
  goal_pose4.pose.position.z = 0.0
  goal_pose4.pose.orientation.x = -0.00014569786535915873
  goal_pose4.pose.orientation.y = 0.00011120789820033709
  goal_pose4.pose.orientation.z = 0.9999999118611629
  goal_pose4.pose.orientation.w = -0.0004104539996143256
  goal_poses.append(goal_pose4)

  goal_pose5 = PoseStamped()
  goal_pose5 = initial_pose
 
  
   
  goal_poses.append(goal_pose5)
  

  

  



#   goal_pose3 = PoseStamped()
#   goal_pose3 = initial_pose
 
  
   
#   goal_poses.append(goal_pose3)
  
 
#   # sanity check a valid path exists
#   # path = navigator.getPathThroughPoses(initial_pose, goal_poses)
 
#   # Go through the goal poses
#   # navigator.goThroughPoses(goal_poses, behavior_tree='./src/nav2_local/behaviour_tree/nav_through_pose.xml')
#   #navigator.goThroughPoses(goal_poses, behavior_tree='./src/nav2_local/behavior_tree/drive_to_pose_bt.xml')
#   navigator.goToPose(goal_poses[0])
  
  
#   #navigator.followWaypoints(goal_poses)
  
  navigator.followWaypoints(goal_poses)

 
  i = 0
  # Keep doing stuff as long as the robot is moving towards the goal poses
  while not navigator.isTaskComplete():
    ################################################
    #
    # Implement some code here for your application!
    #
    ################################################
 
    # Do something with the feedback
    i = i + 1
    feedback = navigator.getFeedback()
    # print(feedback)
    if feedback and i % 7 == 0:
        print('Executing current waypoint: ' + str(feedback.current_waypoint + 1) + '/' + str(len(goal_poses)))
        # print("Executing one waypoint")

        


  # Do something depending on the return code
  result = navigator.getResult()
  if result == TaskResult.SUCCEEDED:
      print('Goal succeeded!')
  elif result == TaskResult.CANCELED:
      print('Goal was canceled!')
  elif result == TaskResult.FAILED:
      print('Goal failed!')
  else:
      print('Goal has an invalid return status!')
      
  # Close the ROS 2 Navigation Stack
  
  
  
 
if __name__ == '__main__':
    # Start the ROS 2 Python Client Library
    rclpy.init()
    global navigator
    navigator = BasicNavigator()
    
    try:
        while True:
            main()     
    except:
        navigator.lifecycleShutdown()
        exit(0)
 
        
            

   