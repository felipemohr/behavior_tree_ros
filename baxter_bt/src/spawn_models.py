#!/usr/bin/env python

import rospy
import rospkg
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose, Point, Quaternion


def main():

  rospy.init_node('spawn_models')
  
  rpkg = rospkg.RosPack()
  
  table_pose = Pose()
  table_pose.position = Point(x=1.0, y=0.0, z=0.0)
  table_pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

  block_pose = Pose()
  block_pose.position = Point(x=0.6725, y=0.1265, z=0.7825)
  block_pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=0.0)


  try:
    spawn_table_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    spawn_table_client(
      model_name='table',
      model_xml=open(rpkg.get_path('baxter_sim_examples')+'/models/cafe_table/model.sdf', 'r').read(),
      robot_namespace='/',
      initial_pose=table_pose,
      reference_frame='world'
    )
  except rospy.ServiceException as e:
    print("Table Service call failed: ",e)

  try:
    spawn_block_client = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
    spawn_block_client(
      model_name='block',
      model_xml=open(rpkg.get_path('baxter_sim_examples')+'/models/block/model.urdf', 'r').read(),
      robot_namespace='/',
      initial_pose=block_pose,
      reference_frame='world'
    )
  except rospy.ServiceException as e:
    print("Block Service call failed: ",e)



if __name__ == "__main__":
  main()
