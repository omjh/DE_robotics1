#!/usr/bin/env python

import sys
import rospy
import rospkg

from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel,
)
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import (
    Header,
    Empty,
)

import baxter_interface


def load_UP(block_no, block_side, block_pose_left=Pose(position=Point(x=0.1, y= 0.7, z=0.74)),
                       block_reference_frame="world",
                       block_pose_right=Pose(position=Point(x=0.1, y= -0.7, z=0.74))):
    # Get Models' Path
    model_path = rospkg.RosPack().get_path('baxter_sim_examples')+"/models/" 
    block_xml = ''
    with open (model_path + "Upright/object.urdf", "r") as block_file:
          block_xml = block_file.read().replace('\n', '')

    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    #Spawn bricks on left side
    if block_side == 'l':
        try:
            spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
            resp_urdf = spawn_urdf(block_no, block_xml, "/",
                                    block_pose_left, block_reference_frame)
        except rospy.ServiceException, e:
            rospy.logerr("Spawn SDF service call failed: {0}".format(e))

    elif block_side == 'r':
        try:
            spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
            resp_urdf = spawn_urdf(block_no, block_xml, "/",
                                   block_pose_right, block_reference_frame)
        except rospy.ServiceException, e:
            rospy.logerr("Spawn SDF service call failed: {0}".format(e))
    else:
      print('You dingus, thats not a side!')


def main():

  rospy.init_node("spawm_sim")
  
  brick_no = raw_input('Brick no =')
  block_side = raw_input('Brick side =')


if __name__ == '__main__':
  sys.exit(main())


