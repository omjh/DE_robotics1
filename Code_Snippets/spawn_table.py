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


def load_tables(table_pose1=Pose(position=Point(x=1.1, y=0, z=0.73)),
                       table_reference_frame1="world",
                       table_pose2=Pose(position=Point(x=0.6, y=1.1, z=0.73)),
                       table_reference_frame2="world",
                       table_pose3=Pose(position=Point(x=0.6, y=-1.1, z=0.73)),
                       table_reference_frame3="world"):
    #Get models from directory
    model_path = rospkg.RosPack().get_path('baxter_sim_examples')+"/models/"
    #Load 1
    table1_xml = ''
    with open (model_path + "real_table/object.urdf", "r") as table_file:
        table1_xml=table_file.read().replace('\n', '')
    #Load 2
    table2_xml = ''
    with open (model_path + "real_table2/object.urdf", "r") as table_file:
        table2_xml=table_file.read().replace('\n', '')
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    #Load 3
    table3_xml = ''
    with open (model_path + "real_table2/object.urdf", "r") as table_file:
        table3_xml=table_file.read().replace('\n', '')
    rospy.wait_for_service('/gazebo/spawn_urdf_model')

    #Spawn 1
    try:
        spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        resp_urdf1 = spawn_urdf("table1", table1_xml, "/",
                             table_pose1, table_reference_frame1)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))

    #Spawn 2
    try:
        spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        resp_urdf2 = spawn_urdf("table2", table2_xml, "/",
                             table_pose2, table_reference_frame2)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))

    #Spawn 3
    try:
        spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        resp_urdf3 = spawn_urdf("table3", table3_xml, "/",
                             table_pose3, table_reference_frame3)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))



def main():

    rospy.init_node("ik_pick_and_place_demo")
    # Load Gazebo Models via Spawning Services
    # Note that the models reference is the /world frame
    load_tables()


if __name__ == '__main__':
    sys.exit(main())


