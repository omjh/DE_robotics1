#!/usr/bin/env python

# Copyright (c) 2013-2015, Rethink Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the Rethink Robotics nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
#!/usr/bin/env python

# Copyright (c) 2013-2015, Rethink Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the Rethink Robotics nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
Baxter RSDK Inverse Kinematics Pick and Place Demo
"""
import argparse
import struct
import sys
import copy

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

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

import baxter_interface


def load_UP1(table_pose=Pose(position=Point(x=0.5, y=-0.6, z=-0.73)),
                       table_reference_frame="world",
                       block_pose=Pose(position=Point(x=0.1, y= 0.7, z=0.74)),
                       block_reference_frame="world"):
    # Get Models' Path
    model_path = rospkg.RosPack().get_path('baxter_sim_examples')+"/models/" 
    block_xml = ''
    with open (model_path + "Upright/object.urdf", "r") as block_file:
        block_xml = block_file.read().replace('\n', '')
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_urdf2 = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        resp_urdf2 = spawn_urdf2("UP1", block_xml, "/",
                               block_pose, block_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))

def load_UP2(table_pose=Pose(position=Point(x=0.5, y=-0.6, z=-0.73)),
                       table_reference_frame="world",
                       block_pose=Pose(position=Point(x=0.1, y=-0.7, z=0.74)),
                       block_reference_frame="world"):
    # Get Models' Path
    model_path = rospkg.RosPack().get_path('baxter_sim_examples')+"/models/"
    block_xml = ''
    with open (model_path + "Upright/object.urdf", "r") as block_file:
        block_xml = block_file.read().replace('\n', '')
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_urdf2 = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        resp_urdf2 = spawn_urdf2("UP2", block_xml, "/",
                               block_pose, block_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))

def load_UP3(table_pose=Pose(position=Point(x=0.5, y=-0.6, z=-0.73)),
                       table_reference_frame="world",
                       block_pose=Pose(position=Point(x=0.1, y=0.7, z=0.74)),
                       block_reference_frame="world"):
    # Get Models' Path
    model_path = rospkg.RosPack().get_path('baxter_sim_examples')+"/models/"
    block_xml = ''
    with open (model_path + "Upright/object.urdf", "r") as block_file:
        block_xml = block_file.read().replace('\n', '')
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_urdf2 = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        resp_urdf2 = spawn_urdf2("UP3", block_xml, "/",
                               block_pose, block_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))

def load_UP4(table_pose=Pose(position=Point(x=0.5, y=-0.6, z=-0.73)),
                       table_reference_frame="world",
                       block_pose=Pose(position=Point(x=0.1, y=-0.70, z=0.74)),
                       block_reference_frame="world"):
    # Get Models' Path
    model_path = rospkg.RosPack().get_path('baxter_sim_examples')+"/models/"
    block_xml = ''
    with open (model_path + "Upright/object.urdf", "r") as block_file:
        block_xml = block_file.read().replace('\n', '')
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_urdf2 = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        resp_urdf2 = spawn_urdf2("UP4", block_xml, "/",
                               block_pose, block_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))

def load_UP5(table_pose=Pose(position=Point(x=0.5, y=-0.6, z=-0.73)),
                       table_reference_frame="world",
                       block_pose=Pose(position=Point(x=0.1, y=0.7, z=0.74)),
                       block_reference_frame="world"):
    # Get Models' Path
    model_path = rospkg.RosPack().get_path('baxter_sim_examples')+"/models/"
    block_xml = ''
    with open (model_path + "Upright/object.urdf", "r") as block_file:
        block_xml = block_file.read().replace('\n', '')
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_urdf2 = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        resp_urdf2 = spawn_urdf2("UP5", block_xml, "/",
                               block_pose, block_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))

def load_UP6(table_pose=Pose(position=Point(x=0.5, y=-0.6, z=-0.73)),
                       table_reference_frame="world",
                       block_pose=Pose(position=Point(x=0.1, y=-0.7, z=0.74)),
                       block_reference_frame="world"):
    # Get Models' Path
    model_path = rospkg.RosPack().get_path('baxter_sim_examples')+"/models/"
    block_xml = ''
    with open (model_path + "Upright/object.urdf", "r") as block_file:
        block_xml = block_file.read().replace('\n', '')
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_urdf2 = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        resp_urdf2 = spawn_urdf2("UP6", block_xml, "/",
                               block_pose, block_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))

def load_UP7(table_pose=Pose(position=Point(x=0.5, y=-0.6, z=-0.73)),
                       table_reference_frame="world",
                       block_pose=Pose(position=Point(x=0.1, y=0.7, z=0.74)),
                       block_reference_frame="world"):
    # Get Models' Path
    model_path = rospkg.RosPack().get_path('baxter_sim_examples')+"/models/"
    block_xml = ''
    with open (model_path + "Upright/object.urdf", "r") as block_file:
        block_xml = block_file.read().replace('\n', '')
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_urdf2 = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        resp_urdf2 = spawn_urdf2("UP7", block_xml, "/",
                               block_pose, block_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))

def load_UP8(table_pose=Pose(position=Point(x=0.5, y=-0.6, z=-0.73)),
                       table_reference_frame="world",
                       block_pose=Pose(position=Point(x=0.1, y=-0.7, z=0.74)),
                       block_reference_frame="world"):
    # Get Models' Path
    model_path = rospkg.RosPack().get_path('baxter_sim_examples')+"/models/"
    block_xml = ''
    with open (model_path + "Upright/object.urdf", "r") as block_file:
        block_xml = block_file.read().replace('\n', '')
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_urdf2 = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        resp_urdf2 = spawn_urdf2("UP8", block_xml, "/",
                               block_pose, block_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))

def load_UP9(table_pose=Pose(position=Point(x=0.5, y=-0.6, z=-0.73)),
                       table_reference_frame="world",
                       block_pose=Pose(position=Point(x=0.1, y=0.7, z=0.74)),
                       block_reference_frame="world"):
    # Get Models' Path
    model_path = rospkg.RosPack().get_path('baxter_sim_examples')+"/models/"
    block_xml = ''
    with open (model_path + "Upright/object.urdf", "r") as block_file:
        block_xml = block_file.read().replace('\n', '')
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_urdf2 = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        resp_urdf2 = spawn_urdf2("UP9", block_xml, "/",
                               block_pose, block_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))

def load_UP10(table_pose=Pose(position=Point(x=0.5, y=-0.6, z=-0.73)),
                       table_reference_frame="world",
                       block_pose=Pose(position=Point(x=0.1, y=-0.7, z=0.74)),
                       block_reference_frame="world"):
    # Get Models' Path
    model_path = rospkg.RosPack().get_path('baxter_sim_examples')+"/models/"
    block_xml = ''
    with open (model_path + "Upright/object.urdf", "r") as block_file:
        block_xml = block_file.read().replace('\n', '')
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_urdf2 = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        resp_urdf2 = spawn_urdf2("UP10", block_xml, "/",
                               block_pose, block_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))

def load_UP11(table_pose=Pose(position=Point(x=0.5, y=-0.6, z=-0.73)),
                       table_reference_frame="world",
                       block_pose=Pose(position=Point(x=0.1, y=0.7, z=0.74)),
                       block_reference_frame="world"):
    # Get Models' Path
    model_path = rospkg.RosPack().get_path('baxter_sim_examples')+"/models/"
    block_xml = ''
    with open (model_path + "Upright/object.urdf", "r") as block_file:
        block_xml = block_file.read().replace('\n', '')
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_urdf2 = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        resp_urdf2 = spawn_urdf2("UP11", block_xml, "/",
                               block_pose, block_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))

def load_UP12(table_pose=Pose(position=Point(x=0.5, y=-0.6, z=-0.73)),
                       table_reference_frame="world",
                       block_pose=Pose(position=Point(x=0.1, y=-0.7, z=0.74)),
                       block_reference_frame="world"):
    # Get Models' Path
    model_path = rospkg.RosPack().get_path('baxter_sim_examples')+"/models/"
    block_xml = ''
    with open (model_path + "Upright/object.urdf", "r") as block_file:
        block_xml = block_file.read().replace('\n', '')
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_urdf2 = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        resp_urdf2 = spawn_urdf2("UP12", block_xml, "/",
                               block_pose, block_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))

def load_UP13(table_pose=Pose(position=Point(x=0.5, y=-0.6, z=-0.73)),
                       table_reference_frame="world",
                       block_pose=Pose(position=Point(x=0.1, y=0.7, z=0.74)),
                       block_reference_frame="world"):
    # Get Models' Path
    model_path = rospkg.RosPack().get_path('baxter_sim_examples')+"/models/"
    block_xml = ''
    with open (model_path + "Upright/object.urdf", "r") as block_file:
        block_xml = block_file.read().replace('\n', '')
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_urdf2 = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        resp_urdf2 = spawn_urdf2("UP13", block_xml, "/",
                               block_pose, block_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))

def load_UP14(table_pose=Pose(position=Point(x=0.5, y=-0.6, z=-0.73)),
                       table_reference_frame="world",
                       block_pose=Pose(position=Point(x=0.1, y=-0.7, z=0.74)),
                       block_reference_frame="world"):
    # Get Models' Path
    model_path = rospkg.RosPack().get_path('baxter_sim_examples')+"/models/"
    block_xml = ''
    with open (model_path + "Upright/object.urdf", "r") as block_file:
        block_xml = block_file.read().replace('\n', '')
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_urdf2 = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        resp_urdf2 = spawn_urdf2("UP5", block_xml, "/",
                               block_pose, block_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))

def load_UP15(table_pose=Pose(position=Point(x=0.5, y=-0.6, z=-0.73)),
                       table_reference_frame="world",
                       block_pose=Pose(position=Point(x=0.1, y=0.7, z=0.74)),
                       block_reference_frame="world"):
    # Get Models' Path
    model_path = rospkg.RosPack().get_path('baxter_sim_examples')+"/models/"
    block_xml = ''
    with open (model_path + "Upright/object.urdf", "r") as block_file:
        block_xml = block_file.read().replace('\n', '')
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_urdf2 = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        resp_urdf2 = spawn_urdf2("UP15", block_xml, "/",
                               block_pose, block_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))


def delete_gazebo_models():
    # This will be called on ROS Exit, deleting Gazebo models
    # Do not wait for the Gazebo Delete Model service, since
    # Gazebo should already be running. If the service is not
    # available since Gazebo has been killed, it is fine to error out
    try:
        delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        resp_delete = delete_model("UP1")
        resp_delete = delete_model("UP2")
    except rospy.ServiceException, e:
        rospy.loginfo("Delete Model service call failed: {0}".format(e))

def main():

    rospy.init_node("spawm_sim")

    brickno = input('Brick no =')
    if brickno == 1:
        load_UP1()
    if brickno == 2:
        load_UP2()
    if brickno == 3:
        load_UP3()
    if brickno == 4:
        load_UP4()
    if brickno == 5:
        load_UP5()
    if brickno == 6:
        load_UP6()
    if brickno == 7:
        load_UP7()
    if brickno == 8:
        load_UP8()
    if brickno == 9:
        load_UP9()
    if brickno == 10:
        load_UP10()
    if brickno == 11:
        load_UP11()
    if brickno == 12:
        load_UP12()
    if brickno == 13:
        load_UP13()
    if brickno == 14:
        load_UP14()
    if brickno == 15:
        load_UP15()
    


if __name__ == '__main__':
    sys.exit(main())


