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

class PickAndPlace(object):
    def __init__(self, limb, hover_distance = 0.15, verbose=True):
        self._limb_name = limb # string
        self._hover_distance = hover_distance # in meters
        self._verbose = verbose # bool
        self._limb = baxter_interface.Limb(limb)
        self._gripper = baxter_interface.Gripper(limb)
        ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
        self._iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
        rospy.wait_for_service(ns, 5.0)
        # verify robot is enabled
        print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()

    def move_to_start(self, start_angles=None):
        print("Moving the {0} arm to start pose...".format(self._limb_name))
        if not start_angles:
            start_angles = dict(zip(self._joint_names, [0]*7))
        self._guarded_move_to_joint_position(start_angles)
        self.gripper_open()
        rospy.sleep(1.0)
        print("Running. Ctrl-c to quit")

    def ik_request(self, pose):
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        ikreq = SolvePositionIKRequest()
        ikreq.pose_stamp.append(PoseStamped(header=hdr, pose=pose))
        try:
            resp = self._iksvc(ikreq)
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call failed: %s" % (e,))
            return False
        # Check if result valid, and type of seed ultimately used to get solution
        # convert rospy's string representation of uint8[]'s to int's
        resp_seeds = struct.unpack('<%dB' % len(resp.result_type), resp.result_type)
        limb_joints = {}
        if (resp_seeds[0] != resp.RESULT_INVALID):
            seed_str = {
                        ikreq.SEED_USER: 'User Provided Seed',
                        ikreq.SEED_CURRENT: 'Current Joint Angles',
                        ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                       }.get(resp_seeds[0], 'None')
            if self._verbose:
                print("IK Solution SUCCESS - Valid Joint Solution Found from Seed Type: {0}".format(
                         (seed_str)))
            # Format solution into Limb API-compatible dictionary
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
            if self._verbose:
                print("IK Joint Solution:\n{0}".format(limb_joints))
                print("------------------")
        else:
            rospy.logerr("INVALID POSE - No Valid Joint Solution Found.")
            return False
        return limb_joints

    def _guarded_move_to_joint_position(self, joint_angles):
        if joint_angles:
            self._limb.move_to_joint_positions(joint_angles)
        else:
            rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")

    def gripper_open(self):
        self._gripper.open()
        rospy.sleep(1.0)

    def gripper_close(self):
        self._gripper.close()
        rospy.sleep(1.0)

    def _approach(self, pose):
        approach = copy.deepcopy(pose)
        # approach with a pose the hover-distance above the requested pose
        approach.position.z = approach.position.z + self._hover_distance
        joint_angles = self.ik_request(approach)
        self._guarded_move_to_joint_position(joint_angles)

    def _retract(self):
        # retrieve current pose from endpoint
        current_pose = self._limb.endpoint_pose()
        ik_pose = Pose()
        ik_pose.position.x = current_pose['position'].x
        ik_pose.position.y = current_pose['position'].y
        ik_pose.position.z = current_pose['position'].z + self._hover_distance
        ik_pose.orientation.x = current_pose['orientation'].x
        ik_pose.orientation.y = current_pose['orientation'].y
        ik_pose.orientation.z = current_pose['orientation'].z
        ik_pose.orientation.w = current_pose['orientation'].w
        joint_angles = self.ik_request(ik_pose)
        # servo up from current pose
        self._guarded_move_to_joint_position(joint_angles)

    def _servo_to_pose(self, pose):
        # servo down to release
        joint_angles = self.ik_request(pose)
        self._guarded_move_to_joint_position(joint_angles)

    def pick(self, pose):
        # open the gripper
        self.gripper_open()
        # servo above pose
        self._approach(pose)
        # servo to pose
        self._servo_to_pose(pose)
        # close gripper
        self.gripper_close()
        # retract to clear object
        self._retract()

    def place(self, pose):
        # servo above pose
        self._approach(pose)
        # servo to pose
        self._servo_to_pose(pose)
        # open the gripper
        self.gripper_open()
        # retract to clear object
        self._retract()

def load_gazebo_models(table_pose=Pose(position=Point(x=1.0, y=0.0, z=0.0)),
                       table_reference_frame="world",
                       block_pose=Pose(position=Point(x=0.6725, y=0.1265, z=0.7825)),
                       block_reference_frame="world"):
    # Get Models' Path
    model_path = rospkg.RosPack().get_path('baxter_sim_examples')+"/models/"
    # Load Table SDF
    table_xml = ''
    with open (model_path + "cafe_table/model.sdf", "r") as table_file:
        table_xml=table_file.read().replace('\n', '')
    # Load Block URDF
    block_xml = ''
    with open (model_path + "block/model.urdf", "r") as block_file:
        block_xml=block_file.read().replace('\n', '')
    # Spawn Table SDF
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:
        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        resp_sdf = spawn_sdf("cafe_table", table_xml, "/",
                             table_pose, table_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))
    # Spawn Block URDF
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        resp_urdf = spawn_urdf("block", block_xml, "/",
                               block_pose, block_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn URDF service call failed: {0}".format(e))

def delete_gazebo_models():
    # This will be called on ROS Exit, deleting Gazebo models
    # Do not wait for the Gazebo Delete Model service, since
    # Gazebo should already be running. If the service is not
    # available since Gazebo has been killed, it is fine to error out
    try:
        delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        resp_delete = delete_model("cafe_table")
        resp_delete = delete_model("block")
    except rospy.ServiceException, e:
        rospy.loginfo("Delete Model service call failed: {0}".format(e))

def main():
    """RSDK Inverse Kinematics Pick and Place Example

    A Pick and Place example using the Rethink Inverse Kinematics
    Service which returns the joint angles a requested Cartesian Pose.
    This ROS Service client is used to request both pick and place
    poses in the /base frame of the robot.

    Note: This is a highly scripted and tuned demo. The object location
    is "known" and movement is done completely open loop. It is expected
    behavior that Baxter will eventually mis-pick or drop the block. You
    can improve on this demo by adding perception and feedback to close
    the loop.
    """
    rospy.init_node("ik_pick_and_place_demo")
    # Load Gazebo Models via Spawning Services
    # Note that the models reference is the /world frame
    # and the IK operates with respect to the /base frame
    load_gazebo_models()
    # Remove models from the scene on shutdown
    rospy.on_shutdown(delete_gazebo_models)

    # Wait for the All Clear from emulator startup
    rospy.wait_for_message("/robot/sim/started", Empty)

    limb = 'left'
    hover_distance = 0.15 # meters - ADJUST
    # Starting Joint angles for left arm
    starting_joint_angles_left = {'left_w0': 0.6699952259595108,
                             'left_w1': 1.030009435085784,
                             'left_w2': -0.4999997247485215,
                             'left_e0': -1.189968899785275,
                             'left_e1': 1.9400238130755056,
                             'left_s0': -0.08000397926829805,
                             'left_s1': -0.9999781166910306}
    limb = 'right'

    # Starting Joint angles for right arm
    starting_joint_angles_right = {'right_w0': 0.6699952259595108,
                             'right_w1': 1.030009435085784,
                             'right_w2': -0.4999997247485215,
                             'right_e0': -1.189968899785275,
                             'right_e1': 1.9400238130755056,
                             'right_s0': -0.08000397926829805,
                             'right_s1': -0.9999781166910306}

    pnp = PickAndPlace(limb, hover_distance)
    # An orientation for gripper fingers to be overhead and parallel to the obj
    overhead_orientation = Quaternion(
                             x=-0.0249590815779,
                             y=0.999649402929,
                             z=0.00737916180073,
                             w=0.00486450832011)

    block_poses_vertical = list()
    block_poses_horizontal = list()

    # ----------------------VERTICAL BRICKS POSITIONS------------------------

    for i in
    # spawning position of vertical bricks
    block_poses_vertical.append(Pose(
        position=Point(x=0.7, y=0.15, z=-0.129),
        orientation=overhead_orientation))

    # first drop position
    block_poses_vertical.append(Pose(
        position=Point(x=0.75, y=0.0, z=-0.129),
        orientation=overhead_orientation))
    # second drop position
    block_poses_vertical.append(Pose(
        position=Point(x=0.75, y=0.0, z=-0.129),
        orientation=overhead_orientation))


    # --------------------HORIZONTAL BRICKS POSITIONS---------------------------

    # spawning position of horizontal bricks
    block_poses_horizontal.append(Pose(
        position=Point(x=0.7, y=0.15, z=-0.129),
        orientation=overhead_orientation))

    # first drop position
    block_poses_horizontal.append(Pose(
        position=Point(x=0.75, y=0.0, z=-0.129),
        orientation=overhead_orientation))
    # second drop position
    block_poses_horizontal.append(Pose(
        position=Point(x=0.75, y=0.0, z=-0.129),
        orientation=overhead_orientation))


    # this creates 2 lists of (initial position of brick, place position of first brick, place position of second brick. place position of third brick) for horizontal and vertical

    # -------------------- MOVE TO START POSITION------------------
    # starting angles
    pnp.move_to_start(starting_joint_angles_left)
    pnp.move_to_start(starting_joint_angles_right)

    # ---------------------- Final Structure -----------------------
        _
       | |
       _ _
      | | |
      _ _ _
     | | | |
     _ _ _ _
    | | | | |


    # --------------------PICKING AND PLACING------------------

    # need the list to be in the right way so 2 arms move correctly far left, far right, next in left, next in right retract
    # put in another counter in for the right and left arm (needs to be reset for each layer)

    # LAYER 1
    # VERTICAL BRICKS
    idx = 1
    for i in range (1,5) # first 5 bricks - LIST 1 1-5
        if idx%2 = 1 # 1%2 = 1, 2%2 = 0
        # alternate left and right arms
        limb = 'left'
        else limb = 'right'
        while not rospy.is_shutdown():
            print("\nPicking...")
            pnp.pick(block_poses_vertical[0])
            print("\nPlacing...")
            pnp.place(block_poses_vertical[idx])
            idx = idx + 1
        return 0
    # LAYER 2
    # HORIZONTAL BRICKS
    idx = 1
    for i in range (1,4) # next 4 bricks - LIST 2 1-4
        #if idx%2 = 1
        #limb = 'left'
        #else limb = 'right'
        while not rospy.is_shutdown():
            print("\nPicking...")
            pnp.pick(block_poses_horizontal[0])
            print("\nPlacing...")
            pnp.place(block_poses_horizontal[idx])
            idx = idx + 1
        return 0
    # LAYER 3
    # VERTICAL BRICKS
    idx = 6
    for i in range (1,4) # next 4 bricks - LIST 1 6-9
        if idx%2 = 1
        limb = 'left'
        else limb = 'right'
        while not rospy.is_shutdown():
            print("\nPicking...")
            pnp.pick(block_poses[0])
            print("\nPlacing...")
            pnp.place(block_poses[idx])
            idx = idx + 1
        return 0
    # LAYER 4
    # HORIZONTAL BRICKS
    idx = 5
    for i in range (1,3) # next 3 bricks - LIST 2 5-7
        if idx%2 = 1
        limb = 'left'
        else limb = 'right'
        while not rospy.is_shutdown():
            print("\nPicking...")
            pnp.pick(block_poses_horizontal[0])
            print("\nPlacing...")
            pnp.place(block_poses_horizontal[idx])
            idx = idx + 1
        return 0
    # LAYER 5
    # VERTICAL BRICKS
    idx = 10
    for i in range (1,3) # next 3 bricks - LIST 10-12
        if idx%2 = 1
        limb = 'left'
        else limb = 'right'
        while not rospy.is_shutdown():
            print("\nPicking...")
            pnp.pick(block_poses[0])
            print("\nPlacing...")
            pnp.place(block_poses[idx])
            idx = idx + 1
        return 0
    # LAYER 6
    # HORIZONTAL BRICKS
    idx = 8
    for i in range (1,2) # next 2 bricks - LIST 2 8-9
        if idx%2 = 1
        limb = 'left'
        else limb = 'right'
        while not rospy.is_shutdown():
            print("\nPicking...")
            pnp.pick(block_poses_horizontal[0])
            print("\nPlacing...")
            pnp.place(block_poses_horizontal[idx])
            idx = idx + 1
        return 0
    # LAYER 7
    # VERTICAL BRICKS
    idx = 13
    for i in range (1,2) # next 2 bricks - LIST 13-14
        if idx%2 = 1
        limb = 'left'
        else limb = 'right'
        while not rospy.is_shutdown():
            print("\nPicking...")
            pnp.pick(block_poses[0])
            print("\nPlacing...")
            pnp.place(block_poses[idx])
            idx = idx + 1
        return 0
    # LAYER 8
    # HORIZONTAL BRICKS
    idx = 10
    for i in range (1) # next 2 bricks - LIST 2 10
        if idx%2 = 1
        limb = 'left'
        else limb = 'right'
        while not rospy.is_shutdown():
            print("\nPicking...")
            pnp.pick(block_poses_horizontal[0])
            print("\nPlacing...")
            pnp.place(block_poses_horizontal[idx])
            idx = idx + 1
        return 0

if __name__ == '__main__':
    sys.exit(main())
