def main():
    """Team House of Cards
    A modification of the  the Pick and Place example which uses the
    Rethink Inverse Kinematics Service which returns the joint angles a
    requested Cartesian Pose. This ROS Service client is used to request
    both pick and place poses in the /base frame of the robot.
    """
    rospy.init_node("ik_house_of_cards")

    # Wait for the All Clear from emulator startup
    rospy.wait_for_message("/robot/sim/started", Empty)

    # Starting Joint angles for both arms
    starting_joint_angles = {'left_w0': 0.6699952259595108,
                             'left_w1': 1.030009435085784,
                             'left_w2': -0.4999997247485215,
                             'left_e0': -1.189968899785275,
                             'left_e1': 1.9400238130755056,
                             'left_s0': -0.08000397926829805,
                             'left_s1': -0.9999781166910306}

    # Two pick and place opjects for each of DENIRO's arms
    hocl = PickAndPlace('left')
    hocr = PickAndPlace('right')

    # An orientation for gripper fingers to be overhead and parallel to the bricks
    orientation = Quaternion(
                             x=-0.0249590815779,
                             y=0.999649402929,
                             z=0.00737916180073,
                             w=0.00486450832011)

    lv_pick = Pose(
        position=Point(x=0., y=0., z=-0.),
        orientation=overhead_orientation)
    rv_pick = Pose(
        position=Point(x=0., y=0., z=-0.),
        orientation=overhead_orientation)
    lh_pick = Pose(
        position=Point(x=0.1, y=0, z=0.125312),
        orientation=overhead_orientation)
    rh_pick = Pose(
        position=Point(x=0., y=0., z=-0.),
        orientation=overhead_orientation)

    base = 3
    height = 1

    # l_block_poses, r_block_poses = house_of_cards_duo(base, height)
    L_block_poses = list()
    L_block_poses.append(Pose(
        position=Point(x=0.3, y=0, z=0.125312),
        orientation=overhead_orientation))

    # Move to the desired starting angles
    hocl.move_to_start(starting_joint_angles)

    i = 0

    while not rospy.is_shutdown() & i > height:
        print("\nPicking...")
        hocl.pick(lh_pick)
        print("\nPlacing...")
        #i = ++
        hocl.place(L_block_poses[i])
    return 0

if __name__ == '__main__':
    sys.exit(main())
