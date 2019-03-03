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
    left_start = {'left_w0': 0.6699952259595108,
                    'left_w1': 1.030009435085784,
                    'left_w2': -0.4999997247485215,
                    'left_e0': -1.189968899785275,
                    'left_e1': 0.6700238130755056,
                    'left_s0': -0.08000397926829805,
                    'left_s1': -0.9999781166910306}
    right_start = {'right_w0': -0.6699952259595108,
                    'right_w1': 1.030009435085784,
                    'right_w2': -0.4999997247485215,
                    'right_e0': 1.189968899785275,
                    'right_e1': 0.6700238130755056,
                    'right_s0': 0.08000397926829805,
                    'right_s1': -0.9999781166910306}

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

    # base and height for the generated house of cards
    base = 3
    height = 1

    block_poses = house_builder(base, height)

    # move to the desired starting angles
    hocl.move_to_start(left_start)
    hocr.move_to_start(right_start)

    # spawn environment
    load_tables()

    # loop to pick and place the entire structure
    i = 0

    while not rospy.is_shutdown() & i > height:
        for j in range(block_poses[i]):
            if i % 2:
                print("\nHorizontal block row")
                if j % 2:
                    print("\nUsing left")
                    #spawn
                    print("\nPicking...")
                    hocl.pick(lh_pick)
                    print("\nPlacing...")
                    hocl.pick(block_poses[i][j])
                else:
                    print("\nUsing right")
                    #spawn
                    print("\nPicking...")
                    hocr.pick(rh_pick)
                    print("\nPlacing...")
                    hocr.pick(block_poses[i][j])
            else:
                print("\nVertical block row")
                if j % 2:
                    print("\nUsing left")
                    #spawn
                    print("\nPicking...")
                    hocl.pick(lv_pick)
                    print("\nPlacing...")
                    hocl.pick(block_poses[i][j])
                else:
                    print("\nUsing right")
                    #spawn
                    print("\nPicking...")
                    hocr.pick(rv_pick)
                    print("\nPlacing...")
                    hocr.pick(block_poses[i][j])
            j += 1
        i += 1
    return

if __name__ == '__main__':
    sys.exit(main())
