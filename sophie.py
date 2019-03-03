#pose and orientation etc. should have already been defined? so just the coordinates need to be added

Coordinates = Pyramid_Builder() #Pyramid_builder is the list of coordinates that Frankie has created
def block_pose_coordinates():
    
    block_poses = list()
    #if block_poses[i][j] = int 
    for i in range(0,5):
        p =  Coordinates[i][0]  
        u =  Coordinates[i][1]
        v =  Coordinates[i][2]
        block_poses.append(Pose(
                position=Point(x = p, y = u, z = v),
                orientation=overhead_orientation))
        
    if len(block_poses) == 5: #checks the bottom layer has been added to the list
        for i in range(5,9):
            p =  Coordinates[i][0]  
            u =  Coordinates[i][1]
            v =  Coordinates[i][2]
            block_poses.append(Pose(
                       position=Point(x = p, y = u, z = v),
                      orientation=overhead_orientation))       
            
    if len(block_poses) == 9: #Checks the bottom two layers
        for i in range(9,13):
            p =  Coordinates[i][0]  
            u =  Coordinates[i][1]
            v =  Coordinates[i][2]
            block_poses.append(Pose(
                       position=Point(x = p, y = u, z = v),
                      orientation=overhead_orientation))
            
    if len(block_poses) == 13: #Checks the bottom three layers
        for i in range(13,16):
            p =  Coordinates[i][0]  
            u =  Coordinates[i][1]
            v =  Coordinates[i][2]
            block_poses.append(Pose(
                       position=Point(x = p, y = u, z = v),
                      orientation=overhead_orientation))
            
    if len(block_poses) == 16: #Checks the bottom four layers
        for i in range(16,19):
            p =  Coordinates[i][0]  
            u =  Coordinates[i][1]
            v =  Coordinates[i][2]
            block_poses.append(Pose(
                       position=Point(x = p, y = u, z = v),
                      orientation=overhead_orientation))  
            
    if len(block_poses) == 19: #Checks the bottom four layers
        for i in range(19,21):
            p =  Coordinates[i][0]  
            u =  Coordinates[i][1]
            v =  Coordinates[i][2]
            block_poses.append(Pose(
                       position=Point(x = p, y = u, z = v),
                      orientation=overhead_orientation))
                
    if len(block_poses) == 21: #Checks the bottom four layers
        for i in range(21,23):
            p =  Coordinates[i][0]  
            u =  Coordinates[i][1]
            v =  Coordinates[i][2]
            block_poses.append(Pose(
                       position=Point(x = p, y = u, z = v),
                      orientation=overhead_orientation)) 
                
    if len(block_poses) == 23: #Checks the bottom four layers
        for i in range(23,24):
            p =  Coordinates[i][0]  
            u =  Coordinates[i][1]
            v =  Coordinates[i][2]
            block_poses.append(Pose(
                       position=Point(x = p, y = u, z = v),
                      orientation=overhead_orientation))
            
    print (block_poses)      
    return block_poses

block_pose_coordinates()


    