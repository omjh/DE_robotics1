import numpy as np
#----- Brick Dimensions -----

# -- Orientation 1: Upright --

bx1 = 60  #thickness
by1 = 90  #width
bz1 = 200  #height

# -- Orientation 2: Flat --

bx2 = 200
by2 = 90
bz2 = 60

# ----- Brick Spawning Points -----
#Read as x,y,z
'''
def Brickspawn(a,b,c):
    x1 = 0 + a
    y1 = b
    z1 = 180 + c
    x2 = 245 + a  #see diagram
    y2 = b
    z2 = 30 + c
    bs1 = [x1,x2,x3] #a,b and c are offset of coordinates from deniro's origin
    bs2 = [x2,y2,z2]
    return bs1, bs2'''
    
# ----- Final Structure -----

''' _
   | |
   _ _ 
  | | | 
  _ _ _ 
 | | | |
 _ _ _ _ 
| | | | |   '''


def Pyramid_Builder():
    
    Brickcoordinates = np.zeros((1,24), dtype='i,i,i')
    Brickcoordinates = Brickcoordinates[0]  #Read coordinates as (x,y,z)
    print(len(Brickcoordinates))
    
    for i in range(0,5):
        Brickcoordinates[i][0] = i*220 - 440
        Brickcoordinates[i][2] = 180
        
    for i in range(5,9):
        Brickcoordinates[i][0] = (i-5)*220 - 330
        Brickcoordinates[i][2] = 230
    
    for i in range(9,13):
        Brickcoordinates[i][0] = (i-9)*220 - 330
        Brickcoordinates[i][2] = 440
        
    for i in range(13,16):
        Brickcoordinates[i][0] = (i-13)*220 - 220
        Brickcoordinates[i][2] = 490
        
    for i in range(16,19):
        Brickcoordinates[i][0] = (i-16)*220 - 220
        Brickcoordinates[i][2] = 700
        
    for i in range(19,21):
        Brickcoordinates[i][0] = (i-19)*220 - 110
        Brickcoordinates[i][2] = 750
        
    for i in range(21,23):
        Brickcoordinates[i][0] = (i-21)*220 - 110
        Brickcoordinates[i][2] = 960
        
    i = 23
    Brickcoordinates[i][0] = 0
    Brickcoordinates[i][2] = 1100
        
    print(Brickcoordinates)
    return Brickcoordinates
                   
        
Pyramid_Builder()        
    
    