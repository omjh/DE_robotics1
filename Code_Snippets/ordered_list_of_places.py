
t = 60  #thickness
w = 90  #width
h = 200  #height

# coordinates are (x,y,z)
# middle block is (0,0,170)
# bottom of table is (0,0,0)

# one list of all positions in order
array_of_positions = []



# number of bricks in each layer
layers = [5, 4, 4, 3, 3, 2, 2, 1]

# coefficients for alternating brick picking from middle
coefficient_odd = [0, -1, 1, -2, 2]
coefficient_even = [-0.5, 0.5, -1.5, 1.5]

#count layers
count_layer = 1

for i in range(len(layers)):

  # iterate through layers
  current_layer = layers[i]

  # count bricks
  count_brick = 1

  # iterate through bricks in layer
  for x in range(current_layer):
    

    #if layer is ODD NUMBER OF BRICKS and VERTICAL then... (layer 1, 5)
    if current_layer%2 = 1 and count_layer%2 = 1:

      y = coefficient_odd[count_brick]*h
    
      z= int((count_layer/2)+1)*h - 30


    # if layer is ODD NUMBER OF BRICKS and HORIZONTAL then... (layer 4, 8)
    elif current_layer%2 = 1 and count_layer%2 = 0:

      y= coefficient_odd[count_brick]*h
    
      z= int((count_layer/2)+1)*t - 30
  
  
    # if layer is EVEN NUMBER OF BRICKS and VERTICAL then... (layer 3, 7)
    elif current_layer%2 = 1 and count_layer%2 = 1:
  
      y= coefficient_even[count_brick]*h
    
      z= int((count_layer/2)+1)*h - 30
  
    # if layer is EVEN NUMBER OF BRICKS and HORIZONTAL then... (layer 2, 6)
    elif current_layer%2 = 1 and count_layer%2 = 0:
  
      y= coefficient_even[count_brick]*h
    
      z= int((count_layer/2)+1)*t - 30
  
    count_brick = count_brick + 1
  
    array_of_positions.append(0,y,z)

    # move on to next layer
  count_layer = count_layer + 1
