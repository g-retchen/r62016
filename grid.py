#!/usr/bin/env python2
# UMKC Senior Design Robotics Team R6
# Grid information functions.


import math


#each obstacle node represents a square on the grid. will be contained within a grid object.
class ObstacleNode:
    def __init__(self, node_number):
        self.node_number = node_number
        self.home = False
        self.is_corner = False
        self.is_perimeter = False
        self.is_tunnel = False
        self.cache_present = False
        slef.is_obstacle =False
        self.die_value = 0
        self.row_number = 1
        self.col_number = 1
        #self.before_corner = False
        self.traversed = False      #change to node_searched.
        self.node_type = '*'      #saves whether node is tunnel, obstacle, deadend, or no tunnel
                            #'u' for unknown, 't' for tunnel, 'd' for deadend, 'n' for no tunnel


#container object that holds array of obstacle nodes
class Grid(object):
    def __init__(self, number_of_nodes):
        self.number_of_nodes = number_of_nodes
        self.current_node = 1
        self.next_node = 2
        self.previous_node = 1
        self.current_side = 's'             # always starts at A7, the bottom left corner
        self.orientation = 'e'              # always starts facing east.
        self.grid = []
        self.is_searching = False
        self.is_grid_search = False
        self.avoidingObstacle = False       # Obstacle is in path.
        self.obstacleDetected = False       # Obstacle Detected
        self.obstacleDirection = 0          # Direction of obstacle sensor.
        self.obstacleRange = 0              # How many squeares away is the obstacle.
        
        for i in range(number_of_nodes + 1):# Number of nodes will be 50?
            self.grid.append(ObstacleNode(i))

    def __getitem__(self,index):
        return self.grid[index]

    def __setitem__(self,index,value):
        self.grid[index] = value

    def get_value(self,which_node):
        return self.grid[which_node].node_number, self.grid[which_node].which_side

    #initialize grid values
    def initialize(self):
        self.orientation = 'e'
        # set corner flags.
        self[1].is_corner = True
        self[7].is_corner = True
        self[43].is_corner = True
        self[49].is_corner = True
        # set home flag.
        self[1].home = True
        
        #initialize row and column numbers
        for i in range(self.number_of_nodes + 1):
            self.grid[i].col_number = (i - 1)%7 + 1
            self.grid[i].row_number = math.floor((i - 1) / 7 + 1)
            #add perimeter flags # and corner flags. removed corner flags.
            if i%7 == 0 or i%7 == 1 or i > 42 or i < 8:
                self.grid[i].is_perimeter = True
                self.grid[i].cache_present = 1
            #if i == 1 or i == 7 or i == 43 or i == 49:
            #    self.grid[i].is_corner = True
            #    self.grid[i].cache_present = 0
                
    def update_node(self, is_tunnel):
        self.grid[self.current_node].is_tunnel = is_tunnel
       # Updates tunnel value if services are detected.

    # 0 go straight
    # 1 turn left, go straight
    # 2 turn left, go straight, turn left
    # 3 turn right, go straight, turn right
    # 4 stop
    # 5 straight, right
    # 6 straight, left
    # 7 right
    # 8 left
    # 9 180 degree turn
    # 10 turn right, go straight

    # Calculates ammount to increment node based on orientatin. 
    def increment_node(self):
        if self.orientation == 's':
            increment = int(-7)
        elif self.orientation == 'e':
            increment = int(1)
        elif self.orientation == 'n':
            increment = int(7)
        elif self.orientation == 'w':
            increment = int(-1)
        else:
            increment = int(0)
            print ("error here. current orientation is: ", self.orientation)
        return increment
    
    # Perimeter search routine
    def next_node_perim(self):
        # At A corner
        # test if node has been searched.
        # drive straight in not on corner.
        while (self.current_node.node_traversed  == False):
            self.previous_node = self.current_node
            if self.grid[self.current_node].is_corner == True:
                self.orientation = self.change_orientation('l')
                self.current_node = self.current_node + self.increment_node()
                turn_type = 8 #turn left and update orientation
            elif self.current_node == 1:            
                turn_type = 8 #turn left and update orientation
                self.is_searching = False
                self.orientation = self.change_orientation('l')
                self.current_node = self.current_node + self.increment_node()
            else:
                self.current_node = self.current_node + self.increment_node()
                turn_type = 0   #go straight 1 square
            print ("current node is: ", self.previous_node, " Turn type", turn_type, "Next node is: ", self.current_node )
            return turn_type



#code for changing the robot's internal oriention whenever it turns
    def change_orientation(self, turnDirection):
        if turnDirection == 'r':
            if self.orientation == 'n':
                orientation = 'e'
            if self.orientation == 'e':
                orientation = 's'
            if self.orientation == 's':
                orientation = 'w'
            if self.orientation == 'w':
                orientation = 'n'
        elif turnDirection == 'l':
            if self.orientation == 'n':
                orientation = 'w'
            if self.orientation == 'w':
                orientation = 's'
            if self.orientation == 's':
                orientation = 'e'
            if self.orientation == 'e':
                orientation = 'n'
        # U-Turn  - consists of two lefts or two rights.
        elif turnDirection == 't':
            if self.orientation == 'n':
                orientation = 's'
            if self.orientation == 'e':
                orientation = 'w'
            if self.orientation == 's':
                orientation = 'n'
            if self.orientation == 'w':
                orientation = 'e'
        return orientation


    # 0 go straight
    # 1 turn left, go straight
    # 2 turn left, go straight, turn left
    # 3 turn right, go straight, turn right
    # 4 stop
    # 5 straight, right
    # 6 straight, left
    # 7 right
    # 8 left
    # 9 180 degree turn
    # 10 turn right, go straight

    #code for grid search
    def next_node_grid(self):
        #if we are at the final node in the search
        if self.current_node == 41:
            self.is_searching = False
            turnType = 4
            self.next_node = self.current_node + self.increment_node()
        elif (self.grid[self.current_node + 1].is_perimeter == True and self.orientation == 'e') or (self.grid[self.current_node ].col_number == 7 and self.orientation == 'n'):
            self.current_node += self.increment_node()
            self.orientation = self.change_orientation('l')
            turnType = 6
        # if we are at or right before the inner perimeter, on west side
        elif (self.grid[self.current_node - 1].is_perimeter == True and self.orientation == 'w') or (self.grid[self.current_node].col_number == 1 and self.orientation == 'n'):
            self.current_node += self.increment_node()
            self.orientation = self.change_orientation('r')
            turnType = 5
        else:
            self.current_node += self.increment_node()
            turnType = 0
        print ("current node:", self.current_node, " Turn type", turnType )
        return turnType


    # 0 go straight
    # 1 turn left, go straight
    # 2 turn left, go straight, turn left
    # 3 turn right, go straight, turn right
    # 4 stop
    # 5 straight, right
    # 6 straight, left
    # 7 right
    # 8 left
    # 9 180 degree turn
    # 10 turn right, go straight

    # Obstacle Avoidance
    # code to avoid an obstacle.
    # Robot will be alerted when there is an object in front of it.
    # Detects range and direction of obstacle.
    # Checks that obstacle is set in grid node.
     

    def obstacleAvoidance(self, startRow, sensorValues, startNode):
       rightObstacle = sensorValues[1]
       middleObstacle = sensorValues[2]
       leftObstacle = sensorValues[3]
    #   direction = 'z'#get rid of this code later!
       
       if rightObstacle == '1':    #no obstacle is to the right of the robot. Turn right and go straight
            self.change_orientation('r')
            self.current_node += self.increment_node()
            self.isEnd(startRow, startNode, rightObstacle, middleObstacle, leftObstacle)
            return 10               
       elif middleObstacle == '1':#no obstacle is in front of the robot. go straight
            self.current_node += self.increment_node()
            self.isEnd(startRow, startNode, rightObstacle, middleObstacle, leftObstacle)
            return 0                
       elif leftObstacle == '1':    #no obstacle is to the left of the robot. Turn left and go straight
            self.change_orientation('l') 
            self.current_node += self.increment_node()
            self.isEnd(startRow, startNode, rightObstacle, middleObstacle, leftObstacle)
            return 1
       else:
            print ("not quite right.")
            
    def obstacleDetected(self, sensorValue, sensorRange):
        # modified to allow input of range distance value.  range is in grid blocks, ie 12" increments.
        # consider replacing value as boolean with single range.  if range == 0, no obstacle detected.
        
        rightObstacle   = sensorValue[1]
        forwardObstacle = sensorValue[2]
        leftObstacle    = sensorValue[3]
        #obstacle is to the right of the robot.
        if rightObstacle == '1':        
            self.change_orientation('r')
            obstacle_node = self.grid[(self.current_node + self.increment_node() * sensorRange)]
            self.grid[obstacle_node].is_obstacle = True
            print ("Obstacles detected to right at node :", self.grid[(self.current_node + self.increment_node() * sensorRange)])
        #obstacle is in front of the robot.
        elif forwardObstacle == '1':    
            obstacle_node = self.grid[(self.current_node + self.increment_node() * sensorRange)]
            self.grid[obstacle_node].is_obstacle = True
            print ("Obstacles detected to left at node :", self.grid[(self.current_node + self.increment_node() * sensorRange)])
        #obstacle is to the left of the robot.
        elif leftObstacle == '1':
            self.change_orientation('l')
            obstacle_node = self.grid[(self.current_node + self.increment_node() * sensorRange)]
            self.grid[obstacle_node].is_obstacle = True
            print ("Obstacles detected ahead at node :", self.grid[(self.current_node + self.increment_node() * sensorRange)])
        else:
            print ("No obstacles detected.")


    def obstacleAvoidance(self, startRow, sensorValue, sensorRange, startNode):
        rightObstacle = sensorValue[1]
        forwardObstacle = sensorValue[2]
        leftObstacle = sensorValue[3]
        #   direction = 'z'#get rid of this code later!
        #
        #read current orientatation
        #read next node.
        #check if next node has obstruction bit set.
        #continue with move
        
        rightNode = self.current_node += self.increment_node()  self.change_orientation('r')

        self.grid[self.current_node += self.increment_node() * sensorRange].is_obstruction

        forwardNode = self.change_orientation('r')
        leftNode = self.change_orientation('r')
        


        if self.current_node += self.increment_node() == '1':    #no obstacle is to the right of the robot. Turn right and go straight
            self.change_orientation('r')
            self.current_node += self.increment_node()
            self.isEnd(startRow, startNode, rightObstacle, forwardObstacle, leftObstacle)
            return 10               
        elif forwardObstacle == '1':#no obstacle is in front of the robot. go straight
            self.current_node += self.increment_node() * 
            self.isEnd(startRow, startNode, rightObstacle, forwardObstacle, leftObstacle)
            return 0                
        elif leftObstacle == '1':#no obstacle is to the left of the robot. Turn left and go straight
            self.change_orientation('l') 
            self.current_node += self.increment_node()
            self.isEnd(startRow, startNode, rightObstacle, forwardObstacle, leftObstacle)
            return 1
        else:
            print ("pinned in.  must back up.")
            

#checks to see if the robot is at the end of the perimeter search. will eventually need to check to see if
#there is an object left behind the robot
    def isEnd(self, startRow, startNode, rightObstacle, forwardObstacle, leftObstacle):
        print ("Start Row: ", startRow, " Row Number:", self[self.current_node].row_number, " Current Node: ", self.current_node)
        print ("Right Obstacle: ", rightObstacle, " Middle Obstacle: ", forwardObstacle, "Left Obstacle: ", leftObstacle)
        if self[self.current_node].row_number == startRow:
            if self.current_node != startNode:
                self.avoidingObstacle = False
        else:
            self.avoidingObstacle = True

    # Verify robot returned home,  ie node 1.            
    def isHome(self)     
        if self[self.current_node].node_number == 1:
            print("Search Complete.  Robot returned home.")
        else:
            print("do you have directions?")
            self.go_Home
        
    # Return to start routine
    def go_Home(self):
        # find current node.  Check for obstructions between here and home. **obstructions are checked in move? I hope. Jeremy
        # go home.

        #self.current_node % 7
        #for i=1:7:
        #self.grid[self.grid[i].col_number + = self.grid[i].row_number * 7]

        while (self.current_node != 1):  #(self.current_node.home == False):
            self.previous_node = self.current_node
            # South first
            if (self.current_node / 7 > 1): 
                if (self.orientation =='w'):
                    self.change_orientation('l')
                    self.current_node = self.current_node + self.increment_node()
                    turn_type = 8 #turn left
                elif (self.orientation =='e'):
                    self.change_orientation('r')
                    self.current_node = self.current_node + self.increment_node()
                    turn_type = 7 #turn right
                elif (self.orientation =='n'):
                    self.change_orientation('t')
                    self.current_node = self.current_node + self.increment_node()
                    turn_type = 9 # u- turn
                else:           #must be facing south already
                    self.current_node = self.current_node + self.increment_node()
                    turn_type = 0 # forward
            # then West
            if (self.current_node % 7 > 1): 
                if (self.orientation =='n'):
                    self.change_orientation('l')
                    self.current_node = self.current_node + self.increment_node()
                    turn_type = 8 #turn left
                elif (self.orientation =='s'):
                    self.change_orientation('r')
                    self.current_node = self.current_node + self.increment_node()
                    turn_type = 7 #turn right
                elif (self.orientation =='e'):
                    self.change_orientation('t')
                    self.current_node = self.current_node + self.increment_node()
                    turn_type = 9 # u- turn
                else:           #must be facing west already
                    self.current_node = self.current_node + self.increment_node()
                    turn_type = 0 # forward
            if (self.current_node.home ==true): print("***   We are Home!   ***")
            return turn_type
