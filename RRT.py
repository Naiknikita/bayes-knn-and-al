#RRT

import sys, random, math, pygame
from pygame.locals import *
from math import sqrt,cos,sin,atan2,pi
from RRT_collision import *

#constants
XDIM = 700
YDIM = 500
WINSIZE = [XDIM, YDIM]
EPSILON = 10.0
NUMNODES = 5000
GOAL_RADIUS = 20
MIN_DISTANCE_TO_ADD = 1.0
GAME_LEVEL = 1

#1/2 car like model
MAX_ACCEL = 2
MIN_ACCEL = -2
MAX_STEER_ACCEL = pi/2
MIN_STEER_ACCEL = -pi/2
MAX_VELO = 5
MIN_VELO = -5
MAX_STEER_VELO = pi/2
MIN_STEER_VELO = -pi/2
DELTA = 0.2

#node points
START_X = 525
START_Y = 425
GOAL_X = 700
GOAL_Y = 0
ROBOT_SPEED = 1
THETA = 0

pygame.init()
fpsClock = pygame.time.Clock()

#initialize and prepare screen
screen = pygame.display.set_mode(WINSIZE)
pygame.display.set_caption('Rapidly Exploring Random Tree')
white = 255, 240, 200
black = 20, 20, 40
red = 255, 0, 0
blue = 0, 255, 0
green = 0, 0, 255
cyan = 0,255,255

# setup program variables
count = 0
rectObs = []


def dist(p1,p2):
    return sqrt((p1[0]-p2[0])*(p1[0]-p2[0])+(p1[1]-p2[1])*(p1[1]-p2[1]))

def step_from_to(p1,p2):
    if dist(p1,p2) < EPSILON:
        return p2
    else:
        theta = atan2(p2[1]-p1[1],p2[0]-p1[0])
        return p1[0] + EPSILON*cos(theta), p1[1] + EPSILON*sin(theta)

def findtheta(p1,p2):
    THETA = atan2(p2[1]-p1[1],p2[0]-p1[0])
    return THETA

def collides(p):
    for rect in rectObs:
        if rect.collidepoint(p) == True:
            # print ("collision with object: " + str(rect))
            return True
    return False


def get_random():
    return random.random()*XDIM, random.random()*YDIM

def get_random_clear():
    while True:
        p = get_random()
        noCollision = collides(p)
        if noCollision == False:
            return p


def init_obstacles(configNum):
    global rectObs
    rectObs = []
    print("config "+ str(configNum))
    if (configNum == 0):
        rectObs.append(pygame.Rect((XDIM / 2.0 - 50, YDIM / 2.0 - 100),(100,200)))
    if (configNum == 1):
        pygame.draw.circle(screen, blue, (350,250), 8)
        pygame.draw.circle(screen, blue, (350,300), 8)
        pygame.draw.circle(screen, blue, (385,420), 8)
        pygame.draw.circle(screen, blue, (476,420), 8)
        pygame.draw.circle(screen, blue, (525,420), 8)
        pygame.draw.circle(screen, blue, (350,480), 8)
        pygame.draw.circle(screen, blue, (525,250), 8)
        pygame.draw.circle(screen, blue, (595,250), 8)
        pygame.draw.circle(screen, blue, (560,200), 8)
        pygame.draw.circle(screen, blue, (511,150), 8)
        pygame.draw.circle(screen, blue, (441,125), 8)
        pygame.draw.circle(screen, blue, (371,125), 8)
        pygame.draw.circle(screen, blue, (301,130), 8)
        pygame.draw.circle(screen, blue, (385,450), 8)
        pygame.draw.circle(screen, blue, (385,490), 8)
        pygame.draw.circle(screen, blue, (281,125), 8)
        pygame.draw.circle(screen, blue, (196,175), 8)
        pygame.draw.circle(screen, blue, (175,235), 8)
        pygame.draw.circle(screen, blue, (175,285), 8)
        pygame.draw.circle(screen, blue, (175,335), 8)
        pygame.draw.circle(screen, blue, (175,385), 8)
        pygame.draw.circle(screen, blue, (700,500), 8)
        pygame.draw.circle(screen, blue, (665,475), 8)
        pygame.draw.circle(screen, blue, (665,425), 8)
    if (configNum == 2):
        rectObs.append(pygame.Rect((40,10),(100,200)))
    if (configNum == 3):
        rectObs.append(pygame.Rect((40,10),(100,200)))

    for rect in rectObs:
        pygame.draw.rect(screen, red, rect)


def reset():
    global count
    screen.fill(black)
    init_obstacles(GAME_LEVEL)
    count = 0

def half_car_robot(GOAL_X, GOAL_Y, theta, p1, p2):
    # 1/2 car like model
    MAX_ACCEL = 2
    MIN_ACCEL = -2
    MAX_STEER_ACCEL = pi/2
    MIN_STEER_ACCEL = -pi/2
    MAX_VELO = 5
    MIN_VELO = -5
    MAX_STEER_VELO = pi/2
    MIN_STEER_VELO = -pi/2
    DELTA = 0.2
    robotSpeed = 1
    distanceGoal = sqrt((p1 - GOAL_X) * (p1 - GOAL_X) + (p2 - GOAL_Y) * (p2- GOAL_Y));
    angleGoal = atan2(GOAL_Y - p2, GOAL_X - p1);
    PosX = max(pow((1.0 / distanceGoal), 2), p1) * sin(angleGoal)
    PosY = max(pow((1.0 / distanceGoal), 2), p2) * cos(angleGoal)
    steer = atan2(robotSpeed * cos(theta) + PosX,
                  robotSpeed * sin(theta) + PosY) - theta;
    			  
    while (steer > MAX_STEER_ACCEL): 
       steer -= 2 * pi; 
    # check to get the angle between -pi and pi
    while (steer < MIN_STEER_ACCEL): 
       steer += 2 * pi;
    X = robotSpeed * sin(theta);
    Y = robotSpeed * cos(theta);
    P = step_from_to(p1,p2);
    theta = steer;
    # return X, Y
    # , theta

def main():
    global count

    initPoseSet = False
    initialPoint = Node(None, None)
    goalPoseSet = False
    goalPoint = Node(None, None)
    currentState = 'init'

    nodes = []
    reset()

    while True:
        if currentState == 'init':
            print('goal point not yet set')
            fpsClock.tick(10)
        elif currentState == 'goalFound':
            #traceback
            currNode = goalNode.parent
            while currNode.parent != None:
                pygame.draw.line(screen,cyan,currNode.point,currNode.parent.point)
                currNode = currNode.parent
            optimizePhase = True

        elif currentState == 'optimize':
            fpsClock.tick(0.5)
            pass
        elif currentState == 'buildTree':
            count = count+1
            if count < NUMNODES:
                foundNext = False
                while foundNext == False:
                    rand = get_random_clear()
                    # print("random num = " + str(rand))
                    parentNode = nodes[0]

                    for p in nodes: #find nearest vertex
                        if dist(p.point,rand) <= dist(parentNode.point,rand): #check to see if this vertex is closer than the previously selected closest
                            newPoint = step_from_to(p.point,rand)
                            if collides(newPoint) == False: # check if a collision would occur with the newly selected vertex
                                parentNode = p #the new point is not in collision, so update this new vertex as the best
                                foundNext = True

                # newnode = step_from_to(parentNode.point,rand)
                thetaval = findtheta(p.point,rand)
                newnode = half_car_robot(GOAL_X, GOAL_Y, thetaval, START_X, START_Y)
                nodes.append(Node(newnode, parentNode))
                pygame.draw.line(screen,white,parentNode.point,newnode)

                if point_circle_collision(newnode, goalPoint.point, GOAL_RADIUS):
                    currentState = 'goalFound'
                    goalNode = nodes[len(nodes)-1]

                if count%100 == 0:
                    print("node: " + str(count))
            else:
                print("Ran out of nodes... :(")
                return;

        #handle events
        for e in pygame.event.get():
            if e.type == QUIT or (e.type == KEYUP and e.key == K_ESCAPE):
                sys.exit("Exiting")
            if e.type == MOUSEBUTTONDOWN:
                print('mouse down')
                if currentState == 'init':
                    if initPoseSet == False:
                        nodes = []
                        if collides(e.pos) == False:
                            # print('initiale pose set: '+str(e.pos))

                            initialPoint = Node((START_X,START_Y), None)
                            nodes.append(initialPoint) # Start in the center
                            initPoseSet = True
                            pygame.draw.circle(screen, blue, initialPoint.point, GOAL_RADIUS)
                    elif goalPoseSet == False:
                        # print('goal pose set: '+str(e.pos))
                        if collides(e.pos) == False:
                            goalPoint = Node((GOAL_X,GOAL_Y),None)
                            goalPoseSet = True
                            pygame.draw.circle(screen, green, goalPoint.point, GOAL_RADIUS)
                            currentState = 'buildTree'
                else:
                    currentState = 'init'
                    initPoseSet = False
                    goalPoseSet = False
                    reset()

        pygame.display.update()
        fpsClock.tick(10000)


# if python says run, then we should run
if __name__ == '__main__':
    main()
    input("press Enter to quit")