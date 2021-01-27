#!/usr/bin/env python2
import time
import math
import rospy
import numpy as np
from geometry_msgs.msg import Twist, Pose2D
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry

#Added by me
from geometry_msgs.msg import Vector3

from evry_project_plugins.srv import DistanceToFlag

class Robot:
    def __init__(self, group, robot_name, nb_flags):
        self.speed = 0.0
        self.angle = 0.0
        self.sonar = 0.0 #Sonar distance
        self.position = 0.0
        self.positionx = 0.0
        self.positiony = 0.0
        self.theta = 0.0
        ########################################################################
        ############################# New ######################################
        self.flagID = 0
        self.xflag = [0 for i in range(8)]
        self.yflag = [0 for i in range(8)]
        self.flagsInfoFromRobots = []
        self.discoverdflags = [[] for i in range(8)]
        ########################################################################


        #ns : Name of the robot, like robot_A1, robot_A2 etc.
        #To be used for your subscriber and publisher with the robot itself
        self.group = group
        self.robot_name = robot_name
        self.ns = self.group + "/" + self.robot_name
        # to listen to each robot
        ns1 = self.group + "/" + "robot_1"
        ns2 = self.group + "/" + "robot_2"
        ns3 = self.group + "/" + "robot_3"

        self.nb_flags = nb_flags    #Number of flags to discover in the environment

        '''Listener and publisher'''

        rospy.Subscriber(self.ns + "/sensor/sonar_front", Range, self.callbacksonar)
        rospy.Subscriber(self.ns + "/odom", Odometry, self.callbackodom)
        rospy.Subscriber(self.ns + '/cmd_vel',Twist,self.callbackvelocity)
        # Listen to published flag position
        rospy.Subscriber(ns1 + "/flagInfo",Vector3, self.callbackflags)
        rospy.Subscriber(ns2 + "/flagInfo",Vector3, self.callbackflags)
        rospy.Subscriber(ns3 + "/flagInfo",Vector3, self.callbackflags)

        self.cmd_vel_pub = rospy.Publisher(self.ns + "/cmd_vel", Twist, queue_size = 1)
        # publish estimated flags
        self.flags = rospy.Publisher(self.ns + "/flagInfo", Vector3, queue_size = 10)

        self.pub_velocity() #Run the publisher once

    def callbacksonar(self,data):
        self.sonar = data.range

    def callbackvelocity(self, dataVel):
        pass

    '''get position from other robots '''
    def callbackflags(self,dataflag):
        # print(dataflag)
        self.flagsInfoFromRobots.append(dataflag)

    def callbackodom (self, message):

        self.positionx = message.pose.pose.position.x
        self.positiony = message.pose.pose.position.y
        self.theta = math.atan2(self.positiony, self.positionx)

    def get_position(self):
        return self.positionx,self.positiony

    def get_sonar(self):
        return self.sonar



    def set_speed_angle(self,speed,angle):
        self.speed = speed
        self.angle = angle
        self.pub_velocity()

    def pub_velocity(self):
        self.speed = min(2, self.speed) # Maximum speed at 2 m/s

        cmd_vel = Twist()
        cmd_vel.linear.x = self.speed
        cmd_vel.linear.y = 0.0
        cmd_vel.linear.z = 0.0

        cmd_vel.angular.x = 0.0
        cmd_vel.angular.y = 0.0
        cmd_vel.angular.z = self.angle

        self.cmd_vel_pub.publish(cmd_vel)

    def getDistanceToFlag(this):
        rospy.wait_for_service('/distanceToFlag')
        try:
            service = rospy.ServiceProxy('/distanceToFlag', DistanceToFlag)
            pose = Pose2D()
            pose.x = this.positionx
            pose.y = this.positiony
            pose.theta = this.theta
            result = service(pose)
            #print(result)
            return (result.id_flag, result.distance)
        except rospy.ServiceException as e :
            print("Service call failed: %s"%e)

#################################################################################
##########################       My methods            ##########################

    '''
    %     Move the robots 1,2 and 3
    %     arguments:
    %         direction - String(forward, turnRight, turnLeft, backward) '''
    def move(self,direction):
        if direction == 'forward':
            self.speed = 2
            self.angle = 0
        elif direction == 'turnRight':
            self.speed = 2
            self.angle = -0.26179
        elif direction == 'turnLeft':
            self.speed = 2
            self.angle = 0.26179
        elif direction == 'backward':
            self.speed = 2
            self.angle = 1
        elif direction == 'random':
            rand = np.random.rand()
            self.speed = 2
            if rand > 0:
                self.angle = -1.57
            else:
                self.angle = 1.57

        else:
            print('Direction Error')
            ###################################################
    '''
    %     calculation of location of 3 circle intersection
    %     arguments:
    %         points
    %      returns;
    %         [x, y] - intersection point'''
    def CalcIntersection3(self,x1, y1, r1,
                               x2, y2, r2,
                               x3, y3, r3):
        a = 2 * (x2 - x1)
        b = 2 * (y2 - y1)
        c = (r1**2 - r2**2) - (x1**2 - x2**2)  - (y1**2 - y2**2)
        d = 2 * (x3 - x2)
        e = 2 * (y3 - y2)
        f = (r2**2 - r3**2) - (x2**2 - x3**2)  - (y2**2 - y3**2)
        y = (f-d*c/a) / (e - b*d/a)
        x = (c - b*y) / a
        return x,y
        #########################################################

    '''publish estimated flag position '''
    def pub_flags_position(self):
        flagsPosition = Vector3()
        #print('xxxxxxxxx = ' + str(self.xflag[self.flagID]))
        #print('yyyyyyyyyy = ' + str(self.yflag[self.flagID]))
        flagsPosition.x = self.xflag[self.flagID -1]
        flagsPosition.y = self.yflag[self.flagID-1]
        flagsPosition.z = self.flagID
        #print(flagsPosition)
        self.flags.publish(flagsPosition)
        #########################################################
    '''update discovered flags '''
    def discoveredflagsID(self,id):
        self.discoverdflags.append(id)
        #########################################################
    '''check if the estimated flag position inside the max limit of the map(here I assume that the map is 50x50) '''
    def checkEstimationErrors(self,xflag,yflag):
        if (xflag > 50) or (yflag > 50) or (math.isinf(xflag)) or (math.isinf(xflag)) or (xflag < -50) or (yflag < -50):
            status = False
        else:
            status = True
        return status
        #########################################################
    '''obsticles avoidance (based on the distance between the robot and the obsticle)'''
    #function for object avoidance
    def object_avoidance(self,sonar):
        self.speed = 2 - (5-sonar) / 5
        self.angle = (1/sonar) * (5-sonar) + 0.01
        return self.speed, self.angle
        #########################################################
    '''take the average between the robots estimation'''
    def PositionCorrection(self,xnew,ynew,id):
        #print('#########' + str(self.xflag[id]))
        if self.xflag[id] == 0 or self.yflag[id] == 0 :
            xavg = xnew
            yavg = ynew
        else:
            xavg = (xnew + self.xflag[id]) / 2
            yavg = (ynew + self.yflag[id]) / 2
        return xavg, yavg
        #########################################################
        '''Read other robots estimation and correct the current estimation by taking the average'''
    def communicate(self):
        for i in range(len(self.flagsInfoFromRobots)):
            xflag = self.flagsInfoFromRobots[i].x
            yflag = self.flagsInfoFromRobots[i].y
            id = int(self.flagsInfoFromRobots[i].z)
            #print(self.ns + 'correct flag id:' + str(id))
            #print('xxxxxxxxxxxxxxxxx = ' + str(xflag))
            if id in self.discoverdflags:
                self.xflag[id-1], self.yflag[id-1] = self.PositionCorrection(xflag, yflag, id-1)
            else:
                self.xflag[id-1], self.yflag[id-1] = xflag, yflag
                self.discoveredflagsID(self.flagID)
    def get_dist(self, x_robot, y_robot, x_area, y_area):
        delta_x = abs(x_robot) - abs(x_area)
        delta_y = abs(y_robot) - abs(y_area)
        dist = np.linalg.norm([delta_x, delta_y])
        if dist > 5:
            return 5
        else:
            return dist

    def goTogoal(self,x_goal, y_goal, sonar):

        theta_goal = math.atan2(y_goal, x_goal)
        theta_robot = self.theta
        omega =  (theta_robot - theta_goal)
        print('')
        print(self.ns + ' angle_g = ' + str(theta_goal) + ' angle_r = ' +str(theta_robot))
        print(self.ns + ' x = ' + str(self.positionx) + ' y = ' +str(self.positiony) + ' x_g = ' + str(x_goal) + ' y_g = ' + str(y_goal))
        print('')
        self.speed = 2 - (5-sonar) / 5
        self.angle = (1/sonar) * (5-sonar) + 0.01 + omega
        delta_x = self.positionx - x_goal
        delta_y = self.positiony - y_goal
        dist = np.linalg.norm([delta_x, delta_y])
        return dist
















#################################################################################


def run_demo():
    '''Main loop'''
    group = rospy.get_param("~group")
    robot_name = rospy.get_param("~robot_name")
    nb_flags = rospy.get_param("nb_flags")
    robot = Robot(group, robot_name, nb_flags)
    print("Robot : " + str(robot_name) +" from Group : " + str(group) + " is starting..")
    # Initialsation of variables:
    i = 0
    x = np.zeros(4)
    y = np.zeros(4)
    r = np.zeros(4)
    search_area = [[(45,-13), (-45,-13)],
                   [(45,0), (-45,0)],
                   [(45,13), (-45,13)] ]
    j = 0          # goal points number

    while not rospy.is_shutdown():
        #Write here your strategy..
        ###################################################################
        t = rospy.get_time()
        # Initialsation:
        if t < 11:
            # robot 1 is in on the right of robot 2
            if robot.robot_name == 'robot_1':
                robot.move('turnRight')
                xgoal, ygoal = search_area[0][j]
            elif robot.robot_name == 'robot_2':
                robot.move('forward')
                xgoal, ygoal = search_area[1][j]
            elif robot.robot_name == 'robot_3':
                robot.move('turnLeft')
                xgoal, ygoal = search_area[2][j]
            elif robot.robot_name == 'robot_4':
                robot.move('backward')
            else:
                print('ID Error')
                # Do Nothing
            x[0], y[0] = robot.get_position()
            robot.flagID, dist  = robot.getDistanceToFlag()
            r[0] = dist

        ###################################################################
        else:
            # Define search area for each robot by setting goals for each one:

            sonar = float(robot.get_sonar())
            distTogoal = robot.goTogoal(xgoal, ygoal, sonar)
            if distTogoal < 3:
                if robot.robot_name == 'robot_1':
                    xgoal, ygoal = search_area[0][1]
                elif robot.robot_name == 'robot_2':
                    xgoal, ygoal = search_area[1][1]
                elif robot.robot_name == 'robot_3':
                    xgoal, ygoal = search_area[2][1]


            #robot.speed,robot.angle = robot.object_avoidance(0.7*sonar + 0.3*boundaries)
            print(robot.ns + ' speed = '+ str(robot.speed)+ ' angle = ' + str(robot.angle) + ' distTogoal = '+ str(distTogoal) + ' sonar = ' + str(sonar))
            # wait untill the feedback of position is changed
            if (x[0] == 0) and (y[0] == 0):
                x[0], y[0] = robot.get_position()
            else:
                robot.communicate()


                # get 3 points position and distanceToFlag
                x[i+1], y[i+1] = robot.get_position()
                robot.flagID, r[i+1]  = robot.getDistanceToFlag()
                dx = abs(x[i+1] - x[i])
                dy = abs(y[i+1] - y[i])
                # if the robot position is changed consider this point in the algorithm
                if (dx > 1) or (dy > 1):
                        i = i + 1

                # if 3 points are detected calc the intersection
                if i == 3 and (prevID == robot.flagID):
                    xNewflag1, yNewflag1 = robot.CalcIntersection3(x[0],y[0],r[0],
                                                                 x[1],y[1],r[1],
                                                                 x[2],y[2],r[2])
                    xNewflag2, yNewflag2 = robot.CalcIntersection3(x[1],y[1],r[1],
                                                                   x[2],y[2],r[2],
                                                                   x[3],y[3],r[3])

                   # Check if there is error in calculations
                    if (robot.checkEstimationErrors(xNewflag1, yNewflag1)) and (robot.checkEstimationErrors(xNewflag2, yNewflag2)):
                        delta_xy_norm = np.linalg.norm([abs(xNewflag1 - xNewflag2), abs(yNewflag1 - yNewflag2)])
                        if delta_xy_norm < 5:
                            xNewflag = (xNewflag1 + xNewflag2) / 2
                            yNewflag = (yNewflag1 + yNewflag2) / 2
                            # if the flag is already discovered correct the previous estimation by average
                            if robot.flagID in robot.discoverdflags:
                                robot.xflag[robot.flagID -1], robot.yflag[robot.flagID-1] = robot.PositionCorrection(xNewflag, yNewflag, robot.flagID -1)
                            else:
                                robot.xflag[robot.flagID-1], robot.yflag[robot.flagID-1] = xNewflag, yNewflag
                            # publish the discovered flag
                            robot.pub_flags_position()
                            # add the flag id to the discovered flag to the flag ids
                            robot.discoveredflagsID(robot.flagID)
                            robot.move('random')

                    i = 0
                    x[0], y[0] = robot.get_position()
                    robot.flagID, dist  = robot.getDistanceToFlag()
                    r[0] = dist
                    print(robot.ns +' List of discovered flags: \n xflag: ' + str(robot.xflag)+ '\n yflag : '+ str(robot.yflag))
                    print('#################################################')
                    #print('List of flags from others= ' + str(robot.flagsInfoFromRobots))
                # if the flag id is changed Initialse the algorithm again
                elif (prevID != robot.flagID):
                    i = 0
                    x[0], y[0] = robot.get_position()
                    robot.flagID, dist  = robot.getDistanceToFlag()
                    r[0] = dist
        prevID = robot.flagID
        # Publish velocity and angle
        robot.set_speed_angle(robot.speed,robot.angle)
        rospy.sleep(0.5)




if __name__ == "__main__":
    print("Running ROS..")
    rospy.init_node("Controller", anonymous = True)


    run_demo()
