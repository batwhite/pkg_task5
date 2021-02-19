#!/usr/bin/env python

# ROS Node - Action Server - IoT ROS Bridge

import rospy
import actionlib
import threading
import sys
import copy
import time
import rospkg
import yaml
import json

import cv2

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import numpy as np

import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
from pkg_vb_sim.msg import *
from pkg_vb_sim.srv import *
from std_srvs.srv import Empty
import tf2_ros
import tf2_geometry_msgs


from pkg_task5.msg import msgPickPlaceAction
from pkg_task5.msg import msgPickPlaceGoal
from pkg_task5.msg import msgPickPlaceFeedback
from pkg_task5.msg import msgPickPlaceResult


class Camera1:

    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/eyrc/vb/camera_1/image_raw", Image,self.callback)
        self.count=0
        self.package_info = {}
        self.store={'red': [],
         "yellow": [],
         "green": []}

  # Ref: GeekforGeeks.com, https://www.geeksforgeeks.org/detect-the-rgb-color-from-a-webcam-using-python-opencv/
    def get_dominant_colour(self, arg_img):
        # setting values for base colors
        b = arg_img[:, :, :1]
        g = arg_img[:, :, 1:2]
        r = arg_img[:, :, 2:]


        # computing the mean
        b_mean = np.mean(b)
        g_mean = np.mean(g)
        r_mean = np.mean(r)


        # displaying the most prominent color
        if (g_mean > r_mean and g_mean > b_mean):
            return 'green'
        elif (r_mean > g_mean and r_mean > b_mean):
            return 'red'
        elif (b_mean > g_mean and b_mean > r_mean):
            return 'blue'
        else:
            return 'yellow'

    #camera callback function
    def callback(self,data):

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)

        (rows,cols,channels) = cv_image.shape

        #cropping the image to contain just the shelf
        image = cv_image[300:900,90:630,:]

        # Resize a 720x1280 image to 360x640 to fit it on the screen
        resized_image_original = cv2.resize(image, (720/2, 1280/2))
        cv2.imshow('photo',resized_image_original)

        #dictionary to store the package-colour info
        package_info=dict()
        store={
          'red': [],
          "yellow": [],
          "green": []
        }
        rowend=1
        colend=1

        #cropping images of the boxes and getting the dominant colour from them
        if self.count==0:
            for i in [1,2,3,4]:
                for j in [1,2,3]:
                    A = image[rowend:150*i,colend:180*j,:]

                    colour=(self.get_dominant_colour(A))
                    package_info[str(i)+str(j)]=colour
                    store[str(colour)].append('package'+str(i-1)+str(j-1))

                    colend=180*j
                rowend=150*i
                colend=1

            print(store)
            self.count=1
            self.store=store
            self.package_info=package_info

        cv2.waitKey(3)

class pick:

    #constructor
    def __init__(self, arg_robot_name):



        self._robot_ns = '/'  + arg_robot_name
        self._planning_group = "manipulator"

        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._tfBuffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._tfBuffer)
        self._robot = moveit_commander.RobotCommander(robot_description= self._robot_ns +
            "/robot_description", ns=self._robot_ns)
        self._scene = moveit_commander.PlanningSceneInterface(ns=self._robot_ns)
        self._group = moveit_commander.MoveGroupCommander(self._planning_group,
            robot_description= self._robot_ns + "/robot_description", ns=self._robot_ns)
        self._display_trajectory_publisher = rospy.Publisher( self._robot_ns +
            '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=10)
        self._exectute_trajectory_client = actionlib.SimpleActionClient( self._robot_ns +
            '/execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()
        self._box_name = ''

        self.exit=0
        # Attribute to store computed trajectory by the planner
        self._computed_plan = ''

        # Current State of the Robot is needed to add box to planning scene
        self._curr_state = self._robot.get_current_state()

        rp = rospkg.RosPack()
        self._pkg_path = rp.get_path('pkg_task5')
        self._file_path = self._pkg_path + '/config/saved_trajectories/'
        rospy.loginfo( "Package Path: {}".format(self._file_path) )

        rospy.loginfo('\033[94m' + " >>> Init done." + '\033[0m')


    def clear_octomap(self):
        clear_octomap_service_proxy = rospy.ServiceProxy(self._robot_ns + "/clear_octomap", Empty)
        return clear_octomap_service_proxy()

    #Destructor
    def __del__(self):

        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class CartesianPath Deleted." + '\033[0m')

    #to attach the objects to the EE in Gazebo simulator
    def attach_detach_gazebo(self,arg):

        # rospy.loginfo('sending request to vaccum gripper')
        rospy.wait_for_service('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1')
        s= rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1',vacuumGripper)

        if arg=='attach':
            attach_request=s(True)
        elif arg=='detach':
            attach_request=s(False)
        return attach_request

    #method to control the working of the conveyer belt
    def conveyor(self,arg):

        rospy.loginfo("turning on the conveyer")
        rospy.wait_for_service('/eyrc/vb/conveyor/set_power')
        s=rospy.ServiceProxy('/eyrc/vb/conveyor/set_power',conveyorBeltPowerMsg)

        control=s(arg)
        return control

    #method to set the joint values
    def set_joint_angles(self, arg_list_joint_angles):

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
        # rospy.loginfo(list_joint_values)

        self._group.set_joint_value_target(arg_list_joint_angles)
        self._computed_plan=self._group.plan()
        flag_plan = self._group.go(wait=True)

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        if (flag_plan == True):
            rospy.loginfo(
                '\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> set_joint_angles() Failed." + '\033[0m')

        return flag_plan


    def hard_set_joint_angles(self, arg_list_joint_angles, arg_max_attempts):

        number_attempts = 0
        flag_success = False

        while ( (number_attempts <= arg_max_attempts) and  (flag_success is False) ):
            number_attempts += 1
            self._computed_plan=''
            flag_success = self.set_joint_angles(arg_list_joint_angles)
            rospy.logwarn("attempts: {}".format(number_attempts) )
            self.clear_octomap()

            rospy.logwarn("attempts: {}".format(number_attempts) )

    #function to play trajectories from saved yaml file
    def moveit_play_planned_path_from_file(self, arg_file_path, arg_file_name):
        file_path = arg_file_path + arg_file_name

        with open(file_path, 'r') as file_open:
            loaded_plan = yaml.load(file_open)

        ret = self._group.execute(loaded_plan)
        # rospy.logerr(ret)
        return ret

    #to repeat moveit_play_planned_path_from_file if it fails
    def moveit_hard_play_planned_path_from_file(self, arg_file_path, arg_file_name, arg_max_attempts):
        number_attempts = 0
        flag_success = False

        while ( (number_attempts <= arg_max_attempts) and (flag_success is False) ):
            number_attempts += 1
            flag_success = self.moveit_play_planned_path_from_file(arg_file_path, arg_file_name)
            rospy.logwarn("attempts: {}".format(number_attempts) )
            # # self.clear_octomap()

        return True
    def key_play1(self,key):
        rospy.logwarn("1. Playing home_to_"+key)
        self.moveit_hard_play_planned_path_from_file(self._file_path, 'home_to_'+key+'.yaml', 5)

    def key_play2(self,key):
        self.attach_detach_gazebo('attach')
        rospy.logwarn("1. Playing "+key+'to home pose')

        self.moveit_hard_play_planned_path_from_file(self._file_path, key+'_to_home.yaml', 5)
        self.attach_detach_gazebo('detach')
        self.conveyor(100)

class place:

    # Constructor
    def __init__(self, arg_robot_name,camera_info):

        self._robot_ns = '/'  + arg_robot_name
        self._planning_group = "manipulator"

        self._sas = actionlib.ActionServer('/action_place',
                                         msgPickPlaceAction,
                                         self.func_on_rx_goal,
                                         self.func_on_cancel,
                                         auto_start=False)
        self._sas.start()

        rospy.loginfo('Started Place Action Server')
        self.colour={'Medicine':'red','Food':'yellow','Clothes':'green'}
        self.item={'red':'Medicine','yellow':'Food','green':'Clothes'}
        self.priority={'red':'HP','yellow':'MP','green':'LP'}
        self.cost={'red':450,'yellow':250,'green':150}

        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._tfBuffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._tfBuffer)
        self._robot = moveit_commander.RobotCommander(robot_description= self._robot_ns +
            "/robot_description", ns=self._robot_ns)
        self._scene = moveit_commander.PlanningSceneInterface(ns=self._robot_ns)
        self._group = moveit_commander.MoveGroupCommander(self._planning_group,
            robot_description= self._robot_ns + "/robot_description", ns=self._robot_ns)
        self._display_trajectory_publisher = rospy.Publisher( self._robot_ns +
            '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=10)
        self._exectute_trajectory_client = actionlib.SimpleActionClient( self._robot_ns +
            '/execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()
        self._box_name = ''
        self._logical_camera_pose = rospy.Subscriber('/eyrc/vb/logical_camera_2',LogicalCameraImage,
            self.Camera_callback_function)

        #camera info variable to store the dictionary info from class Camera
        self.camera_info=camera_info

        self.x = {}

        #to store the current box name
        self.box_name=''

        # Attribute to store computed trajectory by the planner
        self._computed_plan = ''

        # Current State of the Robot is needed to add box to planning scene
        self._curr_state = self._robot.get_current_state()

        rospy.loginfo('\033[94m' + " >>> Init done." + '\033[0m')

    def func_on_cancel(self, goal_handle):
        rospy.loginfo("Received cancel request.")
        goal_id = goal_handle.get_goal_id()


    def func_on_rx_goal(self, goal_handle):

        self._goal_handle=goal_handle
        goal = goal_handle.get_goal()
        self.x = json.loads(goal.message)
        rospy.loginfo('Recieved new goal from place action client')
        self._goal_handle.set_accepted()

        # parameters = {'id':'OrdersShipped','Team ID':'Vb#0620','Unique Id':'FAAMAMYU','Order ID':x['order_id'],'Shipped Date and Time':x['order_time'],'Item':x['item'],'Priority':self.priority[self.colour[x['item']]],
        # "Shipped Quantity":'1','City':x['city'],'Cost':self.cost[self.colour[x['item']]],'Shipped Status':'Yes'}
        # parameters = json.dumps(parameters)

    #function to sort the boxes in their respective bins
    def box_sorter(self,box_id,new_pose_infox,new_pose_infoy):

        ee_pose=self._group.get_current_pose().pose

        #to get the distance the EE should move to grasp the box
        trans_x=ee_pose.position.x-new_pose_infox
        trans_y=ee_pose.position.y-new_pose_infoy


        self.hard_ee_cartesian_translation(-trans_x,0,0,0.01,5)

        boolv=(self.attach_detach_gazebo('attach'))

        while boolv.result == False:
            boolv=(self.attach_detach_gazebo('attach'))
        print(boolv)

        results = msgPickPlaceResult()

        print('printing x',self.x)
        parameter = {'id':'OrdersShipped','Team Id':'VB#0620','Unique Id':'FAAMAMYU','Order ID':self.x['Order ID'],'Item':self.x['Item'],'Priority':self.priority[self.colour[self.x['Item']]],
        "Shipped Quantity":'1','City':self.x['City'],'Cost':self.cost[self.colour[self.x['Item']]],'Shipped Status':'Yes'}
        parameters = json.dumps(parameter)
        print('paraaaaaaaaaaaaaaaaaaaaaa',parameters)
        #check the diffrent boxes and put them into their respective bins and then return to the home position
        if box_id=='red':

            self.hard_ee_cartesian_translation((0.044638+0.7),(0.713571-0.3),0,0.7,5)

            self.attach_detach_gazebo('detach')


            # self.ee_cartesian_translation(-(0.044638+0.8),-(0.713571),0,1)

        elif box_id=='yellow':

            lst_joint_angles_1 = [-0.2256,
                                  0,
                                  0,
                                  -1.5700,
                                  -1.5794,
                                  0]
            self.hard_set_joint_angles(lst_joint_angles_1,5)

            self.attach_detach_gazebo('detach')

        elif box_id=='green':

            lst_joint_angles_1 = [1.6453901250740568, -2.243975595151765, -1.378693271924699, -1.0897202799753245, 1.5707963586574083, 1.6453901250739564]
            self.hard_set_joint_angles(lst_joint_angles_1,5)

            self.attach_detach_gazebo('detach')

        results.result = parameters
        results.state = 3
        results.success = True
        print('resultsssssssssssssssssssssssssssss',results)


        self._goal_handle.set_succeeded(results)

        lst_joint_angles_1 = [0.13683998732796088, -2.4423682116751824, -1.0174361654883883, -1.2520121813310645, 1.57071913195598, 0.13659848941419117]
        self.hard_set_joint_angles(lst_joint_angles_1,5)




    #camera callback function for the Logical camera messages
    def Camera_callback_function(self,msg):


        #check whether the message isnt empty
        if msg.models != []:
            #to exclude the ur5 in the messages
            if msg.models[-1].type!='ur5':
                pose_info=msg.models[-1].pose

                #using manual transformation info get the coordinates of the boxes wrt to the world frame
                new_pose_infox=-0.8+pose_info.position.z
                new_pose_infoy=0+pose_info.position.y
                new_pose_infoz=2-pose_info.position.x

                #check the boxes and assign the respective box ids and send them to the sorter and stop the conveyer so to grasp the object
                if new_pose_infoy<=0.05 and self.box_name != msg.models[-1].type:
                    self.conveyor(0)
                    self.box_name=msg.models[-1].type
                    box_name = str(int(self.box_name[8])+1)+str(int(self.box_name[9])+1)
                    box_id=self.camera_info[box_name]
                    self.box_sorter(box_id,new_pose_infox,new_pose_infoy)

    #function to move EE by cartesian path
    def ee_cartesian_translation(self, trans_x, trans_y, trans_z, check):
        # 1. Create a empty list to hold waypoints
        waypoints = []

        # 2. Add Current Pose to the list of waypoints
        waypoints.append(self._group.get_current_pose().pose)

        # 3. Create a New waypoint
        wpose = geometry_msgs.msg.Pose()
        wpose.position.x = waypoints[0].position.x + (trans_x)
        wpose.position.y = waypoints[0].position.y + (trans_y)
        wpose.position.z = waypoints[0].position.z + (trans_z)
        # This to keep EE parallel to Ground Plane
        wpose.orientation.x = -0.5
        wpose.orientation.y = -0.5
        wpose.orientation.z = 0.5
        wpose.orientation.w = 0.5

        # 4. Add the new waypoint to the list of waypoints
        waypoints.append(copy.deepcopy(wpose))

        # 5. Compute Cartesian Path connecting the waypoints in the list of waypoints

        (plan, fraction) = self._group.compute_cartesian_path(
            waypoints,   # waypoints to follow
            check,        # Step Size, distance between two adjacent computed waypoints will be 1 cm
            0.0)         # Jump Threshold
        rospy.loginfo("Path computed successfully. Moving the arm.")

        # The reason for deleting the first two waypoints from the computed Cartisian Path can be found here,
        # https://answers.ros.org/question/253004/moveit-problem-error-trajectory-message-contains-waypoints-that-are-not-strictly-increasing-in-time/?answer=257488#post-id-257488

        num_pts = len(plan.joint_trajectory.points)
        if (num_pts >= 3):
            del plan.joint_trajectory.points[0]
            del plan.joint_trajectory.points[1]

        # 6. Make the arm follow the Computed Cartesian Path
        self._group.execute(plan)

    #to move to the given position
    def go_to_pose(self, arg_pose):

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Current Pose:" + '\033[0m')
        # rospy.loginfo(pose_values)

        self._group.set_pose_target(arg_pose)

        flag_plan = self._group.go(wait=True)  # wait=False for Async Move

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        # rospy.loginfo(pose_values)

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        # rospy.loginfo(list_joint_values)

        if (flag_plan == True):
            rospy.loginfo('\033[94m' + ">>> go_to_pose() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> go_to_pose() Failed. Solution for Pose not Found." + '\033[0m')

        return flag_plan

    #Destructor
    def __del__(self):

        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class CartesianPath Deleted." + '\033[0m')

    #to attach the objects to the EE in Gazebo simulator
    def attach_detach_gazebo(self,arg):

        # rospy.loginfo('sending request to vaccum gripper')
        rospy.wait_for_service('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_2')
        s= rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_2',vacuumGripper)

        if arg=='attach':
            attach_request=s(True)
        elif arg=='detach':
            attach_request=s(False)
        return attach_request

    #method to control the working of the conveyer belt
    def conveyor(self,arg):

        rospy.loginfo("turning on the conveyer")
        rospy.wait_for_service('/eyrc/vb/conveyor/set_power')
        s=rospy.ServiceProxy('/eyrc/vb/conveyor/set_power',conveyorBeltPowerMsg)

        control=s(arg)
        return control

    #method to set the joint values
    def set_joint_angles(self, arg_list_joint_angles):

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        self._group.set_joint_value_target(arg_list_joint_angles)
        self._group.plan()
        flag_plan = self._group.go(wait=True)

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        if (flag_plan == True):
            rospy.loginfo(
                '\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> set_joint_angles() Failed." + '\033[0m')

        return flag_plan

    #to repeat set_joint_angles if failed
    def hard_set_joint_angles(self, arg_list_joint_angles, arg_max_attempts):

        number_attempts = 0
        flag_success = False

        while ( (number_attempts <= arg_max_attempts) and  (flag_success is False) ):
            number_attempts += 1
            self._computed_plan=''
            flag_success = self.set_joint_angles(arg_list_joint_angles)
            rospy.logwarn("attempts: {}".format(number_attempts) )

    #to repeat ee_cartesian_translation if failed
    def hard_ee_cartesian_translation(self,trans_x, trans_y, trans_z, check, arg_max_attempts):

        number_attempts = 0
        flag_success = False

        while ( (number_attempts <= arg_max_attempts) and  (flag_success is False) ):
            number_attempts += 1
            self._computed_plan=''
            flag_success = self.ee_cartesian_translation(trans_x, trans_y, trans_z, check)
            rospy.logwarn("attempts: {}".format(number_attempts) )

class pick_actionserver:

    def __init__(self):


        self._as = actionlib.SimpleActionServer('/action_pick',
                                          msgPickPlaceAction,
                                          execute_cb=self.on_goal,
                                          auto_start=False)
        cam = Camera1()
        rospy.sleep(5)
        self.stored_in = cam.package_info
        self.camera_info = cam.store
        self.ur5_1=pick('ur5_1')
        self.ur5_2=place('ur5_2',self.stored_in)
        lst_joint_angles_1 = [0.13683998732796088, -2.4423682116751824, -1.0174361654883883, -1.2520121813310645, 1.57071913195598, 0.13659848941419117]

        self.ur5_1.hard_set_joint_angles(lst_joint_angles_1,5)
        self.ur5_2.hard_set_joint_angles(lst_joint_angles_1,5)

        self.count = 0
        self.goal_info = {}
        self._goal_handles = {}
        self.colour={'Medicine':'red','Food':'yellow','Clothes':'green'}
        self.item={'red':'Medicine','yellow':'Food','green':'Clothes'}
        self.priority={'red':'HP','yellow':'MP','green':'LP'}
        self.cost={'red':450,'yellow':250,'green':150}
        self._as.start()
        rospy.loginfo("Started Pick Action Server.")

    def on_cancel(self, goal_handle):
        rospy.loginfo("Received cancel request.")
        goal_id = goal_handle.get_goal_id()
        goal_handle.set_cancelled()
        lst_joint_angles_1 = [0.13683998732796088, -2.4423682116751824, -1.0174361654883883, -1.2520121813310645, 1.57071913195598, 0.13659848941419117]

        self.ur5_1.hard_set_joint_angles(lst_joint_angles_1,5)

    def on_goal(self, goal):


        rospy.loginfo("Received new goal from Pick Action Client, This is pick Action Server")
        rospy.loginfo(goal)
        goal = goal.message

        if goal != 'None':

            x = json.loads(goal)

            # self.count=self.count+1
            # self.goal_info.update({self.count:x})
            # self._goal_handles.update({self.count:goal_handle})

        if goal == 'None':
            # goal_handle.set_accepted()


            stored_info = self.stored_in
            parameters={}
            i=0

            if stored_info != {}:
                for j in stored_info:
                    i=i+1

                    parameters[i] = {'id':'Inventory','Team Id':'VB#0620','Unique Id':'FAAMAMYU','SKU':stored_info[j][0]+str(j),'Item':self.item[str(stored_info[j])],'Priority':self.priority[stored_info[j]],'Storage Number':'R'+str(int(j[0])-1)
                    +' '
                    +'C'+str(int(j[1])-1),
                    "Cost":self.cost[stored_info[j]],"Quantity":'1'}
                parameters = json.dumps(parameters)
                feed = msgPickPlaceResult()
                feed.result = parameters
                feed.state = 0
                feed.success = True

                self._as.set_succeeded(feed)

        else:



            parameters = {'id':'OrdersDispatched','Team Id':'VB#0620','Unique Id':'FAAMAMYU','Order ID':x['order_id'],'Dispatch Date and Time':x['order_time'],'Item':x['item'],'Priority':self.priority[self.colour[x['item']]],
            "Dispatch Quantity":'1','City':x['city'],'Longitude':x['lon'],'Latitude':x['lat'],'Cost':self.cost[self.colour[x['item']]],'Dispatch Status':'Yes'}

            parameters = json.dumps(parameters)

            items=['package21','package02','package01','package10','package11','package12','package22','package20','package00']

            #to move to the initial home position
            # lst_joint_angles_1 = [0.13683998732796088, -2.4423682116751824, -1.0174361654883883, -1.2520121813310645, 1.57071913195598, 0.13659848941419117]
            #
            # self.ur5_2.hard_set_joint_angles(lst_joint_angles_1,5)

            result = msgPickPlaceResult()


            # if x['item'] == 'Medicine':
            #
            #
            #     # goal_handle.set_accepted()
            #     key = self.camera_info['red'][0]
            #
            #     self.ur5_1.key_play(key)
            #
            #     self.camera_info['red'].remove(key)
            #     self.goal_info.pop(self.count)
            #     self._goal_handles.pop(self.count)
            #     result.success = True
            #     result.result = parameters
            #     result.state = 2
            #     goal_handle.set_succeeded(result)
            #
            # elif len(self._goal_handles) <=1:

            # goal_handle.set_accepted()
            key = self.camera_info[self.colour[x['item']]][0]

            self.ur5_1.key_play1(key)

            if self._as.is_new_goal_available():

                rospy.logwarn('New goal is availiable with higher priority')
                lst_joint_angles_1 = [0.13683998732796088, -2.4423682116751824, -1.0174361654883883, -1.2520121813310645, 1.57071913195598, 0.13659848941419117]

                self.ur5_1.hard_set_joint_angles(lst_joint_angles_1,5)

                self._as.accept_new_goal()
                return
            else:
                self.ur5_1.key_play2(key)



            self.camera_info[self.colour[x['item']]].remove(key)
            # self.goal_info.pop(self.count)
            # self._goal_handles.pop(self.count)
            result.success = True
            result.result = parameters
            result.state = 2
            self._as.set_succeeded(result)

            # else:
            #
            #     for j in [1,2,3]:
            #
            #         for i in self.goal_info.keys():
            #
            #             goal = self._goal_handles[i].get_goal()
            #             x = json.loads(goal)
            #
            #             parameters = {'id':'OrdersDispatched','Team ID':'Vb#0620','Unique Id':'FAAMAMYU','Order ID':x['order_id'],'Dispatch Date and Time':x['order_time'],'Item':x['item'],'Priority':self.priority[self.colour[x['item']]],
            #             "Dispatch Quantity":'1','City':x['city'],'Longitude':x['lon'],'Latitude':x['lat'],'Cost':self.cost[self.colour[x['item']]],'Dispatch Status':'Yes'}
            #             parameters = json.dumps(parameters)
            #
            #             if j == 1 and self.goal_info[i]['item'] == 'Medicine':
            #
            #                 self._goal_handles[i].set_accepted()
            #                 key = self.camera_info['red'][0]
            #
            #                 self.ur5_1.key_play(key)
            #
            #                 # feed = msgPickPlaceFeedback()
            #                 # feed.feedback = json.dumps(self.goal_info[i])
            #                 # feed.state = 1
            #                 # self._as.publish_feedback(2,feed)
            #
            #                 self.camera_info['red'].remove(key)
            #                 self.goal_info.pop(i)
            #                 self._goal_handles.pop(i)
            #                 result.success = True
            #                 result.result = parameters
            #                 result.state = 2
            #
            #                 self._goal_handles[i].set_succeeded(result)
            #
            #             elif j == 2 and self.goal_info[i]['item'] == 'Food':
            #
            #                 self._goal_handles[i].set_accepted()
            #                 key = self.camera_info['yellow'][0]
            #
            #                 self.ur5_1.key_play(key)
            #
            #                 # feed = msgPickPlaceFeedback()
            #                 # feed.feedback = json.dumps(self.goal_info[i])
            #                 # feed.state = 1
            #                 # self._as.publish_feedback(2,feed)
            #
            #                 self.camera_info['yellow'].remove(key)
            #                 self.goal_info.pop(i)
            #                 self._goal_handles.pop(i)
            #                 result.success = True
            #                 result.result = parameters
            #                 result.state = 2
            #                 self._goal_handles[i].set_succeeded(result)
            #
            #             elif j == 3 and self.goal_info[i]['item'] == 'Clothes':
            #
            #                 self._goal_handles[i].set_accepted()
            #                 key = self.camera_info['green'][0]
            #
            #                 self.ur5_1.key_play(key)
            #
            #                 # feed = msgPickPlaceFeedback()
            #                 # feed.feedback = json.dumps(self.goal_info[i])
            #                 # feed.state = 1
            #                 # self._as.publish_feedback(2,feed)
            #
            #                 self.camera_info['green'].remove(key)
            #                 self.goal_info.pop(i)
            #                 self._goal_handles.pop(i)
            #                 result.success = True
            #                 result.result = parameters
            #                 result.state = 2
            #                 self._goal_handles[i].set_succeeded(result)
        print('goal completed by pick action server')
# class place_actionserver():
#
#     def __init__ (self):
#
#         self.ur5_1 = place('ur5_2',camera_info)
#
#         self.as = actionlib.SimplpeActionServer('/action_place',msgPickPlaceAction,execute_cb=self.on_goal,auto_start=False)
#         self.as.start()
#         rospy.loginfo('Started Place Action Server')
#         self.colour={'Medicine':'red','Food':'yellow','Clothes':'green'}
#         self.item={'red':'Medicine','yellow':'Food','green':'Clothes'}
#         self.priority={'red':'HP','yellow':'MP','green':'LP'}
#         self.cost={'red':450,'yellow':250,'green':150}
#
#     def on_goal(self, goal_handle):
#
#         goal = goal_handle.get_goal()
#         x = json.loads(goal)
#         rospy.loginfo('Recieved new goal from place action client')
#         ur5_2 = place('ur5_2')
#
#         parameters = {'id':'OrdersShipped','Team ID':'Vb#0620','Unique Id':'FAAMAMYU','Order ID':x['order_id'],'Shipped Date and Time':x['order_time'],'Item':x['item'],'Priority':self.priority[self.colour[x['item']]],
#         "Shipped Quantity":'1','City':x['city'],'Cost':self.cost[self.colour[x['item']]],'Shipped Status':'Yes'}
#         parameters = json.dumps(parameters)







def main():
    rospy.sleep(20)

    rospy.init_node('pick_place_actionserver', anonymous=True)
    obj1 = pick_actionserver()


if __name__ == '__main__':

    main()
