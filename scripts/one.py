#!/usr/bin/env python

# ROS Node - Action Server - IoT ROS Bridge

import rospy
import actionlib
import threading
import json

from pkg_ros_iot_bridge.msg import msgRosIotAction  # Message Class that is used by ROS Actions internally
from pkg_ros_iot_bridge.msg import msgRosIotGoal    # Message Class that is used for Goal Messages
from pkg_ros_iot_bridge.msg import msgRosIotResult  # Message Class that is used for Result Messages
from pkg_ros_iot_bridge.msg import msgMqttSub       # Message Class that is used for msgMqttSub Messages

from pkg_task5.msg import msgPickPlaceAction
from pkg_task5.msg import msgPickPlaceGoal
from pkg_task5.msg import msgPickPlaceFeedback
from pkg_task5.msg import msgPickPlaceResult

class ActionClient:

    # Class Constructor
    def __init__(self):

        self._ac1 = actionlib.ActionClient('/action_ros_iot', msgRosIotAction)

        self._ac2 = actionlib.ActionClient('/action_pick', msgPickPlaceAction)

        self._ac3 = actionlib.ActionClient('/action_place',msgPickPlaceAction)

        rospy.init_node('node_pick_place_iot_client')

        self._goal_handles1 = {}

        self._goal_handles2 = {}

        self._goal_handles3 = {}
        self.count1 = 0

        self.count2 = 0

        self.count3 = 0

        self._ac1.wait_for_server()

        rospy.loginfo('Action Iot server up ready....')

        self._ac2.wait_for_server()

        rospy.loginfo('Pick server up ready....')

        self.colour={'Medicine':'red','Food':'yellow','Clothes':'green'}
        self.item={'red':'Medicine','yellow':'Food','green':'Clothes'}
        self.priority={'red':'HP','yellow':'MP','green':'LP'}
        self.cost={'red':450,'yellow':250,'green':150}

        self._ac3.wait_for_server()

        rospy.loginfo('Place server up ready....')

        self._sub_mqtt_sub = rospy.Subscriber('/ros_iot_bridge/mqtt/sub', msgMqttSub, self.func_callback_msgMqttSub)

        goal = msgPickPlaceGoal()

        goal.message = 'None'

        goal_handle = self._ac2.send_goal(goal,self.on_transition2,None)
        self.count2 = self.count2 + 1
        self._goal_handles2.update({str(self.count2):goal_handle})


    def func_callback_msgMqttSub(self,msg):

        print('message recieved from msgMqttsub',msg)

        x=json.loads(msg.message)

        feed = msgPickPlaceFeedback()
        feed.state = 1
        parameters = {'id':'IncomingOrders','Team Id':'VB#0620','Unique Id':'FAAMAMYU','Order ID':x['order_id'],'Order Date and Time':x['order_time'],'Item':x['item'],'Priority':self.priority[self.colour[x['item']]],
        "Order Quantity":'1','City':x['city'],'Longitude':x['lon'],'Latitude':x['lat'],'Cost':self.cost[self.colour[x['item']]]}

        feed.result = json.dumps(parameters)
        self.send_goal_iot(feed)


        self.send_goal_pick_place(msg.message)

    def send_goal_pick_place(self,msg):


        goal = msgPickPlaceGoal()
        goal.message = msg

        goal_handle = self._ac2.send_goal(goal,self.on_transition2,None)
        print('Sending goal to pick action client for picking package',goal)
        self.count2 = self.count2 + 1
        self._goal_handles2.update({str(self.count2):goal_handle})

    def feedback2(self,goal_handle,feedback):

        for i in self.goal_handles2:
            if self._goal_handles2[i] == goal_handle:
                index = i
                break
        rospy.loginfo("Feedback Callback. Client Goal Handle #: " + str(index))


        self.send_goal_iot(feedback)

    def send_goal_iot(self,msg):

        goal = msgRosIotGoal()

        goal.message = msg.result
        goal.protocol = 'http'
        goal.topic = str(msg.state)
        goal.mode = 'pub'


        goal_handle = self._ac1.send_goal(goal,self.on_transition1,None)
        print('sending goal to iot server to push data',goal)
        self.count1 = self.count1 + 1
        self._goal_handles1.update({str(self.count1):goal_handle})

    def on_transition1(self,goal_handle):

        result = msgRosIotResult()

        index = 0
        for i in self._goal_handles1:
            if self._goal_handles1[i] == goal_handle:
                index = i
                break

        rospy.loginfo("Transition Callback. From iot clent Client Goal Handle #: " + str(index))
        rospy.loginfo("Comm. State: " + str(goal_handle.get_comm_state()) )
        rospy.loginfo("Goal Status: " + str(goal_handle.get_goal_status()) )

        # Comm State - Monitors the State Machine of the Client which is different from Server's
        # Comm State = 2 -> Active
        # Comm State = 3 -> Wating for Result
        # Comm State = 7 -> Done

        # if (Comm State == ACTIVE)
        if goal_handle.get_comm_state() == 2:
            rospy.loginfo(str(index) + ": Goal just went active.")

        # if (Comm State == DONE)
        if goal_handle.get_comm_state() == 7:
            rospy.loginfo(str(index) + ": Goal is DONE")
            rospy.loginfo(goal_handle.get_terminal_state())

            # get_result() gets the result produced by the Action Server
            result = goal_handle.get_result()
            rospy.loginfo(result.flag_success)


            if (result.flag_success == True):
                rospy.loginfo("Goal successfully completed. Client Goal Handle #: " + str(index))
            else:
                rospy.loginfo("Goal failed. Client Goal Handle #: " + str(index))
            #rospy.sleep(5)

    def on_transition2(self,goal_handle):

        result = msgPickPlaceResult()

        index = 0
        for i in self._goal_handles2:
            if self._goal_handles2[i] == goal_handle:
                index = i
                break

        rospy.loginfo("Transition Callback.From pick action Client Goal Handle #: " + str(index))
        rospy.loginfo("Comm. State: " + str(goal_handle.get_comm_state()) )
        rospy.loginfo("Goal Status: " + str(goal_handle.get_goal_status()) )

        # Comm State - Monitors the State Machine of the Client which is different from Server's
        # Comm State = 2 -> Active
        # Comm State = 3 -> Wating for Result
        # Comm State = 7 -> Done

        # if (Comm State == ACTIVE)
        if goal_handle.get_comm_state() == 2:
            rospy.loginfo(str(index) + ": Goal just went active.")

        # if (Comm State == DONE)
        if goal_handle.get_comm_state() == 7:
            rospy.loginfo(str(index) + ": Goal is DONE")
            rospy.loginfo(goal_handle.get_terminal_state())



            # get_result() gets the result produced by the Action Server
            result = goal_handle.get_result()
            rospy.loginfo(result)




            if (result.success == True):

                if result.state==0:
                    res=json.loads(result.result)
                    sen = msgPickPlaceResult()
                    for i in res:

                        sen.result = json.dumps(res[i])
                        sen.success = True
                        sen.state = 0
                        self.send_goal_iot(sen)

                else:
                    rospy.loginfo("Goal successfully completed. Client Goal Handle #: " + str(index))

                    self.send_goal_place(result)
                    self.send_goal_iot(result)

            else:
                rospy.loginfo("Goal failed. Client Goal Handle #: " + str(index))

            #rospy.sleep(5)
    def send_goal_place(self,msg):

        goal = msgPickPlaceGoal()
        print('msg in send_goal_place', msg)
        goal.message = msg.result

        goal_handle=self._ac3.send_goal(goal,self.on_transition3,None)
        self.count3 = self.count3 + 1
        self._goal_handles3.update({str(self.count3):goal_handle})

    def on_transition3(self,goal_handle):

        result = msgPickPlaceResult()

        index = 0
        for i in self._goal_handles3:
            if self._goal_handles3[i] == goal_handle:
                index = i
                break

        rospy.loginfo("Transition Callback. from place action Client Goal Handle #: " + str(index))
        rospy.loginfo("Comm. State: " + str(goal_handle.get_comm_state()) )
        rospy.loginfo("Goal Status: " + str(goal_handle.get_goal_status()) )

        # Comm State - Monitors the State Machine of the Client which is different from Server's
        # Comm State = 2 -> Active
        # Comm State = 3 -> Wating for Result
        # Comm State = 7 -> Done

        # if (Comm State == ACTIVE)
        if goal_handle.get_comm_state() == 2:
            rospy.loginfo(str(index) + ": Goal just went active.")

        # if (Comm State == DONE)
        if goal_handle.get_comm_state() == 7:
            rospy.loginfo(str(index) + ": Goal is DONE")
            rospy.loginfo(goal_handle.get_terminal_state())



            # get_result() gets the result produced by the Action Server
            result = goal_handle.get_result()
            rospy.loginfo(result)




            if (result.success == True):




                rospy.loginfo("Goal successfully completed. Client Goal Handle #: " + str(index))

                self.send_goal_iot(result)

            else:
                rospy.loginfo("Goal failed. Client Goal Handle #: " + str(index))









if __name__ == '__main__':
    x=ActionClient()
    rospy.spin()
