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

        self._ac2 = actionlib.SimpleActionClient('/action_pick', msgPickPlaceAction)

        self._ac3 = actionlib.ActionClient('/action_place',msgPickPlaceAction)

        rospy.init_node('node_pick_place_iot_client')

        self._goal_handles1 = {}

        self._goal_handles2 = {}

        self._goal_handles3 = {}

        self._goal_info = {}

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

        self._processing_goal = ''

        self._processing_goal_handle = ''

        self.largest_goal_handle = ''

        self.largest_goal = ''

        rospy.loginfo('Place server up ready....')

        self._sub_mqtt_sub = rospy.Subscriber('/ros_iot_bridge/mqtt/sub', msgMqttSub, self.func_callback_msgMqttSub)

        goal = msgPickPlaceGoal()

        goal.message = 'None'

        self._processing_goal_handle = self._ac2.send_goal(goal,done_cb = self.done_callback)
        self._processing_goal = goal.message

        self.count2 = self.count2 + 1
        self._goal_info.update({self.count2:goal.message})

        # self._goal_handles2.update({self.count2:goal_handle})
        # self._goal_info.update({self.count2:goal.message})


    def func_callback_msgMqttSub(self,msg):

        print('message recieved from msgMqttsub',msg)

        x=json.loads(msg.message)

        feed = msgPickPlaceFeedback()
        feed.state = 1
        parameters = {'id':'IncomingOrders','Team Id':'VB#0620','Unique Id':'FAAMAMYU','Order ID':x['order_id'],'Order Date and Time':x['order_time'],'Item':x['item'],'Priority':self.priority[self.colour[x['item']]],
        "Order Quantity":'1','City':x['city'],'Longitude':x['lon'],'Latitude':x['lat'],'Cost':self.cost[self.colour[x['item']]]}

        feed.result = json.dumps(parameters)
        self.send_goal_iot(feed)
        if self._goal_info == {}:
            self.send_goal_pick_place(msg.message)
        else:
            self.count2 = self.count2 + 1
            self._goal_info.update({self.count2:msg.message})


    def send_goal_pick_place(self,msg):

        goal = msgPickPlaceGoal()

        goal.message = msg
        self._processing_goal_handle = self._ac2.send_goal(goal,done_cb = self.done_callback)
        self._processing_goal = goal.message
        self.count2 = self.count2 + 1
        self._goal_info.update({self.count2:goal.message})

    def check_goal_pick_place(self,msg):

        rospy.logwarn('waiting for result complete')
        goal = msgPickPlaceGoal()
        goal.message = msg
        print('the goals collected are ',self._goal_info)

        if self._goal_info != {} :

            y = sorted(self._goal_info)
            # curr_goal = json.loads(self._processing_goal)
            self._largest_goal = json.loads(self._goal_info[y[0]])
            # self._largest_goal_handle = self._goal_handles2[y[0]]

            for i in range(len(y)-1):
                next_goal = json.loads(self._goal_info[y[i+1]])
                index = i
                print('inside the for loop',self._goal_info)
                if self.cost[self.colour[next_goal['item']]] > self.cost[self.colour[self._largest_goal['item']]]:
                    self._largest_goal = next_goal
                    # self._largest_goal_handle = self._goal_handles2[y[i+1]]
                    index = i+1

            # if self.cost[self.colour[self._largest_goal['item']]] > self.cost[self.colour[curr_goal['item']]]:
                # self._processing_goal_handle.cancel()

                goal.message = json.dumps(self._largest_goal)
                # self._goal_handles2.pop(y[index])
                # self._goal_info.pop(y[index])
            print('the largest among goals')
            rospy.logwarn(self._largest_goal)
        self._processing_goal_handle = self._ac2.send_goal(goal,done_cb = self.done_callback)

        self._processing_goal = goal.message
        rospy.logwarn('waiting for result')


        print('Sending goal to pick action client for picking package',goal)
        # self.count2 = self.count2 + 1
        # self._goal_handles2.update({self.count2:goal_handle})
        # self._goal_info.update({self.count2:msg})

    def done_callback(self,status,result):
        rospy.loginfo('The goal has been completed by the pick simple action server')

        rospy.loginfo(result)
        rospy.logwarn(self._processing_goal)
        rospy.logwarn(self._goal_info)
        index = 0

        for i in self._goal_info:
            if self._goal_info[i] == self._processing_goal:
                index = i
                break

        self._processing_goal = ''
        self._processing_goal_handle = ''

        if (result.success == True):

            self._goal_info.pop(index)
            if result.state==0:

                res=json.loads(result.result)
                sen = msgPickPlaceResult()
                for i in res:

                    sen.result = json.dumps(res[i])
                    sen.success = True
                    sen.state = 0
                    self.send_goal_iot(sen)

            else:

                rospy.loginfo("Goal successfully completed by pick action server. Client Goal Handle #: " + str(index))
                y = sorted(self._goal_info)
                if self._goal_info != {}:
                    self.check_goal_pick_place(self._goal_info[y[0]])
                self.send_goal_place(result)
                self.send_goal_iot(result)



    def feedback2(self,goal_handle,feedback):

        for i in self._goal_handles2:
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

        rospy.loginfo("Transition Callback IOT. From iot clent Client Goal Handle #: " + str(index))
        rospy.loginfo("Comm. State: IOT " + str(goal_handle.get_comm_state()) )
        rospy.loginfo("Goal Status: IOT " + str(goal_handle.get_goal_status()) )

        # Comm State - Monitors the State Machine of the Client which is different from Server's
        # Comm State = 2 -> Active
        # Comm State = 3 -> Wating for Result
        # Comm State = 7 -> Done

        # if (Comm State == ACTIVE)
        if goal_handle.get_comm_state() == 2:
            rospy.loginfo(str(index) + ":IOT Goal just went active.")

        # if (Comm State == DONE)
        if goal_handle.get_comm_state() == 7:
            rospy.loginfo(str(index) + ": IOT Goal is DONE")
            rospy.loginfo(goal_handle.get_terminal_state())

            # get_result() gets the result produced by the Action Server
            result = goal_handle.get_result()
            rospy.loginfo(result.flag_success)


            if (result.flag_success == True):
                rospy.loginfo("Goal successfully completed. IOT Client Goal Handle #: " + str(index))
            else:
                rospy.loginfo("IOT Goal failed. Client Goal Handle #: " + str(index))
            #rospy.sleep(5)

    # def on_transition2(self,goal_handle):
    #
    #     result = msgPickPlaceResult()
    #     print(goal_handle,type(goal_handle))
    #     index = 0
    #     for i in self._goal_handles2:
    #         if self._goal_handles2[i] == goal_handle:
    #             index = i
    #             break
    #
    #     rospy.loginfo("Transition Callback.From pick action Client Goal Handle #: " + str(index))
    #     rospy.loginfo("Comm. State: pick action client " + str(goal_handle.get_comm_state()) )
    #     rospy.loginfo("Goal Status: pick action client " + str(goal_handle.get_goal_status()) )
    #
    #     # Comm State - Monitors the State Machine of the Client which is different from Server's
    #     # Comm State = 2 -> Active
    #     # Comm State = 3 -> Wating for Result
    #     # Comm State = 7 -> Done
    #
    #     # if (Comm State == ACTIVE)
    #     if goal_handle.get_comm_state() == 2:
    #         rospy.loginfo(str(index) + ": Goal just went active. in pick action server")
    #
    #     if goal_handle.get_comm_state() == 3:
    #         self._processing_goal_handle = self._goal_handles2[index]
    #         self._processing_goal = self._goal_info[index]
    #         print('current_processing goal',self._processing_goal)
    #
    #     # if (Comm State == DONE)
    #     if goal_handle.get_comm_state() == 7:
    #         rospy.loginfo(str(index) + ": Goal is DONE by pick action server" )
    #         rospy.loginfo(goal_handle.get_terminal_state())
    #
    #
    #
    #         # get_result() gets the result produced by the Action Server
    #         result = goal_handle.get_result()
    #         rospy.loginfo(result)
    #
    #
    #
    #
    #         if (result.success == True):
    #             self._goal_handles2.pop(index)
    #             self._goal_info.pop(index)
    #             if result.state==0:
    #
    #                 res=json.loads(result.result)
    #                 sen = msgPickPlaceResult()
    #                 for i in res:
    #
    #                     sen.result = json.dumps(res[i])
    #                     sen.success = True
    #                     sen.state = 0
    #                     self.send_goal_iot(sen)
    #
    #             else:
    #
    #                 rospy.loginfo("Goal successfully completed by pick action server. Client Goal Handle #: " + str(index))
    #                 y = sorted(self._goal_handles2)
    #                 if self._goal_handles2 != {}:
    #                     self.send_goal_pick_place(self._goal_info[y[0]])
    #                 self.send_goal_place(result)
    #                 self.send_goal_iot(result)
    #
    #         else:
    #             rospy.loginfo("Goal failed. Client Goal Handle #: " + str(index))

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
        rospy.loginfo("Comm. State:PL " + str(goal_handle.get_comm_state()) )
        rospy.loginfo("Goal Status:PL " + str(goal_handle.get_goal_status()) )

        # Comm State - Monitors the State Machine of the Client which is different from Server's
        # Comm State = 2 -> Active
        # Comm State = 3 -> Wating for Result
        # Comm State = 7 -> Done

        # if (Comm State == ACTIVE)
        if goal_handle.get_comm_state() == 2:
            rospy.loginfo(str(index) + ": PL Goal just went active.")

        # if (Comm State == DONE)
        if goal_handle.get_comm_state() == 7:
            rospy.loginfo(str(index) + ": PL Goal is DONE")
            rospy.loginfo(goal_handle.get_terminal_state())



            # get_result() gets the result produced by the Action Server
            result = goal_handle.get_result()
            rospy.loginfo(result)




            if (result.success == True):




                rospy.loginfo("Goal successfully completed PL. Client Goal Handle #: " + str(index))

                self.send_goal_iot(result)

            else:
                rospy.loginfo("Goal failed. Client Goal Handle #: " + str(index))









if __name__ == '__main__':
    x=ActionClient()
    rospy.spin()
