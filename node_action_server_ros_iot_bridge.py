#!/usr/bin/env python
"""This python script will done related to ros_iot_bridge"""

import datetime
import threading
import time
import json
import paho.mqtt.client as mqtt
import actionlib
import rospy
import requests

from pkg_ros_iot_bridge.msg import msgRosIotAction
from pkg_ros_iot_bridge.msg import msgRosIotGoal
from pkg_ros_iot_bridge.msg import msgRosIotResult
from pkg_ros_iot_bridge.msg import msgRosIotFeedback

from pkg_ros_iot_bridge.msg import msgMqttSub
from pkg_task5.msg import msgDispatchedOrder
from pkg_task5.msg import msgShippedOrder

from pkg_task5.srv import *
from std_srvs.srv import Empty
import numpy as np

from pyiot import iot


class RosIotBridgeActionServer:
    """RosIotBridgeActionServer Class"""
    def __init__(self):
        """Initialize the Action Server"""
        self._as = actionlib.ActionServer('/action_ros_iot',
                                          msgRosIotAction,
                                          self.on_goal,
                                          self.on_cancel,
                                          auto_start=False)
        '''
            self.on_goal - It is the fuction pointer which points to a function which will be
                             called when the Action Server receives a Goal.

            self.on_cancel - It is the fuction pointer which points to a function which will be
                             called when the Action Server receives a Cancel Request.
        '''
        # Read and Store IoT Configuration data from Parameter Server
        param_config_iot = rospy.get_param('config_pyiot')
        self._config_mqtt_server_url = param_config_iot['mqtt']['server_url']
        self._config_mqtt_server_port = param_config_iot['mqtt']['server_port']
        self._config_mqtt_sub_topic = param_config_iot['mqtt']['topic_sub']
        self._config_mqtt_pub_topic = param_config_iot['mqtt']['topic_pub']
        self._config_mqtt_qos = param_config_iot['mqtt']['qos']
        self._config_mqtt_sub_cb_ros_topic = param_config_iot['mqtt']['sub_cb_ros_topic']
        self._config_google_apps_spread_sheet_id = param_config_iot['google_apps']['spread_sheet_id']
        self._config_google_apps_submission_spread_sheet_id = param_config_iot['google_apps']['submission_spread_sheet_id']
        self._config_google_apps_team_id = "VB#1321"
        self._config_google_apps_unique_id = "eyrcRAVS"
        print param_config_iot
        self.url_spread_sheet = "https://script.google.com/macros/s/"+self._config_google_apps_spread_sheet_id+"/exec"
        self.url_sumission_spread_sheet = "https://script.google.com/macros/s/"+self._config_google_apps_submission_spread_sheet_id+"/exec"

        # Call inventorydatas function
        self.inventorydatas(True)

        # Subscribe ROSTOPIC eyrc/publish/dispatchedorder
        rospy.Subscriber("eyrc/publish/dispatchedorder", msgDispatchedOrder, self.dispatchedorders)

        # Subscribe ROSTOPIC eyrc/publish/shippedorder
        rospy.Subscriber("eyrc/publish/shippedorder", msgShippedOrder, self.shippedorders)

        self.pub_client = mqtt.Client()
        self.pub_client.connect(self._config_mqtt_server_url, self._config_mqtt_server_port)

        # Initialize ROS Topic Publication
        # Incoming message from MQTT Subscription will be published on a ROS Topic (/ros_iot_bridge/mqtt/sub).
        # ROS Nodes can subscribe to this ROS Topic (/ros_iot_bridge/mqtt/sub) to get messages from MQTT Subscription.
        self._handle_ros_pub = rospy.Publisher(self._config_mqtt_sub_cb_ros_topic, msgMqttSub, queue_size=10)

        # Subscribe to MQTT Topic (eyrc/xYzqLm/iot_to_ros) which is defined in 'config_ros_iot.yaml'.
        # self.mqtt_sub_callback() function will be called when there is a message from MQTT Subscription.
        ret = iot.mqtt_subscribe_thread_start(self.mqtt_sub_callback,
                                              self._config_mqtt_server_url,
                                              self._config_mqtt_server_port,
                                              self._config_mqtt_sub_topic,
                                              self._config_mqtt_qos)
        if ret == 0:
            rospy.loginfo("MQTT Subscribe Thread Started")
        else:
            rospy.logerr("Failed to start MQTT Subscribe Thread")

        # Start the Action Server
        self._as.start()

        rospy.loginfo("Started ROS-IoT Bridge Action Server.")

    def dispatchedorders(self, mymsg):
        """
        This is a callback function of ROSTOPIC eyrc/publish/dispatchedorder
        And Send Data of Dispatched Order to OrdersDispatched Sheet
        """

        # Load JSON object as a dictionary
        dict_dispatched_data = json.loads(mymsg.message)

        # Update "dict_dispatched_data" dictionary
        dict_dispatched_data["id"] = "OrdersDispatched"
        dict_dispatched_data["Team Id"] = self._config_google_apps_team_id
        dict_dispatched_data["Unique Id"] = self._config_google_apps_unique_id

        rospy.loginfo("Dispatched Order :- "+ str(dict_dispatched_data))

        # Send data to OrdersDispatched sheet
        response_spreed_sheet = requests.get(self.url_spread_sheet, params=dict_dispatched_data)
        response_submission_spreed_sheet = requests.get(self.url_sumission_spread_sheet,
                                                        params=dict_dispatched_data)
        print response_spreed_sheet.content
        print response_submission_spreed_sheet.content

    def shippedorders(self, mymsg):
        """
        This is a callback function of ROSTOPIC eyrc/publish/shippedorder
        And Send Data of Shipped Order to OrdersShipped Sheet
        """

        # Load JSON object as a dictionary
        dict_shipped_data = json.loads(mymsg.message)

        # Update "dict_shipped_data" dictionary
        dict_shipped_data["id"] = "OrdersShipped"
        dict_shipped_data["Team Id"] = self._config_google_apps_team_id
        dict_shipped_data["Unique Id"] = self._config_google_apps_unique_id

        rospy.loginfo("Shipped Order :- "+ str(dict_shipped_data))

        # Send data to OrdersShipped sheet
        response_spreed_sheet = requests.get(self.url_spread_sheet, params=dict_shipped_data)
        response_submission_spreed_sheet = requests.get(self.url_sumission_spread_sheet,
                                                        params=dict_shipped_data)
        print response_spreed_sheet.content
        print response_submission_spreed_sheet.content

    def incomingorders(self, note):
        """
        Send Data of Incoming Order to IncomingOrders Sheet
        """

        # Load JSON object as a dictionary
        dict_incoming_data = json.loads(note)

        # Attribute to priority of item
        priority = ''

        # Attribute to cost of item
        cost = 0

        # Check if item if Food
        if dict_incoming_data["item"] == 'Food':
            priority = 'MP'
            cost = 250

        # Check if item is Medicine
        elif dict_incoming_data["item"] == 'Medicine':
            priority = 'HP'
            cost = 450

        # Check if item is Clothes
        else:
            priority = 'LP'
            cost = 150

        # Datas which will send to IncomingOrders sheet
        parameters = {"id" : "IncomingOrders",
                      "Team Id" : self._config_google_apps_team_id,
                      "Unique Id" : self._config_google_apps_unique_id,
                      "Order ID" : dict_incoming_data["order_id"],
                      "Order Date and Time" : dict_incoming_data["order_time"],
                      "Order Quantity" : dict_incoming_data["qty"],
                      "Longitude" : dict_incoming_data["lon"],
                      "Latitude" : dict_incoming_data["lat"],
                      "City" : dict_incoming_data["city"],
                      "Item" : dict_incoming_data["item"],
                      "Priority" : priority,
                      "Cost" : cost}

        rospy.loginfo("Incoming Order :- "+ str(parameters))

        # Send data to IncomingOrders sheet
        response_spreed_sheet = requests.get(self.url_spread_sheet, params=parameters)
        response_submission_spreed_sheet = requests.get(self.url_sumission_spread_sheet,
                                                        params=parameters)
        print response_spreed_sheet.content
        print response_submission_spreed_sheet.content

    def inventorydatas(self, send):
        """
        Send Data of Packages present on shelf to Inventory Sheet
        """

        # Call ROSSERVICE eyrc/vb/sendcolourspackages
        colourspackages = rospy.ServiceProxy('eyrc/vb/sendcolourspackages', sendColoursPackages)
        jai = colourspackages(send)
        colours = jai.colours
        packages = jai.packages
        all_colours = np.array(colours)
        all_packages = np.array(packages)

        # red packages on shelf
        red_packages = list(all_packages[all_colours == 'red'])

        # green packages on shelf
        green_packages = list(all_packages[all_colours == 'green'])

        # yellow packages on shelf
        yellow_packages = list(all_packages[all_colours == 'yellow'])

        for package in packages:
            timestamp = int(time.time())
            value = datetime.datetime.fromtimestamp(timestamp)
            str_time = value.strftime('%Y-%m-%d %H:%M:%S')

            # Attribute for Storage Number of package
            storagenumber = 'R'+package[-2]+' C'+package[-1]

            # Attribute for item name of package
            item = ''

            # Attribute for priority of package
            priority = ''

            # Attribute for cost of package
            cost = 0

            # Attribute for sku of package
            sku = ''
            quantity = 1

            # Check if package is Red
            if package in red_packages:
                item = 'Medicine'
                priority = 'HP'
                cost = 450
                sku = 'R'+package[-2:]+str_time[5:7]+str_time[2:4]

            # Check if package is yellow
            elif package in yellow_packages:
                item = 'Food'
                priority = 'MP'
                cost = 250
                sku = 'Y'+package[-2:]+str_time[5:7]+str_time[2:4]

            # Check if package is green
            elif package in green_packages:
                item = 'Clothes'
                priority = 'LP'
                cost = 150
                sku = 'G'+package[-2:]+str_time[5:7]+str_time[2:4]
            else:
                pass

            parameters = {"id" : "Inventory",
                          "Team Id" : self._config_google_apps_team_id,
                          "Unique Id" : self._config_google_apps_unique_id,
                          "SKU" : sku,
                          "Storage Number" : storagenumber,
                          "Quantity" : quantity,
                          "Item" : item,
                          "Priority" : priority,
                          "Cost" : cost}
            
            # push data to Inventory sheet
            response_spreed_sheet = requests.get(self.url_spread_sheet, params=parameters)
            response_submission_spreed_sheet = requests.get(self.url_sumission_spread_sheet,
                                                            params=parameters)
            print response_spreed_sheet.content
            print response_submission_spreed_sheet.content

    def mqtt_sub_callback(self, client, userdata, message):
        """
        This is a callback function for MQTT Subscriptions
        """
        payload = str(message.payload.decode("utf-8"))

        print("[MQTT SUB CB] Message: ", payload)
        print("[MQTT SUB CB] Topic: ", message.topic)

        # Check if payload is null
        if payload == "null":
            rospy.logerr("Message:= "+payload)
        else:
            msg_mqtt_sub = msgMqttSub()
            msg_mqtt_sub.timestamp = rospy.Time.now()
            msg_mqtt_sub.topic = self._config_mqtt_sub_cb_ros_topic
            msg_mqtt_sub.message = payload

            # publish message on ROSTOPIC /ros_iot_bridge/mqtt/sub
            self._handle_ros_pub.publish(msg_mqtt_sub)

            # Call incomingorders function
            self.incomingorders(payload)

    def on_goal(self, goal_handle):
        """
        This function will be called when Action Server receives a Goal
        """
        goal = goal_handle.get_goal()

        rospy.loginfo("Received new goal from Client")
        rospy.loginfo(goal)

        # Validate incoming goal parameters
        if goal.protocol == "mqtt":

            if((goal.mode == "pub") or (goal.mode == "sub")):
                goal_handle.set_accepted()

                # Start a new thread to process new goal from the client 
                # (For Asynchronous Processing of Goals)
                # 'self.process_goal' - is the function pointer which points 
                # to a function that will process incoming Goals
                thread = threading.Thread(name="worker",
                                          target=self.process_goal,
                                          args=(goal_handle,))
                thread.start()

            else:
                goal_handle.set_rejected()
                return

        else:
            goal_handle.set_rejected()
            return

    def process_goal(self, goal_handle):
        """
        This function is called is a separate thread to process Goal.
        """

        flag_success = False
        result = msgRosIotResult()

        goal_id = goal_handle.get_goal_id()
        rospy.loginfo("Processing goal : " + str(goal_id.id))

        goal = goal_handle.get_goal()


        # Goal Processing
        if goal.protocol == "mqtt":
            rospy.logwarn("MQTT")

            if goal.mode == "pub":
                rospy.logwarn("MQTT PUB Goal ID: " + str(goal_id.id))

                rospy.logwarn(goal.topic + " > " + goal.message)
                ret = iot.mqtt_publish(self._config_mqtt_server_url,
                                       self._config_mqtt_server_port,
                                       goal.topic,
                                       goal.message,
                                       self._config_mqtt_qos)

                if ret == 0:
                    rospy.loginfo("MQTT Publish Successful.")
                    result.flag_success = True
                else:
                    rospy.logerr("MQTT Failed to Publish")
                    result.flag_success = False

            elif goal.mode == "sub":
                rospy.logwarn("MQTT SUB Goal ID: " + str(goal_id.id))
                rospy.logwarn(goal.topic)

                ret = iot.mqtt_subscribe_thread_start(self.mqtt_sub_callback,
                                                      self._config_mqtt_server_url,
                                                      self._config_mqtt_server_port,
                                                      goal.topic,
                                                      self._config_mqtt_qos)
                if ret == 0:
                    rospy.loginfo("MQTT Subscribe Thread Started")
                    result.flag_success = True
                else:
                    rospy.logerr("Failed to start MQTT Subscribe Thread")
                    result.flag_success = False

        rospy.loginfo("Send goal result to client")
        if result.flag_success is True:
            rospy.loginfo("Succeeded")
            goal_handle.set_succeeded(result)
        else:
            rospy.loginfo("Goal Failed. Aborting.")
            goal_handle.set_aborted(result)

        rospy.loginfo("Goal ID: " + str(goal_id.id) + " Goal Processing Done.")

    def on_cancel(self, goal_handle):
        """
        This function will be called when Goal Cancel request is send to the Action Server
        """
        rospy.loginfo("Received cancel request.")
        goal_id = goal_handle.get_goal_id()


def main():
    """Main"""
    rospy.init_node('node_action_server_ros_iot_bridge')
    rospy.sleep(14)
    action_server = RosIotBridgeActionServer()

    rospy.spin()

if __name__ == '__main__':
    main()
