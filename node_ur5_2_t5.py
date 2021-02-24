#! /usr/bin/env python
""" This python script is done all the work related to ur5_2 arm"""

import sys
import copy
import os
import time
import datetime
import json
import math
import rospy

import rospkg
import yaml

import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib

from pkg_vb_sim.srv import *
from pkg_vb_sim.msg import LogicalCameraImage
from std_srvs.srv import Empty

from pkg_task5.srv import *
from pkg_task5.msg import msgShippedOrder
from pkg_task5.msg import msgDispatchedOrder

import numpy as np


class CartesianPath:
    """CartesianPath class"""
    def __init__(self):
        """
        The constructor for CartesianPath class.
        Connect ur5_21 move group

        Change position and orientation of End Effector
        to Home(Which i have decided) from allZeros
        """
        rospy.init_node('node_ur5_2_t5', anonymous=True)
        rospy.sleep(12)
        self._robot_ns = '/ur5_2'
        self._planning_group = "manipulator"

        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander(robot_description=self._robot_ns
                                                      + "/robot_description",
                                                      ns=self._robot_ns)
        self._scene = moveit_commander.PlanningSceneInterface(ns=self._robot_ns)
        self._group = moveit_commander.MoveGroupCommander(self._planning_group,
                                                          robot_description=self._robot_ns
                                                          + "/robot_description",
                                                          ns=self._robot_ns)
        self._display_trajectory_publisher = rospy.Publisher(self._robot_ns
                                                             + '/move_group/display_planned_path',
                                                             moveit_msgs.msg.DisplayTrajectory,
                                                             queue_size=1)
        self._exectute_trajectory_client = actionlib.SimpleActionClient(
            self._robot_ns
            + '/execute_trajectory',
            moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()
        self._box_name = ''

        self._order_shipped = rospy.Publisher("eyrc/publish/shippedorder",
                                              msgShippedOrder,
                                              queue_size=10)

        # Attribute to store computed trajectory by the planner
        self._computed_plan = ''
        self.red_packages = []
        self.green_packages = []
        self.yellow_packages = []
        self.dispatched_order = []
        # call a fnction to detect which package is what color
        self.colour_package(True)

        # Current State of the Robot is needed to add box to planning scene
        self._curr_state = self._robot.get_current_state()

        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')

        ros_pkg = rospkg.RosPack()
        self._pkg_path = ros_pkg.get_path('pkg_task5')
        self._file_path = self._pkg_path + '/config/saved_trajectories/ur5_2/'
        rospy.loginfo("Package Path: {}".format(self._file_path))

        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

        rospy.logwarn("1. Playing AllZeros to Pose#1 Trajectory File")
        self.moveit_hard_play_planned_path_from_file(self._file_path, 'go_to_homepose.yaml', 5)

        rospy.loginfo('\033[94m' + " >>> Init done." + '\033[0m')

    def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
        '''
        If the Python node dies before publishing a collision object update message,
        the message could get lost and the box will not appear.
        To ensure that the updates are made, wait until see the changes reflected in the
        get_known_object_names() and get_known_object_names() lists. we call this function after
        adding, removing, attaching or detaching an object in the planning scene.
        '''
        box_name = self._box_name
        scene = self._scene
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = scene.get_attached_objects([box_name])
            is_attached = len(attached_objects.keys()) > 0

            # Test if the box is in the scene.
            # Note that attaching the box will remove it from known_objects
            is_known = box_name in scene.get_known_object_names()

        # Test if we are in the expected state
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

        # Sleep so that we give other threads time on the processor
        seconds = rospy.get_time()

        return False

    def add_box(self, box_position_x, box_position_y, box_position_z, name_box, timeout=4):
        """
        Adding Object(Box) to the Planning Scene.

        Parameters:
            box_position_x (float): position of box in x
            box_position_y (float): position of box in y
            box_position_z (float): position of box in z
            name_box (str): name of box
            timeout (int): Timeout
        Returns:
            Update "wait_for_state_update" function
        """
        box_name = self._box_name
        scene = self._scene
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "world"
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.x = box_position_x
        box_pose.pose.position.y = box_position_y
        box_pose.pose.position.z = box_position_z
        box_name = name_box
        scene.add_box(box_name, box_pose, size=(0.15, 0.15, 0.15))
        self._box_name = box_name
        return self.wait_for_state_update(box_is_known=True, timeout=timeout)

    def attach_box(self, timeout=4):
        """
        Attach box to End Effector of Robot.

        Parameters:
            timeout (int): Timeout
        Returns:
            Update "wait_for_state_update" function
        """
        box_name = self._box_name
        robot = self._robot
        scene = self._scene
        eef_link = self._eef_link
        group_names = self._group_names
        grasping_group = 'manipulator'
        touch_links = robot.get_link_names(group=grasping_group)
        scene.attach_box(eef_link, box_name, touch_links=touch_links)
        rospy.loginfo("  >>>>     Box Attached")
        return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)

    def detach_box(self, timeout=4):
        """
        Detach Object to End Effector of ur5_1 arm.

        Parameters:
            timeout (int): Timeout
        Returns:
            Update "wait_for_state_update" function
        """
        box_name = self._box_name
        scene = self._scene
        eef_link = self._eef_link
        scene.remove_attached_object(eef_link, name=box_name)
        rospy.loginfo("  >>>>     Box Detached")
        return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)

    def remove_box(self, timeout=4):
        """
        Remove Box from planning scene

        Parameters:
            timeout (int): Timeout
        Returns:
            Update "wait_for_state_update" function
        """
        box_name = self._box_name
        scene = self._scene
        scene.remove_world_object(box_name)
        rospy.loginfo("   >>>>>    Box removed from planning scene")
        return self.wait_for_state_update(box_is_attached=False,
                                          box_is_known=False,
                                          timeout=timeout)


    def belt_response(self, power):
        """
        This function call a ROSSERVICE ```/eyrc/vb/conveyor/set_power```

        Parameters:
            power (int): Power(in between 0 to 100) for conveyor belt.
                         Speed of conveyor belt = power/100 (m/sec)
        """
        conveyorBelt_response = rospy.ServiceProxy('/eyrc/vb/conveyor/set_power',
                                                   conveyorBeltPowerMsg)
        conveyorBelt_response(power)

    def clear_octomap(self):
        """
        Clear all octmap
        """
        clear_octomap_service_proxy = rospy.ServiceProxy(self._robot_ns + "/clear_octomap", Empty)
        return clear_octomap_service_proxy()

    def moveit_play_planned_path_from_file(self, arg_file_path, arg_file_name):
        """
        Load yaml file and execute them

        Parameters:
            arg_file_path (str): path of folder where saved trajectry stored
            arg_file_name (str): file name
        """
        file_path = arg_file_path + arg_file_name

        with open(file_path, 'r') as file_open:
            loaded_plan = yaml.load(file_open, Loader=yaml.Loader)

        ret = self._group.execute(loaded_plan)
        # rospy.logerr(ret)
        return ret

    def moveit_hard_play_planned_path_from_file(self,
                                                arg_file_path,
                                                arg_file_name,
                                                arg_max_attempts):
        """
        Call "moveit_play_planned_path_from_file" function multiple times.

        Parameters:
            arg_file_path (str): path of folder where saved trajectry stored
            arg_file_name (str): file name
            arg_max_attempts (int): maximum for "moveit_play_planned_path_from_file" function
        """
        number_attempts = 0
        flag_success = False

        while ((number_attempts <= arg_max_attempts) and (flag_success is False)):
            number_attempts += 1
            flag_success = self.moveit_play_planned_path_from_file(arg_file_path, arg_file_name)
            rospy.logwarn("attempts: {}".format(number_attempts))
            # self.clear_octomap()
        return True

    def colour_package(self, send):
        """
        This function call a ROSSERVICE "eyrc/vb/sendcolourspackages"
        and update red_packages, green_packages, and yellow_packages lists

        Parameters:
        send (bool): rosservice work only when value of send is True
        """
        colourspackages = rospy.ServiceProxy('eyrc/vb/sendcolourspackages', sendColoursPackages)
        jai = colourspackages(send)
        colours = jai.colours
        packages = jai.packages
        all_colours = np.array(colours)
        all_packages = np.array(packages)
        self.red_packages = list(all_packages[all_colours == 'red'])
        self.red_packages.sort()
        self.green_packages = list(all_packages[all_colours == 'green'])
        self.green_packages.sort()
        self.yellow_packages = list(all_packages[all_colours == 'yellow'])
        self.yellow_packages.sort()

    def dispatched_orders(self, mymsg):
        """
        Callback function of ROSTOPIC "eyrc/publish/dispatchedorder"
        """
        if mymsg == "":
            pass
        else:
            dict_dispatched_data = json.loads(mymsg.message)
            self.dispatched_order.append(dict_dispatched_data)

    def get_time_str(self):
        """
        Return a list of two different times
        one is Shipped Time
        second one is Estimated Delivery Time (1 day after Shipped Time)
        """
        time_lis = []
        timestamp = int(time.time())
        shipped_date = datetime.datetime.fromtimestamp(timestamp)
        str_shipped_time = shipped_date.strftime('%Y-%m-%d %H:%M:%S')
        time_lis.append(str_shipped_time)

        delivery_date = shipped_date + datetime.timedelta(days=1)
        str_delivery_time = delivery_date.strftime('%Y-%m-%d %H:%M:%S')
        time_lis.append(str_delivery_time)

        return time_lis

    def __del__(self):
        """
        Destructor
        """
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class CartesianPath Deleted." + '\033[0m')

def main():
    """Main"""
    ur5 = CartesianPath()

    # Subscribe ROSTOPIC eyrc/publish/dispatchedorder
    rospy.Subscriber("eyrc/publish/dispatchedorder", msgDispatchedOrder, ur5.dispatched_orders)

    # Call ROSSERVICE /eyrc/vb/ur5/activate_vacuum_gripper/ur5_2
    gripper_response = rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_2',
                                          vacuumGripper)

    print '\033[91m'+'RED PACKAGES DETECTED BY 2D CAMERA :- '+str(ur5.red_packages)+'\033[0m'
    print '\033[93m'+'YELLOW PACKAGES DETECTED BY 2D CAMERA :- '+str(ur5.yellow_packages)+'\033[0m'
    print '\033[92m'+'GREEN PACKAGES DETECTED BY 2D CAMERA :- '+str(ur5.green_packages)+'\033[0m'

    i = 0

    # Run until rospy is not shutdown
    while not rospy.is_shutdown():
        # Attribute to dispatched order
        dispatched_ord = ur5.dispatched_order

        # Subscribe ROSTOPIC /eyrc/vb/logical_camera_2 (Logical Camera 2)
        msgmy = rospy.wait_for_message("/eyrc/vb/logical_camera_2", LogicalCameraImage)

        # Check there is any model present in view of logical camera 2
        if (msgmy.models) == []:
            pass
        else:
            # Attribute to cost of item
            cost = 0

            # Attribute to priority of item
            priority = ""

            # Check if there is only one model present in view of logical camera 2
            if len(msgmy.models) == 1:

                # Check if model type is packagen or not
                if msgmy.models[0].type[:8] == 'packagen':

                    # Check if packgen in between -0.0315-0.0315
                    if (msgmy.models[0].pose.position.y <= 0.0315 and
                            msgmy.models[0].pose.position.y >= -0.0315):
                        print msgmy.models[0].type
                        print 'Fine'

                        # Stop conveyor belt
                        ur5.belt_response(0)

                        # Attribute to save model type name
                        box = msgmy.models[0].type
                        rospy.sleep(0.1)

                        # Check if box is red or not
                        if box in ur5.red_packages:
                            cost = 450
                            priority = "HP"

                            # Attribute to name of saved trajectory yaml file for
                            # shipping package from conveyor belt
                            play_bin = "go_to_redbin_from_homepose.yaml"
                            play_home = "go_to_homepose_from_redbin.yaml"

                        # Check if box is yellow or not
                        elif box in ur5.yellow_packages:
                            cost = 250
                            priority = "MP"
                            play_bin = "go_to_yellowbin_from_homepose.yaml"
                            play_home = "go_to_homepose_from_yellowbin.yaml"

                        # Check if box is green or not
                        elif box in ur5.green_packages:
                            cost = 150
                            priority = "LP"
                            play_bin = "go_to_greenbin_from_homepose.yaml"
                            play_home = "go_to_homepose_from_greenbin.yaml"

                        else:
                            rospy.logwarn((msgmy.models[0].type+'is not detected my 2d camera'))

                        # Add box in rviz
                        ur5.add_box(-0.8, 0, 0.995, box)

                        # Attach box by arm in rviz
                        ur5.attach_box()

                        # Attach box by arm in gazebo
                        gripper_response(True)
                        ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, play_bin, 5)
                        ur5.detach_box()
                        gripper_response(False)
                        dict_shipped_data = {
                            "Order ID" : dispatched_ord[i]["Order ID"],
                            "City" : dispatched_ord[i]["City"],
                            "Cost" : cost,
                            "Item" : dispatched_ord[i]["Item"],
                            "Priority" : priority,
                            "Shipped Quantity" : dispatched_ord[i]["Dispatch Quantity"],
                            "Shipped Status" : "YES",
                            "Shipped Date and Time" : ur5.get_time_str()[0],
                            "Estimated Time of Delivery" : ur5.get_time_str()[1]
                            }
                        str_shippeddata = json.dumps(dict_shipped_data)

                        msg_shipped_order = msgShippedOrder()
                        msg_shipped_order.message = str_shippeddata
                        ur5._order_shipped.publish(msg_shipped_order)
                        ur5.remove_box()
                        ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, play_home, 5)

                        i = i+1

                    else:
                        pass
                else:
                    pass
            elif len(msgmy.models) == 2:
                if msgmy.models[0].type[:8] == 'packagen':
                    if (msgmy.models[0].pose.position.y <= 0.0315 and
                            msgmy.models[0].pose.position.y >= -0.0315):
                        print msgmy.models[0].type
                        print 'Fine'
                        ur5.belt_response(0)
                        box = msgmy.models[0].type
                        rospy.sleep(0.1)
                        if box in ur5.red_packages:
                            cost = 450
                            priority = "HP"
                            play_bin = "go_to_redbin_from_homepose.yaml"
                            play_home = "go_to_homepose_from_redbin.yaml"

                        elif box in ur5.yellow_packages:
                            cost = 250
                            priority = "MP"
                            play_bin = "go_to_yellowbin_from_homepose.yaml"
                            play_home = "go_to_homepose_from_yellowbin.yaml"

                        elif box in ur5.green_packages:
                            cost = 150
                            priority = "LP"
                            play_bin = "go_to_greenbin_from_homepose.yaml"
                            play_home = "go_to_homepose_from_greenbin.yaml"

                        else:
                            rospy.logwarn((msgmy.models[0].type+'is not detected my 2d camera'))

                        ur5.add_box(-0.8, 0, 0.995, box)
                        ur5.attach_box()
                        gripper_response(True)
                        ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, play_bin, 5)
                        ur5.detach_box()
                        gripper_response(False)

                        dict_shipped_data = {
                            "Order ID" : dispatched_ord[i]["Order ID"],
                            "City" : dispatched_ord[i]["City"],
                            "Cost" : cost,
                            "Item" : dispatched_ord[i]["Item"],
                            "Priority" : priority,
                            "Shipped Quantity" : dispatched_ord[i]["Dispatch Quantity"],
                            "Shipped Status" : "YES",
                            "Shipped Date and Time" : ur5.get_time_str()[0],
                            "Estimated Time of Delivery" : ur5.get_time_str()[1]
                            }
                        str_shippeddata = json.dumps(dict_shipped_data)

                        msg_shipped_order = msgShippedOrder()
                        msg_shipped_order.message = str_shippeddata
                        ur5._order_shipped.publish(msg_shipped_order)

                        ur5.remove_box()
                        ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, play_home, 5)

                        i = i+1

                    else:
                        pass
                elif msgmy.models[1].type[:8] == 'packagen':
                    if (msgmy.models[1].pose.position.y <= 0.0315 and
                            msgmy.models[1].pose.position.y >= -0.0315):
                        print msgmy.models[1].type
                        print 'Fine'
                        ur5.belt_response(0)
                        box = msgmy.models[1].type
                        rospy.sleep(0.1)
                        if box in ur5.red_packages:
                            cost = 450
                            priority = "HP"
                            play_bin = "go_to_redbin_from_homepose.yaml"
                            play_home = "go_to_homepose_from_redbin.yaml"

                        elif box in ur5.yellow_packages:
                            cost = 250
                            priority = "MP"
                            play_bin = "go_to_yellowbin_from_homepose.yaml"
                            play_home = "go_to_homepose_from_yellowbin.yaml"

                        elif box in ur5.green_packages:
                            cost = 150
                            priority = "LP"
                            play_bin = "go_to_greenbin_from_homepose.yaml"
                            play_home = "go_to_homepose_from_greenbin.yaml"

                        else:
                            rospy.logwarn((msgmy.models[0].type+'is not detected my 2d camera'))

                        ur5.add_box(-0.8, 0, 0.995, box)
                        ur5.attach_box()
                        gripper_response(True)
                        ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, play_bin, 5)
                        ur5.detach_box()
                        gripper_response(False)
                        dict_shipped_data = {
                            "Order ID" : dispatched_ord[i]["Order ID"],
                            "City" : dispatched_ord[i]["City"],
                            "Cost" : cost,
                            "Item" : dispatched_ord[i]["Item"],
                            "Priority" : priority,
                            "Shipped Quantity" : dispatched_ord[i]["Dispatch Quantity"],
                            "Shipped Status" : "YES",
                            "Shipped Date and Time" : ur5.get_time_str()[0],
                            "Estimated Time of Delivery" : ur5.get_time_str()[1]
                            }
                        str_shippeddata = json.dumps(dict_shipped_data)

                        msg_shipped_order = msgShippedOrder()
                        msg_shipped_order.message = str_shippeddata
                        ur5._order_shipped.publish(msg_shipped_order)
                        ur5.remove_box()
                        ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, play_home, 5)

                        i = i+1
                    else:
                        pass
                else:
                    pass
            else:
                pass

    del ur5

if __name__ == '__main__':
    main()
