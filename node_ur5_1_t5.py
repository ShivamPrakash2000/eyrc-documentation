#! /usr/bin/env python
""" This python script is done all the work related to ur5_1 arm"""

import sys
import time
import datetime
import os
import copy
import math
import json
import rospkg

import rospy
import yaml

import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib

from pkg_vb_sim.srv import *
from pkg_vb_sim.msg import LogicalCameraImage
from std_srvs.srv import Empty

from pkg_ros_iot_bridge.msg import msgMqttSub
from pkg_task5.srv import *
from pkg_task5.msg import msgDispatchedOrder

import numpy as np


class CartesianPath:
    """CartesianPath class"""

    def __init__(self):
        """
        The constructor for CartesianPath class.
        Connect ur5_1 move group

        Change position and orientation of End Effector
        to Home(Which i have decided) from allZeros
        """
        rospy.init_node('node_ur5_1_t5', anonymous=True)
        rospy.sleep(10)
        self._robot_ns = '/ur5_1'
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

        # Attribute to store name of added box in rviz
        self.box_list = []

        # Attribute to store incoming orders
        self.incoming_orders = []

        # Attribute to store name of red packages
        self.red_packages = []

        # Attribute to store name of green packages
        self.green_packages = []

        # Attribute to store name of yellow packages
        self.yellow_packages = []

        # Call colour_package function
        self.colour_package(True)

        # Initialize ROS Topic Publication
        # Dispatched message will be published on a ROSTOPIC (eyrc/publish/dispatchedorder).
        self._order_dispatched = rospy.Publisher("eyrc/publish/dispatchedorder",
                                                 msgDispatchedOrder,
                                                 queue_size=10)
        # Attribute to store computed trajectory by the planner
        self._computed_plan = ''
        self._group.set_planning_time(10)

        # Current State of the Robot is needed to add box to planning scene
        self._curr_state = self._robot.get_current_state()

        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')

        rp = rospkg.RosPack()
        self._pkg_path = rp.get_path('pkg_task5')
        self._file_path = self._pkg_path + '/config/saved_trajectories/ur5_1/'
        rospy.loginfo("Package Path: {}".format(self._file_path))

        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

        # Add all boxes in planning scene
        self.add_box(0.28, -0.41, 1.90, "box00")
        self.add_box(0, -0.41, 1.90, "box01")
        self.add_box(-0.28, -0.41, 1.90, "box02")
        self.add_box(0.28, -0.41, 1.63, "box10")
        self.add_box(0, -0.41, 1.63, "box11")
        self.add_box(-0.28, -0.41, 1.63, "box12")
        self.add_box(0.28, -0.41, 1.41, "box20")
        self.add_box(0, -0.41, 1.41, "box21")
        self.add_box(-0.28, -0.41, 1.41, "box22")
        self.add_box(0.28, -0.41, 1.18, "box30")
        self.add_box(0, -0.41, 1.18, "box31")
        self.add_box(-0.28, -0.41, 1.18, "box32")

        rospy.sleep(5)
        rospy.logwarn("Playing allZeros to homepose Trajectory File")
        self.moveit_hard_play_planned_path_from_file(self._file_path, 'go_to_homepose.yaml', 5)

        param_config_iot = rospy.get_param('config_pyiot')

        # Get ROSTOPIC on which incoming message will be published
        self._config_mqtt_sub_cb_ros_topic = param_config_iot['mqtt']['sub_cb_ros_topic']

        rospy.loginfo('\033[94m' + " >>> Init done." + '\033[0m')

    def colour_package(self, send):
        """
        This function will call ROSSERVICE eyrc/vb/sendcolourspackages
        and update red_packages, green_packages, and yellow_packages lists

        Parameters:
        send (bool): rosservice work only when value of send is True
        """
        colourspackages = rospy.ServiceProxy('eyrc/vb/sendcolourspackages',
                                             sendColoursPackages)

        jai = colourspackages(send)
        colours = jai.colours
        packages = jai.packages
        all_colours = np.array(colours)
        all_packages = np.array(packages)

        # Update red_packages list
        self.red_packages = list(all_packages[all_colours == 'red'])
        self.red_packages.sort()

        # Update green_packages list
        self.green_packages = list(all_packages[all_colours == 'green'])
        self.green_packages.sort()

        # Update yellow_packages list
        self.yellow_packages = list(all_packages[all_colours == 'yellow'])
        self.yellow_packages.sort()

    def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
        """
        If the Python node dies before publishing a collision object update message,
        the message could get lost and the box will not appear.
        To ensure that the updates are made, wait until see the changes reflected in the
        get_known_object_names() and get_known_object_names() lists. we call this function after
        adding, removing, attaching or detaching an object in the planning scene.
        """
        box_name = self._box_name
        self.box_list.append(box_name)
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
        rospy.loginfo("  >>>>   Box Attached")
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
        conveyorbelt_response = rospy.ServiceProxy('/eyrc/vb/conveyor/set_power',
                                                   conveyorBeltPowerMsg)
        conveyorbelt_response(power)

    def set_joint_angles(self, arg_list_joint_angles):
        """
        Planning to a Joint Angle Goal

        Parameters:
            arg_list_joint_angles (list): value of joint angles
        """
        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        self._group.set_joint_value_target(arg_list_joint_angles)
        self._computed_plan = self._group.plan()
        flag_plan = self._group.go(wait=True)

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        if flag_plan:
            rospy.loginfo(
                '\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> set_joint_angles() Failed." + '\033[0m')

        return flag_plan

    def ee_cartesian_translation(self, trans_x, trans_y, trans_z, orx, ory, orz, orw):
        """
        End Effector Cartesian Translation

        parameters:
            trans_x (float): Translation in x
            trans_y (float): Translation in y
            trans_y (float): Translation in z
            orx (float): orientation in x
            ory (float): orientation in y
            orz (float): orientation in z
            orw (float): orientation in w
        """
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
        wpose.orientation.x = orx
        wpose.orientation.y = ory
        wpose.orientation.z = orz
        wpose.orientation.w = orw

        # 4. Add the new waypoint to the list of waypoints
        waypoints.append(copy.deepcopy(wpose))

        # 5. Compute Cartesian Path connecting the waypoints in the list of waypoints
        (plan, fraction) = self._group.compute_cartesian_path(
            waypoints,   # waypoints to follow
            0.01,        # Step Size, distance between two adjacent computed waypoints will be 1 cm
            0.0)         # Jump Threshold
        rospy.loginfo("Path computed successfully. Moving the arm.")

        # The reason for deleting the first two waypoints from the
        # computed Cartisian Path can be found here,
        num_pts = len(plan.joint_trajectory.points)
        if num_pts >= 3:
            del plan.joint_trajectory.points[0]
            del plan.joint_trajectory.points[1]

        # 6. Make the arm follow the Computed Cartesian Path
        self._group.execute(plan)

    def clear_octomap(self):
        """
        Clear Octomap
        """
        clear_octomap_service_proxy = rospy.ServiceProxy(self._robot_ns + "/clear_octomap", Empty)
        return clear_octomap_service_proxy()

    def hard_set_joint_angles(self, arg_list_joint_angles, arg_max_attempts):
        """
        Call "set_joint_angles" function multiple times.

        Parameters:
            arg_list_joint_angles (list): value of joint angles
            arg_max_attempts (int): maximum attempt for "set_joint_angles" function
        """
        number_attempts = 0
        flag_success = False

        while ((number_attempts <= arg_max_attempts) and (flag_success is False)):
            number_attempts += 1
            flag_success = self.set_joint_angles(arg_list_joint_angles)
            rospy.logwarn("attempts: {}".format(number_attempts))
        # self.clear_octomap()

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

    def func_callback_mqtt_messages(self, mymsg):
        """
        This is a callback function of ROSTOPIC /ros_iot_bridge/mqtt/sub
        """
        if not mymsg:
            pass
        else:
            dict_incoming_data = json.loads(mymsg.message)
            print "Incoming Order to ur5_1 Node:- "+str(dict_incoming_data)
            self.incoming_orders.append(dict_incoming_data)

    def check_box_number(self, box):
        """
        Check Which box have to be attach in Rviz.

        Parameters:
            box (str): name of package which would attach
        """
        if box[-2:] == '00':
            index_of_box = 0
        elif box[-2:] == '01':
            index_of_box = 1
        elif box[-2:] == '02':
            index_of_box = 2
        elif box[-2:] == '10':
            index_of_box = 3
        elif box[-2:] == '11':
            index_of_box = 4
        elif box[-2:] == '12':
            index_of_box = 5
        elif box[-2:] == '20':
            index_of_box = 6
        elif box[-2:] == '21':
            index_of_box = 7
        elif box[-2:] == '22':
            index_of_box = 8
        elif box[-2:] == '30':
            index_of_box = 9
        elif box[-2:] == '31':
            index_of_box = 10
        else:
            index_of_box = 11

        return index_of_box

    def get_time_str(self):
        """
        This function for Real time

        Returns:
            str_time (str): return current time
        """
        timestamp = int(time.time())
        value = datetime.datetime.fromtimestamp(timestamp)
        str_time = value.strftime('%Y-%m-%d %H:%M:%S')

        return str_time

    def __del__(self):
        """Destructor"""
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class CartesianPath Deleted." + '\033[0m')


def main():
    """Main"""
    ur5 = CartesianPath()

    lst_joint_angles_box30 = [math.radians(-55.7697018912),
                              math.radians(-91.6153558747),
                              math.radians(120.21567527),
                              math.radians(-30.2972939321),
                              math.radians(124.20002058),
                              math.radians(2.01811152119)]

    lst_joint_angles_box31 = [math.radians(-119.269105736),
                              math.radians(-114.470870298),
                              math.radians(137.528298101),
                              math.radians(-23.6240681769),
                              math.radians(60.6219389521),
                              math.radians(3.5)]

    lst_joint_angles_box32 = [math.radians(-160.343462481),
                              math.radians(-91.0812493213),
                              math.radians(120.346419972),
                              math.radians(-30.735709149),
                              math.radians(19.5479556107),
                              math.radians(3.5)]

    lst_joint_angles_homepose = [math.radians(7.8424263585),
                                 math.radians(-138.164502906),
                                 math.radians(-55.4994262919),
                                 math.radians(-76.2989685681),
                                 math.radians(89.992910236),
                                 math.radians(7.79846884669)]

    # Subscribe ROSTOPIC /ros_iot_bridge/mqtt/sub.
    rospy.Subscriber(ur5._config_mqtt_sub_cb_ros_topic, msgMqttSub, ur5.func_callback_mqtt_messages)

    # Call ROSSERVICE /eyrc/vb/ur5/activate_vacuum_gripper/ur5_1
    vacuum_response = rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1',
                                         vacuumGripper)

    # No of red packages dispatched
    reddispatched = 0

    # No of yellow packages dispatched
    yellowdispatched = 0

    # No of green packages dispatched
    greendispatched = 0

    # No of packages dispatched
    packagedispatched = 0

    # Run until rospy is not shutdown
    while not rospy.is_shutdown():
        incoming_ord = ur5.incoming_orders

        # Check lenght of incoming_orders list.
        if len(incoming_ord) == 0:
            pass

        else:

            # Attribute to chcek ordered item is Medicine or not
            red = False

            # Attribute to chcek ordered item is Foof or not
            yellow = False

            # Attribute to chcek ordered item is Clothes or not
            green = False

            indexord = 0
            # Check incoming_orders is Medicine or not
            for incord in range(len(incoming_ord)):
                if incoming_ord[incord]["item"] == "Medicine":
                    red = True
                    indexord = incord
                    break

            # If item is Medicine
            if red:
                priority = "HP"
                cost = 450

                # Attribute to which package will be dispatched
                box = ur5.red_packages[reddispatched]

                # Attribute to name of saved trajectory yaml file for grab package from shelf
                play_box = "go_to_box" + box[-2:] + "_from_homepose.yaml"

                # Attribute to name of saved trajectory yaml file for dispatch package
                # on conveyor belt
                play_home = "go_to_homepose_from_box" + box[-2:] +".yaml"

                # Attribute to attach which box in Rviz planning scene
                index_of_box = ur5.check_box_number(box)
                reddispatched = reddispatched+1
                print "Red :-  "+str(incoming_ord[indexord])

            # If item is not Medicine
            else:
                # Check Item is Food or not
                for incord in range(len(incoming_ord)):
                    if incoming_ord[incord]["item"] == "Food":
                        yellow = True
                        indexord = incord
                        break

                # If item is Food
                if yellow:
                    priority = "MP"
                    cost = 250
                    box = ur5.yellow_packages[yellowdispatched]
                    play_box = "go_to_box" + box[-2:] + "_from_homepose.yaml"
                    play_home = "go_to_homepose_from_box" + box[-2:] +".yaml"
                    index_of_box = ur5.check_box_number(box)
                    yellowdispatched = yellowdispatched+1
                    print "Yellow :-  "+str(incoming_ord[indexord])

                # If item is not Food
                else:
                    # Check Item is Clothes or not
                    for incord in range(len(incoming_ord)):
                        if incoming_ord[incord]["item"] == "Clothes":
                            green = True
                            indexord = incord
                            break

                    # If item is Clothes
                    if green:
                        priority = "LP"
                        cost = 150
                        box = ur5.green_packages[greendispatched]
                        play_box = "go_to_box" + box[-2:] + "_from_homepose.yaml"
                        play_home = "go_to_homepose_from_box" + box[-2:] +".yaml"
                        index_of_box = ur5.check_box_number(box)
                        greendispatched = greendispatched+1
                        print "Green :-  "+str(incoming_ord[indexord])

            ur5._box_name = ur5.box_list[index_of_box]
            time1 = rospy.get_time()
            # Time taken by these box is low
            if packagedispatched != 0:
                if index_of_box == 2:
                    rospy.sleep(1.5)
                elif index_of_box == 5:
                    rospy.sleep(1)
                elif index_of_box == 6:
                    rospy.sleep(1.2)
                elif index_of_box == 8:
                    rospy.sleep(0.5)
                else:
                    pass

            # Check wheather save trajectry is present in config folder or not
            if index_of_box in range(0, 7):
                ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, play_box, 5)
                ur5.attach_box()
                vacuum_response(True)

                ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, play_home, 5)

            else:
                if index_of_box == 7:
                    ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, play_box, 5)
                elif index_of_box == 8:
                    ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, play_box, 5)
                elif index_of_box == 9:
                    ur5.hard_set_joint_angles(lst_joint_angles_box30, 5)
                elif index_of_box == 10:
                    ur5.hard_set_joint_angles(lst_joint_angles_box31, 5)
                else:
                    ur5.hard_set_joint_angles(lst_joint_angles_box32, 5)

                ur5.attach_box()
                vacuum_response(True)

                if index_of_box in [7, 8]:
                    if index_of_box == 7:
                        ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'box21_middle.yaml', 5)
                    else:
                        ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'box22_middle.yaml', 5)

                    ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, play_home, 5)

                else:
                    rospy.sleep(0.002)
                    jaishreekrishna = ur5._group.get_current_pose().pose
                    ur5.ee_cartesian_translation(0,
                                                0.1,
                                                0.002,
                                                jaishreekrishna.orientation.x,
                                                jaishreekrishna.orientation.y,
                                                jaishreekrishna.orientation.z,
                                                jaishreekrishna.orientation.w)
                    ur5.hard_set_joint_angles(lst_joint_angles_homepose, 5)

            ur5.detach_box()
            vacuum_response(False)

            ur5.belt_response(100)
            ur5.remove_box()
            time2 = rospy.get_time()

            if (time2-time1) > 7.0:

                dict_dispatched_data = {
                    "Order ID" : incoming_ord[indexord]["order_id"],
                    "City" : incoming_ord[indexord]["city"],
                    "Cost" : cost,
                    "Item" : incoming_ord[indexord]["item"],
                    "Priority" : priority,
                    "Dispatch Quantity" : incoming_ord[indexord]["qty"],
                    "Dispatch Status" : "YES",
                    "Dispatch Date and Time" : ur5.get_time_str()
                }
                str_dispatcheddata = json.dumps(dict_dispatched_data)

                msg_dispatched_order = msgDispatchedOrder()
                msg_dispatched_order.message = str_dispatcheddata
                ur5._order_dispatched.publish(msg_dispatched_order)
                del ur5.incoming_orders[indexord]
                packagedispatched = packagedispatched+1

    del ur5


if __name__ == '__main__':
    main()
