#!/usr/bin/python
# Software License Agreement (BSD License)
#
# Copyright (c) 2014, PAL Robotics SL
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the PAL Robotics nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# author: Bence Magyar
# author: Sammy Pfeiffer

import rospy
from actionlib import SimpleActionClient
from geometry_msgs.msg import PoseArray, Pose, Point, PoseStamped
from std_msgs.msg import Header
from moveit_simple_grasps.msg import GenerateGraspsAction, GenerateGraspsGoal, GraspGeneratorOptions
from moveit_msgs.msg import Grasp
from visualization_msgs.msg import MarkerArray, Marker
import copy

grasp_publisher = None
text_publisher = None
grasps_ac = None

MOVEIT_SIMPLE_GRASPS_AS = '/moveit_simple_grasps_server/generate'
GENERATED_GRASPS_TOPIC = 'generated_grasps'
GENERATED_GRASPS_TEXT_TOPIC = 'generated_grasps_text'


def generate_grasps(pose, width, options):
    """Send the request to the grasp generator with the options given"""
    grasps_ac.wait_for_server()
    rospy.loginfo("Successfully connected.")
    goal = GenerateGraspsGoal()
    goal.pose = pose.pose
    goal.width = width
    goal.options.extend(options)
    grasps_ac.send_goal(goal)
    rospy.loginfo("Sent goal, waiting:\n" + str(goal))
    t_start = rospy.Time.now()
    grasps_ac.wait_for_result()
    t_end = rospy.Time.now()
    t_total = t_end - t_start
    rospy.loginfo("Result received in " + str(t_total.to_sec()))
    grasp_list = grasps_ac.get_result().grasps
    return grasp_list


def publish_grasps_as_poses(grasps):
    rospy.loginfo("Publishing PoseArray on /grasp_pose_from_block_bla for grasp_pose")
    graspmsg = Grasp()
    grasp_PA = PoseArray()
    header = Header()
    header.frame_id = "base_link"
    header.stamp = rospy.Time.now()
    grasp_PA.header = header
    for graspmsg in grasps:
        p = Pose(graspmsg.grasp_pose.pose.position, graspmsg.grasp_pose.pose.orientation)
        grasp_PA.poses.append(p)
    # Publish twice as Rviz sometimes does not get the first publication
    grasp_publisher.publish(grasp_PA)
    rospy.sleep(0.1)
    grasp_publisher.publish(grasp_PA)
    rospy.loginfo('Published ' + str(len(grasp_PA.poses)) + ' poses')


def create_text_marker(text, text_pose, text_id):
    m = Marker()
    m.header.frame_id = copy.deepcopy(text_pose.header.frame_id)
    m.pose = copy.deepcopy(text_pose.pose)
    m.pose.position.z += 0.4
    m.id = text_id
    m.type = Marker.TEXT_VIEW_FACING
    m.action = Marker.ADD
    m.text = text
    m.scale.z = 0.07
    m.color.r = 0.5
    m.color.g = 0.5
    m.color.a = 1.0
    return m


def create_grasp_options(axis, direction, rotation):
    """Create a GraspGeneratorOptions msg with the input specified in a friendly way.
    i.e. axis = X,Y,Z direction = up, down, rotation = half, full"""
    opts = GraspGeneratorOptions()
    if   "X" in axis:
        opts.grasp_axis = GraspGeneratorOptions.GRASP_AXIS_X
    elif "Y" in axis:
        opts.grasp_axis = GraspGeneratorOptions.GRASP_AXIS_Y
    elif "Z" in axis:
        opts.grasp_axis = GraspGeneratorOptions.GRASP_AXIS_Z

    if   "UP" in direction:
        opts.grasp_direction = GraspGeneratorOptions.GRASP_DIRECTION_UP
    elif "DOWN" in direction:
        opts.grasp_direction = GraspGeneratorOptions.GRASP_DIRECTION_DOWN

    if "HALF" in rotation:
        opts.grasp_rotation = GraspGeneratorOptions.GRASP_ROTATION_HALF
    elif "FULL" in rotation:
        opts.grasp_rotation = GraspGeneratorOptions.GRASP_ROTATION_FULL
    return opts


def do_something_between_tests():
    """Placeholder function to do something between tests, to sleep, wait for keyboard input... whatever"""
    rospy.sleep(1)


def generate_and_publish_grasps(pose, width, options):
    """Generate grasps and publish in rviz the generated grasps"""
    if type(options) != type([]):  # if it's just one options element
        print "We got an options which is just one element, not a list or tuple, fixing it"
        g_options = []
        g_options.append(options)
    else:
        g_options = options
    grasp_list = generate_grasps(pose, width, g_options)
    publish_grasps_as_poses(grasp_list)

    return grasp_list


def run_tests(tests, obj_pose, width, step=-0.5):
    """Given a list of lists of strings representing what to test, run it's tests
    tests should look like [("X", "DOWN", "HALF"), ("Y", "UP", "FULL")]
    obj_pose will be the starting pose to represent them in rviz advancing every step distance"""
    total_grasp_list = []
    markers_poses_list = []
    markers_pose = copy.deepcopy(obj_pose)
    texts_ma = MarkerArray()
    idx = 0
    for test in tests:
        curr_grasp_opts = create_grasp_options(*test)
        grasp_list = generate_and_publish_grasps(markers_pose, width, curr_grasp_opts)
        markers_poses_list.append(markers_pose)
        total_grasp_list.extend(grasp_list)
        text_to_show = ""
        for word in test:
            text_to_show += word + " "
        t_marker = create_text_marker(text_to_show, markers_pose, idx)
        texts_ma.markers.append(t_marker)
        markers_pose = copy.deepcopy(markers_pose)
        markers_pose.pose.position.y += step
        idx += 1

    # print all the markers together:
    publish_grasps_as_poses(total_grasp_list)
    text_publisher.publish(texts_ma)
    # publish text twice because rviz sometimes misses updates
    rospy.sleep(0.5)
    text_publisher.publish(texts_ma)

if __name__ == '__main__':
    name = 'grasp_object_server_test'
    rospy.init_node(name, anonymous=False)
    rospy.loginfo("Connecting to grasp generator AS '" + MOVEIT_SIMPLE_GRASPS_AS + "'")
    grasps_ac = SimpleActionClient(MOVEIT_SIMPLE_GRASPS_AS, GenerateGraspsAction)
    grasp_publisher = rospy.Publisher(GENERATED_GRASPS_TOPIC, PoseArray)
    text_publisher = rospy.Publisher(GENERATED_GRASPS_TEXT_TOPIC, MarkerArray)

    # Create PoseStamped to start generation from
    object_pose = PoseStamped()
    object_pose.pose.position.x = 0.5
    object_pose.pose.position.y = -0.5
    object_pose.pose.position.z = 1.0
    object_pose.pose.orientation.w = 1.0
    object_pose.pose.orientation.x = 0.0
    object_pose.pose.orientation.y = 0.0
    object_pose.pose.orientation.z = 0.0
    object_pose.header.frame_id = "base_link"

    # Also the imaginary object width
    object_width = 0.06

    tests_to_run = []

    # All down and half options
    tests_to_run.append(("X", "DOWN", "HALF"))
    tests_to_run.append(("Y", "DOWN", "HALF"))
    tests_to_run.append(("Z", "DOWN", "HALF"))

    # All up and half options
    tests_to_run.append(("X", "UP", "HALF"))
    tests_to_run.append(("Y", "UP", "HALF"))
    tests_to_run.append(("Z", "UP", "HALF"))

    # All down and full options
    tests_to_run.append(("X", "DOWN", "FULL"))
    tests_to_run.append(("Y", "DOWN", "FULL"))
    tests_to_run.append(("Z", "DOWN", "FULL"))

    # All UP and full options
    tests_to_run.append(("X", "UP", "FULL"))
    tests_to_run.append(("Y", "UP", "FULL"))
    tests_to_run.append(("Z", "UP", "FULL"))

    run_tests(tests_to_run, object_pose, object_width)

    # TODO: options with angle steps, when implemented

    # TODO: options with NEGATIVE axis (-X, -Y, -Z), , when implemented
