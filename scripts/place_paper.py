#!/usr/bin/env python

import roslib; roslib.load_manifest("interactive_markers")
import rospy

import tf

br = tf.TransformBroadcaster()

from interactive_markers.interactive_marker_server import *

server = None

frame_pose = None

def processFeedback(feedback):
    global frame_pose
    p = feedback.pose.position
    o = feedback.pose.orientation
    frame_pose = feedback.pose

def a4_sheet():

    w = 0.21
    h = 0.297

    sheet = Marker()
    sheet.pose.orientation.w = 1.0
    sheet.pose.position.z = -.0005
    sheet.pose.position.x = w/2
    sheet.pose.position.y = h/2
    sheet.id = 99
    sheet.type = Marker.CUBE
    sheet.scale.x = w
    sheet.scale.y = h
    sheet.scale.z = 0.0005
    sheet.color.b = 1.0
    sheet.color.g = 1.0
    sheet.color.r = 1.0
    sheet.color.a = 1.0

    return sheet


def make6DofMarker( fixed = False ):
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "/map"
    int_marker.scale = 0.05

    int_marker.pose.position.x =  -0.100988589227
    int_marker.pose.position.y =   0.035845387727
    int_marker.pose.position.z =   0.266128748655

    int_marker.name = "paper_sheet"
    int_marker.description = "Place the sheet of paper"

    # create a grey box marker
    box_marker = Marker()
    box_marker.type = Marker.CUBE
    box_marker.scale.x = 0.21
    box_marker.scale.y = 0.297
    box_marker.scale.z = 0.001
    box_marker.color.r = 1.0
    box_marker.color.g = 1.0
    box_marker.color.b = 1.0
    box_marker.color.a = 1.0

    # create a non-interactive control which contains the box
    box_control = InteractiveMarkerControl()
    box_control.always_visible = True
    box_control.markers.append( a4_sheet() )

    # add the control to the interactive marker
    int_marker.controls.append( box_control )

    if fixed:
        int_marker.name += "_fixed"
        int_marker.description += "\n(fixed orientation)"

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 1
    control.orientation.y = 0
    control.orientation.z = 0
    control.name = "rotate_x"
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    if fixed:
        control.orientation_mode = InteractiveMarkerControl.FIXED
    int_marker.controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 1
    control.orientation.y = 0
    control.orientation.z = 0
    control.name = "move_x"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    if fixed:
        control.orientation_mode = InteractiveMarkerControl.FIXED
    int_marker.controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 1
    control.orientation.z = 0
    control.name = "rotate_z"
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    if fixed:
        control.orientation_mode = InteractiveMarkerControl.FIXED
    int_marker.controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 1
    control.orientation.z = 0
    control.name = "move_z"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    if fixed:
        control.orientation_mode = InteractiveMarkerControl.FIXED
    int_marker.controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 0
    control.orientation.z = 1
    control.name = "rotate_y"
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    if fixed:
        control.orientation_mode = InteractiveMarkerControl.FIXED
    int_marker.controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 0
    control.orientation.z = 1
    control.name = "move_y"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    if fixed:
        control.orientation_mode = InteractiveMarkerControl.FIXED
    int_marker.controls.append(control)

    server.insert(int_marker, processFeedback)

if __name__=="__main__":
    rospy.init_node("paper_placer")
    
    server = InteractiveMarkerServer("paper_placer")

    make6DofMarker(fixed = True)

    # 'commit' changes and send to all clients
    server.applyChanges()

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        if frame_pose:
            p = frame_pose.position
            o = frame_pose.orientation
            br.sendTransform((p.x, p.y, p.z),
                            (o.x, o.y, o.z, o.w),
                            rospy.Time.now(),
                            "paper_sheet",
                            "map")
        rate.sleep()
