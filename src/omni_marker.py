#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
import tf
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
import tf_conversions.posemath as pm

# todo: get rid of all these globals- use classes instead
omni_control = False
listener = None
br = None
server = None
menu_handler = MenuHandler()
feedback_client_id = '/rviz/InteractiveMarkers'
feedback_pub = rospy.Publisher('omni_marker/feedback', InteractiveMarkerFeedback)

# User clicked on menu entry, i.e. 'Omni Control'
def processMenuFeedback(feedback):
    global omni_control, omni_tf, feedback_client_id
    if feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
        handle = feedback.menu_entry_id
        state = menu_handler.getCheckState(handle)
        if state == MenuHandler.CHECKED:
            menu_handler.setCheckState(handle, MenuHandler.UNCHECKED)
            omni_control = False
        else:
            menu_handler.setCheckState(handle, MenuHandler.CHECKED)
            omni_control = True
            omni_tf = listener.lookupTransform('/stylus', '/marker', rospy.Time(0))
            br.sendTransform(omni_tf[0], omni_tf[1], rospy.Time.now(), "/proxy", "/stylus")            
        menu_handler.reApply(server)
    if feedback.client_id != feedback_client_id:
        rospy.logwarn('Different client_id! This could cause feedback to be ignored. i.e., break EVERYTHING.')
    server.applyChanges()

# Marker moved - just save its new pose
def processMarkerFeedback(feedback):
    global marker_tf, feedback_client_id
    if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
        marker_tf = pm.toTf(pm.fromMsg(feedback.pose))
    if feedback.client_id != feedback_client_id:
        rospy.logwarn('Different client_id! This could cause feedback to be ignored. i.e., break EVERYTHING.')
    server.applyChanges()

# Gets called whenever omni position (joint state) changes
# The idea here is that we publish the omni position to the omni_marker feedback topic,
# but only if 'omni_control' is currently selected.
def omni_callback(joint_state):
    global omni_control, omni_tf, feedback_client_id, feedback_pub

    br.sendTransform(omni_tf[0], omni_tf[1], rospy.Time.now(), "/proxy", "/stylus")            

    if omni_control:
        try:
            # Get pose corresponding to transform between base and proxy.
            p = pm.toMsg(pm.fromTf(listener.lookupTransform ('/base', '/proxy', rospy.Time(0))))

            # Construct feedback message.
            feedback = InteractiveMarkerFeedback()
            feedback.pose = p
            feedback.marker_name = "omni_marker"
            feedback.event_type = feedback.POSE_UPDATE
            feedback.client_id = feedback_client_id

            # Publish feedback.
            feedback_pub.publish(feedback)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr("Couldn't look up transform. These things happen...")

if __name__=="__main__":
    global omni_tf, omni_tf, marker_tf

    rospy.init_node("omni_marker")

    listener = tf.TransformListener()
    br = tf.TransformBroadcaster()
    
    omni_tf = ((0, 0, 0), tf.transformations.quaternion_from_euler(0, 0, 0))
    marker_tf = omni_tf

    rospy.Subscriber("omni1_joint_states", JointState, omni_callback)
    # create an interactive marker server on the topic namespace omni_marker
    server = InteractiveMarkerServer("omni_marker")

    entry = menu_handler.insert("Omni control", callback=processMenuFeedback)
    menu_handler.setCheckState(entry, MenuHandler.UNCHECKED)

    # create an interactive marker for our server
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "/base"
    int_marker.name = "omni_marker"
    int_marker.description = "Phantom Omni Control"
    int_marker.scale = 0.1

    # create a grey box marker
    box_marker = Marker()
    box_marker.type = Marker.CUBE
    box_marker.scale.x = 0.045
    box_marker.scale.y = 0.045
    box_marker.scale.z = 0.045
    box_marker.color.r = 0.0
    box_marker.color.g = 0.5
    box_marker.color.b = 0.5
    box_marker.color.a = 1.0

    # create a non-interactive control which contains the box
    box_control = InteractiveMarkerControl()
    box_control.always_visible = True
    box_control.markers.append( box_marker )
    # add the control to the interactive marker
    int_marker.controls.append( box_control )

    # create a control which will move the box
    # this control does not contain any markers,  
    # which will cause RViz to insert two arrows
    rotate_control = InteractiveMarkerControl()
    rotate_control.name = "move_x"
    rotate_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    # add the control to the interactive marker
    int_marker.controls.append(rotate_control)

    menu_control = InteractiveMarkerControl()
    menu_control.name = "menu_only_control"
    menu_control.interaction_mode = InteractiveMarkerControl.BUTTON
    menu_control.always_visible = True;
    int_marker.controls.append(menu_control)

    server.insert(int_marker, processMarkerFeedback)

    menu_handler.apply(server, int_marker.name)

    # 'commit' changes and send to all clients
    server.applyChanges()
    
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        br.sendTransform(marker_tf[0], marker_tf[1], rospy.Time.now(), "/marker", "/base")
        rate.sleep()
