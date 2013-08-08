#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from phantom_omni.msg import PhantomButtonEvent
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
feedback_pub = rospy.Publisher('omni_im/feedback', InteractiveMarkerFeedback)
button_clicked = False
feedback_client_id = '/rviz/InteractiveMarkers'

def updateRefs():
    global marker_ref, stylus_ref

    try:
        stylus_ref = listener.lookupTransform('/world', '/stylus', rospy.Time(0))
        marker_ref = listener.lookupTransform('/world', '/marker', rospy.Time(0))
    except:
        pass

def checkFeedback(client_id):
    if client_id != feedback_client_id:
        rospy.logwarn("Different client_id! This could cause feedback to be ignored. i.e., break EVERYTHING.")

# User clicked on menu entry, i.e. 'Omni Control'
def processMenuFeedback(feedback):
    global omni_control
    if feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
        handle = feedback.menu_entry_id
        state = menu_handler.getCheckState(handle)
        if state == MenuHandler.CHECKED:
            menu_handler.setCheckState(handle, MenuHandler.UNCHECKED)
            omni_control = False
        else:
            menu_handler.setCheckState(handle, MenuHandler.CHECKED)
            omni_control = True
        menu_handler.reApply(server)
    checkFeedback(feedback.client_id)
    server.applyChanges()

# Marker moved - just save its new pose
def processMarkerFeedback(feedback):
    global marker_tf
    if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
        marker_tf = pm.toTf(pm.fromMsg(feedback.pose))
    checkFeedback(feedback.client_id)
    server.applyChanges()

def omni_button_callback(button_event):
    global button_clicked
    button_clicked = (button_event.grey_button or button_event.white_button)

# Gets called whenever omni position (joint state) changes
# The idea here is that we publish the omni position to the omni_im feedback topic,
# but only if 'omni_control' is currently selected.
def omni_callback(joint_state):
    global feedback_pub

    if omni_control and button_clicked:
        try:
            # Get pose corresponding to transform between base and proxy.
            p = pm.toMsg(pm.fromTf(listener.lookupTransform('/world', '/proxy', rospy.Time(0))))

            # Construct feedback message.
            feedback = InteractiveMarkerFeedback()
            feedback.pose = p
            feedback.marker_name = 'omni_im'
            feedback.event_type = feedback.POSE_UPDATE
            feedback.client_id = feedback_client_id

            # Publish feedback.
            feedback_pub.publish(feedback)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr("Couldn't look up transform. These things happen...")
    else:
        updateRefs()

def sendTf(transform, target, source):
    global br
    br.sendTransform(transform[0], transform[1], rospy.Time.now(), target, source)

def makeControl(w, x, y, z, name, mode):
    control = InteractiveMarkerControl()
    control.orientation.w = w
    control.orientation.x = x
    control.orientation.y = y
    control.orientation.z = z
    control.name = name
    control.interaction_mode = mode
    return control

def makeMarker():
    # create an interactive marker for our server
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = '/world'
    int_marker.name = 'omni_im'
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

    control = InteractiveMarkerControl()
    control.always_visible = True
    control.interaction_mode = InteractiveMarkerControl.MOVE_ROTATE_3D
    control.markers.append(box_marker)
    int_marker.controls.append(control)

    control = makeControl(1, 1, 0, 0, 'rotate_x', InteractiveMarkerControl.ROTATE_AXIS)
    int_marker.controls.append(control)
    
    control = makeControl(1, 1, 0, 0, 'move_x', InteractiveMarkerControl.MOVE_AXIS)
    int_marker.controls.append(control)
    
    control = makeControl(1, 0, 1, 0, 'rotate_z', InteractiveMarkerControl.ROTATE_AXIS)
    int_marker.controls.append(control)

    control = makeControl(1, 0, 1, 0, 'move_z', InteractiveMarkerControl.MOVE_AXIS)
    int_marker.controls.append(control)

    control = makeControl(1, 0, 0, 1, 'rotate_y', InteractiveMarkerControl.ROTATE_AXIS)
    int_marker.controls.append(control)

    control = makeControl(1, 0, 0, 1, 'move_y', InteractiveMarkerControl.MOVE_AXIS)
    int_marker.controls.append(control)

    menu_control = InteractiveMarkerControl()
    menu_control.name = 'menu_only_control'
    menu_control.interaction_mode = InteractiveMarkerControl.BUTTON
    menu_control.always_visible = True;
    int_marker.controls.append(menu_control)

    server.insert(int_marker, processMarkerFeedback)
    menu_handler.apply(server, int_marker.name)

if __name__=='__main__':
    global marker_tf, zero_tf, marker_ref, stylus_ref

    rospy.init_node('omni_im')

    listener = tf.TransformListener()
    br = tf.TransformBroadcaster()

    zero_tf = ((0, 0, 0), tf.transformations.quaternion_from_euler(0, 0, 0))
    marker_tf = zero_tf
    marker_ref = zero_tf
    stylus_ref = zero_tf

    rospy.Subscriber('omni1_joint_states', JointState, omni_callback)
    rospy.Subscriber('omni1_button', PhantomButtonEvent, omni_button_callback)

    # create an interactive marker server on the topic namespace omni_im
    server = InteractiveMarkerServer('omni_im')

    entry = menu_handler.insert("Omni control", callback=processMenuFeedback)
    menu_handler.setCheckState(entry, MenuHandler.UNCHECKED)
    makeMarker()

    # 'commit' changes and send to all clients
    server.applyChanges()
    
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        sendTf(marker_tf, '/marker', '/world')
        sendTf(zero_tf, '/base', '/world')
        sendTf(marker_ref, '/marker_ref', '/world')
        sendTf(stylus_ref, '/stylus_ref', '/world')
        
        try:
            rel_tf = listener.lookupTransform('/stylus_ref', '/stylus', rospy.Time(0))
            sendTf(rel_tf, '/proxy', '/marker_ref')
        except:
            continue
        rate.sleep()
