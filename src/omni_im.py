#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from phantom_omni.msg import PhantomButtonEvent
import tf
from interactive_markers.interactive_marker_server import *
import tf_conversions.posemath as pm
from interaction_cursor_msgs.msg import InteractionCursorUpdate
import numpy

marker_name = ''
topic_name = rospy.get_param('~/omni_im/topic_name', '')
fixed_frame = rospy.get_param('~/omni_im/fixed_frame', '/world')
control_frame = rospy.get_param('~/omni_im/control_frame', '/world')
control_offset = rospy.get_param('~/omni_im/control_offset', 3.14159)
# TODO: get rid of marker_pos - can get the true marker position from topic_name/update
marker_offset = rospy.get_param('~/omni_im/marker_offset', 0)
last_button_state = 0

# TODO: get rid of all these globals- use classes instead
listener = None
br = None
update_pub = rospy.Publisher('/interaction_cursor/update', InteractionCursorUpdate)
button_clicked = False
feedback_client_id = '/rviz/InteractiveMarkers'

def updateRefs():
    global marker_ref, stylus_ref
    try:
        stylus_ref = listener.lookupTransform(fixed_frame, '/stylus', rospy.Time(0))
        (control_pos, control_rot) = listener.lookupTransform('/marker', control_frame, rospy.Time(0))
        (marker_pos, marker_rot) = listener.lookupTransform(fixed_frame, '/marker', rospy.Time(0))
        marker_ref = (marker_pos, tf.transformations.quaternion_multiply(control_rot, marker_rot))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rospy.logerr("Couldn't look up transform. These things happen...")

# Marker moved - just save its new pose
def processMarkerFeedback(feedback):
    global marker_tf, marker_name
    marker_name = feedback.marker_name
    marker_offset_tf = ((0, 0, marker_offset), tf.transformations.quaternion_from_euler(0, 0, 0))
    marker_tf = pm.toTf(pm.fromMatrix(numpy.dot(pm.toMatrix(pm.fromMsg(feedback.pose)), pm.toMatrix(pm.fromTf(marker_offset_tf)))))
    if feedback.menu_entry_id:
        marker_tf = zero_tf

def omni_button_callback(button_event):
    global button_clicked
    button_clicked = (button_event.grey_button or button_event.white_button)

# Gets called whenever omni position (joint state) changes
# The idea here is that we publish the omni position to the omni_im feedback topic,
def omni_callback(joint_state):
    global update_pub, last_button_state
    sendTf(marker_tf, '/marker', fixed_frame)
    sendTf(zero_tf, '/base', fixed_frame)
    sendTf(marker_ref, '/marker_ref', fixed_frame)
    sendTf(stylus_ref, '/stylus_ref', fixed_frame)
        
    try:
        update = InteractionCursorUpdate()
        update.pose.header = std_msgs.msg.Header()
        update.pose.header.stamp = rospy.Time.now()
        update.pose.header.frame_id = 'marker_ref'
        if button_clicked and last_button_state == update.GRAB:
            update.button_state = update.KEEP_ALIVE
        elif button_clicked and last_button_state == update.KEEP_ALIVE:
            update.button_state = update.KEEP_ALIVE
        elif button_clicked:
            update.button_state = update.GRAB
        elif last_button_state == update.KEEP_ALIVE:
            update.button_state = update.RELEASE
        else:
            update.button_state = update.NONE
            updateRefs()
        update.key_event = 0

        control_tf = ((0, 0, 0), tf.transformations.quaternion_from_euler(0, 0, control_offset))
        if button_clicked:
            # Get pose corresponding to transform between stylus reference and current position.
            stylus_tf = listener.lookupTransform('/stylus_ref', '/stylus', rospy.Time(0))
            stylus_matrix = pm.toMatrix(pm.fromTf(stylus_tf))
            control_matrix = pm.toMatrix(pm.fromTf(control_tf))
            p = pm.toMsg(pm.fromMatrix(numpy.dot(control_matrix, stylus_matrix)))
        else:
            p = pm.toMsg(pm.fromTf(control_tf))
        
        # Simply scale this up a bit to increase the workspace.
        workspace = 4
        p.position.x = p.position.x * workspace
        p.position.y = p.position.y * workspace
        p.position.z = p.position.z * workspace

        update.pose.pose = p

        last_button_state = update.button_state

        # Publish feedback.
        update_pub.publish(update)
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rospy.logerr("Couldn't look up transform. These things happen...")


def sendTf(transform, target, source):
    global br
    br.sendTransform(transform[0], transform[1], rospy.Time.now(), target, source)

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
    rospy.Subscriber(topic_name + '/feedback', InteractiveMarkerFeedback, processMarkerFeedback)
    
    rospy.spin()
