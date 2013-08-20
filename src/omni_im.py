#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from phantom_omni.msg import PhantomButtonEvent
import tf
from interactive_markers.interactive_marker_server import *
import tf_conversions.posemath as pm
import interaction_cursor_msgs

marker_name = ''
topic_name = rospy.get_param('~/omni_im/topic_name', '')

# todo: get rid of all these globals- use classes instead
listener = None
br = None
update_pub = rospy.Publisher('/interaction_cursor/update', InteractiveMarkerFeedback)
button_clicked = False
feedback_client_id = '/rviz/InteractiveMarkers'

def updateRefs():
    global marker_ref, stylus_ref
    try:
        stylus_ref = listener.lookupTransform('/world', '/stylus', rospy.Time(0))
        marker_ref = listener.lookupTransform('/world', '/marker', rospy.Time(0))
    except:
        pass

# Marker moved - just save its new pose
def processMarkerFeedback(feedback):
    global marker_tf, marker_name
    marker_name = feedback.marker_name
    marker_tf = pm.toTf(pm.fromMsg(feedback.pose))

def omni_button_callback(button_event):
    global button_clicked
    button_clicked = (button_event.grey_button or button_event.white_button)

# Gets called whenever omni position (joint state) changes
# The idea here is that we publish the omni position to the omni_im feedback topic,
def omni_callback(joint_state):
    global update_pub
    if button_clicked:
        try:
            # Get pose corresponding to transform between base and proxy.
            p = pm.toMsg(pm.fromTf(listener.lookupTransform('/world', '/proxy', rospy.Time(0))))

            update = InteractionCursorUpdate()
            update.pose = p
            update.button_state = update.KEEP_ALIVE
            update.key_event = update.NONE

            # Publish feedback.
            update_pub.publish(update)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr("Couldn't look up transform. These things happen...")
    else:
        updateRefs()

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
