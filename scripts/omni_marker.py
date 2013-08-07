#!/usr/bin/env python

import roslib; roslib.load_manifest("interactive_markers")
import rospy
from geometry_msgs.msg import PoseStamped, Pose
from sensor_msgs.msg import JointState
import tf
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *

omni_control = False
listener = None
br = None
server = None
menu_handler = MenuHandler()
feedback_client_id = '/rviz/InteractiveMarkers'

def processFeedback(feedback):
    global omni_control, omni_trans, omni_rot, marker_trans, marker_rot, feedback_client_id
    if feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
        handle = feedback.menu_entry_id
        state = menu_handler.getCheckState(handle)
        if state == MenuHandler.CHECKED:
            menu_handler.setCheckState(handle, MenuHandler.UNCHECKED)
            omni_control = False
        else:
            menu_handler.setCheckState( handle, MenuHandler.CHECKED )
            omni_control = True
            (omni_trans, omni_rot) = listener.lookupTransform('/stylus', '/marker', rospy.Time(0))
            br.sendTransform(omni_trans, omni_rot, rospy.Time.now(), "/proxy", "/stylus")            
        menu_handler.reApply(server)
    elif feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
        p = feedback.pose
        pp = p.position
        po = p.orientation
        marker_trans = (pp.x, pp.y, pp.z)
        marker_rot = (po.x, po.y, po.z, po.w)
    if feedback.client_id != feedback_client_id:
        rospy.logwarn('Different client_id! This could cause feedback to be ignored. i.e., break EVERYTHING.')
    server.applyChanges()

# Gets called whenever omni position (joint state) changes
def omni_callback(joint_state):
    global omni_control, omni_trans, omni_rot, feedback_client_id

    br.sendTransform(omni_trans, omni_rot, rospy.Time.now(), "/proxy", "/stylus")            

    pub = rospy.Publisher('omni_marker/feedback', InteractiveMarkerFeedback)
    
    if omni_control:
        try:
            (trans, rot) = listener.lookupTransform ('/base', '/proxy', rospy.Time(0))
            p = Pose()
            p.position.x = trans[0]
            p.position.y = trans[1]
            p.position.z = trans[2]
            p.orientation.x = rot[0]
            p.orientation.y = rot[1]
            p.orientation.z = rot[2]
            p.orientation.w = rot[3]
            feedback = InteractiveMarkerFeedback()
            feedback.pose = p
            feedback.marker_name = "omni_marker"
            feedback.event_type = feedback.POSE_UPDATE
            feedback.client_id = feedback_client_id
            pub.publish(feedback)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr("Couldn't look up transform. These things happen...")

if __name__=="__main__":
    global omni_trans, omni_rot, marker_trans, marker_rot

    rospy.init_node("omni_marker")

    listener = tf.TransformListener()
    br = tf.TransformBroadcaster()

    omni_trans = (0, 0, 0)
    omni_rot = tf.transformations.quaternion_from_euler(0, 0, 0)
    marker_trans = omni_trans
    marker_rot = omni_rot

    rospy.Subscriber("omni1_joint_states", JointState, omni_callback)
    # create an interactive marker server on the topic namespace omni_marker
    server = InteractiveMarkerServer("omni_marker")

    entry = menu_handler.insert("Omni control", callback=processFeedback)
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

    # add the interactive marker to our collection &
    # tell the server to call processFeedback() when feedback arrives for it
    server.insert(int_marker, processFeedback)

    menu_handler.apply(server, int_marker.name)

    # 'commit' changes and send to all clients
    server.applyChanges()
    
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        br.sendTransform(marker_trans, marker_rot, rospy.Time.now(), "/marker", "/base")
        rate.sleep()
