omni_im
=======

Interactive Marker (IM) Server integrated with the Phantom Omni

This package provides a backend for the interaction_cursor_rviz package, allowing you to control the 3D cursor using the Phantom Omni. Download the omni_im_demo package as well, and simply:

roslaunch omni_im_demo omni_im_demo

Then add the InteractionCursor display to rviz.

To see it in action on an actual robot, you can also download the pr2_dane package.

Parameters:
* /omni_im/topic_name (default null): The base topic name for the interactive markers you wish to control with the omni
* /omni_im/fixed_frame (default '/world'): A fixed 'world' frame.
* /omni_im/control_frame (default '/world'): A frame that determines the 'perspective' for the omni controls. It should roughly match the camera's position in Rviz.
* /omni_im/control_offset (default PI): The z-rotation for the Phantom omni control frame. Usually PI if your control frame roughly matches the camera frame.
* /omni_im/marker_offset (default 0): If the marker and controls aren't actually centered in the marker frame, use this offset to adjust the z-position of the interaction cursor.

Behind the scenes (although visible if you look at the TFs), you'll see that this works by storing reference positions for the marker and stylus whenever you switch to omni control mode. While in omni control, the transform between the stylus reference and actual position is applied to the marker reference position to get the desired marker pose, which is published to the interactive marker feedback topic.

