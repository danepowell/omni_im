omni_im
=======

Interactive Marker (IM) Server integrated with the Phantom Omni

This package provides a backend for the interaction_cursor_rviz package, allowing you to control the 3D cursor using the Phantom Omni. Download the omni_im_demo package as well, and simply:

roslaunch omni_im_demo omni_im_demo

Then add the InteractionCursor display to rviz.

Behind the scenes (although visible if you look at the TFs), you'll see that this works by storing reference positions for the marker and stylus whenever you switch to omni control mode. While in omni control, the transform between the stylus reference and actual position is applied to the marker reference position to get the desired marker pose, which is published to the interactive marker feedback topic.

