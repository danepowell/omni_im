omni_im
=======

Interactive Marker (IM) Server integrated with the Phantom Omni

This provides a node (omni_im.py) that, when given the name of an IM feedback topic, will allow the user to control markers on that topic using a Phantom Omni.

All you have to do is click on a marker to select it, and then press a button on the Omni to begin moving it (this is a 'clutch' system.)

Behind the scenes (although visible if you look at the TFs), you'll see that this works by storing reference positions for the marker and stylus whenever you switch to omni control mode. While in omni control, the transform between the stylus reference and actual position is applied to the marker reference position to get the desired marker pose, which is published to the interactive marker feedback topic.

To try it out, download omni_im_demo, and simply:
roslaunch omni_im_demo omni_im_demo.launch

