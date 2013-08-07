omni_im
=======

Interactive Marker Server integrated with the Phantom Omni

This provides interactive marker server nodes, based on those in the interactive marker tutorials package, that allow control of interactive markers using the Phantom Omni.

This is mostly a proof-of-concept package that you would need to adapt for use with an actual robot. Basically, you'd want to copy the callback functions into your own project's existing interactive marker server node, and modify them to send feedback to an arbitrary marker, rather than just "omni_im". (Maybe this can be pulled out into a separate package, so you don't even have to modify your existing server? Doubtful, but maybe...)

To launch: roslaunch omni_im omni_im.launch
This will launch rviz, add the Interactive Marker display, the whole #!.

How it works:
The launch file runs four nodes: the phantom omni and associated robot state publisher, omni_im.py, and rviz.

Of course, omni_im is the guts of the operation. It subscribes to the omni's joint state topic. If the omni moves, and the user has currently selected omni control mode, the subscriber callback publishes feedback to the interactive marker feedback topic, based on the omni's position. This is what causes the marker to move (yay!).

There's also a menu handler callback- this just keeps track of whether the user has selected the omni control mode or not.

Finally, there's a interactive marker feedback callback. This just keeps track of the marker's pose, storing it as a tf. The main thread constantly publishes this tf (from the omni base to the marker position).