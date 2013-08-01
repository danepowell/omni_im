omni_im
=======

Interactive Marker Server integrated with the Phantom Omni

This provides interactive marker server nodes, based on those in the interactive marker tutorials package, that allow control of interactive markers using the Phantom Omni.

This is mostly a proof-of-concept package that you would need to adapt for use with an actual robot. Basically, you'd want to copy the callback function into your own project's existing interactive marker server node, and modify it to send feedback to an arbitrary marker, rather than just "my_marker". (Maybe this can be pulled out into a separate package, so you don't even have to modify your existing server? Doubtful, but maybe...)

