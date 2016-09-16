### roboteam_vision

This program receives the messages from the SSL vision program and converts them into ROS messages.
It splits the three types of messages into their own topic.

- DetectionFrame -> vision_detection
- GeometryData -> vision_geometry
- RefboxCmd -> vision_refbox


It keeps a frame counter for every camera.
When a packet has a lower or equal frame number than the frame number in memory it gets dropped.
This negates duplicate frames and old frames arriving after newer frames.
To reset the frame counters, call the `/reset` service, which is an std_srvs/Empty..
