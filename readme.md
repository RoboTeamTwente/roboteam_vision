### roboteam_vision

This program receives the messages from the SSL vision program and converts them into ROS messages.
It splits the three types of messages into their own topic.

- DetectionFrame -> vision_detection
- GeometryData -> vision_geometry
- RefboxCmd -> vision_refbox


It keeps a frame counter for every camera.
When a packet has a lower or equal frame number than the frame number in memory it gets dropped.
This negates duplicate frames and old frames arriving after newer frames.
To reset the frame counters, call the `/reset` service, which is an std_srvs/Empty.


### Dependencies

Protobuf


### Services

`/reset`
Call to reset the frame counters.
Also reads in the `our_color` parameter.

### Parameters

`/our_color`
Determines which color is our team.
Valid values are: `blue` and `yellow`.
Defaults to `yellow`.

`/our_field_side`
Determines which of the field halves is ours.
Valid values are: `left` and `right`.
Defaults to `right`.

`/use_legacy_packets`
Switch to true when the vision system you are using sends legacy vision packets.
Boolean.
Defaults to false.
