#include "net/robocup_ssl_client.h"

#include "roboteam_msgs/DetectionFrame.h"
#include "roboteam_msgs/GeometryData.h"


/**
 * Converts the vision unit to meters.
 * millimeters -> meters
 */
float vision_to_m(float scalar); 


/**
 * Converts a protoBuf DetectionBall to the ROS version.
 */
roboteam_msgs::DetectionBall convert_detection_ball(SSL_DetectionBall protoBall); 


/**
 * Converts a protoBuf DetectionRobot to the ROS version.
 */
roboteam_msgs::DetectionRobot convert_detection_robot(SSL_DetectionRobot protoBot); 


/**
 * Converts a protoBuf DetectionFrame to the ROS version.
 */
roboteam_msgs::DetectionFrame convert_detection_frame(SSL_DetectionFrame protoFrame); 


/**
 * Converts a protoBuf FieldLineSegment to the ROS version.
 */
roboteam_msgs::FieldLineSegment convert_geometry_field_line_segment(SSL_FieldLineSegment protoLine); 


/**
 * Converts a protoBuf FieldCicularArc to the ROS version.
 */
roboteam_msgs::FieldCicularArc convert_geometry_field_Cicular_arc(SSL_FieldCicularArc protoArc); 


/**
 * Converts a protoBuf GeometryFieldSize to the ROS version.
 */
roboteam_msgs::GeometryFieldSize convert_geometry_field_size(SSL_GeometryFieldSize protoSize); 


/**
 * Converts a protoBuf GeometryCameraCalibration to the ROS version.
 */
roboteam_msgs::GeometryCameraCalibration convert_geometry_camera_calibration(SSL_GeometryCameraCalibration protoCal); 


/**
 * Converts a protoBuf GeometryData to the ROS version.
 */
roboteam_msgs::GeometryData convert_geometry_data(SSL_GeometryData protoData); 
