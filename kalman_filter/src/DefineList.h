// essential header for ROS-OpenCV operation
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_broadcaster.h>

/// publish message type
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int8MultiArray.h>
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Twist.h"
#include <nav_msgs/Odometry.h>
#include "sensor_msgs/Range.h"

/// Opencv Include
#include "opencv2/opencv.hpp"
#include "opencv2/video/video.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

// for using serial communication
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <math.h>

#pragma pack(1)
struct Odometry_zed// Always Make 40 Bytes
{
    float 		x;
    float 		y;
    float 		z;
};
#pragma pack()

#pragma pack(1)
struct OpticalFlow// Always Make 40 Bytes
{
    float 		x;
    float 		y;
};
#pragma pack()


#pragma pack(1)
struct struct_t_imu// Always Make 40 Bytes
{
    float               r;
    float 		p;
    float 		y;
    float 		ax;
    float 		ay;
    float 		az;
    float               q0;
    float               q1;
    float               q2;
    float               q3;
};
#pragma pack()
