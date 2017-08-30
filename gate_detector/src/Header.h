#pragma once

//ROS include
#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int8MultiArray.h>
#include <geometry_msgs/Twist.h>


//OpenCV include
#include "opencv2/core/core.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>


//CAFFE include
#include "caffe/caffe.hpp"
#include <caffe/proto/caffe.pb.h>


//STANDARD include
#include <stdlib.h>
#include <time.h>
#include <iostream>
#include <vector>
#include <stdio.h>
#include <math.h>

#include <stdint.h>

// User Include
#include "Gvar.h"

#define DETECTION_THRESHOLD 0.7

using namespace caffe;
using std::string;
using namespace std;



