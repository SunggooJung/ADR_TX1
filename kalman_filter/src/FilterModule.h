#ifndef FilterModule_H
#define FilterModule_H

#include "DefineList.h"
//#include "opencv2/video/tracking.hpp"
//#include "opencv2/highgui.hpp"
using namespace ros;
using namespace cv;
using namespace std;\

struct Odometry_zed     Odom_zed;

int stateSize = 4;
int measSize = 2;
int contrSize = 0;

int zerosize = 0;
int fivesize = 5;
unsigned int type = CV_32F;

cv::KalmanFilter kf(stateSize, measSize, contrSize, type);
cv::Mat state(stateSize, 1, type);  // [x,y,v_x,v_y]
cv::Mat meas(measSize, 1, type);    // [z_x,z_y]

float optflow_x;
float optflow_y;

class Filter_Module
{

public:

    void AltFilter(void);
    void KalmanFilter_Init(void);
    void KalmanFilter(void);

    // kalman fitler


    // Geolocation
    float phi,theta,psi;
    float rho, F, s;
    float u, v;

    float R11,R12,R13;
    float R21,R22,R23;
    float R31,R32,R33;

    float x_tar, y_tar;
    float x_data_pre, y_data_pre;
    float h_ref, h_ref_pre;
    float ticks = 0;

    float alt_cur, alt_pre, alt_vel;
    int   alt_flag;
};

void Filter_Module::KalmanFilter_Init(void)
{

    cv::setIdentity(kf.transitionMatrix);
    cv::setIdentity(kf.processNoiseCov, cv::Scalar(1e-4));
    cv::setIdentity(kf.measurementNoiseCov, cv::Scalar(0.001));

    kf.measurementMatrix = cv::Mat::zeros(measSize, stateSize, type);
    kf.measurementMatrix.at<float>(0) = 1.0;
    kf.measurementMatrix.at<float>(5) = 1.0;

    kf.processNoiseCov.at<float>(10) = 1e-3;
    kf.processNoiseCov.at<float>(15) = 1e-3;

    kf.errorCovPre.at<float>(0) = 1.0; // px
    kf.errorCovPre.at<float>(5) = 1.0; // px
    kf.errorCovPre.at<float>(10) = 1.0;
    kf.errorCovPre.at<float>(15) = 1.0;

    state.at<float>(0) = Odom_zed.x;
    state.at<float>(1) = Odom_zed.y;
    state.at<float>(2) = 0.0;
    state.at<float>(3) = 0.0;

    kf.statePost = state;
    //cout << "transition :" << endl << kf.transitionMatrix << endl;
    //cout << "measure :" << endl << kf.measurementMatrix << endl;
    //cout << "process nos :" << endl << kf.processNoiseCov << endl;
    //cout << "measure nos :" << endl << kf.measurementNoiseCov << endl;
}


void Filter_Module::KalmanFilter(void)
{
    float dT = 1.0/60.0;

    kf.transitionMatrix.at<float>(2) = dT;
    kf.transitionMatrix.at<float>(7) = dT;

    state = kf.predict();
    //cout << "State post:" << endl << state << endl;

    meas.at<float>(0) = Odom_zed.x;
    meas.at<float>(1) = Odom_zed.y;

    kf.correct(meas);
/*
    if(fmod(imdata.t_cur*5,1.0)==0.0)
    {
        //cout << "Measure matrix:" << endl << meas << endl;
        //cout << "state matrix:" << endl << state << endl;
    }
    */
    optflow_x = state.at<float>(2);
    optflow_y = state.at<float>(3);

}

#endif
