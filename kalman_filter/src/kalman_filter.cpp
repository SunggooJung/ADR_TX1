///Sensor Measurement Data log For IROS ADR 2017
/// OVERALL COMMON SIGN : X -> roll, aileron   Y -> pitch, elevator   Z-> heave
/// ARRAY SEQUENCE: ENUM { X Y Z }

#include "FilterModule.h"
Filter_Module Filt;

// custom variables
float   count_ros           = 0.0   ;
float   height_m            = 0.0   ;
int fcc_Flight_Mode         =   0   ;
int gate_num                =   0   ;
int cur_h                   =   0   ;


void ZedOdom(const nav_msgs::Odometry& zed_odom_msg)
{
    Odom_zed.x = -zed_odom_msg.pose.pose.position.y; //lateral
    Odom_zed.y = zed_odom_msg.pose.pose.position.z; //longitudinal
    Odom_zed.z = zed_odom_msg.pose.pose.position.x;  //heave
}


void FccCallback(const std_msgs::Int8MultiArray &msg)
{
    fcc_Flight_Mode	 = msg.data[0];
    gate_num             = msg.data[1];
    cur_h                = msg.data[2];
}


int main(int argc, char** argv)
{
    static FILE* pFile;
    char OutFileName[12] = {" "};

    // node name initialization
    init(argc, argv, "data_log");

    // assign node handler
    ros::NodeHandle nh_;

    Subscriber zed_odom_sub_ = nh_.subscribe("/zed/odom", 10, &ZedOdom);
    Publisher  opt_flow_pub = nh_.advertise<std_msgs::Float32MultiArray>("/zedcamera/opt_flow", 10);
    Subscriber fcc_sub_ = nh_.subscribe("/fcc_info", 20, &FccCallback);

    ros::Rate loop_rate(60);

    float fdt = (float)(1/60);

    sprintf(OutFileName,"/home/ubuntu/catkin_ws/src/kalman_filter/src/%s", "filtered_data");
    pFile = fopen(strcat(OutFileName, ".out"), "w+t");

    Filt.KalmanFilter_Init();

    while( ok() )
    {
        float t_cur = (float)count_ros / 60.0;

        std_msgs::Float32MultiArray opt_flow_msg;

        Filt.KalmanFilter();

        //cout << "Xpos: " << Odom_zed.x << "Ypos: " <<Odom_zed.y << "  " <<optflow_x << "  "  << optflow_y << "  " << t_cur<< "\n";
        cout << fcc_Flight_Mode << "\n";
        fprintf(pFile, "%d %d %.4f %.4f %.4f %.4f %.4f\n", fcc_Flight_Mode, gate_num, t_cur, Odom_zed.x, Odom_zed.y, optflow_x, optflow_y);
        /// Optical flow data read
        opt_flow_msg.data.resize(2);
        opt_flow_msg.data[0] = optflow_x;
        opt_flow_msg.data[1] = optflow_y;

	count_ros = count_ros +1 ;
        opt_flow_pub.publish(opt_flow_msg);
        loop_rate.sleep();
        spinOnce();
    }
    return 0;
}
