/// OVERALL COMMON SIGN : X -> roll, aileron   Y -> pitch, elevator   Z-> heave
/// ARRAY SEQUENCE: ENUM { X Y Z }
///
//#define ATT_TEST
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


// for using serial communication
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <pthread.h>
#include "DefineList.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <math.h>

#define Kp_x                         (       0.5)
#define Kd_x                         (       0.1)
#define Kp_y                         (       0.5)
#define Kd_y                         (       0.1)
#define Kp_z                         (       0.8)
#define Kd_z                         (      0.05)

#define Kp_psi                       (         1)
#define HOVER_ALTITUDE               (     -1.50)
#define IMAGE_WIDTH         (float ) (     672.0)
#define IMAGE_HEIGHT        (float ) (     376.0)
#define eps                          ( 0.0000001)
#define AVOIDANCE_VEL       (float ) (       0.3)
#define VEL_FWD             (float ) (       0.3)
#define D2R                 (float ) (3.141592 / 180.0)
#define SONAR_OFFSET        (float ) (       0.0)
#define TAKEOFF_ALT         (float ) (       1.5)
#define K_ZED               (float)  (       3.0)

// Serial Communication initialization
int 	FdPort1;
int     count_ros   = 0;
int     time_flag   = 1;
int     flag_sonar  = 0;
int     count_fwd   = 0;
float   t_capt = 0.0;
float   t_rel = 0.0;
float   t_cur = 0.0;
float   a_param, b_param;

float   Ka = 0.98;
float   Ks = 0.02;
float   dt = 0.05;
float   a = 2.84;

float posX_error = 0.0;
float posY_error = 0.0;
float posZ_error = 0.0;

bool flag_FM            = 0     ;
float Y_cmd             = 2.0   ;
float pos_error_x_m     = 0.0   ;
float cmd_pos_psi       = 0.0   ;
float cmd_pos_z         = 0.0   ;
float 	h;
int wp_num              = 0     ;

char    filename[50];

// Mission control
float WP_x[3] = {0,0,0};
float WP_y[3] = {0,0,0};
float WP_z[3] = {0,0,0};

int OpenSerial(char *device_name);
int CloseSerial(int fd);
void updatedata(void);
void serialsend(int fd);
void *serialreceive(void *fd);
int DS_ParsingMainFuncArgs(int Arg_argc, char ** Arg_argv);
void DS_OnboardLog(void);

struct senddata     	     tx;         
struct struct_t_MessageBody  tx_data;
struct struct_t_RXttyO       StrRXttyO;
struct struct_t_MainFuncArgs StrMainFuncArgs;
struct Odometry_zed          Odom_zed;
struct velocity_command      cmd;


// setup the initial name
using namespace ros;
using namespace std;

// for publishing the data
std_msgs::Float32MultiArray receive_data;

float YawAngleMod(float ArgYawAngle_deg)
{
    float Ret = fmodf(ArgYawAngle_deg, 360.0);

    if(Ret > 180.0)
    {
        Ret -= 360.0;
    }
    return Ret;
}

float sat(float data, float max)
{

    float res;

    if(abs(data) > max)
        res = (data + eps)/abs(data + eps)*max;
    else
        res = data;

    return res;
}

void ZedOdom(const nav_msgs::Odometry& zed_odom_msg)
{
    Odom_zed.x = -zed_odom_msg.pose.pose.position.y;
    Odom_zed.y = zed_odom_msg.pose.pose.position.x;
    Odom_zed.z = zed_odom_msg.pose.pose.position.z;
}

// node main loop, for ROS
int main(int argc, char** argv)
{
    static FILE* pFile;
    char OutFileName[12] = {" "};
    if(DS_ParsingMainFuncArgs(argc, argv) == 1)
	{
		printf("[DONE] Parsing main function arguments\n");
	}
	else
	{
		printf("[ERROR] 'DS_ParsingMainFuncArgs()'\n");
        return -1;
    }

	// node name initialization
	init(argc, argv, "FCC_serial");    

	// assign node handler
	ros::NodeHandle nh_;

	// for debugging
	printf("Initiate: FCC_Serial_node\n");

	// subscribing the image processing results (x_pos, y_pos)
    Subscriber zed_odom_sub_ = nh_.subscribe("/zedstereocam/odom", 1, &ZedOdom);
    Publisher  imu_pub = nh_.advertise<sensor_msgs::Imu>("imu/data_raw", 1000);
    Publisher  pose_down_pub = nh_.advertise<std_msgs::Float32MultiArray>("pose_down/data_raw", 100);
    Publisher  gps_pub = nh_.advertise<std_msgs::Float32MultiArray>("gps/data_raw", 100);
    Publisher  opt_flow_pub = nh_.advertise<geometry_msgs::Twist>("camera/opt_flow", 1);

	receive_data.data.resize(10);

	// setup the loop speed, [Hz], synchronizing the hector slam loop
	ros::Rate loop_rate(20);

	float fdt = (float)(1/20);

	//===== Open Serial =====//
	FdPort1 = OpenSerial(PORT1);

	//===== pthread create =====//
	pthread_t p_thread;
	int thread_rx;

	thread_rx = pthread_create(&p_thread, NULL, serialreceive, (void *)FdPort1);

	if(thread_rx < 0)
	{
		perror("thread create error : ");
		exit(0);
	}

    // node loop, for ROS, check ros status, ros::ok()
    sprintf(OutFileName,"/home/ubuntu/catkin_ws/src/fcc_serial/src/%s", "ODOM");
    pFile = fopen(strcat(OutFileName, ".txt"), "w+t");

    float pos_x_2nd = 0.0;
    float pos_y_2nd = 0.0;
    float Err_Pos_x_int = 0.0;
    float Err_Pos_y_int = 0.0;


	while( ok() )
    {
        /// messages
        sensor_msgs::Imu imu_msg;
        std_msgs::Float32MultiArray pose_down_msg;
        std_msgs::Float32MultiArray gps_msg;
        geometry_msgs::Twist opt_flow_msg;
        std_msgs::Int8MultiArray obs_through_msg;

        /// Imu data read fcc -> Tk1
        imu_msg.header.stamp = ros::Time::now();
        imu_msg.header.frame_id = "imu";

        for (int i = 0; i < 9; i++)
         {
           imu_msg.linear_acceleration_covariance[i] = 0.0;
           imu_msg.angular_velocity_covariance[i] = 0.0;
           imu_msg.orientation_covariance[i] = 0.0;
         }

        imu_msg.orientation.x = StrRXttyO.Cur_Att_deg[1];
        imu_msg.orientation.y = StrRXttyO.Cur_Att_deg[0];
        imu_msg.orientation.z = StrRXttyO.Cur_Att_deg[2];

        imu_msg.angular_velocity.x = StrRXttyO.Cur_AngularRate_dps[1];
        imu_msg.angular_velocity.y = StrRXttyO.Cur_AngularRate_dps[0];
        imu_msg.angular_velocity.z = StrRXttyO.Cur_AngularRate_dps[2];

        imu_msg.linear_acceleration.x = StrRXttyO.Cur_LinAccAED_mpss[1];
        imu_msg.linear_acceleration.y = StrRXttyO.Cur_LinAccAED_mpss[0];
        imu_msg.linear_acceleration.z = StrRXttyO.Cur_LinAccAED_mpss[2];

        /// Lidar altimeter & barometer altitude data read fcc -> tk1
        pose_down_msg.data.clear();
        pose_down_msg.data.resize(3);
        pose_down_msg.data[0] = StrRXttyO.LidarPosDown_m;
        pose_down_msg.data[1] = StrRXttyO.SonarPosDown_m;
        pose_down_msg.data[2] = StrRXttyO.BaroPosDown_m;


        gps_msg.data.clear();
        gps_msg.data.resize(3);
        gps_msg.data[0] = StrRXttyO.GNSS_SatNum;
        gps_msg.data[1] = StrRXttyO.GNSS_hAcc_m;
        gps_msg.data[2] = StrRXttyO.GNSS_vAcc_m;

        t_cur = count_ros / 20.0;

        /// Optical flow data read
        opt_flow_msg.linear.x = StrRXttyO.FlowXY_mps[0];
        opt_flow_msg.linear.y = StrRXttyO.FlowXY_mps[1];

        // complimentary filter with zed odometry and px4flow velocity
        float Err_Pos_x_2nd = (pos_x_2nd - Odom_zed.x*K_ZED);
        Err_Pos_x_int = Err_Pos_x_int + Err_Pos_x_2nd*dt;
        float tempPos_x_2nd = opt_flow_msg.linear.x - Ka*Err_Pos_x_2nd - Ks*Err_Pos_x_int;
        pos_x_2nd = pos_x_2nd + tempPos_x_2nd*dt;
        //pos_x_2nd =  - pos_x_2nd;

        float Err_Pos_y_2nd = (pos_y_2nd - Odom_zed.y);
        Err_Pos_y_int = Err_Pos_y_int + Err_Pos_y_2nd*dt;
        float tempPos_y_2nd = opt_flow_msg.linear.y - Ka*Err_Pos_y_2nd - Ks*Err_Pos_y_int;
        pos_y_2nd = pos_y_2nd + tempPos_y_2nd*dt;

        fprintf(pFile, "%d %.4f %.4f %.4f %.4f %.4f %.4f %.4f %d\n", StrRXttyO.Mode_FlightMode,  sqrt((posX_error)*(posX_error) + (posY_error)*(posY_error)), Odom_zed.x, Odom_zed.y, opt_flow_msg.linear.x, opt_flow_msg.linear.y, pos_x_2nd, pos_y_2nd, wp_num);

        if (StrRXttyO.Mode_FlightMode == 1)
        {
            cout << "\n1. Attitude control Mode" << "\n";
            wp_num = 0;

            WP_x[0] = ( 0.5 ) ;
            WP_x[1] = ( 0.0 ) ;
            WP_x[2] = ( 0.5 ) ;

            WP_y[0] = ( 0.5 ) ;
            WP_y[1] = ( 1.0 ) ;
            WP_y[2] = ( 1.5 ) ;

            WP_z[0] = -1.5          ;
            WP_z[1] = -1.5          ;
            WP_z[2] = -1.5          ;
        }

        if (StrRXttyO.Mode_FlightMode == 2)
        {
            flag_FM = 1;            
            cout << "\n2. Optical Flow Mode" << "\n";
        }

        /////////////////////////////////////MISSION MODE START//////////////////////////////////////////////////////////

        if (StrRXttyO.Mode_FlightMode == 3 && flag_FM == 1)   
        {
            cout << "\n3. Mission Mode" << "\n";
            posX_error = WP_x[wp_num] - pos_x_2nd;
            posY_error = WP_y[wp_num] - pos_y_2nd;
            posZ_error = WP_z[wp_num] - StrRXttyO.LidarPosDown_m;

            cmd.X_out = Kp_x*posX_error - opt_flow_msg.linear.x*Kd_x ;
            cmd.Y_out = Kp_y*posY_error - opt_flow_msg.linear.y*Kd_y ;
            cmd.Z_out = posZ_error*Kp_z;
            cmd.PSI_out = 0.0;


            if ( sqrt((posX_error)*(posX_error) + (posY_error)*(posY_error)) < 0.2)
            {
                    wp_num = wp_num+1;
            }
        }



        updatedata();
        //===== Serial TX part=====//
        serialsend(FdPort1);

        if(StrMainFuncArgs.Flag_Args[0] == 1) // Case of Activating Onboard Logging
		{
			//printf("Debug OnboadLog\n");
			DS_OnboardLog();
        }

		// loop rate [Hz]
		loop_rate.sleep();

        imu_pub.publish(imu_msg);
        pose_down_pub.publish(pose_down_msg);
        gps_pub.publish(gps_msg);
        opt_flow_pub.publish(opt_flow_msg);       

        count_ros = count_ros + 1;

        // loop sampling, ros
        spinOnce();
	}
	// for debugging
	printf("Terminate: FCC_Serial_node\n");
	return 0;
}


void updatedata(void)
{
    /// tx_data update
    tx_data.FlagA   =0;
    tx_data.FlagB   =0;
    tx_data.FlagC   =0;
    tx_data.FlagD   =0;

    tx_data.CmdVelAil = sat(cmd.X_out, 2.0);
    tx_data.CmdVelEle = sat(cmd.Y_out, 2.0);
    tx_data.CmdVelDown = sat(cmd.Z_out, 0.6);
    tx_data.CmdR_dps = sat(cmd.PSI_out, 15);

    unsigned char *data = (unsigned char *)&tx_data;
    memcpy((void *)(tx.Data),(void *)(data),sizeofdata);
}

void *serialreceive(void *fd)
{

	int datasize;
	unsigned char RXRawData[sizeof(StrRXttyO)];

	printf("pthread RX process start!\n");
	while(1)
	{
		int ParsingMode   = 1;
        int ContinueWhile = 1;
		while(ContinueWhile)
		{
			switch(ParsingMode)
			{
			case 1:
				if(read((int)fd, &RXRawData[0], 1) == 1)
				{
					if(RXRawData[0] == 0x12)
					{
						ParsingMode = 2;
					}
					else
					{
						ParsingMode = 1;
					}
				}
				break;

			case 2:
				if(read((int)fd, &RXRawData[1], 1) == 1)
				{
					if(RXRawData[1] == 0x34)
					{
						ParsingMode = 3;
					}
					else
					{
						ParsingMode = 1;
					}
				}
				break;

			case 3:
				if(read((int)fd, &RXRawData[2], 1) == 1)
				{
					if(RXRawData[2] == 0x56)
					{
						ParsingMode = 4;
					}
					else
					{
						ParsingMode = 1;
					}
				}
				break;

			case 4:
				if(read((int)fd, &RXRawData[3], 1) == 1)
				{
					if(RXRawData[3] == 0x78)
					{
						ParsingMode = 5;
					}
					else
					{
						ParsingMode = 1;
					}
				}
				break;

			case 5:
				if(read((int)fd,&RXRawData[4],(sizeof(StrRXttyO)-4)/2)==(sizeof(StrRXttyO)-4)/2)
				{
					if(read((int)fd,&RXRawData[4]+(sizeof(StrRXttyO)-4)/2,(sizeof(StrRXttyO)-4)/2)==(sizeof(StrRXttyO)-4)/2)
					{
						// Calculate Checksum
						unsigned char CalChecksumA = 0;
                        unsigned char CalChecksumB = 0;

						int Ind;

                        for(Ind = 0; Ind<(sizeof(StrRXttyO)-2); Ind++)
						{
							CalChecksumA += RXRawData[Ind];
							CalChecksumB += CalChecksumA;
						}

						if((CalChecksumA == RXRawData[sizeof(StrRXttyO)-2])&&(CalChecksumB == RXRawData[sizeof(StrRXttyO)-1]))
						{
							memcpy((void *)(&StrRXttyO), (void *)(RXRawData), sizeof(StrRXttyO));
							ContinueWhile = 0;
						}
						else
						{
							ParsingMode = 1;
						}
					}
					else
					{
						ParsingMode = 1;
					}
				}
				else
				{
					ParsingMode = 1;
				}
				break;

			default:
				ParsingMode = 1;
				break;
			}
		}
	}

}

void serialsend(int fd)
{
	//===== initial header =====//
	tx.header[0] = header1;
	tx.header[1] = header2;

	tx.IDs[0] = IDs1;
	tx.IDs[1] = IDs2;

	tx.checksum[0] = 0;
	tx.checksum[1] = 0;

	unsigned char *data = (unsigned char *)&tx;

	for(int ind=0; ind<sizeof(senddata)-2;ind++)
	{
		tx.checksum[0] += data[ind];
		tx.checksum[1] += tx.checksum[0];
	}
	//printf("ckA : %d ckB : %d\n",tx.checksum[0],tx.checksum[1]);

	write(fd,&tx,sizeof(senddata));
}


int OpenSerial(char *device_name)
{
	int fd;
	struct termios newtio;

	fd = open(device_name, O_RDWR | O_NOCTTY);

	if(fd < 0)
	{
		printf("Serial Port Open Fail.\n");
		return -1;
	}

	memset(&newtio, 0, sizeof(newtio));
	newtio.c_iflag = IGNPAR;
	newtio.c_oflag = 0;
	newtio.c_cflag = CS8|CLOCAL|CREAD;

	switch(BAUDRATE)
	{
		case 921600 : newtio.c_cflag |= B921600;
		break;
		case 115200 : newtio.c_cflag |= B115200;
		break;
		case 57600  : newtio.c_cflag |= B57600;
		break;
	}

	newtio.c_lflag 		= 0;
	newtio.c_cc[VTIME] 	= 0;
	newtio.c_cc[VMIN] 	= sizeof(StrRXttyO)/2;

	tcflush(fd,TCIFLUSH);
	tcsetattr(fd,TCSANOW, &newtio);

	return fd;
}


int CloseSerial(int fd)
{
	close(fd);
}


int DS_ParsingMainFuncArgs(int Arg_argc, char ** Arg_argv)
{
	// Initialization to set to 0
	memset(&StrMainFuncArgs, 0, sizeof(StrMainFuncArgs));


	// Assign Main Function Arguments
	StrMainFuncArgs.PtrArr_Args[0]  = DF_MAIN_FUNC_ARG_00;
	StrMainFuncArgs.PtrArr_Args[1]  = DF_MAIN_FUNC_ARG_01;
	StrMainFuncArgs.PtrArr_Args[2]  = DF_MAIN_FUNC_ARG_02;
	// ...
	// ...
	// ...
	// ...
	// ...


	// Set Flags
    if(Arg_argc > 2)
	{
		int TempIndA;
        int TempIndB;

		for(TempIndA = 2; TempIndA < Arg_argc; TempIndA++)
		{
			for(TempIndB = 0; TempIndB < DF_MAIN_FUNC_ARGS_MAX; TempIndB++)
			{
				if(strcmp(Arg_argv[TempIndA],StrMainFuncArgs.PtrArr_Args[TempIndB]) == 0)
				{
					StrMainFuncArgs.Flag_Args[TempIndB] = 1;
					break;
				}
			}
		}
	}


	// Get LogFileName
    if(StrMainFuncArgs.Flag_Args[0] == 1) // Case of Activating Onboard Logging
	{
		printf("Type onboard log file name : ");
		scanf("%1023s",&StrMainFuncArgs.OnboardLogFileName[0]);
        sprintf(filename,"/home/ubuntu/%s",StrMainFuncArgs.OnboardLogFileName);
    }
	return 1;
}


void DS_OnboardLog(void)
{
	static int    Flag_Initialized = 0;
	static FILE * FD_ONBOARD_LOG;

	if(Flag_Initialized==0) // Not Initialized
	{
		FD_ONBOARD_LOG = fopen(filename,"wb"); // File Opening
		if(FD_ONBOARD_LOG == NULL) // Open Error
		{
			printf("[ERROR] 'DS_OnboardLog()'\n");
			exit(-1); // Terminate Program
		}
		else // Opening Success
		{
			fclose(FD_ONBOARD_LOG);
			printf("[DONE] Creating onboard log file\n");
			Flag_Initialized = 1; // Initialized
		}
	}

	if(Flag_Initialized==1) // After Initializing
	{
		// Copy Data to Log File
        FD_ONBOARD_LOG = fopen(filename,"ab"); // File Opening with Update Mode
		fwrite(&StrRXttyO, sizeof(StrRXttyO), 1, FD_ONBOARD_LOG);        
		fclose(FD_ONBOARD_LOG);
        cout << StrRXttyO.Cur_Time_sec << "\n";
	}
}




