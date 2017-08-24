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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <math.h>

#define Kx_h          0.005
#define Ky_h 	      0.005
#define Kx_l          0.003
#define Ky_l          0.003
#define h_ref	      3
#define vz_max        0.7
#define vz_landing    0.2

#define Ref_H         2

#define ximg_center   320
#define yimg_center   240
#define sizeofdata    40

#define header1       0x12
#define header2       0x34
#define IDs1          0x01
#define IDs2          0x01

#define PORT1 		"/dev/ttyUSB0"
#define BAUDRATE 	921600

// Main Function Argument
#define DF_MAIN_FUNC_ARGS_MAX   64
#define DF_MAIN_FUNC_ARG_00     "aLOG"
#define DF_MAIN_FUNC_ARG_01     "aLOGODOM"
#define DF_MAIN_FUNC_ARG_02     "pGNC"


#pragma pack(1)
struct senddata
{
	unsigned char   header[2];
	unsigned char 	IDs[2];
	unsigned char 	Data[40];
	unsigned char   checksum[2];
};
#pragma pack()

#pragma pack(1)
struct struct_t_MessageBody // Always Make 40 Bytes
{
	unsigned char   FlagA;
        unsigned char   Flag_Sensing;
	unsigned char   FlagC;
	unsigned char   FlagD;
	float 		CmdVelAil;
	float 		CmdVelEle;
	float 		CmdVelDown;
	float           CmdR_dps;
        float 		Cur_FlowAilEle_mps[2];

	uint16_t PWM_Ch_09_14_PulseWidth_usec[6];
};
#pragma pack()



#pragma pack(1)
struct Odometry_zed// Always Make 40 Bytes
{
    float 		x;
    float 		y;
    float 		z;
};
#pragma pack()



#pragma pack(1)
struct OpticalFlow_zed// Always Make 40 Bytes
{
    float 		x;
    float 		y;
};
#pragma pack()


#pragma pack(1)
struct Image_error// Always Make 40 Bytes
{
    float 		pos_error_pixel[6];
};
#pragma pack()



#pragma pack(1)
struct velocity_command// Always Make 40 Bytes
{
    float X_out;
    float Y_out;
    float Z_out;
    float PSI_out;
    float GIMBAL_out;

};
#pragma pack()




#pragma pack(1)
struct struct_t_RXttyO
{

	// -------------
	// Packet Header
	// -------------
	uint8_t HeaderA; // 0x12;
	uint8_t HeaderB; // 0x34;
	uint8_t HeaderC; // 0x56;
	uint8_t HeaderD; // 0x78;


	// ----
	// Time
	// ----
	float Cur_Time_sec;


	// -----
	// Flags
	// -----
	int8_t UsedSBUSNum;
	int8_t Mode_MotorAct;
	int8_t Mode_FlightMode;
	int8_t Mode_VehicleFlyingStatus;


	// ---------------
	// ADC Information
	// ---------------
	float ADC_volt[6];


	// ----------------
	// GNSS Information
	// ----------------
	float  GNSS_SatNum;
	float  GNSS_hAcc_m;
	float  GNSS_vAcc_m;
	double GNSS_iTOW_msec;
	double GNSS_ECEF_m[3];
	double GNSS_LLH_degdegm[3];


	// ---------------------------------------
	// PosDown Information (Lidar & Barometer)
	// ---------------------------------------
	float LidarPosDown_m;
    float LidarPosRaw_m;
	float BaroPosDown_m;


	// ----------------------------------
	// Optical Flow Information (PX4Flow)
	// ----------------------------------
	float FlowXY_mps[2];
	float SonarPosDown_m;
    float FlowQuality;


	// --------------------------
	// Transmitter Stick Position
	// --------------------------
	float TXStickPosAil_usec;
	float TXStickPosEle_usec;
	float TXStickPosThr_usec;
	float TXStickPosRud_usec;
    float TX_CH_8_9_13_14_15_16_usec[6];


	// ----------------------
	// Controller Information
	// ----------------------
	// Angular Velocity
	float Cmd_R_dps;
	float Cur_AngularRate_dps[3];
	// Attitude
    float Cmd_Att_deg[3];
	float Cur_Att_deg[3];
	// Linear Acceleration, AED
	float Cmd_LinAccAED_mpss[3];
	float Cur_LinAccAED_mpss[3];
	// Velocity, AED
	float Cmd_VelAED_mps[3];
	float Cur_VelAED_mps[3];


	// ---------
	// Actuation
	// ---------
	// Motor Actuation
	float ThrottleTrim_usec;
	float ThrottleOut_usec;
	float MotorPWM_usec[8];


	// -----------
	// Flight Path
	// -----------
	// NED Position
	float Cur_PosNED_m[3];
	// NED Velocity
	float Cur_VelNED_mps[3];


	// ------------------
	// Sensor Calibration
	// ------------------
	// Gyro Calibration
	float GyroTemper_C;
	float GyroRawXYZ_dps[3];
	// Accel Calibration
	float SpeciAccRawXYZ_mpss[3];
	// Magnetometer
	float MagRawXYZ_mG[3];


	// -------------
	// Padding Bytes
	// -------------
	uint8_t PaddingA;
	uint8_t PaddingB;


	// --------------
	// Checksum Bytes
	// --------------
	uint8_t ChecksumA;
	uint8_t ChecksumB;

};
#pragma pack()

struct struct_t_MainFuncArgs
{
	const char *  PtrArr_Args[DF_MAIN_FUNC_ARGS_MAX];
	unsigned char Flag_Args[DF_MAIN_FUNC_ARGS_MAX];
	char          OnboardLogFileName[1025];
};

