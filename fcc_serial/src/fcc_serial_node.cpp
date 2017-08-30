/// OVERALL COMMON SIGN : X -> roll, aileron   Y -> pitch, elevator   Z-> heave
/// ARRAY SEQUENCE: ENUM { X Y Z }
///
#include "DefineList.h"

#define Kp_x                         (       0.4)
#define Kd_x                         (       0.0)
#define Kp_y                         (       0.4)
#define Kd_y                         (       0.1)
#define Kp_z                         (       0.8)
#define Kd_z                         (      0.05)
#define Kp_psi                       (         1)

#define eps                          ( 0.0000001)
#define D2R                 (float ) (3.141592 / 180.0)
#define SECOND                       (        20)


bool    flag_roll          = 1  ;
bool    flag_FM            = 0  ;
int 	FdPort1                 ;
// Serial Communication initialization
int     time_flag   = 1         ;
int     gate_num = 0            ;
float   Y_cmd             = 2.0 ;
float   pos_error_x_m     = 0.0 ;
float   pos_error_y_m     = 0.0 ;
float   cmd_pos_x         = 0.0 ;
float   cmd_pos_y         = 0.0 ;
float   cmd_pos_z         = 0.0 ;
float   cmd_pos_psi       = 0.0 ;
float 	h                       ;
float   r_data                  ;
float   a_param, b_param        ;
char    filename[50]            ;
float height_m = 0.0;

// Mission control
float WP_x[3] = {0,0,0};
float WP_y[3] = {0,0,0};
float WP_z[3] = {0,0,0};
float WP_psi[3] = {0,0,0};

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
struct OpticalFlow_zed       opt_flow;
struct Image_error     	     img;
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

void OpticalFlow(const geometry_msgs::Twist& opt_flow_msg)
{
    opt_flow.x = opt_flow_msg.linear.x;
    opt_flow.y = opt_flow_msg.linear.y;
}

void ZedOdom(const nav_msgs::Odometry& zed_odom_msg)
{
    Odom_zed.x = -zed_odom_msg.pose.pose.position.y; //lateral
    Odom_zed.y = -zed_odom_msg.pose.pose.position.x; //longitudinal
    Odom_zed.z = zed_odom_msg.pose.pose.position.z;  //heave
}

void Lidar(const sensor_msgs::Range &Lidar_Height_msg)
{
    height_m = Lidar_Height_msg.range;
}

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
        Subscriber zed_odom_sub_ = nh_.subscribe("/zed/odom", 1, &ZedOdom);
        Subscriber opt_flow_sub_ = nh_.subscribe("/camera/opt_flow", 1, &OpticalFlow);
        Subscriber Lidar_sub_ = nh_.subscribe("/terarangerone", 1, &Lidar);        
        Publisher  fcc_info_pub = nh_.advertise<std_msgs::Int8MultiArray>("/fcc_info", 20);

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

	while( ok() )
        {
            std_msgs::Int8MultiArray fcc_info_msg;

            fcc_info_msg.data.clear();
            fcc_info_msg.data.resize(4);
            fcc_info_msg.data[0] = StrRXttyO.Mode_FlightMode;
            fcc_info_msg.data[1] = gate_num;
            fcc_info_msg.data[2] = height_m;

            if (StrRXttyO.Mode_FlightMode == 1)
            {
                cout << "\n1. Attitude control Mode" << "\n";
                gate_num = 0;


                WP_psi[0] = ( 0.0 + 0.0 ) ;
                WP_psi[1] = ( WP_psi[0] + 0.0 ) ;
                WP_psi[2] = ( WP_psi[0] + 0.0 ) ;

                WP_x[0] = 0.0       ;
                WP_x[1] = 0.0       ;
                WP_x[2] = 0.0       ;

                WP_y[0] = 0.0       ;
                WP_y[1] = 0.0       ;
                WP_y[2] = 0.0       ;

                WP_z[0] = 1.65          ;
                WP_z[1] = 2.15          ;
                WP_z[2] = 1.65          ;
            }

            if (StrRXttyO.Mode_FlightMode == 2)
            {
                flag_FM = 1;
                cout << "\n2. Optical Flow Mode" << "\n";
            }

            if (StrRXttyO.Mode_FlightMode >= 3 && flag_FM == 1)
            {
                cout << "\n3. Mission Mode" << "\n";

                cmd_pos_psi = WP_psi[gate_num];
                cmd_pos_z   = WP_z[gate_num]  ;
                cmd_pos_x   = WP_x[gate_num]  ;
                cmd_pos_y   = WP_y[gate_num]  ;

                float psi_error     = YawAngleMod(cmd_pos_psi - StrRXttyO.Cur_Att_deg[2]);
                float posZ_error    = cmd_pos_z - height_m;
                float pos_error_x_m = cmd_pos_x - Odom_zed.x;
                float pos_error_y_m = cmd_pos_y - Odom_zed.y;

                cmd.X_out = Kp_x*pos_error_x_m - StrRXttyO.FlowXY_mps[0]*Kd_x ;
                cmd.Y_out = Kp_y*pos_error_y_m - StrRXttyO.FlowXY_mps[1]*Kd_y ;
                cmd.Z_out = -1.0*Kp_z*posZ_error;
                cmd.PSI_out = Kp_psi*psi_error*1.0;

		cout << "x:   "   <<  Odom_zed.x << "   y:     " << Odom_zed.y <<  "  h:   " << height_m << "\n";

             }


            updatedata();
            //===== Serial TX part=====//
            serialsend(FdPort1);

            if(StrMainFuncArgs.Flag_Args[0] == 1) // Case of Activating Onboard Logging
                    {
                            //printf("Debug OnboadLog\n");
                            DS_OnboardLog();
            }

            fcc_info_pub.publish(fcc_info_msg);


            // loop rate [Hz]
            loop_rate.sleep();
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
    tx_data.FlagA        =0; // Flag for auto takeoff/landing/motor-cut, etc...
    tx_data.Flag_Sensing =0; // [1] = flow from high-level computer (TX1, 2, etc...)
    tx_data.FlagC        =0; // N/A
    tx_data.FlagD        =0; // N/A

    tx_data.CmdVelAil = sat(cmd.X_out, 2.0);
    tx_data.CmdVelEle = sat(cmd.Y_out, 1.0);
    tx_data.CmdVelDown = sat(cmd.Z_out, 4.0);
    tx_data.CmdR_dps = sat(cmd.PSI_out, 15);

    tx_data.Cur_FlowAilEle_mps[0] = opt_flow.x;
    tx_data.Cur_FlowAilEle_mps[1] = opt_flow.y;


    unsigned char *data = (unsigned char *)&tx_data;
    memcpy((void *)(tx.Data),(void *)(data),sizeofdata);
}

void *serialreceive(void *fdt)
{
	int fd = *((int*) &fdt);
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




