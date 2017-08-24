#include "Header.h"

//using namespace std::chrono;

cv::Mat raw_frame(480, 640, CV_8UC4);
int glob_count = 0;
Point2f point;
bool addRemovePt = false;

struct timeval tp;
// [m/pixel] = (0.000272754-0.000107189)/(125-50)([height(m)] - 0.5) + 0.000272754
// [m/pixel] = 0.0002207537574*[height(m)] + 0.0001623769610
class ImageConverter
{
public:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    ros::Publisher opt_flow_pub_;

    ImageConverter()
        : it_(nh_)
    {
        // Subscrive to input video feed and publish output video feed
        image_sub_ = it_.subscribe("/zed/rgb/image_raw_color", 1, &ImageConverter::imageRgb, this);
        opt_flow_pub_ = nh_.advertise<geometry_msgs::Twist>("/camera/opt_flow", 1);
    }

    ~ImageConverter()
    {

    }

    void imageRgb(const sensor_msgs::ImageConstPtr& msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        raw_frame = cv_ptr->image;
        glob_count++;
        // Update GUI Window
        cv::waitKey(3);
    }
};

int main(int argc, char** argv)
{

    ros::init(argc, argv, "optical_flow");
    ImageConverter ic;
    ros::Rate loop_rate(30);
    
    static FILE* pFile;
    char OutFileName[200];
    char save_flag = 0;
    sprintf(OutFileName,"/home/ubuntu/catkin_ws/src/optical_flow/%s", "OpticalFlowLog");
    pFile = fopen(strcat(OutFileName, ".out"), "w+t");
    fprintf(pFile,"Vx:Vy:dt:(cm/s,cm/s,msec)\n");


    cv::Mat image, prevGray, gray;

    cv::TermCriteria termcrit(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.03);
    cv::Size subPixWinSize(10, 10), winSize(31, 31);

    const int MAX_COUNT = 100;
    bool needToInit = true;
    bool nightMode = false;

    //namedWindow("LK Demo", 1);

    vector<Point2f> points[2];
    int pre_glob_count = -1;  
  
    gettimeofday(&tp, NULL);
    long int ms1 = tp.tv_sec * 1000 + tp.tv_usec / 1000, duration;
    float height_m;
    float pos_x = 0, pos_y = 0;
    //milliseconds ms = duration_cast< milliseconds >(system_clock::now().time_since_epoch());
    while( ros::ok() )
    {
        /// messages
        geometry_msgs::Twist opt_flow_msg;

        if(pre_glob_count != glob_count){

            gettimeofday(&tp, NULL);
            duration = tp.tv_sec * 1000 + tp.tv_usec / 1000 - ms1;
            ms1 = tp.tv_sec * 1000 + tp.tv_usec / 1000;
            //ms = duration_cast< milliseconds >(system_clock::now().time_since_epoch());
            //printf("%+7ld\n", duration);

            resize(raw_frame,image,cv::Point(256,144));
            //imshow("LK Demo", image);

            cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
            //gray.convertTo(gray, CV_32FC1); gray = gray/255;
            //imshow("LK Demo gray", gray); printf("gray type : %d\n", gray.type());
            if (nightMode)
                image = Scalar::all(0);
            int min_size = (points[0].size() < points[1].size()) ? points[0].size() : points[1].size();

            if (needToInit || min_size < 50)
            {
                // automatic initialization
                goodFeaturesToTrack(gray, points[1], MAX_COUNT, 0.01, 10, cv::Mat(), 3, 0, 0.04);
                if(!points[1].empty()){
                    cornerSubPix(gray, points[1], subPixWinSize, Size(-1, -1), termcrit);
                    addRemovePt = false;
                    needToInit = false;
                }
            }
            else if (!points[0].empty())
            {
                vector<uchar> status;
                vector<float> err;
                if (prevGray.empty())
                    gray.copyTo(prevGray);
                calcOpticalFlowPyrLK(prevGray, gray, points[0], points[1], status, err, winSize, 3, termcrit, 0, 0.001);
                size_t i, k;
                for (i = k = 0; i < points[1].size(); i++)
                {
                    if (addRemovePt)
                    {
                        if (norm(point - points[1][i]) <= 5)
                        {
                            addRemovePt = false;
                            continue;
                        }
                    }

                    if (!status[i])
                        continue;

                    points[1][k++] = points[1][i];
                    circle(image, points[1][i], 3, Scalar(0, 255, 0), -1, 8);
                }
                points[1].resize(k);
            }

            min_size = (points[0].size() < points[1].size()) ? points[0].size() : points[1].size();

            //if ( min_size < 50)		needToInit = true;

            float dx = 0;
            float dy = 0;
            for (int i = 0 ; i < min_size ; i++){
                circle(image, points[0][i], 3, Scalar(0, 0, 255), -1, 8); // pre
                circle(image, points[1][i], 3, Scalar(0, 255, 0), -1, 8); // cur
                // calc flow of each features
                dx += points[1][i].x - points[0][i].x;
                dy += points[1][i].y - points[0][i].y;
            }
            height_m = 0.77; // [m]
            float v_x = 1000.0 * (dx / (float)min_size) * (0.0002207537574 * height_m + 0.0001623769610) / (float)duration;
            float v_y = 1000.0 * (dy / (float)min_size) * (0.0002207537574 * height_m + 0.0001623769610) / (float)duration;
            pos_x += (v_x);
            pos_y += (v_y);

            if(save_flag != 0 ){
                fprintf(pFile,"%5.2f:%5.2f:%4ld\n",100*v_x, 100*v_y, duration);
            }
            else{
                printf("Vx = %+5.2f, Vy = %+5.2f, Px = %+5.2f, Py = %+5.2f, dx = %+5.2f, dy = %+5.2f, dt[msec] = %4ld\n", 100*v_x, 100*v_y, 100*pos_x, 100*pos_y, dx / min_size, dy / min_size, duration);
            }

            /// Optical flow data read
            opt_flow_msg.linear.x = v_x;
            opt_flow_msg.linear.y = v_y;

            if (addRemovePt && points[1].size() < (size_t)MAX_COUNT)
            {
                vector<Point2f> tmp;
                tmp.push_back(point);
                cornerSubPix(gray, tmp, winSize, cvSize(-1, -1), termcrit);
                points[1].push_back(tmp[0]);
                addRemovePt = false;
            }


            needToInit = false;
            imshow("LK Demo", image);

            std::swap(points[1], points[0]);
            cv::swap(prevGray, gray);
            pre_glob_count = glob_count;
        }        else{            pre_glob_count = glob_count;        }


        char c = (char)waitKey(10);
        if (c == 27){
            break;
        }
        else if( c == 'r'){
            needToInit = true;
        }
        else if( c == 'c'){
            points[0].clear();
            points[1].clear();
        }
        else if( c == 'n'){
            nightMode = !nightMode;
        }
        else if( c == 's'){
            if (save_flag == 0 ) save_flag = 1;
            else if (save_flag == 1 ) save_flag = 0;
            printf("save_flag : %d\n", save_flag);
        }

        ic.opt_flow_pub_.publish(opt_flow_msg);
        cv::waitKey( 20 );
        ros::spinOnce();

    }

    fclose(pFile);

    return 0;
}

