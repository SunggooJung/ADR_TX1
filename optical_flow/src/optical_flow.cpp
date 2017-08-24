#include "Header.h"

float height_m = 0.0;

cv::Mat raw_frame(480, 640, CV_8UC4);
int glob_count = 0;
Point2f point;
bool addRemovePt = false;

struct timeval tp;

void Lidar(const sensor_msgs::Range &Lidar_Height_msg)
{
    height_m = Lidar_Height_msg.range;
}


class ImageConverter
{
public:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    ros::Subscriber Lidar_sub_;
    ros::Publisher opt_flow_pub_;
    image_transport::Publisher opt_img_pub_;

    ImageConverter()
        : it_(nh_)
    {
        // Subscrive to input video feed and publish output video feed
        image_sub_ = it_.subscribe("/zed/rgb/image_raw_color", 1, &ImageConverter::imageRgb, this);
        Lidar_sub_ = nh_.subscribe("/terarangerone", 1, &Lidar);
        opt_flow_pub_ = nh_.advertise<geometry_msgs::Twist>("/camera/opt_flow", 1);
        opt_img_pub_ = it_.advertise("/opt_flow/image_flow", 1);
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

    cv::Mat image, prevGray, gray;

    cv::TermCriteria termcrit(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.03);
    cv::Size subPixWinSize(10, 10), winSize(31, 31);

    const int MAX_COUNT = 100;
    bool needToInit = true;
    bool nightMode = false;

    vector<Point2f> points[2];
    int pre_glob_count = -1;  
  
    gettimeofday(&tp, NULL);
    long int ms1 = tp.tv_sec * 1000 + tp.tv_usec / 1000, duration;    
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

            resize(raw_frame,image,cv::Point(256,144));

            cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
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

            float dx = 0;
            float dy = 0;
            for (int i = 0 ; i < min_size ; i++){
                circle(image, points[0][i], 1, Scalar(0, 0, 255), -1, 8); // pre
                circle(image, points[1][i], 1, Scalar(0, 255, 0), -1, 8); // cur
                // calc flow of each features
                dx += points[0][i].x - points[1][i].x;
                dy += points[1][i].y - points[0][i].y;
            }

            float v_x = 1000.0 * (dx / (float)min_size) * (0.0005505673060 * height_m + 0.0004058533918) / (float)duration;
            float v_y = 1000.0 * (dy / (float)min_size) * (0.0007376041302 * height_m + 0.0005413763299) / (float)duration;
            pos_x += (v_x);
            pos_y += (v_y);

            /// Optical flow data read
            opt_flow_msg.linear.x = v_x;
            opt_flow_msg.linear.y = v_y;
            cout << height_m << "    " << opt_flow_msg.linear.x<<"\n";
            sensor_msgs::ImagePtr opt_img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
            ic.opt_flow_pub_.publish(opt_flow_msg);            
            ic.opt_img_pub_.publish(opt_img_msg);

            if (addRemovePt && points[1].size() < (size_t)MAX_COUNT)
            {
                vector<Point2f> tmp;
                tmp.push_back(point);
                cornerSubPix(gray, tmp, winSize, cvSize(-1, -1), termcrit);
                points[1].push_back(tmp[0]);
                addRemovePt = false;
            }


            needToInit = false;

            std::swap(points[1], points[0]);
            cv::swap(prevGray, gray);
            pre_glob_count = glob_count;
        }        else{            pre_glob_count = glob_count;        }

        ros::spinOnce();
        cv::waitKey( 20 );
    }

    return 0;
}

