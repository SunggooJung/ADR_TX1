//Description: For IROS Gate Detection
//Update

#define FOCAL_LENGTH    (float)    (  274.11 )  // /usr/local/zed/settings/sn2914.conf
#define OBJECT_WIDTH    (float)    (    1.35 )

#define VGA_WIDTH   640
#define VGA_HEIGHT  480
#define WIDTH_OFFSET  0
#define HEIGHT_OFFSET 0

#include "Header.h"
struct RectBoundInfo    BndInfo;
int fcc_Flight_Mode;
int gate_num;
int cur_h;

void FccCallback(const std_msgs::Int8MultiArray &msg)
{
    fcc_Flight_Mode	 = msg.data[0];
    gate_num             = msg.data[1];
    cur_h                = msg.data[2];
}



class ImageConverter
{
public:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;    
    image_transport::Publisher fit_pub_;
    ros::Publisher gate_info_pub;
    ros::Subscriber fcc_sub_;

    ImageConverter()
        : it_(nh_)
    {
        // Subscrive to input video feed and publish output video feed
        image_sub_ = it_.subscribe("/firefly_mv/image_raw", 1, &ImageConverter::imageRgb, this);
        gate_info_pub = nh_.advertise<std_msgs::Float32MultiArray>("/gate/pos_info",1);                
        fit_pub_ = it_.advertise("/rgb/detect_result", 1);
        fcc_sub_ = nh_.subscribe("/fcc_info", 20, &FccCallback);
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
        point_gray_frame = cv_ptr->image;
        // Update GUI Window        
        cv::waitKey(3);
    }
};


class Detector {
    public:
    Detector(const string& model_file,
            const string& weights_file,
            const string& mean_file,
            const string& mean_value);

    std::vector<vector<float> > Detect(const cv::Mat& img);

    private:
    void SetMean(const string& mean_file, const string& mean_value);

    void WrapInputLayer(std::vector<cv::Mat>* input_channels);

    void Preprocess(const cv::Mat& img,
                    std::vector<cv::Mat>* input_channels);

    private:
    caffe::shared_ptr<Net<float> > net_;
    cv::Size input_geometry_;
    int num_channels_;
    cv::Mat mean_;
};

Detector::Detector(const string& model_file,
                    const string& weights_file,
                    const string& mean_file,
                    const string& mean_value) {

    Caffe::set_mode(Caffe::GPU);

    /* Load the network. */
    net_.reset(new Net<float>(model_file, TEST));
    net_->CopyTrainedLayersFrom(weights_file);

    CHECK_EQ(net_->num_inputs(), 1) << "Network should have exactly one input.";
    CHECK_EQ(net_->num_outputs(), 1) << "Network should have exactly one output.";

    Blob<float>* input_layer = net_->input_blobs()[0];
    num_channels_ = input_layer->channels();
    CHECK(num_channels_ == 3 || num_channels_ == 1)
    << "Input layer should have 1 or 3 channels.";
    input_geometry_ = cv::Size(input_layer->width(), input_layer->height());

    /* Load the binaryproto mean file. */
//    SetMean(mean_file, mean_value);
}

std::vector<vector<float> > Detector::Detect(const cv::Mat& img) {
    Blob<float>* input_layer = net_->input_blobs()[0];
    input_layer->Reshape(1, num_channels_,
                       input_geometry_.height, input_geometry_.width);
    /* Forward dimension change to all layers. */
    net_->Reshape();

    std::vector<cv::Mat> input_channels;

    WrapInputLayer(&input_channels);

    Preprocess(img, &input_channels);

    net_->Forward();

    /* Copy the output layer to a std::vector */
    Blob<float>* result_blob = net_->output_blobs()[0];
    const float* result = result_blob->cpu_data();
    const int num_det = result_blob->height();
    vector<vector<float> > detections;
    for (int k = 0; k < num_det; ++k) {
        if (result[0] == -1) {
        // Skip invalid detection.
        result += 7;
        continue;
        }
        vector<float> detection(result, result + 7);
        detections.push_back(detection);
        result += 7;
    }
    return detections;
}

/* Load the mean file in binaryproto format. */
void Detector::SetMean(const string& mean_file, const string& mean_value) {
    cv::Scalar channel_mean;
    if (!mean_file.empty()) {
        CHECK(mean_value.empty()) <<
        "Cannot specify mean_file and mean_value at the same time";
        BlobProto blob_proto;
        ReadProtoFromBinaryFileOrDie(mean_file.c_str(), &blob_proto);

        /* Convert from BlobProto to Blob<float> */
        Blob<float> mean_blob;
        mean_blob.FromProto(blob_proto);
        CHECK_EQ(mean_blob.channels(), num_channels_)
            << "Number of channels of mean file doesn't match input layer.";

        /* The format of the mean file is planar 32-bit float BGR or grayscale. */
        std::vector<cv::Mat> channels;
        float* data = mean_blob.mutable_cpu_data();
        for (int i = 0; i < num_channels_; ++i) {
            /* Extract an individual channel. */
            cv::Mat channel(mean_blob.height(), mean_blob.width(), CV_32FC1, data);
            channels.push_back(channel);
            data += mean_blob.height() * mean_blob.width();
        }

        /* Merge the separate channels into a single image. */
        cv::Mat mean;
        cv::merge(channels, mean);

        /* Compute the global mean pixel value and create a mean image
         * filled with this value. */
        channel_mean = cv::mean(mean);
        mean_ = cv::Mat(input_geometry_, mean.type(), channel_mean);
    }
    if (!mean_value.empty()) {
        CHECK(mean_file.empty()) <<
            "Cannot specify mean_file and mean_value at the same time";
        stringstream ss(mean_value);
        vector<float> values;
        string item;
        while (getline(ss, item, ',')) {
            float value = std::atof(item.c_str());
            values.push_back(value);
        }
        CHECK(values.size() == 1 || values.size() == num_channels_) <<
            "Specify either 1 mean_value or as many as channels: " << num_channels_;

        std::vector<cv::Mat> channels;
        for (int i = 0; i < num_channels_; ++i) {
        /* Extract an individual channel. */
            cv::Mat channel(input_geometry_.height, input_geometry_.width, CV_32FC1,
            cv::Scalar(values[i]));
            channels.push_back(channel);
        }
        cv::merge(channels, mean_);
    }
}

/* Wrap the input layer of the network in separate cv::Mat objects
 * (one per channel). This way we save one memcpy operation and we
 * don't need to rely on cudaMemcpy2D. The last preprocessing
 * operation will write the separate channels directly to the input
 * layer. */
void Detector::WrapInputLayer(std::vector<cv::Mat>* input_channels) {
    Blob<float>* input_layer = net_->input_blobs()[0];

    int width = input_layer->width();
    int height = input_layer->height();
    float* input_data = input_layer->mutable_cpu_data();
    for (int i = 0; i < input_layer->channels(); ++i) {
        cv::Mat channel(height, width, CV_32FC1, input_data);
        input_channels->push_back(channel);
        input_data += width * height;
    }
}

void Detector::Preprocess(const cv::Mat& img,
                            std::vector<cv::Mat>* input_channels) {
    /* Convert the input image to the input image format of the network. */
    cv::Mat sample;
    if (img.channels() == 3 && num_channels_ == 1)
        cv::cvtColor(img, sample, cv::COLOR_BGR2GRAY);
    else if (img.channels() == 4 && num_channels_ == 1)
        cv::cvtColor(img, sample, cv::COLOR_BGRA2GRAY);
    else if (img.channels() == 4 && num_channels_ == 3)
        cv::cvtColor(img, sample, cv::COLOR_BGRA2BGR);
    else if (img.channels() == 1 && num_channels_ == 3)
        cv::cvtColor(img, sample, cv::COLOR_GRAY2BGR);
    else
        sample = img;


    cv::Mat sample_resized;
    if (sample.size() != input_geometry_)
        cv::resize(sample, sample_resized, input_geometry_);
    else
        sample_resized = sample;

    cv::Mat sample_float;
    if (num_channels_ == 3)
        sample_resized.convertTo(sample_float, CV_32FC3);
    else
        sample_resized.convertTo(sample_float, CV_32FC1);

    cv::Mat sample_normalized;
    //cv::subtract(sample_float, mean_, sample_normalized);
	sample_normalized = sample_float;

    /* This operation will write the separate BGR planes directly to the
     * input layer of the network because it is wrapped by the cv::Mat
     * objects in input_channels. */
    cv::split(sample_normalized, *input_channels);


    CHECK(reinterpret_cast<float*>(input_channels->at(0).data)
        == net_->input_blobs()[0]->cpu_data())
    << "Input channels are not wrapping the input layer of the network.";
}


int main(int argc, char** argv)
{
//    paths for caffe network inference
    string model_path = "/home/ubuntu/catkin_ws/src/gate_detector/mod_alex_ssd_deploy.prototxt";
    string weights_path = "/home/ubuntu/catkin_ws/src/gate_detector/alx_iter_120000.caffemodel";

    cv::Point pt1;
    cv::Point pt2;

    ros::init(argc, argv, "gate_detector");
    ImageConverter ic;
    ros::Rate loop_rate(20);

    cv::Mat rgb_frame;

    // Initialize the network.
    Detector detector(model_path, weights_path, "", "104,117,123");
    float pose_error = 0.0;
    float distance_to_obs = 0.0;
    while (ros::ok())
    {

        std_msgs::Float32MultiArray msg_gate_pos;
 	rgb_frame = point_gray_frame.clone();

        // run detection
        std::vector<vector<float> > detections = detector.Detect(rgb_frame);

        /* Print the detection results. */
        for (int i = 0; i < detections.size(); ++i)
        {
            const vector<float>& d = detections[i];
            // Detection format: [image_id, label, score, xmin, ymin, xmax, ymax].
            CHECK_EQ(d.size(), 7);
            const float score = d[2];
            if (score >= DETECTION_THRESHOLD)
            {

                pt1.x = d[3] * VGA_WIDTH;
                pt1.y = d[4] * VGA_HEIGHT;
                pt2.x = d[5] * VGA_WIDTH;
                pt2.y = d[6] * VGA_HEIGHT;

 		BndInfo.center[0] = (pt1.x+pt2.x)/2.0;
            	BndInfo.center[1] = (pt1.y+pt2.y)/2.0;

                pose_error = sqrt ( pow(VGA_WIDTH/2+WIDTH_OFFSET - BndInfo.center[0],2) + pow(VGA_HEIGHT/2+HEIGHT_OFFSET - BndInfo.center[1],2) );
                distance_to_obs = (FOCAL_LENGTH * OBJECT_WIDTH) / (fabs(pt1.x - pt2.x));

                // drawing section
                line(rgb_frame, cv::Point(VGA_WIDTH/2+WIDTH_OFFSET,VGA_HEIGHT/2+HEIGHT_OFFSET), cv::Point(BndInfo.center[0],BndInfo.center[1]), cv::Scalar(0, 0, 0), 1, 8, 0);
		circle(rgb_frame, cv::Point(BndInfo.center[0],BndInfo.center[1]), 2, cv::Scalar(255, 0, 0), -1, 8, 0);
            	circle(rgb_frame, cv::Point(640/2+WIDTH_OFFSET, 480/2+HEIGHT_OFFSET), 2, cv::Scalar(0, 0, 255), -1, 8, 0);
                rectangle(rgb_frame,pt1,pt2,cv::Scalar(0,0,255),2,8,0);

                char text_fm[10];
                char text_gate[10];
                char text_curH[10];

                sprintf(text_fm, "FM: %d", fcc_Flight_Mode);
                sprintf(text_gate, "Pass: %d", gate_num);
                sprintf(text_curH, "ALT: %.2f", ((float)cur_h)/10.0);

                cv::putText(rgb_frame, text_fm, cvPoint(20, 20), CV_FONT_HERSHEY_PLAIN, 1.1, cvScalar(255, 0, 0), 2);
                cv::putText(rgb_frame, text_gate, cvPoint(20, 40), CV_FONT_HERSHEY_PLAIN, 1.1, cvScalar(255, 0, 0), 2);
                cv::putText(rgb_frame, text_curH, cvPoint(20, 60), CV_FONT_HERSHEY_PLAIN, 1.1, cvScalar(255, 0, 0), 2);

                msg_gate_pos.data.clear();
                msg_gate_pos.data.resize(4);
                msg_gate_pos.data[0] = (BndInfo.center[0] - (VGA_WIDTH/2+WIDTH_OFFSET));
                msg_gate_pos.data[1] = (BndInfo.center[1] - (VGA_HEIGHT/2+HEIGHT_OFFSET));
                msg_gate_pos.data[2] = distance_to_obs;
                msg_gate_pos.data[3] = pose_error;

                sensor_msgs::ImagePtr fit_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", rgb_frame).toImageMsg();

                ic.gate_info_pub.publish(msg_gate_pos);                
                ic.fit_pub_.publish(fit_msg);
            }
        }
	//DONT TOUCH BELOW HERE 
        ros::spinOnce();
        cv::waitKey(20);
     }
     return 0;
}




