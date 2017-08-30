#pragma once
using namespace std;
cv::Mat point_gray_frame(480, 640, CV_8UC4);

int width = 0;
int height = 0;

#pragma pack(1)
struct RectBoundInfo
{
    float center[2]         = {0.0, 0.0}    ;
    float TopLeft[2]        = {0.0, 0.0}    ;
    float BottomRight[2]    = {0.0, 0.0}    ;

    float area              = 0.0;
    float maxarea           = 0.0;
    float BoxWidth          = 0.0;
    float BoxHeight         = 0.0;
    int   maxboxind         = 0;
};
#pragma pack()





