/*
        Made by: Sreehari Premkumar
        MS Robotics Northeastern University

*/

#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>

using namespace cv;

int main(int argc, char *argv[])
{
    cv::VideoCapture *capdev;

    // open the video device
    // std::string url = "http://10.0.0.53:4747/video";
    // capdev = new cv::VideoCapture(url,cv::CAP_FFMPEG);
    capdev = new cv::VideoCapture(0); // use this instead if webcam
    if (!capdev->isOpened())
    {
        printf("Unable to open video device\n");
        return (-1);
    }
    using namespace std;

    // get some properties of the image
    cv::Size refS((int)capdev->get(cv::CAP_PROP_FRAME_WIDTH),
                  (int)capdev->get(cv::CAP_PROP_FRAME_HEIGHT));
    printf("Video Size: %d %d\n", refS.width, refS.height);

    cv::namedWindow("Video", 1); // identifies a window

    capdev->set(cv::CAP_PROP_FRAME_WIDTH, 640);
    capdev->set(cv::CAP_PROP_FRAME_HEIGHT, 480);

    cv::Mat frame;

    for (;;)
    {
        *capdev >> frame; // get a new frame from the camera, treat as a stream
        if (frame.empty())
        {
            printf("frame is empty\n");
            break;
        }

        cv::flip(frame,frame,1);

        //Template for Surf detector from opencv website
        Ptr<cv::xfeatures2d::SURF> detector = xfeatures2d::SURF::create(1000);

        // Detecting keypoints and computing descriptors
        std::vector<KeyPoint> keypoints;
        Mat descriptors;
        detector->detectAndCompute(frame, Mat(), keypoints, descriptors);

        // Drawing keypoints on image
        Mat img_keypoints;
        drawKeypoints(frame, keypoints, img_keypoints);

        imshow("Video", img_keypoints);
        char key = waitKey(10);

        if(key == 'q')
        {
            break;
        }
    }
    delete capdev;
    return (0);
}