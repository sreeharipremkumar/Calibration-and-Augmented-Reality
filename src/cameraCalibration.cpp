/*
        Made by: Sreehari Premkumar
        MS Robotics Northeastern University

*/
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>

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
        cv::Mat frame;
        cv::Size chess_size(9, 6);
        std::vector<cv::Point2f> corners;
        std::vector<std::vector<cv::Vec3f>> point_list;
        std::vector<std::vector<cv::Point2f>> corner_list;
        int count = 0,min_calibration = 5;


        capdev->set(cv::CAP_PROP_FRAME_WIDTH, 640);
        capdev->set(cv::CAP_PROP_FRAME_HEIGHT, 480);

        for (;;)
        {
                *capdev >> frame; // get a new frame from the camera, treat as a stream
                if (frame.empty())
                {
                        printf("frame is empty\n");
                        break;
                }

                cv::flip(frame,frame,1);
                char key = cv::waitKey(10);
                if (key == 'q')
                {
                        break;
                }
                bool chess_found = cv::findChessboardCorners(frame, chess_size, corners,cv::CALIB_CB_FAST_CHECK);

                if (chess_found)
                {
                        printf("%ld\n",corners.size());
                        printf("Corner %f %f \n",corners[0].x, corners[0].y);

                        cv::Mat gray_image;
                        cv::cvtColor(frame, gray_image, cv::COLOR_BGR2GRAY);
                        cv::cornerSubPix(gray_image, corners, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 25, 0.2));

                        cv::drawChessboardCorners(frame, chess_size, corners, chess_found);

                        cv::imshow("Video", frame);
                }
                else
                {
                        cv::imshow("Video", frame);
                        continue;
                }

                if(key == 's' && chess_found)
                {
                        count++;
                        std::vector<cv::Vec3f> point_set;

                        cv::Mat camera_matrix = cv::Mat::eye(3, 3, CV_64F);
                        camera_matrix.at<double>(0,2) = frame.cols/2;
                        camera_matrix.at<double>(1,2) = frame.rows/2;
                        cv::Mat distortion_coeffs = cv::Mat::zeros(7, 1, CV_64F);
                        std::vector<cv::Mat> rotiation_vecs, translational_vecs;

                        for(int y = 0;y<chess_size.height;y++)
                        {
                                for(int x = 0; x< chess_size.width;x++)
                                {
                                        point_set.push_back(cv::Vec3f(x,y,0));
                                        //printf("%d, %d \n",x,y);
                                }
                        }

                        corner_list.push_back(corners);
                        point_list.push_back(point_set);

                        std::string filename = "../calibration/image_" + std::to_string(count) + ".png";
                        cv::imwrite(filename.c_str(), frame);
                        printf("Image saved \n");

                        if(count>=min_calibration)
                        {
                                double error = cv::calibrateCamera(point_list, corner_list, chess_size, camera_matrix, distortion_coeffs, rotiation_vecs, translational_vecs, cv::CALIB_FIX_ASPECT_RATIO);
                                
                                printf("\nReprojection Error = %f\n",error);
                                cv::FileStorage fs("../calibration/camera_parameters.yaml", cv::FileStorage::WRITE);
                                fs << "camera_matrix" << camera_matrix;
                                fs << "distortion_coeffs" << distortion_coeffs;
                                fs << "rotation_vecs" <<rotiation_vecs;
                                fs << "translational_vecs" <<translational_vecs;
                                fs.release();
                                cv::waitKey(2000);
                        }
                        else{
                                cv::waitKey(1000);
                        }
                }
        }
                delete capdev;
                return (0);
}