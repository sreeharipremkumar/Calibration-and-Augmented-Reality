/*
        Made by: Sreehari Premkumar
        MS Robotics Northeastern University

*/
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <vector>
#include <math.h>

#include <opencv2/opencv.hpp>
#include <vector>
#include <cmath>

int createIcosahedron(std::vector<cv::Point3f>& vertices, std::vector<cv::Vec3i>& edges, float radius) {
     float phi = (1 + sqrt(5)) / 2;
     float a = radius / sqrt(phi * phi + 1);
     float b = a * phi;
    vertices.clear();
    vertices.reserve(12);
    vertices.emplace_back(-a, 0, b);
    vertices.emplace_back(a, 0, b);
    vertices.emplace_back(-a, 0, -b);
    vertices.emplace_back(a, 0, -b);
    vertices.emplace_back(0, b, a);
    vertices.emplace_back(0, b, -a);
    vertices.emplace_back(0, -b, a);
    vertices.emplace_back(0, -b, -a);
    vertices.emplace_back(b, a, 0);
    vertices.emplace_back(-b, a, 0);
    vertices.emplace_back(b, -a, 0);
    vertices.emplace_back(-b, -a, 0);
    edges.clear();
    edges.reserve(30);
    edges.emplace_back(1, 4, 0);
    edges.emplace_back(1, 6, 0);
    edges.emplace_back(1, 6, 10);
    edges.emplace_back(1, 10, 8);
    edges.emplace_back(1, 8, 4);
    edges.emplace_back(0, 4, 9);
    edges.emplace_back(2, 9, 5);
    edges.emplace_back(2, 5, 3);
    edges.emplace_back(2, 3, 7);
    edges.emplace_back(2, 11, 7);
    edges.emplace_back(2, 9, 11);
    edges.emplace_back(3, 5, 8);
    edges.emplace_back(4, 8, 5);
    edges.emplace_back(4, 5, 9);
    edges.emplace_back(7, 10, 6);
    edges.emplace_back(6, 11, 7);
    edges.emplace_back(7, 3, 10);
    edges.emplace_back(3, 8, 10);
    edges.emplace_back(0, 11, 6);    
    edges.emplace_back(0,9, 11);

    return 0;
}

int rotateTranslate(std::vector<cv::Point3f>& point_set, int rotate,cv::Point3f &translate,int rt)
{
    double angle = rotate * CV_PI / 180.0;
    
    // Create the rotation matrix
    cv::Mat rotationMatrix = cv::Mat::zeros(3, 3, CV_64FC1);
    rotationMatrix.at<double>(0, 0) = cos(angle);
    rotationMatrix.at<double>(0, 1) = -sin(angle);
    rotationMatrix.at<double>(1, 0) = sin(angle);
    rotationMatrix.at<double>(1, 1) = cos(angle);
    rotationMatrix.at<double>(2, 2) = 1.0;

    cv::Mat translate_mat = cv::Mat(translate);
    translate_mat.convertTo(translate_mat, CV_64FC1);
    
    // Apply the rotation to each point in the point cloud
    for (auto &point : point_set)
    {
        cv::Mat pointMat = cv::Mat(point);
        pointMat.convertTo(pointMat, CV_64FC1);
        cv::Mat rotatedPointMat;

        if(rt ==1)
        {
            rotatedPointMat = rotationMatrix * pointMat + translate_mat;
        }
        else if(rt ==2)
        {
            rotatedPointMat = pointMat + translate_mat;
            rotatedPointMat = rotationMatrix * rotatedPointMat;
            rotatedPointMat = rotatedPointMat + translate_mat;
        }
        // cv::add(rotatedPointMat,translate_mat,rotatedPointMat);
        // rotatedPointMat.copyTo(pointMat);
        point.x = rotatedPointMat.at<double>(0, 0);
        point.y = rotatedPointMat.at<double>(1, 0);
        point.z = rotatedPointMat.at<double>(2, 0);
    }
    return 0;
}


int drawIcosahedron(cv::Mat& img,std::vector<cv::Point3f>& vertices,  std::vector<cv::Vec3i>& edges,  cv::Mat& cameraMatrix,  cv::Mat& distCoeffs,  cv::Mat& rvec,  cv::Mat& tvec, cv::Scalar color) {
    std::vector<cv::Point2f> imagePoints;

    projectPoints(vertices, rvec, tvec, cameraMatrix, distCoeffs, imagePoints);
    for ( auto& edge : edges) {
        line(img, imagePoints[edge[0]], imagePoints[edge[1]], color, 2);
        line(img, imagePoints[edge[1]], imagePoints[edge[2]], color, 2);
        line(img, imagePoints[edge[2]], imagePoints[edge[0]], color, 2);
    }
    return 0;
}

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
        int rotate_angle = 0;

        //std::vector<std::vector<cv::Vec3f>> point_list;
        //std::vector<std::vector<cv::Point2f>> corner_list;

        cv::Mat camera_matrix,distortion_coeffs;
        std::vector<cv::Mat> rotation_vecs, translational_vecs;

        cv::FileStorage fs("../calibration/camera_parameters.yaml", cv::FileStorage::READ);

        if(!fs.isOpened()){
            printf("Failed to access taml file");
        }

        fs["camera_matrix"]>> camera_matrix;
        fs["distortion_coeffs"] >> distortion_coeffs;
        fs["rotation_vecs"] >> rotation_vecs;
        fs["translational_vecs"] >> translational_vecs;
        fs.release();

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
                        std::vector<cv::Vec3f> point_set;
                        cv::Mat gray_image,rotation,translation;
                        cv::cvtColor(frame, gray_image, cv::COLOR_BGR2GRAY);
                        cv::cornerSubPix(gray_image, corners, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 25, 0.2));

                        //cv::drawChessboardCorners(frame, chess_size, corners, chess_found);

                        for(int y = 0;y<chess_size.height;y++)
                        {
                                for(int x = 0; x< chess_size.width;x++)
                                {
                                        point_set.push_back(cv::Vec3f(x,y,0));
                                }
                        }

                        cv::solvePnP(cv::Mat(point_set), cv::Mat(corners), camera_matrix, distortion_coeffs, rotation, translation);

                        // std::cout << "Rotation: " << rotation<< std::endl;
                        // std::cout << "Translation: " << translation << std::endl;

                        std::vector<cv::Point2f> point_img;

                        // for(int i=0;i<point_set.size();i++)
                        // {
                        //     std::cout << point_set[i] <<std::endl;
                        // }
                        // break;
                        std::vector<cv::Point3f> cornor_points(4);
                        cornor_points[0] = cv::Point3f(0, 0, 0);
                        cornor_points[1] = cv::Point3f(0, chess_size.height-1, 0);
                        cornor_points[2] = cv::Point3f(chess_size.width-1, chess_size.height-1, 0);
                        cornor_points[3] = cv::Point3f(chess_size.width-1, 0, 0);

                        cv::projectPoints(cornor_points, rotation, translation, camera_matrix, distortion_coeffs, point_img);

                        // add 3D axis to the origin
                        cv::Mat axis = cv::Mat::zeros(3, 3, CV_64FC1);
                        axis.at<double>(0, 0) = 0.1;  // length of x-axis
                        axis.at<double>(1, 1) = 0.1;  // length of y-axis
                        axis.at<double>(2, 2) = -0.1; // length of z-axis
                        cv::drawFrameAxes(frame, camera_matrix, distortion_coeffs, rotation, translation, 2, 5);

                        cv::Scalar color(0, 0, 128);

                        for(int i =0;i<point_img.size();i++)
                        {
                            cv::line(frame, point_img[i], point_img[(i+1)%point_img.size()], color, 2);
                        }
                        
                        vector<cv::Point3f> vertices1,vertices2;
                        vector<cv::Vec3i> edges1,edges2;

                        //Trying to imitate planet
                        createIcosahedron(vertices1, edges1, 2);
                        cv::Point3f translate = cv::Point3f(4,4,-5);
                        rotateTranslate(vertices1,rotate_angle,translate,1);
                        rotate_angle = (rotate_angle + 1)%360; 
                        drawIcosahedron(frame, vertices1, edges1, camera_matrix, distortion_coeffs, rotation, translation, cv::Scalar(255, 255, 255));
                        

                        //Imitate moon

                        createIcosahedron(vertices1, edges1, 1);

                        translate = cv::Point3f(4,4,-2.5);
                        rotateTranslate(vertices1,4*rotate_angle,translate,2);
                        rotate_angle = (rotate_angle + 1)%360; 
                        drawIcosahedron(frame, vertices1, edges1, camera_matrix, distortion_coeffs, rotation, translation, cv::Scalar(255, 255, 255));
                        

                        cv::imshow("Video", frame);

                }
                else
                {
                    cv::imshow("Video", frame);
                    continue;
                }
        }
                delete capdev;
                return (0);
}