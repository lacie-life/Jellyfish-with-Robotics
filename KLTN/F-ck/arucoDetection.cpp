#include <iostream>
#include <eigen3/Eigen/Eigen>
#include <math.h>
#include <stdio.h>
#include <stdarg.h>
#include <string>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <librealsense2/rs.hpp>

using namespace cv;
using namespace std;

enum class CameraSide { LEFT, RIGHT };

#define PI 3.14159265

cv::Mat lmapx, lmapy, rmapx, rmapy;

cv::Mat K1 = (cv::Mat_<double>(3,3) << 284.910186767578, 0.0, 418.319610595703,
                              0.0, 284.959289550781, 400.978485107422, 
                              0.0, 0.0, 1.0);

cv::Mat K2 = (cv::Mat_<double>(3,3) << 284.553405761719, 0.0, 416.308288574219,
                              0.0, 284.448303222656, 399.650512695312, 
                              0.0, 0.0, 1.0);

cv::Mat R = (cv::Mat_<double>(3,3) << 0.999995   ,     -0.00069247    ,  -0.00301285    ,
   0.000697846  ,    0.999998 ,        0.00178385  ,  
   0.00301161  ,    -0.00178594 ,      0.999994);

cv::Mat D1 = (cv::Mat_<double>(1,4) << -0.00215030391700566 , 	0.034565981477499 , 	-0.0321723408997059 , 	0.00456085382029414);

cv::Mat D2 = (cv::Mat_<double>(1,4) << -0.00417117308825254 , 	0.0398495607078075  ,	-0.037257868796587  ,	0.00614893808960915);


cv::Mat Q, P1, P2;
cv::Mat R1, R2;
cv::Vec3d T(-0.0635989606380463  ,0.000236903550103307,  0.000122214885777794);
cv::Vec2d size_input(848, 800);
cv::Vec2d size_output(800,800);

int dictitonaryId = 1;
float marker_length_m = 0.14987654321;

Eigen::MatrixXf convertCoordiane(float x_angle, float y_angele, float z_angele, Eigen::MatrixXf Q)
{
    Eigen::Matrix3f R_z, R_x, R_y;
    R_z << cos(z_angele), -sin(z_angele), 0,
           sin(z_angele), cos(z_angele), 0,
           0, 0, 1;
    R_y << cos(y_angele), 0, sin(y_angele),
           0, 1, 0,
           -sin(y_angele), 0, cos(y_angele);
    R_x << 1, 0, 0,
           0, cos(x_angle), -sin(x_angle),
           0, sin(x_angle), cos(x_angle);

    Eigen::Matrix3f rotationMatrix = R_x*R_y*R_z;

    Eigen::MatrixXf transformMatrix(4, 4);

    transformMatrix << rotationMatrix(0,0), rotationMatrix(0,1), rotationMatrix(0,2), Q(0,0),
                       rotationMatrix(1,0), rotationMatrix(1,1), rotationMatrix(1,2), Q(1,0),
                       rotationMatrix(2,0), rotationMatrix(2,1), rotationMatrix(2,2), Q(2,0),
                       0, 0, 0, 1;
  //  std::cout << "Done" << std::endl;
    return transformMatrix;

}

void marker_detection(Mat& dst, const CameraSide& side, rs2_pose pose_data)
{
  cv::Ptr<cv::aruco::Dictionary> dictionary =
        cv::aruco::getPredefinedDictionary( \
        cv::aruco::PREDEFINED_DICTIONARY_NAME(dictitonaryId));
  
  std::ostringstream vector_to_marker;

  float x, y, z, w;
  x = pose_data.rotation.x;
  y = pose_data.rotation.y;
  z = pose_data.rotation.z;
  w = pose_data.rotation.w;
  
  double roll, pitch, yaw;

  pitch =  -asin(2.0 * (x*z - w*y)) * 180.0 / PI;
  roll  =  atan2(2.0 * (w*x + y*z), w*w - x*x - y*y + z*z) * 180.0 / PI;
  yaw   =  atan2(2.0 * (w*z + x*y), w*w + x*x - y*y - z*z) * 180.0 / PI;

//  std::cout << "Roll: " << roll << " " << "Pitch: " << pitch << " " << "Yaw: " << yaw << std::endl;


  if(side == CameraSide::LEFT)
  {
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::aruco::detectMarkers(dst, dictionary, corners, ids);

    Eigen::MatrixXf fisheye1Position(3,1);

    fisheye1Position << -3.2, 
                        0, 
                        0;

    if (ids.size() > 0)
    {
        cv::aruco::drawDetectedMarkers(dst, corners, ids);
        std::vector<cv::Vec3d> rvecs, tvecs;
        cv::aruco::estimatePoseSingleMarkers(corners, marker_length_m,
                K1, D1, rvecs, tvecs);

        // Draw axis for each marker
        for(int i=0; i < ids.size(); i++)
        {
          cv::aruco::drawAxis(dst, K1, D1,
              rvecs[i], tvecs[i], 0.1);

        //  cout << " Marker ID: " << ids[i] << "\t";
          vector_to_marker.str(std::string());
          vector_to_marker << std::setprecision(4)
                           << "x: " << std::setw(8) << tvecs[i](0);
          cv::putText(dst, vector_to_marker.str(),
                          cvPoint(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                          cvScalar(0, 252, 124), 1, LINE_AA);
          // cout << std::setprecision(4)
          //                  << "x: " << std::setw(8) << tvecs[i](0) << "\t";

          vector_to_marker.str(std::string());
          vector_to_marker << std::setprecision(4)
                           << "y: " << std::setw(8) << tvecs[i](1);
          cv::putText(dst, vector_to_marker.str(),
                          cvPoint(10, 50), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                          cvScalar(0, 252, 124), 1, LINE_AA);
          // cout << std::setprecision(4)
          //                  << "y: " << std::setw(8) << tvecs[i](1) << "\t";

          vector_to_marker.str(std::string());
          vector_to_marker << std::setprecision(4)
                           << "z: " << std::setw(8) << tvecs[i](2);
          cv::putText(dst, vector_to_marker.str(),
                          cvPoint(10, 70), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                          cvScalar(0, 252, 124), 1, LINE_AA);
          // cout << std::setprecision(4)
          //                  << "z: " << std::setw(8) << tvecs[i](2) << endl;

          Eigen::MatrixXf point(4,1);
          point << tvecs[i](0), tvecs[i](1), tvecs[i](2) , 0;

          Eigen::MatrixXf T265Point(4,1);
          T265Point << pose_data.translation.x, pose_data.translation.y, pose_data.translation.z, 0; 

          Eigen::MatrixXf fisheye1ToT265Pose = convertCoordiane(PI/2, 0, 0, fisheye1Position);
        //  std::cout << fisheye1ToT265Pose << std::endl;
          Eigen::MatrixXf T265PoseToWorld = convertCoordiane(roll, pitch, yaw, T265Point);

          Eigen::MatrixXf final_(4,1);
        //  final_ = point*fisheye1ToT265Pose*T265PoseToWorld;
       // std::cout << point << std::endl;
        final_ = fisheye1ToT265Pose*point;
        final_ = T265PoseToWorld*final_;
      //  std::cout << final_ << std::endl;

        std::cout << "Final: " << final_(0,0) << " " << final_(1,0) << " " << final_(2,0) << std::endl;

          cout << endl;
        }
    }
  }
  else 
  {
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::aruco::detectMarkers(dst, dictionary, corners, ids);

    if (ids.size() > 0)
    {
        cv::aruco::drawDetectedMarkers(dst, corners, ids);
        std::vector<cv::Vec3d> rvecs, tvecs;
        cv::aruco::estimatePoseSingleMarkers(corners, marker_length_m,
                K2, D2, rvecs, tvecs);

        // Draw axis for each marker
        for(int i=0; i < ids.size(); i++)
        {
          cv::aruco::drawAxis(dst, K2, D2,
              rvecs[i], tvecs[i], 0.1);

          vector_to_marker.str(std::string());
          vector_to_marker << std::setprecision(4)
                           << "x: " << std::setw(8) << tvecs[0](0);
          cv::putText(dst, vector_to_marker.str(),
                          cvPoint(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                          cvScalar(0, 252, 124), 1, LINE_AA);

          vector_to_marker.str(std::string());
          vector_to_marker << std::setprecision(4)
                           << "y: " << std::setw(8) << tvecs[0](1);
          cv::putText(dst, vector_to_marker.str(),
                          cvPoint(10, 50), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                          cvScalar(0, 252, 124), 1, LINE_AA);

          vector_to_marker.str(std::string());
          vector_to_marker << std::setprecision(4)
                           << "z: " << std::setw(8) << tvecs[0](2);
          cv::putText(dst, vector_to_marker.str(),
                          cvPoint(10, 70), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                          cvScalar(0, 252, 124), 1, LINE_AA);
        }
    }
  }
} 

void init_rectification_map(std::string param_file_path) 
{
  // The resolution of the input images used for stereo calibration.
  cv::Size input_img_size(size_input[0], size_input[1]);

  // The resolution of the output rectified images. Lower resolution images require less computation time.
  cv::Size output_img_size(size_output[0], size_output[1]);
  double alpha = 0.0;

  stereoRectify(K1, D1, K2, D2, 
                input_img_size, 
                R, T, 
                R1, R2, P1, P2, 
                Q,
                CV_CALIB_ZERO_DISPARITY, 
                alpha, 
                output_img_size);
 
  cv::fisheye::initUndistortRectifyMap(K1, D1, R1, P1, output_img_size, CV_32FC1, lmapx, lmapy);
  cv::fisheye::initUndistortRectifyMap(K2, D2, R2, P2, output_img_size, CV_32FC1, rmapx, rmapy);
  std::cout << "Hello" << std::endl;

}

//
// This function undistorts and rectifies the src image into dst. 
// The homographic mappings lmapx, lmapy, rmapx, and rmapy are found from OpenCV’s initUndistortRectifyMap function.
//
void undistort_rectify_image(cv::Mat& src, cv::Mat& dst, const CameraSide& side)
{
  if (side == CameraSide::RIGHT) 
  {
    remap(src, dst, lmapx, lmapy, cv::INTER_LINEAR);
  } 
  else 
  {
    remap(src, dst, rmapx, rmapy, cv::INTER_LINEAR);
  }
}

int main(){
    std::cout << "Hello world" << std::endl;

    rs2::pipeline pipe;
    pipe.start();

    const auto fish_right = "Right FishCamera";
    const auto fish_left = "Left FishCamera";

    const auto fish_right_undis = "Right FishCamera Undistorted";
    const auto fish_left_undis = "Left FishCamera Undistorted";

    const auto fish_right_undis_marker = "Right FishCamera Undistorted and Marker";
    const auto fish_left_undis_marker = "Left FishCamera Undistorted and Marker";

    init_rectification_map("bla bla");

    while(cv::waitKey(1) < 0){

        auto frameset = pipe.wait_for_frames();
        rs2::video_frame Left = frameset.get_fisheye_frame(1);
        rs2::video_frame Right = frameset.get_fisheye_frame(2);

        auto frame = frameset.get_pose_frame();
        auto pose_data = frame.get_pose_data();

    //    std::cout << "T265 position x: " << pose_data.translation.x << " " << "y: " << pose_data.translation.y << " " << "z: " << pose_data.translation.z << std::endl;
    //    std::cout << "Quaternion: " << pose_data.rotation.w << " " << pose_data.rotation.x << " " << pose_data.rotation.y << " " << pose_data.rotation.z << std::endl;

        const int rw = Right.get_width();
        const int rh = Right.get_height();

        const int lw = Left.get_width();
        const int lh = Left.get_height();

        cv::Mat right_image(cv::Size(rw, rh), CV_8UC1, (void*)Right.get_data(), cv::Mat::AUTO_STEP);
		    cv::Mat left_image(cv::Size(lw, lh), CV_8UC1, (void*)Left.get_data(), cv::Mat::AUTO_STEP);

	//	cv::imshow(fish_right, right_image);
        cv::Mat right_image_undis;
        cv::remap(right_image, right_image_undis, lmapx, lmapy, cv::INTER_LINEAR);
        marker_detection(right_image_undis, CameraSide::RIGHT, pose_data);
        cv::imshow(fish_right_undis_marker, right_image_undis);



	//	cv::imshow(fish_left, left_image);
        cv::Mat left_image_undis;
        cv::remap(left_image, left_image_undis, rmapx, rmapy, cv::INTER_LINEAR);
        marker_detection(left_image_undis, CameraSide::LEFT, pose_data);
    //    cv::imshow(fish_left_undis_marker, left_image_undis);

    }
    return 0;
}