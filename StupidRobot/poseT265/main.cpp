// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 Intel Corporation. All Rights Reserved.
#include <librealsense2/rs.hpp>
#include <iostream>
#include <iomanip>
#include <math.h>

#define PI 3.14159265

int main(int argc, char * argv[])
{
    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    // Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;
    // Add pose stream
    cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
    // Start pipeline with chosen configuration
    pipe.start(cfg);
    float x, y, z, w;

    // Main loop
    while (true)
    {
        // Wait for the next set of frames from the camera
        auto frames = pipe.wait_for_frames();
        // Get a frame from the pose stream
        auto f = frames.first_or_default(RS2_STREAM_POSE);
        // Cast the frame to pose_frame and get its data
        auto pose_data = f.as<rs2::pose_frame>().get_pose_data();

        // Print the x, y, z values of the translation, relative to initial position
        std::cout << "\r" << "Device Position: " << std::setprecision(3) << std::fixed << pose_data.translation.x << " " <<
            pose_data.translation.y << " " << pose_data.translation.z << " (meters)";

        x = pose_data.rotation.x;
        y = pose_data.rotation.y;
        z = pose_data.rotation.z;
        w = pose_data.rotation.w;

        double roll, pitch, yaw;

        pitch =  -asin(2.0 * (x*z - w*y)) * 180.0 / PI;
        roll  =  atan2(2.0 * (w*x + y*z), w*w - x*x - y*y + z*z) * 180.0 / PI;
        yaw   =  atan2(2.0 * (w*z + x*y), w*w + x*x - y*y - z*z) * 180.0 / PI;

        std::cout << "Roll: " << roll << " " << "Pitch: " << pitch << " " << "Yaw: " << yaw << std::endl;
    }
    return EXIT_SUCCESS;
}