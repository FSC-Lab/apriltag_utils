/* Copyright (C) 2013-2016, The Regents of The University of Michigan.
All rights reserved.
This software was developed in the APRIL Robotics Lab under the
direction of Edwin Olson, ebolson@umich.edu. This software may be
available under alternative licensing terms; contact the address above.
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
The views and conclusions contained in the software and documentation are those
of the authors and should not be interpreted as representing official policies,
either expressed or implied, of the Regents of The University of Michigan.
*/

#include "../include/Detector.hpp"
#include <ros/package.h>

int main(int argc, char *argv[])
{

    ros::init(argc, argv, "detector");

    getopt_t *getopt = getopt_create();
    std::string param_path;
    getopt_add_string(getopt, 'p', "param_path", "", "Location of camera parameter file");
    if (getopt_parse(getopt, argc, argv, 1))
        std::string param_path = getopt_get_string(getopt, "param_path");

    // Initialize Camera through CSI
    int capture_width = 1280;
    int capture_height = 720;
    int display_width = 1280;
    int display_height = 720;
    int framerate = 30;
    int flip_method = 0;

    // Initialize tag detector with options
    Detector dtr(capture_width, capture_height, display_width, display_height, framerate, flip_method);
    std::cout << "Using pipeline: \n\t" << dtr.pipeline_ << "\n";

    bool param_ok;

    if (param_path.empty())
        param_ok = dtr.load_parameters(ros::package::getPath("apriltag_utils") + "/tests/parameter.yaml");
    else
        param_ok = dtr.load_parameters(param_path);

    bool detector_ok = dtr.init();

    if (detector_ok && param_ok)
    {
        std::cout << "Detector initialized successfully" << std::endl;
    }
    else
    {
        std::cout << "Detector initialization failed. Exiting";
        return 1;
    }
    ros::Rate loop_rate(30);

    while (ros::ok())
    {
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

        //ros camera publish
        bool publish = true;
        dtr.get_image(publish);
        dtr.get_pose();
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        double ttrack = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
        std::cout << "\r"
                  << "Tracking time cost is " << ttrack << "ms \r";
        ros::spinOnce();
        loop_rate.sleep();
    }

    dtr.~Detector();
    return 0;
}
