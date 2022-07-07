///////////////////////////////////////////////////////////////////////////\file
///
///   Copyright 2022 SINTEF AS
///
///   This Source Code Form is subject to the terms of the Mozilla
///   Public License, v. 2.0. If a copy of the MPL was not distributed
///   with this file, You can obtain one at https://mozilla.org/MPL/2.0/
///
////////////////////////////////////////////////////////////////////////////////


#include <csignal>
#include <chrono>
#include <thread>
#include <atomic>
#include <string>

#include "stereo_reconstruction.h"
#include "disparity_generator.hpp"


std::atomic<bool> running;

void
signal_handler(int)
{
    running = false;
}

int main(int argc, char *argv[])
{
    std::string calibration_file = "../calibration/calib_params_stereo.xml";
    std::map<std::string, int> reconstruction_parameters;
    reconstruction_parameters["numDisparities"] = 5*16;
    reconstruction_parameters["blockSize"] = 4*2 + 5;
    reconstruction_parameters["preFilterType"] = 0;
    reconstruction_parameters["preFilterSize"] = 6*2 + 5;
    reconstruction_parameters["preFilterCap"] = 16;
    reconstruction_parameters["textureThreshold"] = 12;
    reconstruction_parameters["uniquenessRatio"] = 12;
    reconstruction_parameters["speckleRange"] = 13;
    reconstruction_parameters["speckleWindowSize"] = 5*2;
    reconstruction_parameters["disp12MaxDiff"] = 3;
    reconstruction_parameters["minDisparity"] = 4;

    std::shared_ptr<StereoReconstruction> stereo_reconstruction = std::make_shared<StereoReconstruction>(calibration_file, reconstruction_parameters);

    DisparityGenerator(Context::Ptr context, i3ds_asn1::NodeID node_id,
               i3ds_asn1::NodeID cam_1_node_id, i3ds_asn1::NodeID cam_2_node_id);

    running = true;
    signal(SIGINT, signal_handler);

    while (running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return 0;
}
