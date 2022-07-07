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

#include <i3ds/configurator.hpp>

#include <boost/program_options.hpp>
namespace po = boost::program_options;

std::atomic<bool> running;

void
signal_handler(int)
{
    running = false;
}

int main(int argc, char *argv[])
{

    int merger_node, cam_1_node, cam_2_node;

    po::options_description desc("Service for merging two camera streams into one stereo image stream\n  Available options");
    i3ds::Configurator configurator;

    configurator.add_common_options(desc);
    desc.add_options()
    ("node,n", po::value<int>(&merger_node)->required(), "NodeID of camera_merger")
    ("cam_1_node", po::value<int>(&cam_1_node)->required(), "NodeID of left input camera")
    ("cam_2_node", po::value<int>(&cam_2_node)->required(), "NodeID of right input camera")
    ;

    po::variables_map vm = configurator.parse_common_options(desc, argc, argv);


    i3ds::Context::Ptr context = i3ds::Context::Create();;
    i3ds::Server server(context);


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

    i3ds::DisparityGenerator disparity_generator(context, merger_node,
               cam_1_node, cam_2_node,
               stereo_reconstruction);
    disparity_generator.Attach(server);

    server.Start();
    running = true;
    signal(SIGINT, signal_handler);

    while (running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    server.Stop();

    return 0;
}
