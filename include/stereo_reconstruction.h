#include <string>
#include <map>
#include <memory>

#include "opencv2/core/core.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"

#ifndef STEREO_RECONSTRUCTION_HEADER_FILE
#define STEREO_RECONSTRUCTION_HEADER_FILE

using namespace cv;

class StereoReconstruction {

public:

    StereoReconstruction(std::string file_storage, std::map<std::string, int> reconstruction_parameters);

    cv::Mat disparity_from_stereovision(Mat img_left, Mat img_right);

    cv::Mat pcl_from_disparity(cv::Mat disparity_map);

private:

    cv::Mat stereoMapL_x;
    cv::Mat stereoMapL_y;
    cv::Mat stereoMapR_x;
    cv::Mat stereoMapR_y;
    
    cv::Mat Q;

    Ptr<StereoBM> stereo;

    float minDisparity, numDisparities;

};

#endif
