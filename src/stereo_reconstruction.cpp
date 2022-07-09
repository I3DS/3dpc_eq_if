#include <iostream>
#include "stereo_reconstruction.h"

using namespace cv;

StereoReconstruction::StereoReconstruction(std::string file_storage, std::map<std::string, int> reconstruction_parameters)
{
    FileStorage fs(file_storage, FileStorage::READ);
    if (!fs.isOpened())
    {
        std::cerr << "Failed to open " << file_storage << std::endl;
        return;
    }

    fs["stereoMapL_x"] >> stereoMapL_x;
    fs["stereoMapL_y"] >> stereoMapL_y;
    fs["stereoMapR_x"] >> stereoMapR_x;
    fs["stereoMapR_y"] >> stereoMapR_y;

    fs["q"] >> Q;

    stereo = StereoBM::create();
    
    stereo->setNumDisparities(reconstruction_parameters["numDisparities"]);
    stereo->setBlockSize(reconstruction_parameters["blockSize"]);
    stereo->setPreFilterType(reconstruction_parameters["preFilterType"]);
    stereo->setPreFilterSize(reconstruction_parameters["preFilterSize"]);
    stereo->setPreFilterCap(reconstruction_parameters["preFilterCap"]);
    stereo->setTextureThreshold(reconstruction_parameters["textureThreshold"]);
    stereo->setUniquenessRatio(reconstruction_parameters["uniquenessRatio"]);
    stereo->setSpeckleRange(reconstruction_parameters["speckleRange"]);
    stereo->setSpeckleWindowSize(reconstruction_parameters["speckleWindowSize"]);
    stereo->setDisp12MaxDiff(reconstruction_parameters["disp12MaxDiff"]);
    stereo->setMinDisparity(reconstruction_parameters["minDisparity"]);

    minDisparity = reconstruction_parameters["minDisparity"];
    numDisparities = reconstruction_parameters["numDisparities"];

}

Mat StereoReconstruction::disparity_from_stereovision(Mat img_left, Mat img_right){  

    Mat img_left_gray, img_right_gray;        
    
    //cvtColor(img_left, img_left_gray, cv::COLOR_RGB2GRAY);
    //cvtColor(img_right, img_right_gray, cv::COLOR_RGB2GRAY);

    Mat img_left_nice, img_right_nice; 

    remap(img_left_gray, img_left_nice, stereoMapL_x, stereoMapL_y, INTER_LANCZOS4, BORDER_CONSTANT,  Scalar(0, 0, 0) );

    remap(img_right_gray, img_right_nice, stereoMapR_x, stereoMapR_y, INTER_LANCZOS4, BORDER_CONSTANT,  Scalar(0, 0, 0) );

    Mat disparity_map;
    stereo->compute(img_left_nice, img_right_nice, disparity_map);

    return disparity_map;
}

cv::Mat StereoReconstruction::pcl_from_disparity(cv::Mat disparity_map){

    disparity_map.convertTo(disparity_map, CV_32FC1); 
    disparity_map = disparity_map / 16.0;

    cv::Mat points_3D(disparity_map.size(),CV_32FC1);
    reprojectImageTo3D(disparity_map, points_3D, Q, false, CV_32FC1);

    double min, max;
    cv::minMaxLoc(disparity_map, &min, &max);

    cv::Mat mask = disparity_map > min;

    cv::Mat output_points;

    points_3D.copyTo(output_points, mask);

    return points_3D;
}



