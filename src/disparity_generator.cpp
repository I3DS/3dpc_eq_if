///////////////////////////////////////////////////////////////////////////\file
///
///   Copyright 2022 SINTEF AS
///
///   This Source Code Form is subject to the terms of the Mozilla
///   Public License, v. 2.0. If a copy of the MPL was not distributed
///   with this file, You can obtain one at https://mozilla.org/MPL/2.0/
///
////////////////////////////////////////////////////////////////////////////////

#include <i3ds/time.hpp>
#include <i3ds/opencv_convertion.hpp>
#include <disparity_generator.hpp>
#include <i3ds_asn1/SampleAttribute.hpp>

#ifndef BOOST_LOG_DYN_LINK
#define BOOST_LOG_DYN_LINK
#endif

#include "stereo_reconstruction.h"

#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/expressions.hpp>

#include <string>

i3ds::DisparityGenerator::DisparityGenerator(i3ds::Context::Ptr context, i3ds_asn1::NodeID node_id,
                                 i3ds_asn1::NodeID cam_1_node_id, i3ds_asn1::NodeID cam_2_node_id,
                                 std::shared_ptr<StereoReconstruction> stereo_reconstruction)
  : Sensor(node_id),
    publisher_(context, node_id),
    cam_1_subscriber_(context),
    cam_2_subscriber_(context),
    cam_1_client_(context, cam_1_node_id),
    cam_2_client_(context, cam_2_node_id),
    camera_buffers_(2),
    run_publiser_thread_(false),
    stereo_reconstruction_(stereo_reconstruction)
{
  cam_1_subscriber_.Attach<i3ds::Camera::FrameTopic>(cam_1_node_id, [this](i3ds::Frame data) {handle_frames(data, 0);});
  cam_2_subscriber_.Attach<i3ds::Camera::FrameTopic>(cam_2_node_id, [this](i3ds::Frame data) {handle_frames(data, 1);});
  BOOST_LOG_TRIVIAL(info) << "DisparityGenerator started with node id: " << node_id;
  BOOST_LOG_TRIVIAL(info) << "DisparityGenerator listening to nodes : " << cam_1_node_id << " and " << cam_2_node_id;
}

void
i3ds::DisparityGenerator::set_state(i3ds_asn1::StateCommand state, i3ds_asn1::StateCommand backup_state)
{
  i3ds_asn1::SensorState current_state_cam_1 = cam_1_client_.state();
  i3ds_asn1::SensorState current_state_cam_2 = cam_2_client_.state();

  if (current_state_cam_1 != current_state_cam_2)
    {
      throw CommandError(i3ds_asn1::ResultCode_error_other, "Cameras are in inconsistent states");
    }

  try
    {
      cam_1_client_.set_state(state);
    }
  catch(i3ds::CommandError& e)
    {
      throw CommandError(i3ds_asn1::ResultCode_error_other, std::string("Camera 1: ", e.what()));
    }

  try
    {
      cam_2_client_.set_state(state);
    }
  catch(i3ds::CommandError& e)
    {
      cam_1_client_.set_state(backup_state);
      throw CommandError(i3ds_asn1::ResultCode_error_other, std::string("Camera 2: ", e.what()));
    }

}

void
i3ds::DisparityGenerator::do_activate()
{
  BOOST_LOG_TRIVIAL(trace) << "DisparityGenerator do_activate()";
  set_state(i3ds_asn1::StateCommand_activate, i3ds_asn1::StateCommand_deactivate);
}

void
i3ds::DisparityGenerator::do_start()
{
  BOOST_LOG_TRIVIAL(trace) << "DisparityGenerator do_start()";
  run_publiser_thread_ = true;
  publisher_thread_ = std::thread(&i3ds::DisparityGenerator::publisher_thread_func, this);
  cam_2_subscriber_.Start();
  cam_1_subscriber_.Start();
  set_state(i3ds_asn1::StateCommand_start, i3ds_asn1::StateCommand_stop);
}

void
i3ds::DisparityGenerator::do_stop()
{
  BOOST_LOG_TRIVIAL(trace) << "DisparityGenerator do_stop()";
  run_publiser_thread_ = false;
  if (publisher_thread_.joinable())
    {
      publisher_thread_.join();
    }
  cam_1_subscriber_.Stop();
  cam_2_subscriber_.Stop();
  set_state(i3ds_asn1::StateCommand_stop, i3ds_asn1::StateCommand_stop);
}

void
i3ds::DisparityGenerator::do_deactivate()
{
  BOOST_LOG_TRIVIAL(trace) << "DisparityGenerator do_deactivate()";
  set_state(i3ds_asn1::StateCommand_deactivate, i3ds_asn1::StateCommand_deactivate);
}

bool
i3ds::DisparityGenerator::is_sampling_supported(i3ds_asn1::SampleCommand sample)
{
  // Dummy implementation of pure virtual function. There is currently no way to
  // check this via clients.
  return sample.period > 0;
}

void
i3ds::DisparityGenerator::handle_sample(SampleService::Data& sample)
{
  BOOST_LOG_TRIVIAL(trace) << "DisparityGenerator handle sample";
  Sensor::handle_sample(sample);

  // Set the cameras to free running to avoid problems with missing frames
  cam_1_client_.set_sampling(sample.request.period);
  cam_2_client_.set_sampling(sample.request.period);
}

void
i3ds::DisparityGenerator::handle_frames(i3ds::Frame data, int cam_number)
{
  BOOST_LOG_TRIVIAL(trace) << "DisparityGenerator received frame from camera: " << cam_number;


  if (data.descriptor.attributes.validity == i3ds_asn1::SampleValidity_sample_valid)
    {
      camera_buffers_[cam_number].put_data(data);
    }
  else
    {
      BOOST_LOG_TRIVIAL(warning) << "Received invalid measurement from camera: " << cam_number;
    }
}

void
i3ds::DisparityGenerator::process_and_send(Buffer cam_1_data, Buffer cam_2_data)
{
  DisparityMapTopic::Data disparity_depthmap;
  DisparityMapTopic::Initialize(disparity_depthmap);

  disparity_depthmap.descriptor.width = cam_1_data.mat_.cols;
  disparity_depthmap.descriptor.height = cam_1_data.mat_.rows;
  disparity_depthmap.descriptor.attributes.timestamp = cam_1_data.timestamp_;
  disparity_depthmap.descriptor.attributes.validity = i3ds_asn1::SampleValidity_sample_valid;

  cv::Mat disparity_map = stereo_reconstruction_->disparity_from_stereovision(cam_1_data.mat_, cam_2_data.mat_);

  disparity_depthmap.depths.assign(disparity_map.datastart, disparity_map.dataend);

  BOOST_LOG_TRIVIAL(info) << "DisparityGenerator sending merged frame with timestamp: " <<
                             disparity_depthmap.descriptor.attributes.timestamp;
  publisher_.Send<DisparityMapTopic>(disparity_depthmap);
  update_and_check_batch_count();
}

void
i3ds::DisparityGenerator::publisher_thread_func()
{
  BOOST_LOG_TRIVIAL(trace) << "Starting publisher_thread";
  while (run_publiser_thread_)
    {
      Buffer cam_1_data = camera_buffers_[0].get_data();
      if (!run_publiser_thread_) break;
      Buffer cam_2_data = camera_buffers_[1].get_data();
      if (!run_publiser_thread_) break;

      BOOST_LOG_TRIVIAL(trace) << "DisparityGenerator received frames with timestamps " << cam_1_data.timestamp_ << " and " <<
                               cam_2_data.timestamp_;
      auto t_diff = static_cast<unsigned int>(std::abs(static_cast<long>(cam_1_data.timestamp_) - static_cast<long>
                                              (cam_2_data.timestamp_)));
      BOOST_LOG_TRIVIAL(trace) << "  Timestamp diff: " << t_diff;
      if (t_diff < period()/4)
        {
          process_and_send(cam_1_data, cam_2_data);
        }
      else
        {
          BOOST_LOG_TRIVIAL(warning) << "DisparityGenerator received frames out of sync";
          if (cam_1_data.timestamp_ < cam_2_data.timestamp_)
            {
              camera_buffers_[1].keep_old_value();
            }
          else
            {
              camera_buffers_[0].keep_old_value();
            }
        }
    }
  BOOST_LOG_TRIVIAL(trace) << "Stopping publisher_thread";
}
