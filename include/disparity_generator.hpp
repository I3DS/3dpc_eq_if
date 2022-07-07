///////////////////////////////////////////////////////////////////////////\file
///
///   Copyright 2022 SINTEF AS
///
///   This Source Code Form is subject to the terms of the Mozilla
///   Public License, v. 2.0. If a copy of the MPL was not distributed
///   with this file, You can obtain one at https://mozilla.org/MPL/2.0/
///
////////////////////////////////////////////////////////////////////////////////

#ifndef __I3DS_DISPARITY_GENERATOR
#define __I3DS_DISPARITY_GENERATOR

#include <i3ds/frame.hpp>
#include <i3ds/sensor.hpp>
#include <i3ds/depthmap.hpp>
#include <i3ds/topic.hpp>
#include <i3ds/publisher.hpp>
#include <i3ds/subscriber.hpp>
#include <i3ds/camera_client.hpp>
#include <i3ds/opencv_convertion.hpp>

#include <memory>
#include <condition_variable>

#include "stereo_reconstruction.h"

namespace i3ds
{

class DisparityGenerator : public Sensor
{
public:

  typedef std::shared_ptr<DisparityGenerator> Ptr;

  typedef Topic<128, DepthMapCodec> DisparityMapTopic;

  DisparityGenerator(Context::Ptr context, i3ds_asn1::NodeID node_id,
               i3ds_asn1::NodeID cam_1_node_id, i3ds_asn1::NodeID cam_2_node_id,
               std::shared_ptr<StereoReconstruction> stereo_reconstruction);

  virtual ~DisparityGenerator() = default;

protected:

  virtual void do_activate();

  virtual void do_start();

  virtual void do_stop();

  virtual void do_deactivate();

  virtual bool is_sampling_supported(i3ds_asn1::SampleCommand sample);
private:

  struct Buffer
  {
    cv::Mat mat_;
    unsigned long timestamp_;
    bool data_ready_;
  };

  class BufferPair
  {
  public:
    BufferPair() :
      allocated_(false),
      buffers_(2),
      write_index_(0)
    {}

    virtual ~BufferPair()
    {
      if (allocated_)
        {
          for (auto& buffer : buffers_)
            {
   //           free((void*)(buffer.frame_.image_data(0)));
            }
        }
    }

    // Allocate memory for storing a picture in each Buffers Frame object
    void initialize(Frame frame)
    {
      size_ = frame.image_size(0);
      for (auto& buffer : buffers_)
        {
  //        buffer.frame_.append_image(static_cast<i3ds_asn1::byte*>(malloc(size_)), size_);
          buffer.data_ready_ = false;
        }
      allocated_ = true;
    }

    void put_data(Frame frame)
    {
      if (!allocated_)
      {
        initialize(frame);
      }

      unsigned int current_write_index; //Make sure to use same index during and after write
      {
        std::unique_lock<std::mutex> lock(mutex_);
        current_write_index = write_index_;
        buffers_[current_write_index].data_ready_ = false;
      }
      cv::Mat tmp = frame_to_cv_mat(frame, 0);
      buffers_[current_write_index].mat_ = tmp.clone();
      buffers_[current_write_index].timestamp_ = frame.descriptor.attributes.timestamp;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        buffers_[current_write_index].data_ready_ = true;
        cond_var_.notify_all();
      }
    }

    Buffer get_data()
    {
      std::unique_lock<std::mutex> lock(mutex_);
      cond_var_.wait(lock, [this] {return buffers_[write_index_].data_ready_;});
      buffers_[write_index_].data_ready_ = false;
      unsigned int old_write_index = write_index_;
      write_index_ = (write_index_ + 1) % 2;
      return buffers_[old_write_index];
    }

    void keep_old_value()
    {
      std::unique_lock<std::mutex> lock(mutex_);
      write_index_ = (write_index_ + 1) % 2;
      buffers_[write_index_].data_ready_ = true;
    }

    bool allocated_;
    size_t size_;

  private:
    std::mutex mutex_;
    std::condition_variable cond_var_;
    std::vector<Buffer> buffers_;
    std::atomic<unsigned int> write_index_;
  };


  void set_state(i3ds_asn1::StateCommand state, i3ds_asn1::StateCommand backup_state);

  virtual void handle_sample(SampleService::Data& sample);

  void handle_frames(Frame data, int cam_number);

  void process_and_send(Buffer cam_1_data, Buffer cam_2_data);
  void publisher_thread_func();

  Publisher publisher_;

  Subscriber cam_1_subscriber_;
  Subscriber cam_2_subscriber_;

  CameraClient cam_1_client_;
  CameraClient cam_2_client_;

  std::vector<BufferPair> camera_buffers_;

  std::atomic<bool> run_publiser_thread_;
  std::thread publisher_thread_;

  std::shared_ptr<StereoReconstruction> stereo_reconstruction_;
};

} // namespace i3ds

#endif /* ifndef __I3DS_DISPARITY_GENERATOR */

