// clang-format off
#include <boost/foreach.hpp>
#include <pluginlib/class_list_macros.h>
// clang-format on

#include <nodelet_rosbag/SubscribeAction.h>
#include <nodelet_rosbag/nodelet_rosbag.h>

#include <ros/ros.h>
#include <rosbag/view.h>

#include <iostream>

namespace nodelet_rosbag
{
void NodeRosbagImpl::set_param()
{
  std::cout << "Subscribing to topics: " << std::endl;
  for (size_t i = 0; i < rosbag_record_topics_.size(); ++i)
  {
    std::cout << "\t * " << rosbag_record_topics_[i] << std::endl;

    ros::Subscriber subscriber =
        private_nh_.subscribe(rosbag_record_topics_[i], 10, &NodeRosbagImpl::record_callback, this);
    record_subscribers_.push_back(subscriber);
  }
}

void NodeRosbagImpl::open_bag()
{
  boost::mutex::scoped_lock lock(rosbag_bag_mtx_);
  std::cout << "open_bag(): " << rosbag_path_ << std::endl;
  bag_.open(rosbag_path_, rosbag::bagmode::Write);
}

void NodeRosbagImpl::close_bag()
{
  boost::mutex::scoped_lock lock(rosbag_bag_mtx_);
  bag_.close();
}

void NodeRosbagImpl::record_callback(const ros::MessageEvent<topic_tools::ShapeShifter const>& event)
{
  boost::mutex::scoped_lock lock(rosbag_bag_mtx_);
  ros::M_string& header = event.getConnectionHeader();
  topic_tools::ShapeShifter::ConstPtr message = event.getMessage();
  bag_.write(header["topic"], ros::Time::now(), message);
}
}  // namespace nodelet_rosbag
