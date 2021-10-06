#ifndef NODELET_ROSBAG_H
#define NODELET_ROSBAG_H

#include <boost/thread/mutex.hpp>
#include <boost/utility/in_place_factory.hpp>
#include <nodelet/nodelet.h>
#include <nodelet_rosbag/SubscribeAction.h>
#include <rosbag/bag.h>
#include <topic_tools/shape_shifter.h>
#include <time.h>

namespace nodelet_rosbag {

class NodeRosbagImpl {
public:
  NodeRosbagImpl(ros::NodeHandle *nh, ros::NodeHandle *private_nh)
      : nh_(*nh), private_nh_(*private_nh) {}

  void init()
  {
    private_nh_.getParam("rosbag_path", rosbag_path_);
    private_nh_.getParam("rosbag_record_topics", rosbag_record_topics_);

    if (rosbag_record_topics_.empty())
    {
      std::cout << "ERROR: NodeRosbagImpl::init() rosparam: rosbag_record_topics was empty!" << std::endl;
    }

    time_t rawtime;
    char timebuffer[100];
    struct tm *buf;

    // Get current time
    time(&rawtime);
    buf = localtime(&rawtime);

    // Set rosbag name
    if (std::strftime(timebuffer, sizeof(timebuffer), "%F-%H-%M-%S", buf)) {
      rosbag_path_ = rosbag_path_ + "_" + std::string(timebuffer) + ".bag";
    } else {
      std::cout << "Error on setting the date on rosbag name, setting name without date" << std::endl;
      rosbag_path_ = rosbag_path_ + ".bag";
    }

    // Display info
    std::cout << "NodeRosbagImpl::init(): recording topics in: " +  rosbag_path_ << std::endl;
    for (size_t i = 0; i < rosbag_record_topics_.size(); ++i) {
      std::cout << "\t * " <<  rosbag_record_topics_[i] << std::endl;
    }
  }


  void set_param();
  void close_bag();
  void open_bag();

private:
  ros::NodeHandle &nh_;
  ros::NodeHandle &private_nh_;

  // Write messages coming to this callback to a Bag file
  void record_callback(
      const ros::MessageEvent<topic_tools::ShapeShifter const> &event);

  std::vector<ros::Subscriber> record_subscribers_;
  rosbag::Bag bag_;
  std::string rosbag_path_;
  std::vector<std::string> rosbag_record_topics_;

  boost::mutex rosbag_bag_mtx_;
};

class NodeletRosbag : public nodelet::Nodelet {
public:
  virtual void onInit() {
    NODELET_INFO("Initializing nodelet...");
    node_impl_ = boost::in_place(&getNodeHandle(), &getPrivateNodeHandle());

    node_impl_->init();
    node_impl_->open_bag();
    node_impl_->set_param();
    std::cout << "NodeletRosbag.onInit(): DONE" << std::endl;
  }

  ~NodeletRosbag() { node_impl_->close_bag(); }

private:
  boost::optional<NodeRosbagImpl> node_impl_;
};
}

PLUGINLIB_EXPORT_CLASS(nodelet_rosbag::NodeletRosbag, nodelet::Nodelet);

#endif // NODELET_ROSBAG_H
