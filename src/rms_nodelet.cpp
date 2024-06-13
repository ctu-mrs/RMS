#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <rms/rms.h>

namespace rms
{

// | ------------------------- Header ------------------------- |

/* class RMSNodelet //{ */
class RMSNodelet : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  bool _is_initialized = false;
  bool _verbose        = false;

  ros::Subscriber _sub_points;
  void            callbackPointCloud(const sensor_msgs::PointCloud2::ConstPtr &msg);

  ros::Publisher _pub_points;
  void           publishCloud(const ros::Publisher &pub, const sensor_msgs::PointCloud2::Ptr msg);

private:
  std::unique_ptr<RMS> _rms;

  size_t _frame_no = 0;
  float  _t_total  = 0.0f;
};
//}

// | --------------------- Implementation --------------------- |

/*//{ RMSNodelet onInit() */
void RMSNodelet::onInit() {

  ros::NodeHandle nh = nodelet::Nodelet::getMTPrivateNodeHandle();
  ros::Time::waitForValid();

  // Load parameters and setup objects
  mrs_lib::ParamLoader param_loader = mrs_lib::ParamLoader(nh, "RMSNodelet");

  param_loader.loadParam<bool>("verbose", _verbose);
  _rms = std::make_unique<RMS>(param_loader);

  if (!param_loader.loadedSuccessfully()) {
    NODELET_ERROR_STREAM("[RMSNodelet] Some compulsory parameters were not loaded successfully, ending the nodelet.");
    ros::shutdown();
  }

  // Setup subsribers and publishers
  _sub_points = nh.subscribe("points_in", 1, &RMSNodelet::callbackPointCloud, this, ros::TransportHints().tcpNoDelay());
  _pub_points = nh.advertise<sensor_msgs::PointCloud2>("points_out", 1);

  _is_initialized = true;
}
/*//}*/

/*//{ callbackPointCloud() */
void RMSNodelet::callbackPointCloud(const sensor_msgs::PointCloud2::ConstPtr &msg) {

  if (!_is_initialized) {
    return;
  }

  // Store input metadata for later
  const size_t size_before = msg->width * msg->height;

  // Convert to non-const ptr
  sensor_msgs::PointCloud2::Ptr msg_out = boost::make_shared<sensor_msgs::PointCloud2>(*msg);

  // Sample input message
  mrs_lib::ScopeTimer timer("RMS", nullptr, false);
  _rms->sample(msg_out);
  const float runtime = timer.getLifetime();

  // Print out sampling info
  const size_t size_after = msg_out->width * msg_out->height;
  _frame_no++;
  _t_total += runtime;

  NODELET_INFO_COND(_verbose, "[RMSNodelet] RMS sampling -> %ld/%ld pts | compression: %.2f %s | timing: %.2f ms (avg: %0.2f ms)", size_after, size_before,
                    100.0f - 100.0f * (float(size_after) / float(size_before)), "%", runtime, _t_total / float(_frame_no));

  // Publish
  publishCloud(_pub_points, msg_out);
}
/*//}*/

/*//{ publishCloud() */
void RMSNodelet::publishCloud(const ros::Publisher &pub, const sensor_msgs::PointCloud2::Ptr msg) {
  if (msg && pub.getNumSubscribers() > 0) {
    NODELET_INFO_ONCE("[RMSNodelet] Publishing first sampled cloud.");
    try {
      pub.publish(msg);
    }
    catch (...) {
    }
  }
}
/*//}*/

}  // namespace rms

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rms::RMSNodelet, nodelet::Nodelet);
