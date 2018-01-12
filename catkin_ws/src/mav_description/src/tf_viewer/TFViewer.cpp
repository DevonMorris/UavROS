#include <tf_viewer/TFViewer.h>

namespace tf_viewer
{

  TFViewer::TFViewer() :
    nh_(ros::NodeHandle()), phi_(0.), theta_(0.),
    psi_(0.), n_(0.), e_(0.), d_(0.)
  {
    ros::NodeHandle nh_private("~");

    // publishers and subscribers
    phi_sub_ = nh_private.subscribe("/mav/phi", 5, &TFViewer::phi_cb_, this);
    theta_sub_ = nh_private.subscribe("/mav/theta", 5, &TFViewer::theta_cb_, this);
    psi_sub_ = nh_private.subscribe("/mav/psi", 5, &TFViewer::psi_cb_, this);
    n_sub_ = nh_private.subscribe("/mav/n", 5, &TFViewer::n_cb_, this);
    e_sub_ = nh_private.subscribe("/mav/e", 5, &TFViewer::e_cb_, this);
    d_sub_ = nh_private.subscribe("/mav/d", 5, &TFViewer::d_cb_, this);
  }

  void TFViewer::tick()
  {
    // create transform
    tf::StampedTransform transform;

    /*
     * Transform from world_ned to body
     */

    transform.setIdentity();
    tf::Quaternion qNED2BODY; qNED2BODY.setRPY(phi_, theta_, psi_);
    tf::Vector3 tNED2BODY; tNED2BODY.setValue(n_, e_, d_);

    transform.setRotation(qNED2BODY);
    transform.setOrigin(tNED2BODY);
    tf_br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world_ned", "base_link"));

  }

  void TFViewer::phi_cb_(const std_msgs::Float32ConstPtr& msg)
  {
    phi_ = msg->data;
  }

  void TFViewer::theta_cb_(const std_msgs::Float32ConstPtr& msg)
  {
    theta_ = msg->data;
  }

  void TFViewer::psi_cb_(const std_msgs::Float32ConstPtr& msg)
  {
    psi_ = msg->data;
  }

  void TFViewer::n_cb_(const std_msgs::Float32ConstPtr& msg)
  {
    n_ = msg->data;
  }

  void TFViewer::e_cb_(const std_msgs::Float32ConstPtr& msg)
  {
    e_ = msg->data;
  }

  void TFViewer::d_cb_(const std_msgs::Float32ConstPtr& msg)
  {
    d_ = msg->data;
  }
}
