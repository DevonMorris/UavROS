#include <tf_frames/TFFrames.h>

namespace tf_frames
{

  TFFrames::TFFrames() :
    nh_(ros::NodeHandle())
  {
    ros::NodeHandle nh_private("~");
  }

  void TFFrames::tick()
  {
    // create transform
    tf::StampedTransform transform;

    /*
     * Transform from world (assumed to be nwu) to world_ned
     */

    transform.setIdentity();
    tf::Quaternion qNWU2NED; qNWU2NED.setRPY(M_PI, 0.0, 0.0);
    transform.setRotation(qNWU2NED);
    tf_br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "world_ned"));

  }

}
