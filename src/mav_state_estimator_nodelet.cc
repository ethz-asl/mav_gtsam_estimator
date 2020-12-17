#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "mav_state_estimation/mav_state_estimator.h"

namespace mav_state_estimation {

class MavStateEstimatorNodelet : public nodelet::Nodelet {
  virtual void onInit() {
    try {
      estimator_ = std::make_shared<MavStateEstimator>(getNodeHandle(),
                                                       getPrivateNodeHandle());
    } catch (std::runtime_error e) {
      ROS_ERROR("%s", e.what());
    }
  }

  std::shared_ptr<MavStateEstimator> estimator_;
};
}  // namespace mav_state_estimation

PLUGINLIB_EXPORT_CLASS(mav_state_estimation::MavStateEstimatorNodelet,
                       nodelet::Nodelet)
