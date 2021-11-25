//
// Created by kehan on 2021/11/4.
//

#ifndef DV_ROS_DV_ROS_KNOISE_K_NOISE_H_
#define DV_ROS_DV_ROS_KNOISE_K_NOISE_H_

#include "dv-sdk/processing.hpp"

#include "k_noise_options.h"

namespace dv_ros {

struct kMemCell {
  int64_t timestamp;
  bool passed;
  bool polarity;
  uint16_t other_addr;
};

class KNoise {
 public:
  explicit KNoise(const KNoiseOptions& options);
  virtual ~KNoise();
  void ProcessEvents(dv::EventStore& event_store);
  std::shared_ptr<KNoiseOptions> GetMutableOptions();

 private:
  std::shared_ptr<KNoiseOptions> options_;
  std::vector<kMemCell> x_cols_;
  std::vector<kMemCell> y_rows_;
};

}  // namespace dv_ros

#endif //DV_ROS_DV_ROS_KNOISE_K_NOISE_H_
