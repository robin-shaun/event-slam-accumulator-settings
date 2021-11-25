//
// Created by kehan on 2021/11/4.
//


#include "dv_ros/knoise/k_noise.h"

namespace dv_ros {

KNoise::KNoise(const KNoiseOptions& options) {
  options_ = std::make_shared<KNoiseOptions>(options);
  x_cols_ = std::vector<kMemCell>(options_->frame_width);
  y_rows_ = std::vector<kMemCell>(options_->frame_height);
}

KNoise::~KNoise() = default;

void KNoise::ProcessEvents(dv::EventStore& event_store) {
  if (event_store.isEmpty()) {
    return;
  }

  dv::EventStore output_event_store;
  for (auto& evt : event_store) {
    uint16_t xAddr = evt.x();
    uint16_t yAddr = evt.y();
    bool polarity = evt.polarity();
    int64_t timestamp = evt.timestamp();
    size_t support = 0;

    bool xAddrMinusOne = (xAddr > 0);
    bool xAddrPlusOne = (xAddr < (options_->frame_width - 1));
    bool yAddrMinusOne = (yAddr > 0);
    bool yAddrPlusOne = (yAddr < (options_->frame_height - 1));

    if (xAddrMinusOne) {
      struct kMemCell& xPrevCell = x_cols_[xAddr - 1];
      if ((timestamp - xPrevCell.timestamp) <= options_->delta_t
          && xPrevCell.polarity == polarity) {
        if ((yAddrMinusOne && (xPrevCell.other_addr == (yAddr - 1)))
            || (xPrevCell.other_addr == yAddr)
            || (yAddrPlusOne && (xPrevCell.other_addr == (yAddr + 1)))) {
          support++;
        }
      }
    }

    struct kMemCell& xCell = x_cols_[xAddr];
    if ((timestamp - xCell.timestamp) <= options_->delta_t
        && xCell.polarity == polarity) {
      if ((yAddrMinusOne && (xCell.other_addr == (yAddr - 1)))
          || (yAddrPlusOne && (xCell.other_addr == (yAddr + 1)))) {
        support++;
      }
    }

    if (xAddrPlusOne) {
      struct kMemCell& xNextCell = x_cols_[xAddr + 1];
      if ((timestamp - xNextCell.timestamp) <= options_->delta_t
          && xNextCell.polarity == polarity) {
        if ((yAddrMinusOne && (xNextCell.other_addr == (yAddr - 1)))
            || (xNextCell.other_addr == yAddr)
            || (yAddrPlusOne && (xNextCell.other_addr == (yAddr + 1)))) {
          support++;
        }
      }
    }

    if (yAddrMinusOne) {
      struct kMemCell& yPrevCell = y_rows_[yAddr - 1];
      if ((timestamp - yPrevCell.timestamp) <= options_->delta_t
          && yPrevCell.polarity == polarity) {
        if ((xAddrMinusOne && (yPrevCell.other_addr == (xAddr - 1)))
            || (yPrevCell.other_addr == xAddr)
            || (xAddrPlusOne && (yPrevCell.other_addr == (xAddr + 1)))) {
          support++;
        }
      }
    }

    struct kMemCell& yCell = y_rows_[yAddr];
    if ((timestamp - yCell.timestamp) <= options_->delta_t
        && yCell.polarity == polarity) {
      if ((xAddrMinusOne && (yCell.other_addr == (xAddr - 1)))
          || (xAddrPlusOne && (yCell.other_addr == (xAddr + 1)))) {
        support++;
      }
    }

    if (yAddrPlusOne) {
      struct kMemCell& yNextCell = y_rows_[yAddr + 1];
      if ((timestamp - yNextCell.timestamp) <= options_->delta_t
          && yNextCell.polarity == polarity) {
        if ((xAddrMinusOne && (yNextCell.other_addr == (xAddr - 1)))
            || (yNextCell.other_addr == xAddr)
            || (xAddrPlusOne && (yNextCell.other_addr == (xAddr + 1)))) {
          support++;
        }
      }
    }

    if (support >= options_->supporters) {
      output_event_store << evt;
      xCell.passed = true;
      yCell.passed = true;
    } else {
      xCell.passed = false;
      yCell.passed = false;
    }

    // Update maps.
    xCell.timestamp = timestamp;
    xCell.polarity = polarity;
    xCell.other_addr = yAddr;

    yCell.timestamp = timestamp;
    yCell.polarity = polarity;
    yCell.other_addr = xAddr;
  }
  event_store = output_event_store;
}

std::shared_ptr<KNoiseOptions> KNoise::GetMutableOptions() {
  return options_;
}

}  // namespace dv_ros
