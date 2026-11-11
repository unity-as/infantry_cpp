#include "super_cap.h"
#include <math.h>

void SuperCap::init(const Config& config) {
    feedback_ = {};
    daemon_.init(config.daemon);
    daemon_.online_ = 0;  // 首个有效反馈到达前不能视作在线。
}

void SuperCap::receive(const Feedback& feedback) {
    if (!isfinite(feedback.output_power_w)) return;
    feedback_ = feedback;
    daemon_.reset();
}

bool SuperCap::isOnline() const {
    return daemon_.online_ != 0;
}

const SuperCap::Feedback& SuperCap::feedback() const {
    return feedback_;
}
