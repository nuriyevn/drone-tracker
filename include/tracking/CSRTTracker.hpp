#pragma once

#include <opencv2/tracking.hpp>

#include "interfaces/IObjectTracker.hpp"

namespace drone_tracker {

class CSRTTracker : public IObjectTracker {
public:
    CSRTTracker() = default;

    bool initialize(const cv::Mat& frame, const cv::Rect& roi) override;
    bool update(const cv::Mat& frame, cv::Rect& roi) override;
    void reset() override;
    bool isInitialized() const override;
    cv::Rect getBoundingBox() const override;

private:
    cv::Ptr<cv::TrackerCSRT> tracker_;
    bool initialized_ = false;
    cv::Rect last_roi_;
};

}  // namespace drone_tracker
