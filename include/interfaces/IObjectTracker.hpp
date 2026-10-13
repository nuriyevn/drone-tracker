#pragma once

#include <opencv2/core.hpp>

namespace drone_tracker {

class IObjectTracker {
public:
    virtual ~IObjectTracker() = default;

    virtual bool initialize(const cv::Mat& frame, const cv::Rect& roi) = 0;
    virtual bool update(const cv::Mat& frame, cv::Rect& roi) = 0;
    virtual void reset() = 0;
    virtual bool isInitialized() const = 0;
    virtual cv::Rect getBoundingBox() const = 0;
};

}  // namespace drone_tracker
