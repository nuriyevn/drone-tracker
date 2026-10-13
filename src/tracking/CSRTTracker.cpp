#include "tracking/CSRTTracker.hpp"

namespace drone_tracker {

bool CSRTTracker::initialize(const cv::Mat& frame, const cv::Rect& roi)
{
    if (frame.empty() || roi.area() <= 0) {
        return false;
    }

    tracker_ = cv::TrackerCSRT::create();
    if (tracker_.empty()) {
        return false;
    }

    try {
        tracker_->init(frame, roi);
    }
    catch (const cv::Exception&) {
        tracker_.release();
        initialized_ = false;
        return false;
    }

    initialized_ = true;
    last_roi_ = roi;
    return true;
}

bool CSRTTracker::update(const cv::Mat& frame, cv::Rect& roi)
{
    if (!initialized_ || tracker_.empty() || frame.empty()) {
        return false;
    }

    cv::Rect updated_roi = roi;
    const bool success = tracker_->update(frame, updated_roi);
    if (!success || updated_roi.area() <= 0) {
        return false;
    }

    roi = updated_roi;
    last_roi_ = updated_roi;
    return true;
}

void CSRTTracker::reset()
{
    tracker_.release();
    initialized_ = false;
    last_roi_ = cv::Rect();
}

bool CSRTTracker::isInitialized() const
{
    return initialized_ && !tracker_.empty();
}

cv::Rect CSRTTracker::getBoundingBox() const
{
    return last_roi_;
}

}  // namespace drone_tracker
