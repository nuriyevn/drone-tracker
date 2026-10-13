#pragma once

#include <opencv2/core.hpp>

namespace drone_tracker {

enum class TrackingStatus {
    NOT_INITIALIZED,
    TRACKING,
    LOST
};

class ObjectPosition {
public:
    static ObjectPosition fromRect(const cv::Rect& roi, TrackingStatus status);
    static bool isValidRect(const cv::Rect& roi, const cv::Size& frameSize);

    bool isValid() const noexcept;
    cv::Point2d center() const noexcept;

    double x = 0.0;
    double y = 0.0;
    int width = 0;
    int height = 0;
    cv::Rect rect;
    TrackingStatus status = TrackingStatus::NOT_INITIALIZED;
};

}  // namespace drone_tracker
