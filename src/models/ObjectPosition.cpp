#include "models/ObjectPosition.hpp"

namespace drone_tracker {

ObjectPosition ObjectPosition::fromRect(const cv::Rect& roi, TrackingStatus status)
{
    ObjectPosition position;
    position.rect = roi;
    position.status = status;
    position.width = roi.width;
    position.height = roi.height;

    if (roi.area() <= 0) {
        position.x = 0.0;
        position.y = 0.0;
        return position;
    }

    position.x = roi.x + roi.width / 2.0;
    position.y = roi.y + roi.height / 2.0;
    return position;
}

bool ObjectPosition::isValidRect(const cv::Rect& roi, const cv::Size& frameSize)
{
    if (roi.area() <= 0) {
        return false;
    }

    if (roi.x < 0 || roi.y < 0) {
        return false;
    }

    if (roi.x + roi.width > frameSize.width || roi.y + roi.height > frameSize.height) {
        return false;
    }

    return true;
}

bool ObjectPosition::isValid() const noexcept
{
    return status != TrackingStatus::NOT_INITIALIZED && status != TrackingStatus::LOST && rect.area() > 0;
}

cv::Point2d ObjectPosition::center() const noexcept
{
    return cv::Point2d(x, y);
}

}  // namespace drone_tracker
