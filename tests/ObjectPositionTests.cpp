#include <cassert>
#include <opencv2/core.hpp>

#include "../include/models/ObjectPosition.hpp"

int main()
{
    const cv::Rect roi(100, 80, 50, 40);
    const auto position = drone_tracker::ObjectPosition::fromRect(roi, drone_tracker::TrackingStatus::TRACKING);

    assert(position.isValid());
    assert(position.x == 125.0);
    assert(position.y == 100.0);
    assert(position.width == 50);
    assert(position.height == 40);

    const auto invalid = drone_tracker::ObjectPosition::fromRect(cv::Rect(), drone_tracker::TrackingStatus::LOST);
    assert(!invalid.isValid());

    return 0;
}
