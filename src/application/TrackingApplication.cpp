#include "application/TrackingApplication.hpp"

#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

namespace drone_tracker {
namespace {

std::string statusToString(TrackingStatus status)
{
    switch (status) {
    case TrackingStatus::NOT_INITIALIZED:
        return "NOT INITIALIZED";
    case TrackingStatus::TRACKING:
        return "TRACKING";
    case TrackingStatus::LOST:
        return "LOST";
    default:
        return "UNKNOWN";
    }
}

std::string formatCoordinate(double value)
{
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(1) << value;
    return stream.str();
}

}  // namespace

TrackingApplication::TrackingApplication(std::unique_ptr<IVideoSource> videoSource,
                                         std::unique_ptr<IObjectTracker> tracker,
                                         std::string windowName)
    : video_source_(std::move(videoSource)),
      tracker_(std::move(tracker)),
      window_name_(std::move(windowName)) {}

bool TrackingApplication::selectInitialRegion(cv::Mat& frame, cv::Rect& roi) const
{
    cv::Rect selected = cv::selectROI(window_name_, frame, false, false);
    if (selected.area() <= 0) {
        return false;
    }

    const auto frameSize = frame.size();
    const bool withinFrame = ObjectPosition::isValidRect(selected, frameSize);
    if (!withinFrame) {
        return false;
    }

    roi = selected;
    return true;
}

void TrackingApplication::renderFrame(cv::Mat& frame,
                                     const cv::Rect& roi,
                                     const ObjectPosition& position,
                                     TrackingStatus status) const
{
    const auto statusText = statusToString(status);
    const cv::Scalar green(0, 255, 0);
    const cv::Scalar red(0, 0, 255);
    const cv::Scalar blue(255, 0, 0);

    if (status == TrackingStatus::TRACKING && roi.area() > 0) {
        cv::rectangle(frame, roi, green, 2);
        const auto center = position.center();
        cv::circle(frame, center, 4, green, -1);
        cv::line(frame, cv::Point(static_cast<int>(center.x), static_cast<int>(center.y) - 10),
                 cv::Point(static_cast<int>(center.x), static_cast<int>(center.y) + 10), green, 1);
        cv::line(frame, cv::Point(static_cast<int>(center.x) - 10, static_cast<int>(center.y)),
                 cv::Point(static_cast<int>(center.x) + 10, static_cast<int>(center.y)), green, 1);
    } else if (status == TrackingStatus::LOST) {
        cv::putText(frame, "Tracking lost. Press 'R' to reselect.",
                    cv::Point(20, 40), cv::FONT_HERSHEY_SIMPLEX, 0.8, red, 2);
    } else if (status == TrackingStatus::NOT_INITIALIZED) {
        cv::putText(frame, "Select the drone ROI with the mouse.",
                    cv::Point(20, 40), cv::FONT_HERSHEY_SIMPLEX, 0.8, blue, 2);
    }

    std::string coordinateText = "X: --, Y: --";
    if (position.isValid()) {
        coordinateText = "X: " + formatCoordinate(position.x) + ", Y: " + formatCoordinate(position.y);
    }

    cv::putText(frame, statusText, cv::Point(20, 20), cv::FONT_HERSHEY_SIMPLEX, 0.8, blue, 2);
    cv::putText(frame, coordinateText, cv::Point(20, 60), cv::FONT_HERSHEY_SIMPLEX, 0.8, blue, 2);
    cv::putText(frame, "Esc or Q to quit, R to reinitialize", cv::Point(20, frame.rows - 20),
                cv::FONT_HERSHEY_SIMPLEX, 0.7, blue, 2);
}

int TrackingApplication::run()
{
    if (video_source_ == nullptr || tracker_ == nullptr) {
        std::cerr << "Video source and tracker must be configured." << std::endl;
        return 1;
    }

    if (!video_source_->open()) {
        std::cerr << "Failed to open video source." << std::endl;
        return 1;
    }

    cv::namedWindow(window_name_, cv::WINDOW_NORMAL);
    cv::Mat frame;

    while (true) {
        if (!video_source_->read(frame) || frame.empty()) {
            std::cout << "End of video stream reached." << std::endl;
            break;
        }

        if (!tracker_->isInitialized()) {
            cv::Rect roi;
            if (selectInitialRegion(frame, roi)) {
                if (!tracker_->initialize(frame, roi)) {
                    std::cerr << "Tracker initialization failed for the selected ROI." << std::endl;
                    current_status_ = TrackingStatus::LOST;
                } else {
                    last_roi_ = roi;
                    current_status_ = TrackingStatus::TRACKING;
                }
            } else {
                current_status_ = TrackingStatus::NOT_INITIALIZED;
            }
        } else {
            cv::Rect roi = last_roi_;
            if (!tracker_->update(frame, roi)) {
                current_status_ = TrackingStatus::LOST;
                last_roi_ = cv::Rect();
            } else {
                last_roi_ = roi;
                current_status_ = TrackingStatus::TRACKING;
            }
        }

        ObjectPosition position;
        if (current_status_ == TrackingStatus::TRACKING && last_roi_.area() > 0) {
            position = ObjectPosition::fromRect(last_roi_, current_status_);
        } else {
            position.status = current_status_;
        }

        renderFrame(frame, last_roi_, position, current_status_);
        cv::imshow(window_name_, frame);

        const int key = cv::waitKey(1) & 0xFF;
        if (key == 27 || key == 'q' || key == 'Q') {
            break;
        }
        if (key == 'r' || key == 'R') {
            tracker_->reset();
            current_status_ = TrackingStatus::NOT_INITIALIZED;
            last_roi_ = cv::Rect();
            std::cout << "Tracker reset. Please select a new ROI." << std::endl;
        }
    }

    video_source_->close();
    cv::destroyAllWindows();
    return 0;
}

}  // namespace drone_tracker
