#include "application/TrackingApplication.hpp"

#include <chrono>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

namespace drone_tracker {
namespace {

/*
ACHTUNG! ACHTUNG!


Important limitation: this patch uses visual template matching, not a trained drone detector. 
It can reacquire a target that looks sufficiently similar to the original selection, 
but it may fail if the drone rotates significantly, becomes too small, or is obscured.
 For more reliable drone detection, the next step is to integrate a trained YOLO model and,
  where multiple drones are present, a way to identify the originally selected drone.
*/

// Compare the appearance of the tracked box with the original user-selected ROI.
// If the CSRT box drifts to unrelated background, this score should usually drop.
constexpr double kMinimumTrackingSimilarity = 0.30;

// A more conservative threshold is used for a full-frame reacquisition search.
constexpr double kReacquireSimilarityThreshold = 0.60;

std::string statusToString(TrackingStatus status)
{
    switch (status) {
    case TrackingStatus::NOT_INITIALIZED:
        return "NOT INITIALIZED";
    case TrackingStatus::TRACKING:
        return "TRACKING";
    case TrackingStatus::LOST:
        return "LOST / SEARCHING";
    default:
        return "UNKNOWN";
    }
}

cv::Mat toGray(const cv::Mat& image)
{
    cv::Mat gray;
    if (image.channels() == 1) {
        image.copyTo(gray);
    } else if (image.channels() == 4) {
        cv::cvtColor(image, gray, cv::COLOR_BGRA2GRAY);
    } else {
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    }
    return gray;
}

std::string formatCoordinate(double value)
{
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(1) << value;
    return stream.str();
}

std::filesystem::path makeRecordingPath(const std::filesystem::path& directory)
{
    const auto now = std::chrono::system_clock::now();
    const auto time = std::chrono::system_clock::to_time_t(now);
    std::tm localTime{};
#if defined(_WIN32)
    localtime_s(&localTime, &time);
#else
    localtime_r(&time, &localTime);
#endif

    std::ostringstream timestamp;
    timestamp << std::put_time(&localTime, "%Y-%m-%d_%H-%M-%S");

    const auto basePath = directory / (timestamp.str() + ".mp4");
    auto outputPath = basePath;
    unsigned int suffix = 1;
    while (std::filesystem::exists(outputPath)) {
        outputPath = directory / (timestamp.str() + "_" + std::to_string(suffix++) + ".mp4");
    }
    return outputPath;
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

    if (!ObjectPosition::isValidRect(selected, frame.size())) {
        return false;
    }

    roi = selected;
    return true;
}

bool TrackingApplication::findTemplateMatch(const cv::Mat& frame, cv::Rect& roi) const
{
    if (frame.empty() || target_template_gray_.empty()) {
        return false;
    }

    const cv::Mat gray = toGray(frame);
    double bestScore = -1.0;
    cv::Rect bestRoi;

    // Search at several scales because the target may have moved closer to or farther
    // from the camera since the original ROI was selected.
    for (double scale = 0.5; scale <= 2.001; scale += 0.1) {
        const int width = cvRound(target_template_gray_.cols * scale);
        const int height = cvRound(target_template_gray_.rows * scale);
        if (width < 8 || height < 8 || width > gray.cols || height > gray.rows) {
            continue;
        }

        cv::Mat scaledTemplate;
        const int interpolation = scale < 1.0 ? cv::INTER_AREA : cv::INTER_CUBIC;
        cv::resize(target_template_gray_, scaledTemplate, cv::Size(width, height),
                   0.0, 0.0, interpolation);

        // A near-uniform template has little identifying information; don't trust it.
        cv::Scalar mean;
        cv::Scalar stddev;
        cv::meanStdDev(scaledTemplate, mean, stddev);
        if (stddev[0] < 4.0) {
            continue;
        }

        cv::Mat scores;
        cv::matchTemplate(gray, scaledTemplate, scores, cv::TM_CCOEFF_NORMED);
        double maxScore = -1.0;
        cv::Point maxLocation;
        cv::minMaxLoc(scores, nullptr, &maxScore, nullptr, &maxLocation);

        if (std::isfinite(maxScore) && maxScore > bestScore) {
            bestScore = maxScore;
            bestRoi = cv::Rect(maxLocation.x, maxLocation.y, width, height);
        }
    }

    if (bestScore < kReacquireSimilarityThreshold ||
        !ObjectPosition::isValidRect(bestRoi, frame.size())) {
        return false;
    }

    roi = bestRoi;
    return true;
}

double TrackingApplication::templateSimilarity(const cv::Mat& frame, const cv::Rect& roi) const
{
    if (frame.empty() || target_template_gray_.empty() ||
        !ObjectPosition::isValidRect(roi, frame.size())) {
        return -1.0;
    }

    cv::Mat candidate = toGray(frame(roi));
    cv::resize(candidate, candidate, target_template_gray_.size(), 0.0, 0.0, cv::INTER_AREA);

    cv::Scalar mean;
    cv::Scalar stddev;
    cv::meanStdDev(target_template_gray_, mean, stddev);
    if (stddev[0] < 4.0) {
        // There is not enough texture to score this ROI reliably. Don't declare
        // drift based on an undefined normalized-correlation value. Full-frame
        // reacquisition will still require a textured template to avoid false hits.
        return 1.0;
    }

    cv::Mat scores;
    cv::matchTemplate(candidate, target_template_gray_, scores, cv::TM_CCOEFF_NORMED);
    const double score = static_cast<double>(scores.at<float>(0, 0));
    return std::isfinite(score) ? score : -1.0;
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
        cv::putText(frame, "Tracking lost. Searching for target... Press 'R' to reselect.",
                    cv::Point(20, 40), cv::FONT_HERSHEY_SIMPLEX, 0.7, red, 2);
    } else if (status == TrackingStatus::NOT_INITIALIZED) {
        cv::putText(frame, "Select the target ROI with the mouse.",
                    cv::Point(20, 40), cv::FONT_HERSHEY_SIMPLEX, 0.8, blue, 2);
    }

    std::string coordinateText = "X: --, Y: --";
    if (position.isValid()) {
        coordinateText = "X: " + formatCoordinate(position.x) + ", Y: " + formatCoordinate(position.y);
    }

    cv::putText(frame, statusText, cv::Point(20, 20), cv::FONT_HERSHEY_SIMPLEX, 0.8, blue, 2);
    cv::putText(frame, coordinateText, cv::Point(20, 60), cv::FONT_HERSHEY_SIMPLEX, 0.8, blue, 2);
    cv::putText(frame, "Esc or Q to quit, R to reselect", cv::Point(20, frame.rows - 20),
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
    cv::VideoWriter writer;
    std::filesystem::path recordingPath;
    bool recordingStarted = false;

    while (true) {
        if (!video_source_->read(frame) || frame.empty()) {
            std::cout << "End of video stream reached." << std::endl;
            break;
        }

        if (!tracker_->isInitialized()) {
            if (target_template_gray_.empty()) {
                // First selection (or an explicit request to select a different target).
                cv::Rect roi;
                if (selectInitialRegion(frame, roi)) {
                    target_template_gray_ = toGray(frame(roi)).clone();
                    if (!tracker_->initialize(frame, roi)) {
                        std::cerr << "Tracker initialization failed for the selected ROI." << std::endl;
                        tracker_->reset();
                        last_roi_ = cv::Rect();
                        current_status_ = TrackingStatus::LOST;
                    } else {
                        last_roi_ = roi;
                        current_status_ = TrackingStatus::TRACKING;
                    }
                } else {
                    current_status_ = TrackingStatus::NOT_INITIALIZED;
                }
            } else {
                // CSRT is no longer initialized. Search the frame using the original ROI
                // appearance, then initialize CSRT again only when a sufficiently strong
                // match is found.
                current_status_ = TrackingStatus::LOST;
                cv::Rect recoveredRoi;
                if (findTemplateMatch(frame, recoveredRoi)) {
                    if (tracker_->initialize(frame, recoveredRoi)) {
                        last_roi_ = recoveredRoi;
                        current_status_ = TrackingStatus::TRACKING;
                        std::cout << "Target reacquired at x=" << recoveredRoi.x
                                  << ", y=" << recoveredRoi.y << std::endl;
                    } else {
                        tracker_->reset();
                        last_roi_ = cv::Rect();
                    }
                }
            }
        } else {
            cv::Rect roi = last_roi_;
            const bool updateSucceeded = tracker_->update(frame, roi);
            const bool validBox = updateSucceeded && ObjectPosition::isValidRect(roi, frame.size());
            const double similarity = validBox ? templateSimilarity(frame, roi) : -1.0;

            // Important: CSRT returning true is not proof that the box still contains the
            // selected target. Validate appearance too, so drifting boxes trigger recovery.
            if (!validBox || similarity < kMinimumTrackingSimilarity) {
                tracker_->reset();
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
        if (!recordingStarted) {
            recordingPath = makeRecordingPath(video_source_->outputDirectory());
            double fps = video_source_->framesPerSecond();
            if (!std::isfinite(fps) || fps <= 0.0) {
                fps = 30.0;
                std::cerr << "Video source did not report a valid frame rate; recording at 30 FPS."
                          << std::endl;
            }

            const bool isColor = frame.channels() != 1;
            if (!writer.open(recordingPath.string(),
                             cv::VideoWriter::fourcc('m', 'p', '4', 'v'),
                             fps, frame.size(), isColor)) {
                std::cerr << "Failed to create recording: " << recordingPath.string() << std::endl;
                video_source_->close();
                cv::destroyAllWindows();
                return 1;
            }
            recordingStarted = true;
            std::cout << "Recording rendered video to: " << recordingPath.string() << std::endl;
        }
        writer.write(frame);
        cv::imshow(window_name_, frame);

        const int key = cv::waitKey(1) & 0xFF;
        if (key == 27 || key == 'q' || key == 'Q') {
            break;
        }
        if (key == 'r' || key == 'R') {
            tracker_->reset();
            // R means manually choose a new target, so discard the old appearance template.
            target_template_gray_.release();
            current_status_ = TrackingStatus::NOT_INITIALIZED;
            last_roi_ = cv::Rect();
            std::cout << "Tracker reset. Please select a new ROI." << std::endl;
        }
    }

    if (recordingStarted) {
        writer.release();
    }
    video_source_->close();
    cv::destroyAllWindows();
    return 0;
}

}  // namespace drone_tracker
