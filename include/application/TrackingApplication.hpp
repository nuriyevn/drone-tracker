#pragma once

#include <memory>
#include <string>

#include <opencv2/core.hpp>

#include "interfaces/IObjectTracker.hpp"
#include "interfaces/IVideoSource.hpp"
#include "models/ObjectPosition.hpp"

namespace drone_tracker {

struct TrackingGeometryLimits {
    double maxCenterDisplacementInBoxSizes = 1.5;
    double minFrameToFrameScaleRatio = 0.65;
    double maxFrameToFrameScaleRatio = 1.55;
    double minInitialScaleRatio = 0.35;
    double maxInitialScaleRatio = 2.5;
};

class TrackingApplication {
public:
    TrackingApplication(std::unique_ptr<IVideoSource> videoSource,
                       std::unique_ptr<IObjectTracker> tracker,
                       std::string windowName = "Drone Tracker",
                       TrackingGeometryLimits geometryLimits = {});

    int run();

private:
    bool selectInitialRegion(cv::Mat& frame, cv::Rect& roi) const;
    bool findTemplateMatch(const cv::Mat& frame, cv::Rect& roi, double& similarity) const;
    double templateSimilarity(const cv::Mat& frame, const cv::Rect& roi) const;
    bool hasPlausibleGeometry(const cv::Rect& previousRoi, const cv::Rect& roi) const;
    void renderFrame(cv::Mat& frame, const cv::Rect& roi, const ObjectPosition& position,
                     TrackingStatus status, double similarity,
                     unsigned int reacquisitionMatches) const;

    std::unique_ptr<IVideoSource> video_source_;
    std::unique_ptr<IObjectTracker> tracker_;
    std::string window_name_;
    TrackingGeometryLimits geometry_limits_;
    cv::Rect last_roi_;
    cv::Rect initial_roi_;
    cv::Mat target_template_gray_;
    cv::Rect pending_reacquisition_roi_;
    unsigned int reacquisition_matches_ = 0;
    double current_similarity_ = -1.0;
    TrackingStatus current_status_ = TrackingStatus::NOT_INITIALIZED;
};

}  // namespace drone_tracker
