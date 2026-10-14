#pragma once

#include <deque>
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
    double minAspectRatioToInitial = 0.65;
    double maxAspectRatioToInitial = 1.55;
    unsigned int cumulativeWindowFrames = 12;
    double minCumulativeScaleRatio = 0.75;
    double maxCumulativeScaleRatio = 1.33;
    double maxCumulativeCenterDisplacementInBoxSizes = 1.5;
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
    bool hasPlausibleGeometry(const cv::Rect& previousRoi, const cv::Rect& roi,
                              double& frameScaleRatio, double& frameCenterDisplacement,
                              double& aspectRatioToInitial, double& windowScaleRatio,
                              double& windowCenterDisplacement) const;
    void rememberAcceptedRoi(const cv::Rect& roi);
    void renderFrame(cv::Mat& frame, const cv::Rect& roi, const ObjectPosition& position,
                     TrackingStatus status, double similarity,
                     unsigned int reacquisitionMatches, double frameScaleRatio,
                     double frameCenterDisplacement, double aspectRatioToInitial,
                     double windowScaleRatio, double windowCenterDisplacement) const;

    std::unique_ptr<IVideoSource> video_source_;
    std::unique_ptr<IObjectTracker> tracker_;
    std::string window_name_;
    TrackingGeometryLimits geometry_limits_;
    cv::Rect last_roi_;
    cv::Rect initial_roi_;
    std::deque<cv::Rect> accepted_roi_history_;
    cv::Mat target_template_gray_;
    cv::Rect pending_reacquisition_roi_;
    unsigned int reacquisition_matches_ = 0;
    double current_similarity_ = -1.0;
    double current_frame_scale_ratio_ = -1.0;
    double current_frame_center_displacement_ = -1.0;
    double current_aspect_ratio_to_initial_ = -1.0;
    double current_window_scale_ratio_ = -1.0;
    double current_window_center_displacement_ = -1.0;
    TrackingStatus current_status_ = TrackingStatus::NOT_INITIALIZED;
};

}  // namespace drone_tracker
