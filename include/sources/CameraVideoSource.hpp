#pragma once

#include <opencv2/videoio.hpp>

#include "interfaces/IVideoSource.hpp"

namespace drone_tracker {

class CameraVideoSource : public IVideoSource {
public:
    bool open() override;
    bool read(cv::Mat& frame) override;
    bool isOpened() const override;
    void close() override;

private:
    cv::VideoCapture capture_;
};

}  // namespace drone_tracker
