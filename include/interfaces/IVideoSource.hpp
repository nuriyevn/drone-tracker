#pragma once

#include <filesystem>

#include <opencv2/core.hpp>

namespace drone_tracker {

class IVideoSource {
public:
    virtual ~IVideoSource() = default;

    virtual bool open() = 0;
    virtual bool read(cv::Mat& frame) = 0;
    virtual bool isOpened() const = 0;
    virtual void close() = 0;

    virtual double framesPerSecond() const { return 0.0; }
    virtual std::filesystem::path outputDirectory() const { return {}; }
};

}  // namespace drone_tracker
