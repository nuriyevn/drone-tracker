#pragma once
#include <opencv2/videoio.hpp>s
#include <string>

#include "interfaces/IVideoSource.hpp"

namespace drone_tracker {

class FileVideoSource : public IVideoSource {
public:
    explicit FileVideoSource(std::string path);

    bool open() override;
    bool read(cv::Mat& frame) override;
    bool isOpened() const override;
    void close() override;

    const std::string& path() const noexcept;

private:
    std::string path_;
    cv::VideoCapture capture_;
};

}  // namespace drone_tracker
