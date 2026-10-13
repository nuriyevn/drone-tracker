#include "sources/FileVideoSource.hpp"

#include <opencv2/videoio.hpp>

namespace drone_tracker {

FileVideoSource::FileVideoSource(std::string path) : path_(std::move(path)) {}

bool FileVideoSource::open()
{
    if (path_.empty()) {
        return false;
    }

    capture_.open(path_);
    return capture_.isOpened();
}

bool FileVideoSource::read(cv::Mat& frame)
{
    if (!capture_.isOpened()) {
        return false;
    }

    return capture_.read(frame);
}

bool FileVideoSource::isOpened() const
{
    return capture_.isOpened();
}

void FileVideoSource::close()
{
    capture_.release();
}

const std::string& FileVideoSource::path() const noexcept
{
    return path_;
}

}  // namespace drone_tracker
