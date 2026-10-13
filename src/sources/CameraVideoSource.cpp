#include "sources/CameraVideoSource.hpp"

#include <iostream>
#include <memory>

#if defined(_WIN32)
#include <dshow.h>
#include <objbase.h>
#endif

namespace drone_tracker {
namespace {

#if defined(_WIN32)

template <typename T>
struct ComReleaser {
    void operator()(T* value) const noexcept
    {
        if (value != nullptr) {
            value->Release();
        }
    }
};

class ComApartment {
public:
    ComApartment()
    {
        const HRESULT result = CoInitializeEx(nullptr, COINIT_MULTITHREADED);
        initialized_ = SUCCEEDED(result);
        valid_ = initialized_ || result == RPC_E_CHANGED_MODE;
    }

    ~ComApartment()
    {
        if (initialized_) {
            CoUninitialize();
        }
    }

    bool isValid() const noexcept
    {
        return valid_;
    }

private:
    bool initialized_ = false;
    bool valid_ = false;
};

bool hasVideoInputDevice()
{
    ComApartment apartment;
    if (!apartment.isValid()) {
        std::cerr << "Failed to initialize COM for Windows camera enumeration." << std::endl;
        return false;
    }

    ICreateDevEnum* rawDeviceEnumerator = nullptr;
    const HRESULT createResult =
        CoCreateInstance(CLSID_SystemDeviceEnum, nullptr, CLSCTX_INPROC_SERVER,
                         IID_ICreateDevEnum, reinterpret_cast<void**>(&rawDeviceEnumerator));
    if (FAILED(createResult)) {
        std::cerr << "Failed to create the Windows video-device enumerator." << std::endl;
        return false;
    }
    std::unique_ptr<ICreateDevEnum, ComReleaser<ICreateDevEnum>> deviceEnumerator(
        rawDeviceEnumerator);

    IEnumMoniker* rawCameraEnumerator = nullptr;
    const HRESULT enumerateResult =
        deviceEnumerator->CreateClassEnumerator(CLSID_VideoInputDeviceCategory,
                                                &rawCameraEnumerator, 0);
    if (enumerateResult != S_OK || rawCameraEnumerator == nullptr) {
        std::cerr << "No Windows camera devices were found." << std::endl;
        return false;
    }
    std::unique_ptr<IEnumMoniker, ComReleaser<IEnumMoniker>> cameraEnumerator(
        rawCameraEnumerator);

    IMoniker* rawFirstCamera = nullptr;
    const HRESULT firstCameraResult = cameraEnumerator->Next(1, &rawFirstCamera, nullptr);
    if (firstCameraResult != S_OK || rawFirstCamera == nullptr) {
        std::cerr << "No Windows camera devices were found." << std::endl;
        return false;
    }
    std::unique_ptr<IMoniker, ComReleaser<IMoniker>> firstCamera(rawFirstCamera);
    return true;
}

#endif

}  // namespace

bool CameraVideoSource::open()
{
    capture_.release();

#if defined(_WIN32)
    if (!hasVideoInputDevice()) {
        return false;
    }

    if (!capture_.open(0, cv::CAP_DSHOW)) {
        std::cerr << "Failed to open the first enumerated Windows camera." << std::endl;
        return false;
    }
    return true;
#else
    std::cerr << "Camera video source is only supported on Windows." << std::endl;
    return false;
#endif
}

bool CameraVideoSource::read(cv::Mat& frame)
{
    return capture_.isOpened() && capture_.read(frame);
}

bool CameraVideoSource::isOpened() const
{
    return capture_.isOpened();
}

void CameraVideoSource::close()
{
    capture_.release();
}

}  // namespace drone_tracker
