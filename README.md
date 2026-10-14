# Drone Tracker

A lightweight C++17 drone tracking sample built with OpenCV and CMake. The current implementation follows a clean separation between video acquisition, tracker logic, 2D coordinate calculation, and the application loop.

## Project purpose and scope

This application reads a video file, lets the user select a drone region of interest (ROI), initializes an OpenCV CSRT tracker, and then renders the tracked object in the image while reporting the center of the ROI in pixel coordinates.

The project intentionally stays in the 2D image plane:

- X and Y are image coordinates in pixels.
- The origin is the top-left corner of the frame.
- The application does not estimate GPS, altitude, or real-world distance.

## Architecture

The repository uses interface-based components and dependency injection:

- IVideoSource: abstraction for video acquisition
- FileVideoSource: reads a local video file using cv::VideoCapture
- CameraVideoSource: enumerates Windows video-input devices and streams from the first one
- IObjectTracker: abstraction for tracking algorithms
- CSRTTracker: OpenCV CSRT implementation
- ObjectPosition: stores the center point, ROI, and tracking status
- TrackingApplication: orchestrates the main event loop and UI

## Prerequisites

- Windows 11
- CMake 3.24+
- Ninja
- Visual Studio 2022 C++ toolchain or another compatible MSVC toolchain
- vcpkg installed at C:\Google_Drive_nuriyevn\Cpp\vcpkg
- OpenCV via vcpkg

## vcpkg setup

The project uses vcpkg manifest mode. The parent vcpkg installation is expected at:

C:\Google_Drive_nuriyevn\Cpp\vcpkg

The manifest declares the OpenCV package used by the project:

- opencv4

The CMake preset points to the local vcpkg toolchain at:

../vcpkg/scripts/buildsystems/vcpkg.cmake

This keeps the project portable without hardcoding absolute machine-specific paths into the source.

## Build and run (PowerShell)

From the repository root:

```powershell
$env:VCPKG_ROOT = "C:\Cpp\vcpkg"
cmake --preset windows-ninja
cmake --build --preset windows-ninja --config Release
```

The project builds into the local build directory:

- build/

Run the application:

```powershell
./build/drone-tracker.exe --source file --input "C:/Videos/drone.mp4"
./build/drone-tracker.exe --source camera
```

The camera source uses the Windows DirectShow device enumerator to verify that a camera is available, then opens the first device through OpenCV's DirectShow backend. The app displays the stream and waits for the user to select a drone ROI.

Every rendered frame is also recorded to a timestamped MP4 file. Recordings from a video file are saved beside that input file; camera recordings are saved in the current working directory. The output name uses the format `YYYY-MM-DD_HH-MM-SS.mp4`. The source frame rate is used when available, otherwise the recording uses 30 FPS. The input file or stream is not modified.

## Controls and coordinate conventions

- Left mouse button: select the drone ROI
- Enter/Space: confirm the selection in the native OpenCV ROI selector
- Esc or Q: quit the application
- R: reset the tracker and select a new ROI after loss

Displayed coordinates are in image pixels using the origin at the upper-left corner:

- X increases to the right
- Y increases downward

Example output:

- X: 640.5, Y: 360.0

## Current limitations

- The camera source is Windows-only and always selects the first enumerated camera.
- A user must select the drone ROI manually before tracking begins.
- The current application does not implement network stream handling, neural detection, or GPS-based localization.
- Tracker recovery is manual via the R key.

## Adding a new video source

Implement a new class that derives from IVideoSource and provides:

- open()
- read(cv::Mat& frame)
- isOpened() const
- close()

Then update the application factory or command-line parser to accept the new source type. The tracking logic remains unchanged because it depends only on the generic IVideoSource abstraction.

## Future detector integration

The architecture was designed to support a detector-plus-tracker pipeline later. A detector such as YOLO could be added as a separate component that produces an ROI for the tracker. The tracker itself remains behind the IObjectTracker interface, so a detector could feed the initial selection or re-initialization logic without changing the core app loop.

## Known build note

The repository expects a working C++ toolchain and the local vcpkg installation to be present. If the toolchain or vcpkg dependency is missing, CMake will stop with a configuration error and the exact blocker should be fixed before running the application.
