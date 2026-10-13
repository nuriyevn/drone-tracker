#include <filesystem>
#include <iostream>
#include <string>

#include "application/TrackingApplication.hpp"
#include "sources/FileVideoSource.hpp"
#include "tracking/CSRTTracker.hpp"

namespace {

void printUsage(const char* programName)
{
    std::cout << "Usage: " << programName << " --source file --input <path-to-video>\n"
              << "Example: " << programName << " --source file --input \"C:/Videos/drone.mp4\"\n";
}

bool parseCommandLine(int argc,
                      char** argv,
                      std::string& sourceType,
                      std::string& inputPath,
                      bool& showHelp)
{
    for (int index = 1; index < argc; ++index) {
        const std::string argument = argv[index];

        if (argument == "--help" || argument == "-h") {
            showHelp = true;
            return true;
        }

        if (argument == "--source") {
            if (index + 1 >= argc) {
                std::cerr << "Missing value for --source." << std::endl;
                return false;
            }
            sourceType = argv[++index];
            continue;
        }

        if (argument == "--input") {
            if (index + 1 >= argc) {
                std::cerr << "Missing value for --input." << std::endl;
                return false;
            }
            inputPath = argv[++index];
            continue;
        }

        std::cerr << "Unsupported argument: " << argument << std::endl;
        return false;
    }

    if (sourceType.empty()) {
        std::cerr << "Missing required --source argument." << std::endl;
        return false;
    }

    if (inputPath.empty()) {
        std::cerr << "Missing required --input argument." << std::endl;
        return false;
    }

    return true;
}

}  // namespace

int main(int argc, char** argv)
{
    std::string sourceType;
    std::string inputPath;
    bool showHelp = false;

    if (!parseCommandLine(argc, argv, sourceType, inputPath, showHelp)) {
        printUsage(argv[0]);
        return 1;
    }

    if (showHelp) {
        printUsage(argv[0]);
        return 0;
    }

    if (sourceType != "file") {
        std::cerr << "Unsupported source type: " << sourceType
                  << ". Supported type is: file" << std::endl;
        printUsage(argv[0]);
        return 1;
    }

    if (!std::filesystem::exists(inputPath)) {
        std::cerr << "Input file does not exist: " << inputPath << std::endl;
        return 1;
    }

    auto videoSource = std::make_unique<drone_tracker::FileVideoSource>(inputPath);
    auto tracker = std::make_unique<drone_tracker::CSRTTracker>();
    drone_tracker::TrackingApplication app(std::move(videoSource), std::move(tracker));
    return app.run();
}