/** @file
    @brief Implementation

    @date 2016

    @author
    Sensics, Inc.
    <http://sensics.com/osvr>
*/

// Copyright 2016 Sensics, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//        http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Internal Includes
#include "videotrackershared/BlobExtractor.h"
#include "videotrackershared/BlobParams.h"
#include "videotrackershared/EdgeHoleBasedLedExtractor.h"
#include "videotrackershared/OptionalStream.h"
#include "videotrackershared/ParseBlobParams.h"
#include "videotrackershared/cvUtils.h"

// Library/third-party includes
#include <json/reader.h>
#include <json/value.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// Standard includes
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>
#include <sstream>

namespace videotracker {
static bool g_showAllRejects = false;
static EdgeHoleParams g_holeExtractorParams{};
static BlobParams g_blobParams{};
void showImage(std::string const &title, cv::Mat const &img,
               bool showImages = true) {
    if (showImages) {
        cv::namedWindow(title);
        cv::imshow(title, img);
    }
}

void handleImage(std::string const &fn, cv::Mat color, cv::Mat gray, bool pause,
                 bool showImages = true) {
    BlobParams &p = g_blobParams;

    /// Initial image loading
    std::cout << "Handling image " << fn << std::endl;

    // showImage("Original", gray);
    EdgeHoleBasedLedExtractor extractor{g_holeExtractorParams};
    extractor(gray, p, pause);

    /// Edge detection
    showImage("Edges", extractor.getEdgeDetectedImage(), showImages);
    cv::imwrite(fn + ".edge.png", extractor.getEdgeDetectedImage());

    showImage("Binarized Contour Input",
              extractor.getEdgeDetectedBinarizedImage(), showImages);
    cv::imwrite(fn + ".binarized.png",
                extractor.getEdgeDetectedBinarizedImage());

    /// Extract beacons from the edge detection image

    /// Produce a colored debug image where each accepted contour is
    /// encircled with a different color.
    cv::Mat highlightedContours =
        drawColoredContours(gray, extractor.getContours());

    // Figure out where to put the text - further along the ray
    // that passes from the image center through the reject center, to try
    // to get it "away from the crowd"
    {
        static const auto FONT_FACE = cv::FONT_HERSHEY_PLAIN;
        static const auto FONT_SCALE = 0.8;
        static const int FONT_THICKNESS = 1;

        auto imageSize = highlightedContours.size();
        auto imageCenter =
            cv::Point2d(imageSize.width / 2, imageSize.height / 2);
        std::random_device rd;
        std::mt19937 mt(rd());
        std::uniform_real_distribution<double> vectorScaleDistribution(1.1,
                                                                       1.3);
        auto computeTextLocation = [&](cv::Point2d rejectCenter,
                                       std::string const &text) {
            auto centerToRejectVec = rejectCenter - imageCenter;
            auto textPoint = cv::Point(
                centerToRejectVec * vectorScaleDistribution(mt) + imageCenter);

            /// OK, so that's where we'll center the text, now we have to
            /// see how big the text is to offset it...
            auto textSize = cv::getTextSize(text, FONT_FACE, FONT_SCALE,
                                            FONT_THICKNESS, nullptr);

            auto textOffset =
                cv::Point(textSize.width / 2, textSize.height / 2);
            return textPoint - textOffset;
        };

        /// Draw and label the rejected centers of contours.
        for (auto &reject : extractor.getRejectList()) {
            EdgeHoleBasedLedExtractor::ContourId contourId = 0;
            RejectReason reason;
            cv::Point2d center;
            std::tie(contourId, reason, center) = reject;

            if (!g_showAllRejects &&
                (RejectReason::Area == reason ||
                 RejectReason::CenterPointValue == reason)) {
                // Skip drawing these, they clutter the display
                continue;
            }

            drawSubpixelPoint(highlightedContours, center, cv::Scalar(0, 0, 0),
                              1.2);
            std::ostringstream os;
            // os << "[";
            os << contourId;
            switch (reason) {
            case RejectReason::Area:
                os << ":A";
                break;
            case RejectReason::CenterPointValue:
                os << ":V";
                break;
            case RejectReason::Circularity:
                os << ":CIRC";
                break;
            case RejectReason::Convexity:
                os << ":CONV";
                break;
            default:
                break;
            }
            // os << "]";
            auto text = os.str();
            cv::putText(highlightedContours, text,
                        computeTextLocation(center, text), FONT_FACE,
                        FONT_SCALE, cv::Scalar(0, 0, 255), FONT_THICKNESS);
        }
    }

    showImage("Selected contours", highlightedContours, showImages);
    cv::imwrite(fn + ".contours.png", highlightedContours);
    if (pause) {
        cv::waitKey();
    }
}
void handleImage(std::string const &fn, bool pause) {

    /// Initial image loading
    std::cout << "Loading image " << fn << std::endl;
    cv::Mat color = cv::imread(fn, cv::IMREAD_COLOR);
    if (!color.data) {
        std::cerr << "Could not load image!" << std::endl;
        return;
    }

    cv::Mat gray;
    cv::cvtColor(color, gray, cv::COLOR_BGR2GRAY);

    if (!gray.data) {
        std::cerr << "Conversion to gray failed?" << std::endl;
        return;
    }
    // showImage("Original", gray);
    handleImage(fn, color, gray, pause);
}
} // namespace videotracker

void processAVI(std::string const &fn) {
    cv::VideoCapture capture;
    capture.open(fn);
    if (!capture.isOpened()) {
        std::cerr << "Could not open video file " << fn << std::endl;
        return;
    }
    std::size_t i = 0;
    cv::Mat frame;
    cv::Mat frameGray;
    capture >> frame;
    while (capture.read(frame)) {
        std::ostringstream os;
        os << fn << "." << std::setw(4) << std::setfill('0') << i << ".png";
        auto pngFileName = os.str();
        cv::imwrite(pngFileName, frame);
        cv::cvtColor(frame, frameGray, cv::COLOR_BGR2GRAY);
        videotracker::handleImage(pngFileName, frame, frameGray, false, false);
        i++;
    }
}

void tryLoadingConfigFile() {
    Json::Value root;
    static const auto FN = "blobDemoConfig.json";
    {
        std::ifstream configStream{"blobDemoConfig.json"};
        if (!configStream) {
            std::cout << "Note: Did not find or could not open config file "
                      << FN << ", just using defaults..." << std::endl;
            return;
        }
        Json::Reader reader;
        if (!reader.parse(configStream, root)) {
            std::cout << "Note: Opened, but could not parse as valid JSON ["
                      << reader.getFormattedErrorMessages()
                      << "], the config file " << FN
                      << ", just using the defaults..." << std::endl;
            return;
        }
    }
    std::cout << "Opened config file " << FN << " and parsed as JSON..."
              << std::endl;
    using namespace videotracker;
    if (root.isMember("blobParams")) {
        std::cout << "Found \"blobParams\" element, parsing..." << std::endl;
        parseBlobParams(root["blobParams"], g_blobParams);
    }
    if (root.isMember("extractParams")) {
        std::cout << "Found \"extractParams\" element, parsing..." << std::endl;
        parseEdgeHoleExtractorParams(root["extractParams"],
                                     g_holeExtractorParams);
    }
    getOptionalParameter(g_showAllRejects, root, "showAllRejects");
}

int main(int argc, char *argv[]) {
    /// Look for a config file (optional)
    tryLoadingConfigFile();
    /// Don't stop before exiting if we've got multiple to process.
    if (argc == 2) {
        auto fn = std::string{argv[1]};
        if (fn.find(".avi") != std::string::npos &&
            fn.find(".avi.") == std::string::npos) {
            /// has .avi but isn't a file extracted from an AVI.
            processAVI(fn);
            return 0;
        }
    }
    bool pause = (argc < 3);
    for (int arg = 1; arg < argc; ++arg) {
        videotracker::handleImage(argv[arg], pause);
    }
    return 0;
}
