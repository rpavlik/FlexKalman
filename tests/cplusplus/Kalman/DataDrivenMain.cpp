/** @file
    @brief Header

    @date 2020

    @author
    Ryan Pavlik
    <ryan.pavlik@collabora.com>

*/

// Copyright 2020 Collabora, Ltd.
//
// SPDX-License-Identifier: BSL-1.0

#include "DataDriven.h"

#include "FlexKalman/FlexibleKalmanFilter.h"

#include "ParsePose.h"
#include "csv.hpp"

#include "CLI/CLI11.hpp"

#include <fstream>
#include <iostream>
#include <sstream>

enum class OutputType { Prediction, Correction };

class App {
    PoseFilterInterface &poseFilter;
    bool csvHasHeader = false;
    double timestamp = 0;
    std::string measType;

    void output(OutputType type, size_t iteration,
                Pose const *realPose = nullptr) {
        auto typeString =
            (type == OutputType::Prediction) ? "prediction" : "correction";
        std::cout << "After iteration " << iteration << " " << typeString
                  << ":\n";
        poseFilter.dumpState(std::cout, showCovariance);
        std::cout << "\n";
        if (type == OutputType::Prediction && !outputPredictions) {
            return;
        }
        if (outCsv.is_open()) {
            if (!csvHasHeader) {
                csvHasHeader = true;
                outCsv << "meastype,type,iteration,time,x,y,z,qx,qy,qz,qw,";
                outCsv
                    << "real_x,real_y,real_z,real_qx,real_qy,real_qz,real_qw,";
                outCsv << "err_x,err_y,err_z,";
                outCsv << "\n";
            }
            outCsv << measType << ",";
            measType = "";
            outCsv << typeString << ",";
            outCsv << iteration << ",";
            outCsv << timestamp << ",";
            Eigen::Vector3d pos = poseFilter.getPosition();
            outCsv << pos.x() << "," << pos.y() << "," << pos.z() << ",";
            Eigen::Quaterniond quat = poseFilter.getOrientation();
            outCsv << quat.x() << "," << quat.y() << "," << quat.z() << ","
                   << quat.w() << ",";
            if (realPose) {
                outCsv << realPose->position.x() << ","
                       << realPose->position.y() << ","
                       << realPose->position.z() << ",";
                outCsv << realPose->orientation.x() << ","
                       << realPose->orientation.y() << ","
                       << realPose->orientation.z() << ","
                       << realPose->orientation.w() << ",";
                outCsv << pos.x() - realPose->position.x() << ","
                       << pos.y() - realPose->position.y() << ","
                       << pos.z() - realPose->position.z() << ",";
            }
            outCsv << "\n";
        }
    }

  public:
    App(PoseFilterInterface &filter) : poseFilter(filter) {}
    bool showCovariance = false;
    bool outputPredictions = false;
    std::ofstream outCsv;

    int run(std::string const &fn) {

        csv::CSVReader in(fn);
        bool haveRealPose = csv::CSV_NOT_FOUND != in.index_of("real_x");
        size_t iteration = 0;
        for (csv::CSVRow &row : in) {
            EntryType type;
            Eigen::Quaterniond orientation;
            Eigen::Vector3d position;
            Eigen::Vector3d offset;

            double dt = 0.1;
            if (!parseEntry(row, type, dt, orientation, position, offset)) {
                std::cout << "End of input data.\n";
                break;
            }
            if (dt != 0) {
                timestamp += dt;
                poseFilter.predict(dt);
                output(OutputType::Prediction, iteration);
            }
            switch (type) {
            case EntryType::Orientation: {
                std::cout << "Orientation: " << orientation.coeffs().transpose()
                          << "\n";
                measType = "orientation";
                if (!poseFilter.filterOrientation(
                        orientation, Eigen::Vector3d::Constant(1e-4))) {
                    std::cout
                        << "Non-finite stuff when filtering orientation!\n";
                    return -1;
                }
                break;
            }
            case EntryType::Position: {
                std::cout << "Position: " << position.transpose() << "\n";
                measType = "position";
                if (!poseFilter.filterPosition(
                        position, Eigen::Vector3d::Constant(1e-3))) {
                    std::cout << "Non-finite stuff when filtering pos!\n";
                    return -1;
                }
                break;
            }
            case EntryType::PositionLeverArm: {
                std::cout << "Position: " << position.transpose()
                          << "  at Offset: " << offset.transpose() << "\n";
                measType = "positionLeverArm";
                if (!poseFilter.filterPositionLeverArm(
                        position, Eigen::Vector3d::Constant(1e-3), offset)) {
                    std::cout
                        << "Non-finite stuff when filtering pos lever arm!\n";
                    return -1;
                }
                break;
            }
            }
            if (haveRealPose) {
                Pose pose = parseRealPose(row);
                output(OutputType::Correction, iteration, &pose);
            } else {
                output(OutputType::Correction, iteration);
            }
            ++iteration;
        }
        return 0;
    }
};

enum class Filter { ExpMap, ExternalQuat };
int main(int argc, char *argv[]) {

    CLI::App app{"Data-driven test utility for flexkalman"};
    std::string fn;
    app.add_option("datafile", fn,
                   "CSV file containing measurements to process")
        ->required()
        ->check(CLI::ExistingFile);
    Filter filterType;
    std::map<std::string, Filter> map{{"expmap", Filter::ExpMap},
                                      {"externalquat", Filter::ExternalQuat}};
    app.add_option("--filter", filterType, "Filter to use")
        ->default_str("expmap")
        ->transform(CLI::CheckedTransformer(map, CLI::ignore_case));

    std::string outFile;
    auto write_csv = app.add_option("-o,--output", outFile,
                                    "Filename to write output CSV to.");

    auto show_cov = app.add_flag("-c,--show-covariance",
                                 "Output the state covariance at each "
                                 "step, not just the state itself.");

    auto output_pred = app.add_flag(
        "-p,--output-predictions", "Output the predicted state to file at each "
                                   "step, in addition to the corrected state.");

    CLI11_PARSE(app, argc, argv);

    PoseFilterPtr filter;
    switch (filterType) {
    case Filter::ExpMap:
        std::cout << "Pose with matrix exponential map rotation state.\n";
        filter = makeExpFilter();
        break;
    case Filter::ExternalQuat:
        std::cout << "Pose with externalized quaternion state.\n";
        filter = makeExternalQuatFilter();
        break;
    default:
        std::cerr << "Invalid filter" << std::endl;
        return 1;
    }
    // auto filter = factory();
    auto filterApp = App{*filter};
    if (show_cov->count()) {
        filterApp.showCovariance = true;
    }
    if (output_pred->count()) {
        filterApp.outputPredictions = true;
    }
    if (write_csv->count()) {
        filterApp.outCsv.open(outFile);
    }
    return filterApp.run(fn);
}

