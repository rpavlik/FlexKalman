
#include "FlexKalman/EigenQuatExponentialMap.h"
#include <Eigen/Eigen>
#include <fstream>

#include "../Util/CSV.h"
#include "../Util/CSVCellGroup.h"

using namespace videotracker::util;

int main() {
    CSV csv;
    for (int i = 0; i < 100; ++i) {

        auto rotVec = Eigen::Vector3d{i / 2000., 0, 0};
        auto full_exp = flexkalman::util::quat_exp(rotVec);
        auto small_angle = flexkalman::util::small_angle_quat_exp(rotVec);
        csv.row() << cellGroup(rotVec) << cellGroup("smallAngle.", small_angle)
                  << cellGroup("fullExp.", full_exp);
    }
    std::ofstream os{"data.csv"};
    csv.output(os);
    return 0;
}
