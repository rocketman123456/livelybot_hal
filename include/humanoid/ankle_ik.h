#pragma once

#include <Eigen/Eigen>

// d : motor to center distance
// L1 : motor link length
// h1 : left link length
// h2 : right link length
// tx : ankle target angle x, in rad
// ty : ankle target angle y. in rad
Eigen::Vector2d ankle_ik(double d, double L1, double h1, double h2, double tx, double ty);
