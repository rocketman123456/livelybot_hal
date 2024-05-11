#pragma once

#include <Eigen/Eigen>

// d : motor to center distance
// L1 : motor link length
// h1 : left link length
// h2 : right link length
// m1 : ankle left motor angle, in rad
// m2 : ankle right motor angle. in rad
Eigen::Vector2d ankle_fk(double d, double L1, double h1, double h2, double mL, double mR);
