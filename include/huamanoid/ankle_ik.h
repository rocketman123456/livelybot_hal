#pragma once

#include <array>

// d : motor to center distance
// L1 : motor link length
// h1 : left link length
// h2 : right link length
// tx : ankle target angle x, in rad
// ty : ankle target angle y. in rad
std::array<double, 2> ankle_ik(double d, double L1, double h1, double h2, double tx, double ty);
