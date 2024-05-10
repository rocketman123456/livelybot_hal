#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/algorithm/contact-dynamics.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/multibody/fwd.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <Eigen/Eigen>
#include <ros/package.h>

#include <iostream>

int main()
{
    std::string path = ros::package::getPath("livelybot_urdf");
    path += "/urdf/livelybot_urdf.urdf";

    pinocchio::Model model;
    pinocchio::urdf::buildModel(path, model);
    pinocchio::Data data(model);
    std::cout << model.name << std::endl;

    Eigen::VectorXd q = pinocchio::neutral(model);
    // std::cout << std::setprecision(4) << q.rows() << " q: " << q.transpose() << std::endl;
    pinocchio::forwardKinematics(model, data, q);
    pinocchio::updateFramePlacements(model, data);

    pinocchio::FrameIndex left = 12;
    pinocchio::FrameIndex rght = 22;

    auto j   = pinocchio::computeJointJacobians(model, data, q);
    auto jl  = j.block<6, 6>(0, 0);
    auto jr  = j.block<6, 6>(0, 6);
    auto jal = j.block<6, 4>(0, 12);
    auto jar = j.block<6, 4>(0, 16);
    // std::cout << std::setprecision(4) << j.rows() << "," << j.cols() << "; J: " << j << std::endl;
    std::cout << "JL: " << jl << std::endl;
    std::cout << "JR: " << jr << std::endl;
    std::cout << "JAL: " << jal << std::endl;
    std::cout << "JAR: " << jar << std::endl;

    Eigen::VectorXd v         = pinocchio::neutral(model);
    Eigen::VectorXd a         = pinocchio::neutral(model);
    auto            tau       = pinocchio::rnea(model, data, q, v, a);
    auto            tau_left  = tau.block<6, 1>(0, 0);
    auto            tau_right = tau.block<6, 1>(6, 0);
    auto            tau_arm_left  = tau.block<4, 1>(12, 0);
    auto            tau_arm_right = tau.block<4, 1>(16, 0);
    // std::cout << std::setprecision(4) << tau.rows() << " tau: " << tau.transpose() << std::endl;
    std::cout << "tau_left:" << tau_left.transpose() << std::endl;
    std::cout << "tau_right:" << tau_right.transpose() << std::endl;
    std::cout << "tau_arm_left:" << tau_arm_left.transpose() << std::endl;
    std::cout << "tau_arm_right:" << tau_arm_right.transpose() << std::endl;

    auto m       = pinocchio::crba(model, data, q);
    auto m_left  = m.block<6, 6>(0, 0);
    auto m_right = m.block<6, 6>(6, 6);
    auto m_a_left  = m.block<4, 4>(12, 12);
    auto m_a_right = m.block<4, 4>(16, 16);
    // std::cout << std::setprecision(4) << m.rows() << "," << m.cols() << "; M:" << m << std::endl;
    std::cout << "m_left:" << m_left << std::endl;
    std::cout << "m_right:" << m_right << std::endl;
    std::cout << "m_a_left:" << m_a_left << std::endl;
    std::cout << "m_a_right:" << m_a_right << std::endl;

    std::cout << "Frames in the model:" << std::endl;
    for (pinocchio::FrameIndex i = 0; i < model.nframes; ++i)
    {
        pinocchio::Frame& frame = model.frames[i];
        std::cout << "Frame " << i << ": " << frame.name << std::endl;
    }

    std::cout << "Joints in the model:" << std::endl;
    for (pinocchio::JointIndex joint_id = 0; joint_id < model.njoints; ++joint_id)
    {
        std::cout << std::setw(24) << std::left << model.names[joint_id] << ": "
                  << "id " << joint_id << " " << std::fixed << std::setprecision(4)
                  << data.oMi[joint_id].translation().transpose() << std::endl;
    }

    return 0;
}