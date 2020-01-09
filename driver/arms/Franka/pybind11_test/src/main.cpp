#include <array>
#include <cmath>
#include <functional>
#include <iostream>
#include <algorithm>
#include <string>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <franka/gripper.h>
#include <franka/robot.h>
#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/control_types.h>
#include <franka/robot_state.h>
#include <franka/model.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;
class myRobot: public franka::Robot
{
public:
    franka::Model * model;
    myRobot(const std::string ip):franka::Robot(ip){
        model = new franka::Model(this->loadModel());
    };
    ~myRobot();
    void reloadModel(){
        model = new franka::Model(this->loadModel());
    }
    const std::array<double, 16> pose(franka::Frame frame, const franka::RobotState& robot_state) const { 
        return model->pose(frame,robot_state); 
    }
    const std::array<double, 42> bodyJacobian(franka::Frame frame, const franka::RobotState& robot_state) const{
        return model->bodyJacobian(frame,robot_state);
    }

    const std::array<double, 42> zeroJacobian(franka::Frame frame, const franka::RobotState& robot_state) const{
        return model->zeroJacobian(frame,robot_state);
    }
    const std::array<double, 49> mass(const franka::RobotState& robot_state) const noexcept
    {
        return model->mass(robot_state);
    }
    const std::array<double, 7> coriolis(const franka::RobotState& robot_state) const noexcept{
        return model->coriolis(robot_state);
    }
    const std::array<double, 7> gravity(const franka::RobotState& robot_state,
                                 const std::array<double, 3>& gravity_earth = {{0., 0., -9.81}}) const noexcept{
        return model->gravity(robot_state,gravity_earth);
    }
};
/*
struct myModel {
    myModel(franka::Robot& model_){
        //model = new franka::Model(robot.loadModel());
        model = &model_;
    }

    franka::Model * model;

    const std::array<double, 16> &pose(franka::Frame frame, const franka::RobotState& robot_state) const { 
        return model->pose(frame,robot_state); 
    }

    const std::array<double, 42> bodyJacobian(franka::Frame frame, const franka::RobotState& robot_state) const{
        return model->bodyJacobian(frame,robot_state);
    }

    const std::array<double, 42> zeroJacobian(franka::Frame frame, const franka::RobotState& robot_state) const{
        return model->zeroJacobian(frame,robot_state);
    }
    const std::array<double, 49> mass(const franka::RobotState& robot_state) const noexcept
    {
        return model->mass(robot_state);
    }
    const std::array<double, 7> coriolis(const franka::RobotState& robot_state) const noexcept{
        return model->coriolis(robot_state);
    }
    const std::array<double, 7> gravity(const franka::RobotState& robot_state,
                                 const std::array<double, 3>& gravity_earth = {{0., 0., -9.81}}) const noexcept{
        return model->gravity(robot_state,gravity_earth);
    }
};
*/
PYBIND11_MODULE(franka_, m) {
    m.doc() = "Python3 Franka library"; // optional module docstring
    
    py::enum_<franka::ControllerMode>(m, "ControllerMode")
        .value("kJointImpedance", franka::ControllerMode::kJointImpedance)
        .value("kCartesianImpedance", franka::ControllerMode::kCartesianImpedance)
        .export_values();

    py::enum_<franka::RealtimeConfig>(m, "RealtimeConfig")
        .value("kEnforce", franka::RealtimeConfig::kEnforce)
        .value("kIgnore", franka::RealtimeConfig::kIgnore)
        .export_values();

    py::enum_<franka::RobotMode>(m, "RobotMode")
        .value("Other", franka::RobotMode::kOther)
        .value("Idle", franka::RobotMode::kIdle)
        .value("Move", franka::RobotMode::kMove)
        .value("Guiding", franka::RobotMode::kGuiding)
        .value("Reflex", franka::RobotMode::kReflex)
        .value("UserStopped", franka::RobotMode::kUserStopped)
        .value("AutomaticErrorRecovery", franka::RobotMode::kAutomaticErrorRecovery)
        .export_values();

    py::enum_<franka::Frame>(m, "Frame")
        .value("kJoint1", franka::Frame::kJoint1)
        .value("kJoint2", franka::Frame::kJoint2)
        .value("kJoint3", franka::Frame::kJoint3)
        .value("kJoint4", franka::Frame::kJoint4)
        .value("kJoint5", franka::Frame::kJoint5)
        .value("kJoint6", franka::Frame::kJoint6)
        .value("kJoint7", franka::Frame::kJoint7)
        .value("kFlange", franka::Frame::kFlange)
        .value("kEndEffector", franka::Frame::kEndEffector)
        .value("kStiffness ", franka::Frame::kStiffness)
        .export_values();

    py::class_<franka::Duration>(m, "Duration")
        .def(py::init<>())
        .def(py::init<uint64_t>())
        .def("to_sec", &franka::Duration::toSec)
        .def("to_msec", &franka::Duration::toMSec);

    py::class_<franka::Finishable>(m, "Finishable", py::dynamic_attr())
        .def(py::init<>())
        .def_readwrite("motion_finished", &franka::Finishable::motion_finished);

    py::class_<franka::Torques, franka::Finishable>(m, "Torques", py::dynamic_attr())
        .def(py::init<const std::array<double,7>&>())
        .def_readwrite("tau_J", &franka::Torques::tau_J);

    py::class_<franka::JointPositions, franka::Finishable>(m, "JointPositions", py::dynamic_attr())
        .def(py::init<const std::array<double,7>&>())
        .def_readwrite("q", &franka::JointPositions::q);

    py::class_<franka::JointVelocities, franka::Finishable>(m, "JointVelocities", py::dynamic_attr())
        .def(py::init<const std::array<double,7>&>())
        .def_readwrite("dq", &franka::JointVelocities::dq);

    py::class_<franka::CartesianPose, franka::Finishable>(m, "CartesianPose", py::dynamic_attr())
        .def(py::init<const std::array<double,16>&>())
        .def_readwrite("O_T_EE", &franka::CartesianPose::O_T_EE)
        .def_readwrite("elbow", &franka::CartesianPose::elbow);

    py::class_<franka::CartesianVelocities, franka::Finishable>(m, "CartesianVelocities", py::dynamic_attr())
        .def(py::init<const std::array<double,6>&>())
        .def_readwrite("O_dP_EE", &franka::CartesianVelocities::O_dP_EE)
        .def_readwrite("elbow", &franka::CartesianVelocities::elbow);


    py::class_<franka::Errors>(m, "Errors")
        .def(py::init<>())
        .def_property_readonly("joint_position_limits_violation", [](const franka::Errors& e) { return e.joint_position_limits_violation; })
        .def_property_readonly("cartesian_position_limits_violation", [](const franka::Errors& e) { return e.cartesian_position_limits_violation; })
        .def_property_readonly("self_collision_avoidance_violation", [](const franka::Errors& e) { return e.self_collision_avoidance_violation; })
        .def_property_readonly("joint_velocity_violation", [](const franka::Errors& e) { return e.joint_velocity_violation; })
        .def_property_readonly("cartesian_velocity_violation", [](const franka::Errors& e) { return e.cartesian_velocity_violation; })
        .def_property_readonly("force_control_safety_violation", [](const franka::Errors& e) { return e.force_control_safety_violation; })
        .def_property_readonly("joint_reflex", [](const franka::Errors& e) { return e.joint_reflex; })
        .def_property_readonly("cartesian_reflex", [](const franka::Errors& e) { return e.cartesian_reflex; })
        .def_property_readonly("max_goal_pose_deviation_violation", [](const franka::Errors& e) { return e.max_goal_pose_deviation_violation; })
        .def_property_readonly("max_path_pose_deviation_violation", [](const franka::Errors& e) { return e.max_path_pose_deviation_violation; })
        .def_property_readonly("cartesian_velocity_profile_safety_violation", [](const franka::Errors& e) { return e.cartesian_velocity_profile_safety_violation; })
        .def_property_readonly("joint_position_motion_generator_start_pose_invalid", [](const franka::Errors& e) { return e.joint_position_motion_generator_start_pose_invalid; })
        .def_property_readonly("joint_motion_generator_position_limits_violation", [](const franka::Errors& e) { return e.joint_motion_generator_position_limits_violation; })
        .def_property_readonly("joint_motion_generator_velocity_limits_violation", [](const franka::Errors& e) { return e.joint_motion_generator_velocity_limits_violation; })
        .def_property_readonly("joint_motion_generator_velocity_discontinuity", [](const franka::Errors& e) { return e.joint_motion_generator_velocity_discontinuity; })
        .def_property_readonly("joint_motion_generator_acceleration_discontinuity", [](const franka::Errors& e) { return e.joint_motion_generator_acceleration_discontinuity; })
        .def_property_readonly("cartesian_position_motion_generator_start_pose_invalid", [](const franka::Errors& e) { return e.cartesian_position_motion_generator_start_pose_invalid; })
        .def_property_readonly("cartesian_motion_generator_elbow_limit_violation", [](const franka::Errors& e) { return e.cartesian_motion_generator_elbow_limit_violation; })
        .def_property_readonly("cartesian_motion_generator_velocity_limits_violation", [](const franka::Errors& e) { return e.cartesian_motion_generator_velocity_limits_violation; })
        .def_property_readonly("cartesian_motion_generator_velocity_discontinuity", [](const franka::Errors& e) { return e.cartesian_motion_generator_velocity_discontinuity; })
        .def_property_readonly("cartesian_motion_generator_acceleration_discontinuity", [](const franka::Errors& e) { return e.cartesian_motion_generator_acceleration_discontinuity; })
        .def_property_readonly("cartesian_motion_generator_elbow_sign_inconsistent", [](const franka::Errors& e) { return e.cartesian_motion_generator_elbow_sign_inconsistent; })
        .def_property_readonly("cartesian_motion_generator_start_elbow_invalid", [](const franka::Errors& e) { return e.cartesian_motion_generator_start_elbow_invalid; })
        .def_property_readonly("cartesian_motion_generator_joint_position_limits_violation", [](const franka::Errors& e) { return e.cartesian_motion_generator_joint_position_limits_violation; })
        .def_property_readonly("cartesian_motion_generator_joint_velocity_limits_violation", [](const franka::Errors& e) { return e.cartesian_motion_generator_joint_velocity_limits_violation; })
        .def_property_readonly("cartesian_motion_generator_joint_velocity_discontinuity", [](const franka::Errors& e) { return e.cartesian_motion_generator_joint_velocity_discontinuity; })
        .def_property_readonly("cartesian_motion_generator_joint_acceleration_discontinuity", [](const franka::Errors& e) { return e.cartesian_motion_generator_joint_acceleration_discontinuity; })
        .def_property_readonly("cartesian_position_motion_generator_invalid_frame", [](const franka::Errors& e) { return e.cartesian_position_motion_generator_invalid_frame; })
        .def_property_readonly("force_controller_desired_force_tolerance_violation", [](const franka::Errors& e) { return e.force_controller_desired_force_tolerance_violation; })
        .def_property_readonly("controller_torque_discontinuity", [](const franka::Errors& e) { return e.controller_torque_discontinuity; })
        .def_property_readonly("start_elbow_sign_inconsistent", [](const franka::Errors& e) { return e.start_elbow_sign_inconsistent; })
        .def_property_readonly("communication_constraints_violation", [](const franka::Errors& e) { return e.communication_constraints_violation; })
        .def_property_readonly("power_limit_violation", [](const franka::Errors& e) { return e.power_limit_violation; })
        .def_property_readonly("joint_p2p_insufficient_torque_for_planning", [](const franka::Errors& e) { return e.joint_p2p_insufficient_torque_for_planning; })
        .def_property_readonly("tau_j_range_violation", [](const franka::Errors& e) { return e.tau_j_range_violation; })
        .def_property_readonly("instability_detected", [](const franka::Errors& e) { return e.instability_detected; })
        .def_property_readonly("joint_move_in_wrong_direction", [](const franka::Errors& e) { return e.joint_move_in_wrong_direction; })
        .def("__repr__",[](const franka::Errors& e) {return std::string(e);});

    py::class_<franka::RobotState>(m, "RobotState")
        .def_readonly("O_T_EE", &franka::RobotState::O_T_EE)
        .def_readonly("O_T_EE_d", &franka::RobotState::O_T_EE_d)
        .def_readonly("F_T_EE", &franka::RobotState::F_T_EE)
        .def_readonly("EE_T_K", &franka::RobotState::EE_T_K)
        .def_readonly("m_ee", &franka::RobotState::m_ee)
        .def_readonly("I_ee", &franka::RobotState::I_ee)
        .def_readonly("F_x_Cee", &franka::RobotState::F_x_Cee)
        .def_readonly("m_load", &franka::RobotState::m_load)
        .def_readonly("I_load", &franka::RobotState::I_load)
        .def_readonly("F_x_Cload", &franka::RobotState::F_x_Cload)
        .def_readonly("m_total", &franka::RobotState::m_total)
        .def_readonly("I_total", &franka::RobotState::I_total)
        .def_readonly("F_x_Ctotal", &franka::RobotState::F_x_Ctotal)
        .def_readonly("elbow", &franka::RobotState::elbow)
        .def_readonly("elbow_d", &franka::RobotState::elbow_d)
        .def_readonly("elbow_c", &franka::RobotState::elbow_c)
        .def_readonly("delbow_c", &franka::RobotState::delbow_c)
        .def_readonly("ddelbow_c", &franka::RobotState::ddelbow_c)
        .def_readonly("tau_J", &franka::RobotState::tau_J)
        .def_readonly("tau_J_d", &franka::RobotState::tau_J_d)
        .def_readonly("dtau_J", &franka::RobotState::dtau_J)
        .def_readonly("q", &franka::RobotState::q)
        .def_readonly("q_d", &franka::RobotState::q_d)
        .def_readonly("dq", &franka::RobotState::dq)
        .def_readonly("dq_d", &franka::RobotState::dq_d)
        .def_readonly("ddq_d", &franka::RobotState::ddq_d)
        .def_readonly("joint_contact", &franka::RobotState::m_total)
        .def_readonly("cartesian_contact", &franka::RobotState::cartesian_contact)
        .def_readonly("joint_collision", &franka::RobotState::joint_collision)
        .def_readonly("cartesian_collision", &franka::RobotState::cartesian_collision)
        .def_readonly("tau_ext_hat_filtered", &franka::RobotState::tau_ext_hat_filtered)
        .def_readonly("O_T_EE_c", &franka::RobotState::O_T_EE_c)
        .def_readonly("O_dP_EE_c", &franka::RobotState::O_dP_EE_c)
        .def_readonly("O_ddP_EE_c", &franka::RobotState::O_ddP_EE_c)
        .def_readonly("theta", &franka::RobotState::theta)
        .def_readonly("dtheta", &franka::RobotState::dtheta)
        .def_readonly("current_errors", &franka::RobotState::current_errors)
        .def_readonly("last_motion_errors", &franka::RobotState::last_motion_errors)
        .def_readonly("control_command_success_rate", &franka::RobotState::control_command_success_rate)
        .def_readonly("robot_mode", &franka::RobotState::robot_mode)
        .def_readonly("time", &franka::RobotState::time);

    py::class_<myRobot>(m, "Robot")
        .def(py::init<const std::string &>())
        //read
        .def("readOnce", &myRobot::readOnce)
        .def("loadModel",&myRobot::loadModel)
        .def("serverVersion",&myRobot::serverVersion)
        
        .def("control",(void (myRobot::*)(std::function<franka::Torques(const franka::RobotState&, franka::Duration)>, 
                bool, double)) &myRobot::control,
            py::arg("control_callback"),
            py::arg("limit_rate") = true, 
            py::arg("cutoff_frequency") = 100.0)
        
        .def("control",(void (myRobot::*)(std::function<franka::Torques(const franka::RobotState&, franka::Duration)>,
                std::function< franka::JointPositions(const franka::RobotState &, franka::Duration)>,
                bool, double)) &myRobot::control,
            py::arg("control_callback"),
            py::arg("motion_generator_callback"),
            py::arg("limit_rate") = true, 
            py::arg("cutoff_frequency") = 100.0)
        
        .def("control",(void (myRobot::*)(std::function<franka::Torques(const franka::RobotState&, franka::Duration)>,
                std::function< franka::JointVelocities(const franka::RobotState &, franka::Duration)>,
                bool, double)) &myRobot::control,
            py::arg("control_callback"),
            py::arg("motion_generator_callback"),
            py::arg("limit_rate") = true, 
            py::arg("cutoff_frequency") = 100.0)

        .def("control",(void (myRobot::*)(std::function<franka::Torques(const franka::RobotState&, franka::Duration)>,
                std::function< franka::CartesianPose(const franka::RobotState &, franka::Duration)>,
                bool, double)) &myRobot::control,
            py::arg("control_callback"),
            py::arg("motion_generator_callback"),
            py::arg("limit_rate") = true, 
            py::arg("cutoff_frequency") = 100.0)

        .def("control",(void (myRobot::*)(std::function<franka::Torques(const franka::RobotState&, franka::Duration)>,
                std::function< franka::CartesianVelocities(const franka::RobotState &, franka::Duration)>,
                bool, double)) &myRobot::control,
            py::arg("control_callback"),
            py::arg("motion_generator_callback"),
            py::arg("limit_rate") = true, 
            py::arg("cutoff_frequency") = 100.0)

        .def("control",(void (myRobot::*)(std::function<franka::JointPositions(const franka::RobotState&, franka::Duration)>, 
                franka::ControllerMode, bool, double)) &myRobot::control,
            py::arg("motion_generator_callback"),
            py::arg("controller_mode") = franka::ControllerMode::kJointImpedance, 
            py::arg("limit_rate") = true, 
            py::arg("cutoff_frequency") = 100.0)

        .def("control",(void (myRobot::*)(std::function<franka::JointVelocities(const franka::RobotState&, franka::Duration)>, 
                franka::ControllerMode, bool, double)) &myRobot::control,
            py::arg("motion_generator_callback"),
            py::arg("controller_mode") = franka::ControllerMode::kJointImpedance, 
            py::arg("limit_rate") = true, 
            py::arg("cutoff_frequency") = 100.0)

        .def("control",(void (myRobot::*)(std::function<franka::CartesianPose(const franka::RobotState&, franka::Duration)>, 
                franka::ControllerMode, bool, double)) &myRobot::control,
            py::arg("motion_generator_callback"),
            py::arg("controller_mode") = franka::ControllerMode::kJointImpedance, 
            py::arg("limit_rate") = true, 
            py::arg("cutoff_frequency") = 100.0)

        .def("control",(void (myRobot::*)(std::function<franka::CartesianVelocities(const franka::RobotState&, franka::Duration)>, 
                franka::ControllerMode, bool, double)) &myRobot::control,
            py::arg("motion_generator_callback"),
            py::arg("controller_mode") = franka::ControllerMode::kJointImpedance, 
            py::arg("limit_rate") = true, 
            py::arg("cutoff_frequency") = 100.0)
        
        .def("setCollisionBehavior", (void (myRobot::*)(const std::array<double, 7ul>&,
            const std::array<double, 7ul>&, const std::array<double, 7ul>&, const std::array<double, 7ul>&, 
            const std::array<double, 6ul>&, const std::array<double, 6ul>&, const std::array<double, 6ul>&, 
            const std::array<double, 6ul>&))&myRobot::setCollisionBehavior)

        .def("setCollisionBehavior", (void (myRobot::*)(const std::array<double, 7ul>&, 
            const std::array<double, 7ul>&, const std::array<double, 6ul>&, const std::array<double, 6ul>&))
            &myRobot::setCollisionBehavior)

        .def("setJointImpedance", &myRobot::setJointImpedance)
        .def("setCartesianImpedance", &myRobot::setCartesianImpedance)
        .def("setGuidingMode", &myRobot::setGuidingMode)
        .def("setK", &myRobot::setK)
        .def("setEE", &myRobot::setEE)
        .def("setLoad", &myRobot::setLoad)
        .def("setFilters", &franka::lowpassFilter)
        .def("automaticErrorRecovery", &myRobot::automaticErrorRecovery)
        .def("stop", &myRobot::stop)
        .def("reloadModel", &myRobot::reloadModel)
        .def("pose", &myRobot::pose)
        .def("bodyJacobian", &myRobot::bodyJacobian)
        .def("zeroJacobian", &myRobot::zeroJacobian)
        .def("mass", &myRobot::mass)
        .def("coriolis", &myRobot::coriolis)
        .def("gravity", &myRobot::gravity);
    
    py::class_<franka::Gripper>(m,"Gripper")
        .def(py::init<const std::string &>())
        .def("homing", &franka::Gripper::homing)
        .def("stop", &franka::Gripper::stop)
        .def("grasp", &franka::Gripper::grasp, 
            py::arg("width"),
            py::arg("speed"),
            py::arg("force"),
            py::arg("epsilon_inner") = 0.005,
            py::arg("epsilon_outer") = 0.005)
        .def("move", &franka::Gripper::move, py::arg("width"),py::arg("speed"))
        .def("stop", &franka::Gripper::stop)
        .def("readOnce", &franka::Gripper::readOnce)
        .def("serverVersion", &franka::Gripper::serverVersion);
    /*
    py::class_<franka::Model>(m, "Model")
        .def(py::init<franka::Network &>())
        .def("pose", &franka::Model::pose)
        ;
    */
    py::class_<franka::GripperState>(m, "GripperState")
        .def_readonly("width", &franka::GripperState::width)
        .def_readonly("max_width", &franka::GripperState::max_width)
        .def_readonly("is_grasped", &franka::GripperState::is_grasped)
        .def_readonly("temperature", &franka::GripperState::temperature)
        .def_readonly("time", &franka::GripperState::time);

    py::register_exception<franka::CommandException>(m, "CommandException");
    py::register_exception<franka::ControlException>(m, "ControlException");
    py::register_exception<franka::IncompatibleVersionException>(m, "IncompatibleVersionException");
    py::register_exception<franka::InvalidOperationException>(m, "InvalidOperationException");
    py::register_exception<franka::ModelException>(m, "ModelException");
    py::register_exception<franka::NetworkException>(m, "NetworkException");
    py::register_exception<franka::ProtocolException>(m, "ProtocolException");
    py::register_exception<franka::RealtimeException>(m, "RealtimeException");

    /*
    py::class_<myModel>(m, "Model")
        .def(py::init<const std::string &>())
        .def("pose", &myModel::pose)
        .def("bodyJacobian", &myModel::bodyJacobian)
        .def("zeroJacobian", &myModel::zeroJacobian)
        .def("mass", &myModel::mass)
        .def("coriolis", &myModel::coriolis)
        .def("gravity", &myModel::gravity);*/
}