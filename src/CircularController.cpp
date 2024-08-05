#include "CircularController.h"
#include <Eigen/src/Core/Matrix.h>
#include <SpaceVecAlg/PTransform.h>
#include <cmath>
#include <ctime>
#include <mc_rbdyn/RobotLoader.h>
#include <mc_tasks/EndEffectorTask.h>
#include <mc_tasks/PostureTask.h>
#include <mc_tasks/OrientationTask.h>


CircularController::CircularController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::MCController(rm, dt, config, Backend::TVM)
{
  start_moving_ = false;
  ctlTime_=0;
  dynamicsConstraint = mc_rtc::unique_ptr<mc_solver::DynamicsConstraint>(new mc_solver::DynamicsConstraint(robots(), 0, solver().dt(), {0.1, 0.01, 0.5}, 0.9, false, true));
  config_.load(config);
  solver().addConstraintSet(contactConstraint);
  solver().addConstraintSet(dynamicsConstraint);

  R_=0.15;
  omega_= 3;

  postureTask = std::make_shared<mc_tasks::PostureTask>(solver(), robot().robotIndex(), 5, 1);
  // postureTask->stiffness(3);
  // postureTask->damping(5);
  solver().addTask(postureTask);


  circularTask = std::make_shared<mc_tasks::EndEffectorTask>(robot().frame("tool_frame"), 20.0, 10000);
  Eigen::VectorXd dimweight(6); 
  dimweight << 1., 1., 1., 1., 1., 1. ; 
  circularTask -> dimWeight(dimweight);
  circularTask->reset();

  solver().addTask(circularTask);

  datastore().make<std::string>("ControlMode", "Position"); // entree dans le datastore
  datastore().make<std::string>("Coriolis", "Yes"); 
  datastore().make_call("getPostureTask", [this]() -> mc_tasks::PostureTaskPtr { return postureTask; });

  gui()->addElement(this, {"Control Mode"},
                    mc_rtc::gui::Label("Current Control :", [this]() { return this->datastore().get<std::string>("ControlMode"); }),
                    mc_rtc::gui::Button("Position", [this]() { datastore().assign<std::string>("ControlMode", "Position"); }),
                    mc_rtc::gui::Button("Torque", [this]() { datastore().assign<std::string>("ControlMode", "Torque"); }));

  gui()->addElement({"Tasks"},
    mc_rtc::gui::Checkbox("Circular Moving", start_moving_)
  );

  logger().addLogEntry("IMU Accel",
                       [this]()
                       {
                         return robot().bodySensor("Accelerometer").linearAcceleration();
                       });

  logger().addLogEntry("IMU Gyro",
                       [this]()
                       {
                         return robot().bodySensor("Accelerometer").angularVelocity();
                       });

  mc_rtc::log::success("CircularController init done");
}

bool CircularController::run()
{ 
  ctlTime_ += timeStep;
  if (start_moving_ ) { 
    circularTask->positionTask->position(Eigen::Vector3d(0.55, R_*std::sin(omega_*ctlTime_), 0.4 + R_*std::cos(omega_*ctlTime_))),
    circularTask->positionTask->refVel(Eigen::Vector3d(0, R_*omega_*std::cos(omega_*ctlTime_), -R_*omega_*std::sin(omega_*ctlTime_))),
    circularTask->positionTask->refAccel(Eigen::Vector3d(0, -R_*R_*omega_*std::sin(omega_*ctlTime_), R_*R_*omega_*std::cos(omega_*ctlTime_))), 
    circularTask->orientationTask->orientation(Eigen::Quaterniond(0, 1, 0, 1).normalized().toRotationMatrix());}

  auto ctrl_mode = datastore().get<std::string>("ControlMode");

  if(ctrl_mode.compare("Position") == 0)
  {
    return mc_control::MCController::run(mc_solver::FeedbackType::OpenLoop);
  }
  else
  {
    return mc_control::MCController::run(mc_solver::FeedbackType::ClosedLoopIntegrateReal);
  }
  return false;
}

void CircularController::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::MCController::reset(reset_data);
}

CONTROLLER_CONSTRUCTOR("CircularController", CircularController)