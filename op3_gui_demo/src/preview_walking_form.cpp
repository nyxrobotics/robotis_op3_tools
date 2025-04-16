/*******************************************************************************
 * Copyright 2017 ROBOTIS CO., LTD.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *******************************************************************************/

/* Author: Kayman Jung */

#include "../include/op3_gui_demo/preview_walking_form.h"
#include <cmath>

PreviewWalkingForm::PreviewWalkingForm(QWidget* parent)
  : QWidget(parent), p_walking_ui(new Ui::PreviewWalkingForm), is_updating_(false)
{
  p_walking_ui->setupUi(this);
}

PreviewWalkingForm::~PreviewWalkingForm()
{
  delete p_walking_ui;
}

bool PreviewWalkingForm::init(robotis_op::QNodeOP3* qnode)
{
  bool result = setQNode(qnode);

  if (result == true)
  {
    qRegisterMetaType<geometry_msgs::Point>("geometry_msgs::Point");
    qRegisterMetaType<geometry_msgs::Pose>("geometry_msgs::Pose");
    connect(qnode_op3_, SIGNAL(updateDemoPoint(geometry_msgs::Point)), this,
            SLOT(updatePointPanel(geometry_msgs::Point)));
    connect(qnode_op3_, SIGNAL(updateDemoPose(geometry_msgs::Pose)), this, SLOT(updatePosePanel(geometry_msgs::Pose)));
  }

  return result;
}

void PreviewWalkingForm::on_button_p_walking_turn_l_clicked(bool check)
{
  sendPWalkingCommand("turn_left", false);
}

void PreviewWalkingForm::on_button_p_walking_turn_r_clicked(bool check)
{
  sendPWalkingCommand("turn_right", false);
}

void PreviewWalkingForm::on_button_p_walking_forward_clicked(bool check)
{
  sendPWalkingCommand("forward");
}

void PreviewWalkingForm::on_button_p_walking_backward_clicked(bool check)
{
  sendPWalkingCommand("backward");
}

void PreviewWalkingForm::on_button_p_walking_stop_clicked(bool check)
{
  sendPWalkingCommand("stop");
}

void PreviewWalkingForm::on_button_p_walking_left_clicked(bool check)
{
  sendPWalkingCommand("left", false);
}

void PreviewWalkingForm::on_button_p_walking_right_clicked(bool check)
{
  sendPWalkingCommand("right", false);
}

void PreviewWalkingForm::on_button_set_walking_param_clicked(bool check)
{
  // Reload walking parameters
  reloadGuiOnlineWalkingParameters();
}

void PreviewWalkingForm::on_button_send_body_offset_clicked(bool check)
{
  geometry_msgs::Pose msg;
  msg.position.x = p_walking_ui->dSpinBox_body_offset_x->value();
  msg.position.y = p_walking_ui->dSpinBox_body_offset_y->value();
  msg.position.z = p_walking_ui->dSpinBox_body_offset_z->value();

  qnode_op3_->sendBodyOffsetMsg(msg);
}

void PreviewWalkingForm::on_button_send_foot_distance_clicked(bool check)
{
  // Set footstep left and right distance
  std_msgs::Float64 foot_distance_msg;
  foot_distance_msg.data = p_walking_ui->dSpinBox_foot_distance->value();
  qnode_op3_->sendFootDistanceMsg(foot_distance_msg);
}

void PreviewWalkingForm::on_button_p_walking_init_pose_clicked(bool check)
{
  // Reload walking parameters
  reloadGuiOnlineWalkingParameters();
  reloadGuiFootstepParameters();

  // Initial pose motion
  std::string ini_pose_path = ros::package::getPath(ROS_PACKAGE_NAME) + "/config/init_pose.yaml";
  qnode_op3_->parseIniPoseData(ini_pose_path);

  // Initialize localization
  std_msgs::Bool ini_pose_msg;
  ini_pose_msg.data = true;
  qnode_op3_->sendResetBodyMsg(ini_pose_msg);

  // Initial foot separation
  std_msgs::Float64 foot_distance_msg;
  foot_distance_msg.data = p_walking_ui->dSpinBox_foot_distance->value();
  qnode_op3_->sendFootDistanceMsg(foot_distance_msg);
}

void PreviewWalkingForm::on_button_p_walking_balance_on_clicked(bool check)
{
  std_msgs::String msg;
  msg.data = "balance_on";

  qnode_op3_->sendWholebodyBalanceMsg(msg);
}

void PreviewWalkingForm::on_button_p_walking_balance_off_clicked(bool check)
{
  std_msgs::String msg;
  msg.data = "balance_off";

  qnode_op3_->sendWholebodyBalanceMsg(msg);
}

// Interactive marker
void PreviewWalkingForm::on_button_marker_set_clicked(bool check)
{
  makeInteractiveMarker();
}

void PreviewWalkingForm::on_button_marker_clear_clicked(bool check)
{
  clearMarkerPanel();
}

void PreviewWalkingForm::on_button_footstep_plan_clicked(bool check)
{
  // Reload interactive marker
  updateInteractiveMarker();

  // Reload walking parameters
  reloadGuiOnlineWalkingParameters();
  reloadGuiFootstepParameters();

  geometry_msgs::Pose target_pose;
  getPoseFromMarkerPanel(target_pose);

  qnode_op3_->makeFootstepUsingPlanner(target_pose);
}

void PreviewWalkingForm::on_button_footstep_clear_clicked(bool check)
{
  qnode_op3_->clearFootsteps();
}

void PreviewWalkingForm::on_button_footstep_go_clicked(bool check)
{
  double step_time = p_walking_ui->dSpinBox_p_walking_step_time->value();

  qnode_op3_->setWalkingFootsteps(step_time);

  bool clear_path = p_walking_ui->checkBox_clear_path->isChecked();
  if (clear_path == true)
    qnode_op3_->clearFootsteps();

  bool clear_marker = p_walking_ui->checkBox_clear_marker->isChecked();
  if (clear_marker == true)
    clearMarkerPanel();
}

void PreviewWalkingForm::on_dSpinBox_marker_pos_x_valueChanged(double value)
{
  updateInteractiveMarker();
}
void PreviewWalkingForm::on_dSpinBox_marker_pos_y_valueChanged(double value)
{
  updateInteractiveMarker();
}
void PreviewWalkingForm::on_dSpinBox_marker_ori_y_valueChanged(double value)
{
  updateInteractiveMarker();
}

void PreviewWalkingForm::reloadGuiOnlineWalkingParameters(void)
{
  // Set Linear Inverted Pendulum Model (LIPM) parameters
  op3_online_walking_module_msgs::WalkingParam online_walking_param;
  online_walking_param.dsp_ratio = p_walking_ui->dSpinBox_dsp_ratio->value();
  online_walking_param.foot_height_max = p_walking_ui->dSpinBox_foot_height_max->value();
  online_walking_param.lipm_height = 0.5;
  online_walking_param.zmp_offset_x = 0;
  online_walking_param.zmp_offset_y = 0;
  qnode_op3_->sendWalkingParamMsg(online_walking_param);
  op3_walking_module_msgs::WalkingParam walking_param;

  walking_param.init_x_offset = p_walking_ui->dSpinBox_body_offset_x->value();
  walking_param.init_y_offset = p_walking_ui->dSpinBox_body_offset_y->value();
  walking_param.init_z_offset = p_walking_ui->dSpinBox_body_offset_z->value();
  walking_param.init_roll_offset = p_walking_ui->dSpinBox_body_offset_roll->value() * DEG2RAD;
  walking_param.init_pitch_offset = p_walking_ui->dSpinBox_body_offset_pitch->value() * DEG2RAD;
  walking_param.init_yaw_offset = p_walking_ui->dSpinBox_body_offset_yaw->value() * DEG2RAD;
  walking_param.hip_pitch_offset = p_walking_ui->dSpinBox_hip_offset_pitch->value() * DEG2RAD;
  walking_param.period_time = p_walking_ui->dSpinBox_p_walking_step_time->value() * 2.0;
  walking_param.dsp_ratio = p_walking_ui->dSpinBox_dsp_ratio->value();
  walking_param.x_move_amplitude = p_walking_ui->dSpinBox_p_walking_step_length->value();
  walking_param.y_move_amplitude = p_walking_ui->dSpinBox_p_walking_side_length->value();
  walking_param.z_move_amplitude = p_walking_ui->dSpinBox_foot_height_max->value();
  walking_param.angle_move_amplitude = p_walking_ui->dSpinBox_p_walking_step_angle->value() * DEG2RAD;
  qnode_op3_->applyWalkingParam(walking_param);
}

void PreviewWalkingForm::reloadGuiFootstepParameters(void)
{
  // Set footstep left and right distance
  qnode_op3_->setFootstepSeparation(p_walking_ui->dSpinBox_foot_distance->value());
  qnode_op3_->setFootstepXMax(p_walking_ui->dSpinBox_p_walking_step_length->value());
  qnode_op3_->setFootstepYMax(p_walking_ui->dSpinBox_p_walking_side_length->value());
  qnode_op3_->setFootstepThetaMax(p_walking_ui->dSpinBox_p_walking_step_angle->value() * M_PI / 180.0);
  qnode_op3_->applyFootstepParam();
}

void PreviewWalkingForm::sendPWalkingCommand(const std::string& command, bool set_start_foot)
{
  op3_online_walking_module_msgs::FootStepCommand msg;

  msg.step_time = p_walking_ui->dSpinBox_p_walking_step_time->value();
  msg.step_num = p_walking_ui->dSpinBox_p_walking_step_num->value();
  msg.step_length = p_walking_ui->dSpinBox_p_walking_step_length->value();
  msg.side_length = p_walking_ui->dSpinBox_p_walking_side_length->value();
  msg.step_angle = p_walking_ui->dSpinBox_p_walking_step_angle->value() * DEG2RAD;
  msg.start_leg = set_start_foot ? p_walking_ui->comboBox_p_walking_start_leg->currentText().toStdString() : "";

  msg.command = command;

  qnode_op3_->sendFootStepCommandMsg(msg);
}

bool PreviewWalkingForm::setQNode(robotis_op::QNodeOP3* qnode)
{
  if (qnode == NULL)
    return false;

  qnode_op3_ = qnode;
  return true;
}

// make interactive marker
void PreviewWalkingForm::makeInteractiveMarker()
{
  geometry_msgs::Pose current_pose;
  getPoseFromMarkerPanel(current_pose);

  qnode_op3_->makeInteractiveMarker(current_pose);
}

// update interactive marker pose from ui
void PreviewWalkingForm::updateInteractiveMarker()
{
  if (is_updating_ == true)
    return;

  geometry_msgs::Pose current_pose;
  getPoseFromMarkerPanel(current_pose);

  qnode_op3_->updateInteractiveMarker(current_pose);
}

void PreviewWalkingForm::clearMarkerPanel()
{
  geometry_msgs::Pose init_pose;
  updatePosePanel(init_pose);

  ROS_INFO("Clear Panel");

  qnode_op3_->clearInteractiveMarker();
}

// Update UI - position
void PreviewWalkingForm::updatePointPanel(const geometry_msgs::Point point)
{
  is_updating_ = true;

  setPointToMarkerPanel(point);

  ROS_INFO("Update Position Panel");
  is_updating_ = false;
}

// Update UI - pose
void PreviewWalkingForm::updatePosePanel(const geometry_msgs::Pose pose)
{
  is_updating_ = true;

  setPoseToMarkerPanel(pose);

  ROS_INFO("Update Pose Panel");
  is_updating_ = false;
}

void PreviewWalkingForm::getPoseFromMarkerPanel(geometry_msgs::Pose& current)
{
  // position
  current.position.x = p_walking_ui->dSpinBox_marker_pos_x->value();
  current.position.y = p_walking_ui->dSpinBox_marker_pos_y->value();

  // orientation
  Eigen::Vector3d euler(0, 0, p_walking_ui->dSpinBox_marker_ori_y->value());
  Eigen::Quaterniond orientation = rpy2quaternion(deg2rad<Eigen::Vector3d>(euler));

  tf::quaternionEigenToMsg(orientation, current.orientation);
}

void PreviewWalkingForm::setPoseToMarkerPanel(const geometry_msgs::Pose& current)
{
  // position
  p_walking_ui->dSpinBox_marker_pos_x->setValue(current.position.x);
  p_walking_ui->dSpinBox_marker_pos_y->setValue(current.position.y);

  // orientation
  Eigen::Vector3d euler = rad2deg<Eigen::Vector3d>(quaternion2rpy(current.orientation));
  p_walking_ui->dSpinBox_marker_ori_y->setValue(euler[2]);
}

void PreviewWalkingForm::getPointFromMarkerPanel(geometry_msgs::Point& current)
{
  // position
  current.x = p_walking_ui->dSpinBox_marker_pos_x->value();
  current.y = p_walking_ui->dSpinBox_marker_pos_y->value();
}

void PreviewWalkingForm::setPointToMarkerPanel(const geometry_msgs::Point& current)
{
  // position
  p_walking_ui->dSpinBox_marker_pos_x->setValue(current.x);
  p_walking_ui->dSpinBox_marker_pos_y->setValue(current.y);

  // orientation
  p_walking_ui->dSpinBox_marker_ori_y->setValue(0.0);
}

/*****************************************************************************
 ** Implementation [Util]
 *****************************************************************************/

Eigen::Vector3d PreviewWalkingForm::rotation2rpy(const Eigen::MatrixXd& rotation)
{
  Eigen::Vector3d rpy;

  rpy[0] = atan2(rotation.coeff(2, 1), rotation.coeff(2, 2));
  rpy[1] = atan2(-rotation.coeff(2, 0), sqrt(pow(rotation.coeff(2, 1), 2) + pow(rotation.coeff(2, 2), 2)));
  rpy[2] = atan2(rotation.coeff(1, 0), rotation.coeff(0, 0));

  return rpy;
}

Eigen::MatrixXd PreviewWalkingForm::rpy2rotation(const double& roll, const double& pitch, const double& yaw)
{
  Eigen::MatrixXd rotation = rotationZ(yaw) * rotationY(pitch) * rotationX(roll);

  return rotation;
}

Eigen::Quaterniond PreviewWalkingForm::rpy2quaternion(const Eigen::Vector3d& euler)
{
  return rpy2quaternion(euler[0], euler[1], euler[2]);
}

Eigen::Quaterniond PreviewWalkingForm::rpy2quaternion(const double& roll, const double& pitch, const double& yaw)
{
  Eigen::MatrixXd rotation = rpy2rotation(roll, pitch, yaw);

  Eigen::Matrix3d rotation3d;
  rotation3d = rotation.block(0, 0, 3, 3);

  Eigen::Quaterniond quaternion;

  quaternion = rotation3d;

  return quaternion;
}

Eigen::Quaterniond PreviewWalkingForm::rotation2quaternion(const Eigen::MatrixXd& rotation)
{
  Eigen::Matrix3d rotation3d;

  rotation3d = rotation.block(0, 0, 3, 3);

  Eigen::Quaterniond quaternion;
  quaternion = rotation3d;

  return quaternion;
}

Eigen::Vector3d PreviewWalkingForm::quaternion2rpy(const Eigen::Quaterniond& quaternion)
{
  Eigen::Vector3d rpy = rotation2rpy(quaternion.toRotationMatrix());

  return rpy;
}

Eigen::Vector3d PreviewWalkingForm::quaternion2rpy(const geometry_msgs::Quaternion& quaternion)
{
  Eigen::Quaterniond eigen_quaternion;
  tf::quaternionMsgToEigen(quaternion, eigen_quaternion);

  Eigen::Vector3d rpy = rotation2rpy(eigen_quaternion.toRotationMatrix());

  return rpy;
}

Eigen::MatrixXd PreviewWalkingForm::quaternion2rotation(const Eigen::Quaterniond& quaternion)
{
  Eigen::MatrixXd rotation = quaternion.toRotationMatrix();

  return rotation;
}

Eigen::MatrixXd PreviewWalkingForm::rotationX(const double& angle)
{
  Eigen::MatrixXd rotation(3, 3);

  rotation << 1.0, 0.0, 0.0, 0.0, cos(angle), -sin(angle), 0.0, sin(angle), cos(angle);

  return rotation;
}

Eigen::MatrixXd PreviewWalkingForm::rotationY(const double& angle)
{
  Eigen::MatrixXd rotation(3, 3);

  rotation << cos(angle), 0.0, sin(angle), 0.0, 1.0, 0.0, -sin(angle), 0.0, cos(angle);

  return rotation;
}

Eigen::MatrixXd PreviewWalkingForm::rotationZ(const double& angle)
{
  Eigen::MatrixXd rotation(3, 3);

  rotation << cos(angle), -sin(angle), 0.0, sin(angle), cos(angle), 0.0, 0.0, 0.0, 1.0;

  return rotation;
}
