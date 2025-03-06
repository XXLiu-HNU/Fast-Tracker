#include <ros/ros.h>
#include <Eigen/Geometry>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <object_detection_msgs/BoundingBoxes.h>

ros::Publisher target_odom_pub_;
ros::Time last_update_stamp_;
bool has_target = false;  // 新增：标志变量，指示是否检测到目标

struct Ekf {
  double dt;
  Eigen::MatrixXd A, B, C;
  Eigen::MatrixXd Qt, Rt;
  Eigen::MatrixXd Sigma, K;
  Eigen::VectorXd x;

  Ekf(double _dt) : dt(_dt) {
    A.setIdentity(6, 6);
    Sigma.setZero(6, 6);
    B.setZero(6, 3);
    C.setZero(3, 6);
    A(0, 3) = dt;
    A(1, 4) = dt;
    A(2, 5) = dt;
    double t2 = dt * dt / 2;
    B(0, 0) = t2;
    B(1, 1) = t2;
    B(2, 2) = t2;
    B(3, 0) = dt;
    B(4, 1) = dt;
    B(5, 2) = dt;
    C(0, 0) = 1;
    C(1, 1) = 1;
    C(2, 2) = 1;
    K = C;
    Qt.setIdentity(3, 3);
    Rt.setIdentity(3, 3);
    Qt(0, 0) = 4;
    Qt(1, 1) = 4;
    Qt(2, 2) = 1;
    Rt(0, 0) = 0.1;
    Rt(1, 1) = 0.1;
    Rt(2, 2) = 0.1;
    x.setZero(6);
  }

  inline void predict() {
    x = A * x;
    Sigma = A * Sigma * A.transpose() + B * Qt * B.transpose();
    return;
  }

  inline void reset(const Eigen::Vector3d& z) {
    x.head(3) = z;
    x.tail(3).setZero();
    Sigma.setZero();
  }

  inline bool checkValid(const Eigen::Vector3d& z) const {
    Eigen::MatrixXd K_tmp = Sigma * C.transpose() * (C * Sigma * C.transpose() + Rt).inverse();
    Eigen::VectorXd x_tmp = x + K_tmp * (z - C * x);
    const double vmax = 2;
    if (x_tmp.tail(3).norm() > vmax) {
      return false;
    } else {
      return true;
    }
  }

  inline void update(const Eigen::Vector3d& z) {
    K = Sigma * C.transpose() * (C * Sigma * C.transpose() + Rt).inverse();
    x = x + K * (z - C * x);
    Sigma = Sigma - K * C * Sigma;
  }

  inline const Eigen::Vector3d pos() const {
    return x.head(3);
  }

  inline const Eigen::Vector3d vel() const {
    return x.tail(3);
  }
};

std::shared_ptr<Ekf> ekfPtr_;

void predict_state_callback(const ros::TimerEvent& event) {
  if (!has_target) {
    return;  // 如果没有检测到目标，直接返回
  }

  double update_dt = (ros::Time::now() - last_update_stamp_).toSec();
  if (update_dt < 2.0) {
    ekfPtr_->predict();
  } else {
    ROS_WARN("too long time no update!");
    has_target = false;  // 长时间未更新，重置标志
    return;
  }

  // 发布目标 odom
  nav_msgs::Odometry target_odom;
  target_odom.header.stamp = ros::Time::now();
  target_odom.header.frame_id = "world";
  target_odom.pose.pose.position.x = ekfPtr_->pos().x();
  target_odom.pose.pose.position.y = ekfPtr_->pos().y();
  target_odom.pose.pose.position.z = ekfPtr_->pos().z();
  target_odom.twist.twist.linear.x = ekfPtr_->vel().x();
  target_odom.twist.twist.linear.y = ekfPtr_->vel().y();
  target_odom.twist.twist.linear.z = ekfPtr_->vel().z();
  target_odom.pose.pose.orientation.w = 1.0;
  target_odom_pub_.publish(target_odom);
}

void yolo_cb(const geometry_msgs::PointConstPtr &yolo_detect) {
  Eigen::Vector3d p;
  p(0) = yolo_detect->x;
  p(1) = yolo_detect->y;
  p(2) = yolo_detect->z;

  // 更新目标 odom
  double update_dt = (ros::Time::now() - last_update_stamp_).toSec();
  if (update_dt > 3.0) {
    ekfPtr_->reset(p);
    ROS_WARN("ekf reset!");
  } else if (ekfPtr_->checkValid(p)) {
    ekfPtr_->update(p);
  } else {
    ROS_ERROR("update invalid!");
    return;
  }

  has_target = true;  // 设置标志为 true，表示检测到目标
  last_update_stamp_ = ros::Time::now();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "target_ekf");
  ros::NodeHandle nh("~");
  ros::Time::init();
  last_update_stamp_ = ros::Time::now() - ros::Duration(10.0);

  target_odom_pub_ = nh.advertise<nav_msgs::Odometry>("odom", 10);

  int ekf_rate = 20;
  nh.getParam("ekf_rate", ekf_rate);
  ekfPtr_ = std::make_shared<Ekf>(1.0 / ekf_rate);

  ros::Subscriber single_pose_sub = nh.subscribe("yolo", 100, &yolo_cb, ros::TransportHints().tcpNoDelay());
  ros::Timer ekf_predict_timer_ = nh.createTimer(ros::Duration(1.0 / ekf_rate), &predict_state_callback);

  ros::spin();
  return 0;
}