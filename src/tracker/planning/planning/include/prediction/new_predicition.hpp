#pragma once

#include <mapping/mapping.h>
#include <ros/ros.h>

#include <Eigen/Core>
#include <queue>
#include <vector>
#include <memory>
#include <algorithm>

namespace prediction {

struct Node {
  Eigen::Vector3d p, v, a;
  double t;
  double score;
  double h;
  Node* parent = nullptr;
};
typedef Node* NodePtr;

class NodeComparator {
 public:
  bool operator()(NodePtr& lhs, NodePtr& rhs) {
    return lhs->score + lhs->h > rhs->score + rhs->h;
  }
};

class Predict {
 private:
  static constexpr int MAX_MEMORY = 1 << 22;

  double dt = 0.1;
  double pre_dur = 1.0;
  double rho_a = 1.0;
  double car_z = 0.0;
  double vmax = 5.0;

  mapping::OccGridMap map;
  std::vector<std::unique_ptr<Node>> data;
  int stack_top = 0;

  inline bool isValid(const Eigen::Vector3d& p, const Eigen::Vector3d& v) const {
    if (!map.isOccupied(p)){
      ROS_INFO("When predict, the position is occupied")
    }
    return (v.norm() < vmax) && (!map.isOccupied(p));
  }

 public:
  Predict(ros::NodeHandle& nh) : data(MAX_MEMORY) {
    // 从参数服务器获取参数，带默认值，防止参数未设置时程序崩溃
    nh.param("tracking_dur", pre_dur, 1.0);
    nh.param("tracking_dt", dt, 0.1);
    nh.param("prediction/rho_a", rho_a, 1.0);
    nh.param("prediction/vmax", vmax, 5.0);

    if (dt <= 0 || pre_dur <= 0 || rho_a <= 0 || vmax <= 0) {
      ROS_WARN("[prediction] Invalid parameter value. Please check the configuration.");
    }

    for (int i = 0; i < MAX_MEMORY; ++i) {
      data[i] = std::make_unique<Node>();
    }
  }

  void setMap(const mapping::OccGridMap& _map) {
    map = _map;
  }

  bool predict(const Eigen::Vector3d& target_p,
               const Eigen::Vector3d& target_v,
               std::vector<Eigen::Vector3d>& target_predict,
               const double& max_time = 0.1) {
    auto score = [&](const NodePtr& ptr) -> double {
      return rho_a * ptr->a.norm();
    };

    Eigen::Vector3d end_p = target_p + target_v * pre_dur;
    auto calH = [&](const NodePtr& ptr) -> double {
      return 0.001 * (ptr->p - end_p).norm();
    };

    ros::Time t_start = ros::Time::now();
    std::priority_queue<NodePtr, std::vector<NodePtr>, NodeComparator> open_set;

    stack_top = 0;
    NodePtr curPtr = data[stack_top++].get();
    curPtr->p = target_p;
    curPtr->v = target_v;
    curPtr->a.setZero();
    curPtr->parent = nullptr;
    curPtr->score = 0;
    curPtr->h = 0;
    curPtr->t = 0;

    double dt2_2 = dt * dt / 2;

    while (curPtr->t < pre_dur) {
      for (input.x() = -3; input.x() <= 3; input.x() += 3) {
        for (input.y() = -3; input.y() <= 3; input.y() += 3) {
          Eigen::Vector3d p = curPtr->p + curPtr->v * dt + input * dt2_2;
          Eigen::Vector3d v = curPtr->v + input * dt;

          if (!isValid(p, v)) {
            continue;
          }

          if (stack_top >= MAX_MEMORY) {
            ROS_ERROR("[prediction] Out of memory! Reduce MAX_MEMORY or simplify the problem.");
            return false;
          }

          double t_cost = (ros::Time::now() - t_start).toSec();
          if (t_cost > max_time) {
            ROS_WARN("[prediction] Prediction exceeded time limit.");
            return false;
          }

          NodePtr ptr = data[stack_top++].get();
          ptr->p = p;
          ptr->v = v;
          ptr->a = input;
          ptr->parent = curPtr;
          ptr->t = curPtr->t + dt;
          ptr->score = curPtr->score + score(ptr);
          ptr->h = calH(ptr);

          open_set.push(ptr);
        }
      }

      if (open_set.empty()) {
        ROS_ERROR("[prediction] No valid path found!");
        return false;
      }

      curPtr = open_set.top();
      open_set.pop();
    }

    target_predict.clear();
    while (curPtr != nullptr) {
      target_predict.push_back(curPtr->p);
      curPtr = curPtr->parent;
    }

    std::reverse(target_predict.begin(), target_predict.end());
    return true;
  }
};

}  // namespace prediction
