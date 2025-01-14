/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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
 *
*/

#include <functional>

#include <ignition/math.hh>
#include "gazebo/physics/physics.hh"
#include "gazebo_plugins/custom_actor_plugin.h"

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(CustomActorPlugin)

#define WALKING_ANIMATION "walking"


inline std::vector<std::string> split(const std::string &delim, const std::string& str){
    std::vector<std::string> result;
    if("" == str) return result;

    char * strs = new char[str.length() + 1];
    strcpy(strs, str.c_str());

    char * d = new char[delim.length() + 1];
    strcpy(d, delim.c_str());

    char *p = strtok(strs, d);
    while(p) {
        std::string s = p;
        result.push_back(s);
        p = strtok(NULL, d);
    }
    return result;
}

inline std::vector<float> string_vector2float_vector(std::vector<std::string> input)
{
    std::vector<float> output;
    output.reserve(input.size());
    for (auto& i : input) output.push_back(std::stof(i));
    return output;
}


CustomActorPlugin::CustomActorPlugin() = default;

void CustomActorPlugin::control_callback(const std_msgs::Float32MultiArrayConstPtr& cmd_msg)
{
    forward_speed = cmd_msg->data[0];
    turn_speed = cmd_msg->data[1];
//    ROS_ERROR_STREAM("forward: " << forward_speed << "   turn: " << turn_speed);
//    std::cout << "forward: " << forward_speed << std::endl;
}

void CustomActorPlugin::QueueThread()
{
    static const double timeout = 0.01;
    while (this->rosNode->ok())
    {
        this->rosQueue.callAvailable(ros::WallDuration(timeout));
    }
}

// 加载参数
void CustomActorPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{

    // Initialize ros, if it has not already bee initialized.
    if (!ros::isInitialized())
    {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "actor_plugin",
                  ros::init_options::NoSigintHandler);
    }

    // Create our ROS node. This acts in a similar manner to
    // the Gazebo node
    this->rosNode.reset(new ros::NodeHandle("actor_plugin"));

    // Create a named topic, and subscribe to it.
    ros::SubscribeOptions so =
            ros::SubscribeOptions::create<std_msgs::Float32MultiArray>(
                    "/actor_cmd",
                    1,
                    boost::bind(&CustomActorPlugin::control_callback, this, _1),
                    ros::VoidPtr(), &this->rosQueue);
    this->rosSub = this->rosNode->subscribe(so);

    // Spin up the queue helper thread.
    this->rosQueueThread =
            std::thread(std::bind(&CustomActorPlugin::QueueThread, this));

    this->sdf = _sdf;
    this->actor = boost::dynamic_pointer_cast<physics::Actor>(_model);
    this->world = this->actor->GetWorld();

    this->connections.push_back(event::Events::ConnectWorldUpdateBegin(
            std::bind(&CustomActorPlugin::OnUpdate, this, std::placeholders::_1)));

    this->Reset();

    if (_sdf->HasElement("trajectory")){
        auto trajectory_node = _sdf->GetElement("trajectory");
        sdf::ElementPtr ptr = trajectory_node->GetFirstElement();
        auto time = ptr->Get<float>("time");
        std::vector<float> pose_vector = string_vector2float_vector(split(" ", ptr->Get<std::string>("pose")));
        trajectory_pose.emplace_back(pose_vector[0], pose_vector[1], 1, 1.5707, 0, 0);
        trajectory_time.push_back(time);

        while (true){
            ptr = ptr->GetNextElement("waypoint");
            if (ptr == sdf::ElementPtr(nullptr)){
                break;
            } else {
                auto time = ptr->Get<float>("time");
                std::vector<float> pose_vector = string_vector2float_vector(split(" ", ptr->Get<std::string>("pose")));
                trajectory_pose.emplace_back(pose_vector[0], pose_vector[1], 1, 1.5707, 0, 0);
                trajectory_time.push_back(time);
            }
        }
    //    for (int i=0;i < trajectory_time.size();i++){
    //        std::cout << "pose: " << trajectory_pose[i][0] << trajectory_pose[i][1] << trajectory_pose[i][2] << std::endl;
    //        std::cout << "time: " << trajectory_time[i] << std::endl;
    //    }
    }
    if (_sdf->HasElement("walking_mode"))
        this->walking_mode = _sdf->Get<std::string>("walking_mode");
    else
        this->walking_mode = "origin";

    // Read in the target weight
    if (_sdf->HasElement("target_weight"))
        this->targetWeight = _sdf->Get<double>("target_weight");
    else
        this->targetWeight = 1.15;

    // Read in the obstacle weight
    if (_sdf->HasElement("obstacle_weight"))
        this->obstacleWeight = _sdf->Get<double>("obstacle_weight");
    else
        this->obstacleWeight = 1.5;

    // Read in the animation factor (applied in the OnUpdate function).
    if (_sdf->HasElement("animation_factor"))
        this->animationFactor = _sdf->Get<double>("animation_factor");
    else
        this->animationFactor = 4.5;

    // Add our own name to models we should ignore when avoiding obstacles.
    this->ignoreModels.push_back(this->actor->GetName());

    // Read in the other obstacles to ignore
    if (_sdf->HasElement("ignore_obstacles"))
    {
        sdf::ElementPtr modelElem =
                _sdf->GetElement("ignore_obstacles")->GetElement("model");
        while (modelElem)
        {
            this->ignoreModels.push_back(modelElem->Get<std::string>());
            modelElem = modelElem->GetNextElement("model");
        }
    }

    // 设置初始位姿，在world文件内也可直接定义，注意的是人体姿态的第一个要设置为1.5707即二分之派，不然人体是旋转了90度的
//    ignition::math::Pose3d pose = this->actor->WorldPose();
//    pose.Rot() = ignition::math::Quaterniond(1.5707, 0, 0); // 下一个时刻应该到的位置，距离越近就会越慢。
//    this->actor->SetWorldPose(pose, false, false);
}

//设置skin,walk的方式，跟addwaypoint中 animation类似。
void CustomActorPlugin::Reset()
{
    this->velocity = 0.8;
    this->lastUpdate = 0;

    if (this->sdf && this->sdf->HasElement("target"))
        this->target = this->sdf->Get<ignition::math::Vector3d>("target");
    else
        this->target = ignition::math::Vector3d(0, -5, 1.2138);

    auto skelAnims = this->actor->SkeletonAnimations();
    if (skelAnims.find(WALKING_ANIMATION) == skelAnims.end())
    {
        gzerr << "Skeleton animation " << WALKING_ANIMATION << " not found.\n";
    }
    else
    {
        // Create custom trajectory
        this->trajectoryInfo.reset(new physics::TrajectoryInfo());
        this->trajectoryInfo->type = WALKING_ANIMATION;
        this->trajectoryInfo->duration = 1.0;

        this->actor->SetCustomTrajectory(this->trajectoryInfo);
    }
}

// 选择新的目标点。
void CustomActorPlugin::ChooseNewTarget()
{
    ignition::math::Vector3d newTarget(this->target);
    while ((newTarget - this->target).Length() < 2.0)
    {
        newTarget.X(ignition::math::Rand::DblUniform(-3, 3.5));
        newTarget.Y(ignition::math::Rand::DblUniform(-10, 2));

        for (unsigned int i = 0; i < this->world->ModelCount(); ++i)
        {
            double dist = (this->world->ModelByIndex(i)->WorldPose().Pos()
                           - newTarget).Length();
            if (dist < 2.0) // 距离小于2才行，但是我觉得没必要。
            {
                newTarget = this->target;
                break;
            }
        }
    }
    this->target = newTarget;
}

//处理障碍，ignoreModels是读取.world文件中的ignor_obstacle中的model参数。
//要是自己建立的应该只有墙壁忽略但实际上并不能忽略，所以只需要忽略ground_plane就行了。
void CustomActorPlugin::HandleObstacles(ignition::math::Vector3d &_pos)
{
    for (unsigned int i = 0; i < this->world->ModelCount(); ++i)
    {
        physics::ModelPtr model = this->world->ModelByIndex(i);
        if (std::find(this->ignoreModels.begin(), this->ignoreModels.end(),
                      model->GetName()) == this->ignoreModels.end())
        {
            ignition::math::Vector3d offset = model->WorldPose().Pos() -
                                              this->actor->WorldPose().Pos();
            double modelDist = offset.Length();
            if (modelDist < 4.0)
            {
                double invModelDist = this->obstacleWeight / modelDist;
                offset.Normalize();
                offset *= invModelDist;
                _pos -= offset;
            }
        }
    }
}

void CustomActorPlugin::origin_on_update(const common::UpdateInfo &_info)
{
    /// 随机在咖啡厅闲逛的更新代码
    // Time delta
    double dt = (_info.simTime - this->lastUpdate).Double();

    ignition::math::Pose3d pose = this->actor->WorldPose();
    ignition::math::Vector3d pos = this->target - pose.Pos(); //位置(x,y,z)的差
    //当前actor的角度，0°是gazebo中的Y轴负方向
    ignition::math::Vector3d rpy = pose.Rot().Euler();

    double distance = pos.Length(); //距离

    // 如果距离小于0.3，则认为到达了目的地，就重新选择目的地。
    if (distance < 0.3)
    {
        this->ChooseNewTarget(); // 更新新的target在self.target设置
        pos = this->target - pose.Pos();  // 重新得到相对pos
    }

    // Normalize the direction vector, and apply the target weight
    pos = pos.Normalize() * this->targetWeight; //向量差单位化以后乘一个权重

    // Adjust the direction vector by avoiding obstacles
    this->HandleObstacles(pos);// 处理障碍，这个会在下边解释

    // Compute the yaw orientation 计算需要转过的角度，也会在下边解释
    ignition::math::Angle yaw = atan2(pos.Y(), pos.X()) + 1.5707 - rpy.Z();
    yaw.Normalize(); //这个就是yaw本身

    // 如果差的角度较大，就原地旋转，位置不变，如果角度较小，就直接转过去并开始行走
    if (std::abs(yaw.Radian()) > IGN_DTOR(10))  // IGN_DTOR是将度数转换成弧度数
    {
        pose.Rot() = ignition::math::Quaterniond(1.5707, 0, rpy.Z()+
                                                            yaw.Radian()*0.001); // 下一个时刻应该到的位置，距离越近就会越慢。
        // 始终是+是因为本身yaw会有负值。这样可以避免始终逆时针或者顺时针转。
    }
    else
    { //如果相差角度小于10，就行走，并且加上差的角度。
        pose.Pos() += pos * this->velocity * dt;
        pose.Rot() = ignition::math::Quaterniond(1.5707, 0, rpy.Z()+yaw.Radian());
    }

    // Make sure the actor stays within bounds 确保在某个框内运动。
    pose.Pos().X(std::max(-3.0, std::min(3.5, pose.Pos().X())));
    pose.Pos().Y(std::max(-10.0, std::min(2.0, pose.Pos().Y())));
    pose.Pos().Z(1.2138);

    // Distance traveled is used to coordinate motion with the walking
    // animation动画。
    double distanceTraveled = (pose.Pos() -
                               this->actor->WorldPose().Pos()).Length();

    this->actor->SetWorldPose(pose, false, false);
    this->actor->SetScriptTime(this->actor->ScriptTime() +
                               (distanceTraveled * this->animationFactor));
    this->lastUpdate = _info.simTime;

}

void CustomActorPlugin::custom_on_update(const common::UpdateInfo &_info)
{
    double dt = (_info.simTime - this->lastUpdate).Double();

    ignition::math::Pose3d pose = this->actor->WorldPose();
    ignition::math::Vector3d rpy = pose.Rot().Euler();

    ignition::math::Pose3d target_pose = pose;
    ignition::math::Vector3d rel_pos;
    if (fabs(turn_speed) > 0.0001){
        double new_yaw = turn_speed>0 ? rpy.Z()-0.00157 : rpy.Z()+0.00157;
        target_pose.Rot() = ignition::math::Quaterniond(1.5707, 0, new_yaw); // 下一个时刻应该到的位置，距离越近就会越慢。
    }
    if (fabs(forward_speed) > 0.0001){
        double old_yaw = rpy.Z();
        if (forward_speed > 0){
            target_pose.Pos().X() += ((0.8 * dt) * sin(old_yaw));
            target_pose.Pos().Y() -= ((0.8 * dt) * cos(old_yaw));
        } else {
            target_pose.Pos().X() -= ((0.8 * dt) * sin(old_yaw));
            target_pose.Pos().Y() += ((0.8 * dt) * cos(old_yaw));
        }
    }
    rel_pos = target_pose.Pos() - pose.Pos();

    // Adjust the direction vector by avoiding obstacles
    this->HandleObstacles(rel_pos);

    // Distance traveled is used to coordinate motion with the walking
    double distanceTraveled = (target_pose.Pos() -
                               this->actor->WorldPose().Pos()).Length();

    this->actor->SetWorldPose(target_pose, false, false);
    this->actor->SetScriptTime(this->actor->ScriptTime() +
                               (distanceTraveled * this->animationFactor));
    this->lastUpdate = _info.simTime;
}

void CustomActorPlugin::trajectory_on_update(const common::UpdateInfo &_info)
{
    /// 按照轨迹行走的代码
    double dt = (_info.simTime - this->lastUpdate).Double();
    ignition::math::Pose3d pose = this->actor->WorldPose();
    ignition::math::Pose3d target_pose = trajectory_pose[cur_trajectory_i];
    if ((target_pose.Pos() - pose.Pos()).Length() < 0.3){
        if (cur_trajectory_i + 1 < trajectory_pose.size()){
            target_pose = trajectory_pose[++cur_trajectory_i];
            std::cout << "final one point" << std::endl;
        }
    }
    ignition::math::Vector3d pos = target_pose.Pos() - pose.Pos(); //位置(x,y,z)的差
    ignition::math::Vector3d rpy = pose.Rot().Euler();

    // Normalize the direction vector, and apply the target weight
    pos = pos.Normalize() * this->targetWeight; //向量差单位化以后乘一个权重

    // Adjust the direction vector by avoiding obstacles
    this->HandleObstacles(pos);// 处理障碍，这个会在下边解释

    // Compute the yaw orientation 计算需要转过的角度，也会在下边解释
    ignition::math::Angle yaw = atan2(pos.Y(), pos.X()) + 1.5707 - rpy.Z();
    yaw.Normalize(); //这个就是yaw本身

    // 如果差的角度较大，就原地旋转，位置不变，如果角度较小，就直接转过去并开始行走
    if (std::abs(yaw.Radian()) > IGN_DTOR(10))  // IGN_DTOR是将度数转换成弧度数
    {
        pose.Rot() = ignition::math::Quaterniond(1.5707, 0, rpy.Z()+
                                                            yaw.Radian()*0.001); // 下一个时刻应该到的位置，距离越近就会越慢。
    }
    else
    { //如果相差角度小于10，就行走，并且加上差的角度。
        pose.Pos() += pos * this->velocity * dt;
        pose.Rot() = ignition::math::Quaterniond(1.5707, 0, rpy.Z()+yaw.Radian());
    }

    // Distance traveled is used to coordinate motion with the walking
    // animation动画。
    double distanceTraveled = (pose.Pos() -
                               this->actor->WorldPose().Pos()).Length();

    this->actor->SetWorldPose(pose, false, false);
    this->actor->SetScriptTime(this->actor->ScriptTime() +
                               (distanceTraveled * this->animationFactor));
    this->lastUpdate = _info.simTime;
}

// 回调函数，
void CustomActorPlugin::OnUpdate(const common::UpdateInfo &_info)
{
    if (walking_mode == "origin" || (walking_mode != "custom" && walking_mode != ""))
        origin_on_update(_info);
    else if (walking_mode == "custom")
        custom_on_update(_info);
    else if (walking_mode == "trajectory")
        trajectory_on_update(_info);
}