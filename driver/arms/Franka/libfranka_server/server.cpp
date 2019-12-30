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
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <string.h>
#include <arpa/inet.h>
#include <cstdlib>

#define SERVER_PORT 8080
#define BUFF_LEN 1024
#define SERVER_IP "127.0.0.1"

class MotionGenerator {
 public:

  MotionGenerator(double speed_factor, const std::array<double, 7> q_goal);

  franka::JointPositions operator()(const franka::RobotState& robot_state, franka::Duration period);

 private:
  using Vector7d = Eigen::Matrix<double, 7, 1, Eigen::ColMajor>;
  using Vector7i = Eigen::Matrix<int, 7, 1, Eigen::ColMajor>;

  bool calculateDesiredValues(double t, Vector7d* delta_q_d) const;
  void calculateSynchronizedValues();

  static constexpr double kDeltaQMotionFinished = 1e-6;
  const Vector7d q_goal_;

  Vector7d q_start_;
  Vector7d delta_q_;

  Vector7d dq_max_sync_;
  Vector7d t_1_sync_;
  Vector7d t_2_sync_;
  Vector7d t_f_sync_;
  Vector7d q_1_;

  double time_ = 0.0;

  Vector7d dq_max_ = (Vector7d() << 2.0, 2.0, 2.0, 2.0, 2.5, 2.5, 2.5).finished();
  Vector7d ddq_max_start_ = (Vector7d() << 5, 5, 5, 5, 5, 5, 5).finished();
  Vector7d ddq_max_goal_ = (Vector7d() << 5, 5, 5, 5, 5, 5, 5).finished();
};


MotionGenerator::MotionGenerator(double speed_factor, const std::array<double, 7> q_goal)
    : q_goal_(q_goal.data()) {
  dq_max_ *= speed_factor;
  ddq_max_start_ *= speed_factor;
  ddq_max_goal_ *= speed_factor;
  dq_max_sync_.setZero();
  q_start_.setZero();
  delta_q_.setZero();
  t_1_sync_.setZero();
  t_2_sync_.setZero();
  t_f_sync_.setZero();
  q_1_.setZero();
}

bool MotionGenerator::calculateDesiredValues(double t, Vector7d* delta_q_d) const {
  Vector7i sign_delta_q;
  sign_delta_q << delta_q_.cwiseSign().cast<int>();
  Vector7d t_d = t_2_sync_ - t_1_sync_;
  Vector7d delta_t_2_sync = t_f_sync_ - t_2_sync_;
  std::array<bool, 7> joint_motion_finished{};

  for (size_t i = 0; i < 7; i++) {
    if (std::abs(delta_q_[i]) < kDeltaQMotionFinished) {
      (*delta_q_d)[i] = 0;
      joint_motion_finished[i] = true;
    } else {
      if (t < t_1_sync_[i]) {
        (*delta_q_d)[i] = -1.0 / std::pow(t_1_sync_[i], 3.0) * dq_max_sync_[i] * sign_delta_q[i] *
                          (0.5 * t - t_1_sync_[i]) * std::pow(t, 3.0);
      } else if (t >= t_1_sync_[i] && t < t_2_sync_[i]) {
        (*delta_q_d)[i] = q_1_[i] + (t - t_1_sync_[i]) * dq_max_sync_[i] * sign_delta_q[i];
      } else if (t >= t_2_sync_[i] && t < t_f_sync_[i]) {
        (*delta_q_d)[i] =
            delta_q_[i] + 0.5 *
                              (1.0 / std::pow(delta_t_2_sync[i], 3.0) *
                                   (t - t_1_sync_[i] - 2.0 * delta_t_2_sync[i] - t_d[i]) *
                                   std::pow((t - t_1_sync_[i] - t_d[i]), 3.0) +
                               (2.0 * t - 2.0 * t_1_sync_[i] - delta_t_2_sync[i] - 2.0 * t_d[i])) *
                              dq_max_sync_[i] * sign_delta_q[i];
      } else {
        (*delta_q_d)[i] = delta_q_[i];
        joint_motion_finished[i] = true;
      }
    }
  }
  return std::all_of(joint_motion_finished.cbegin(), joint_motion_finished.cend(),
                     [](bool x) { return x; });
}

void MotionGenerator::calculateSynchronizedValues() {
  Vector7d dq_max_reach(dq_max_);
  Vector7d t_f = Vector7d::Zero();
  Vector7d delta_t_2 = Vector7d::Zero();
  Vector7d t_1 = Vector7d::Zero();
  Vector7d delta_t_2_sync = Vector7d::Zero();
  Vector7i sign_delta_q;
  sign_delta_q << delta_q_.cwiseSign().cast<int>();

  for (size_t i = 0; i < 7; i++) {
    if (std::abs(delta_q_[i]) > kDeltaQMotionFinished) {
      if (std::abs(delta_q_[i]) < (3.0 / 4.0 * (std::pow(dq_max_[i], 2.0) / ddq_max_start_[i]) +
                                   3.0 / 4.0 * (std::pow(dq_max_[i], 2.0) / ddq_max_goal_[i]))) {
        dq_max_reach[i] = std::sqrt(4.0 / 3.0 * delta_q_[i] * sign_delta_q[i] *
                                    (ddq_max_start_[i] * ddq_max_goal_[i]) /
                                    (ddq_max_start_[i] + ddq_max_goal_[i]));
      }
      t_1[i] = 1.5 * dq_max_reach[i] / ddq_max_start_[i];
      delta_t_2[i] = 1.5 * dq_max_reach[i] / ddq_max_goal_[i];
      t_f[i] = t_1[i] / 2.0 + delta_t_2[i] / 2.0 + std::abs(delta_q_[i]) / dq_max_reach[i];
    }
  }
  double max_t_f = t_f.maxCoeff();
  for (size_t i = 0; i < 7; i++) {
    if (std::abs(delta_q_[i]) > kDeltaQMotionFinished) {
      double a = 1.5 / 2.0 * (ddq_max_goal_[i] + ddq_max_start_[i]);
      double b = -1.0 * max_t_f * ddq_max_goal_[i] * ddq_max_start_[i];
      double c = std::abs(delta_q_[i]) * ddq_max_goal_[i] * ddq_max_start_[i];
      double delta = b * b - 4.0 * a * c;
      if (delta < 0.0) {
        delta = 0.0;
      }
      dq_max_sync_[i] = (-1.0 * b - std::sqrt(delta)) / (2.0 * a);
      t_1_sync_[i] = 1.5 * dq_max_sync_[i] / ddq_max_start_[i];
      delta_t_2_sync[i] = 1.5 * dq_max_sync_[i] / ddq_max_goal_[i];
      t_f_sync_[i] =
          (t_1_sync_)[i] / 2.0 + delta_t_2_sync[i] / 2.0 + std::abs(delta_q_[i] / dq_max_sync_[i]);
      t_2_sync_[i] = (t_f_sync_)[i] - delta_t_2_sync[i];
      q_1_[i] = (dq_max_sync_)[i] * sign_delta_q[i] * (0.5 * (t_1_sync_)[i]);
    }
  }
}

franka::JointPositions MotionGenerator::operator()(const franka::RobotState& robot_state,
                                                   franka::Duration period) {
  time_ += period.toSec();

  if (time_ == 0.0) {
    q_start_ = Vector7d(robot_state.q_d.data());
    delta_q_ = q_goal_ - q_start_;
    calculateSynchronizedValues();
  }

  Vector7d delta_q_d;
  bool motion_finished = calculateDesiredValues(time_, &delta_q_d);

  std::array<double, 7> joint_positions;
  Eigen::VectorXd::Map(&joint_positions[0], 7) = (q_start_ + delta_q_d);
  franka::JointPositions output(joint_positions);
  output.motion_finished = motion_finished;
  return output;
}

std::vector<std::string> split(const std::string &s, const std::string &seperator){
    std::vector<std::string> result;
    typedef std::string::size_type string_size;
    string_size i = 0;
    
    while(i != s.size())
    {
        int flag = 0;
        while(i != s.size() && flag == 0)
        {
            flag = 1;
            for(string_size x = 0; x < seperator.size();++x)
            {
                if(s[i] == seperator[x])
                {
                    ++i;
                    flag = 0;
                    break;
                }
            }
        }
        
        flag = 0;
        string_size j = i;
        
        while(j != s.size() && flag == 0)
        {
            for(string_size x = 0; x < seperator.size(); ++x)
            {
                if(s[j] == seperator[x])
                {
                    flag = 1;
                    break;
                }
            }
            if(flag == 0) 
            ++j;
        }
        if(i != j)
        {
            result.push_back(s.substr(i, j-i));
            i = j;
        }
        
    }
    return result;
}

double strtodouble(const std::string& str)
{
    std::stringstream iss;
    double val;
    iss << str;
    iss >> val;
    return val;
}

std::string doubletostr(const double& str)
{
    std::stringstream iss;
    std::string val;
    iss << str;
    iss >> val;
    return val;
}

void handle_udp_msg(int fd)
{
    char buf[BUFF_LEN];
    char return_buf[BUFF_LEN]; 
    socklen_t len;
    int count;
    double speed_factor,width,speed,force,gripper_width,gripper_speed;
    int gripper_temperature;
    struct sockaddr_in clent_addr;
    std::array<double, 7> q, q_get;
    std::string buf_str, cmd, args, ip;
    std::vector<std::string> str_v, args_v, q_v, grasp_v, gripper_move_v;
    franka::Robot * robot;
    franka::Gripper * gripper;
    franka::RobotState robot_state;
    franka::GripperState gripper_state;
    static int init_flag,gripper_init_flag;

    while(1)
    {
        memset(buf, 0, BUFF_LEN);
        memset(return_buf, 0, BUFF_LEN);
        len = sizeof(clent_addr);
        count = recvfrom(fd, buf, BUFF_LEN, 0, (struct sockaddr*)&clent_addr, &len); 
        if(count == -1)
        {
            printf("recieve data fail!\n");
            return;
        }
        //command: control_command,(args_list)
        //command: init,(192.168.1.100)
        //command: get_pose,(0)
        //command: movej,0.5:12 12 21 21 12 12 21
        buf_str = std::string(buf);
        str_v = split(buf, ",");
        try
        {
            if(str_v.size() == 2)
            {
                cmd = str_v[0];
                args = str_v[1];
                std::cout << "recive:"<< cmd << "," << args << std::endl;
                if(cmd == "init") //init
                {
                    ip = args;
                    init_flag = 1;
                    robot = new franka::Robot(ip);
                    robot->automaticErrorRecovery();
                    robot->setCollisionBehavior(
                    	{{40.0, 40.0, 36.0, 36.0, 32.0, 28.0, 24.0}}, {{40.0, 40.0, 36.0, 36.0, 32.0, 28.0, 24.0}},
                    	{{40.0, 40.0, 36.0, 36.0, 32.0, 28.0, 24.0}}, {{40.0, 40.0, 36.0, 36.0, 32.0, 28.0, 24.0}},
                    	{{40.0, 40.0, 40.0, 50.0, 50.0, 50.0}}, {{40.0, 40.0, 40.0, 50.0, 50.0, 50.0}},
                    	{{40.0, 40.0, 40.0, 50.0, 50.0, 50.0}}, {{40.0, 40.0, 40.0, 50.0, 50.0, 50.0}});
                    robot->setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
                	robot->setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});
                    std::cout << "Init good! [ip = "<< ip << "]"<<std::endl;
                    return_buf[0] = '0';
                }
                else if(cmd == "movej" && init_flag==1)
                {
                    args_v = split(args, ":");
                    q_v = split(args_v[1], " ");
                    std::cout << "speed_factor=" << args_v[0] << std::endl
                            << "joints: " << q_v[1]<<std::endl;
                    speed_factor = strtodouble(args_v[0]);
                    if(q_v.size() == 7)
                    {
                        std::cout << speed_factor <<std::endl;
                        for (int i = 0; i < 7; ++i)
                        {
                            q[i] = strtodouble(q_v[i]);
                        }
                        MotionGenerator motion_generator(speed_factor, q);
                        robot->control(motion_generator);
                        return_buf[0] = '0';
                    }
                    else{std::cout << "number of joints = 7!" <<std::endl;}
                }
                else if(cmd == "get_pose" && init_flag==1)
                {
                    std::string pose_str;
                    robot_state = robot->readOnce();
                    std::array<double, 16> pose= robot_state.O_T_EE;
                    for (int i = 0; i < 16; ++i)
                    {
                        pose_str = pose_str+" "+doubletostr(pose[i]);
                    }
                    pose_str.resize(BUFF_LEN);
                    memcpy(return_buf,pose_str.c_str(),BUFF_LEN);
                    //std::cout << return_buf << std::endl;
                }
                else if(cmd == "get_joints" && init_flag==1)
                {
                    std::string pose_str;

                    robot_state = robot->readOnce();
                    q_get = robot_state.q;

                    for (int i = 0; i < 7; ++i)
                    {
                        pose_str = pose_str+" "+doubletostr(q_get[i]);
                    }
                    pose_str.resize(BUFF_LEN);
                    memcpy(return_buf,pose_str.c_str(),BUFF_LEN);
                    //std::cout << return_buf << std::endl;
                }
                else if(cmd == "recover" && init_flag==1)
                {
                    robot->automaticErrorRecovery();
                    return_buf[0] = '0';
                }
                else if(cmd == "gripper_init")
                {
                    ip = args;
                    gripper_init_flag = 1;
                    gripper = new franka::Gripper(ip);
                    return_buf[0] = '0';
                }
                else if(cmd == "gripper_home" && gripper_init_flag == 1)
                {
                    gripper-> homing();
                    return_buf[0] = '0';
                }
                else if(cmd == "gripper_grasp" && gripper_init_flag == 1)
                {
                    grasp_v = split(args, ":");
                    width = strtodouble(grasp_v[0]);
                    speed = strtodouble(grasp_v[1]);
                    force = strtodouble(grasp_v[2]);
                    std::cout << width<<" "<<speed<<" "<<force<<std::endl;
                    gripper-> grasp(width,speed,force);
                    return_buf[0] = '0';
                }
                else if(cmd == "is_grasp" && gripper_init_flag == 1)
                {
                    gripper_state = gripper->readOnce();
                    return_buf[0] = '0';
                    if(gripper_state.is_grasped)
                    {
                        return_buf[0] = '1';
                    }
                    std::cout << gripper_state.max_width << std::endl;
                }
                else if(cmd == "gripper_stop" && gripper_init_flag == 1)
                {
                    gripper-> stop();
                    return_buf[0] = '0';
                }
                else if(cmd == "gripper_width" && gripper_init_flag == 1)
                {
                    gripper_state = gripper->readOnce();
                    gripper_width = gripper_state.width;
                    std::string gripper_width_str = doubletostr(gripper_width);
                    gripper_width_str.resize(BUFF_LEN);
                    memcpy(return_buf,gripper_width_str.c_str(),BUFF_LEN);
                    return_buf[0] = '0';
                }
                else if(cmd == "gripper_temperature" && gripper_init_flag == 1)
                {
                    gripper_state = gripper->readOnce();
                    gripper_temperature = (int)gripper_state.temperature;
                    std::cout << gripper_temperature << std::endl;
                    std::stringstream iss;
                    std::string gripper_temperature_str;
                    iss << gripper_temperature;
                    iss >> gripper_temperature_str;

                    gripper_temperature_str.resize(BUFF_LEN);
                    memcpy(return_buf,gripper_temperature_str.c_str(),BUFF_LEN);
                    return_buf[0] = '0';
                }
                else if(cmd == "gripper_move" && gripper_init_flag == 1)
                {
                    gripper_move_v = split(args, ":");
                    gripper_width = strtodouble(gripper_move_v[0]);
                    gripper_speed = strtodouble(gripper_move_v[1]);
                    gripper->move(gripper_width,gripper_speed);
                    std::cout <<gripper_width<<" "<<gripper_speed<<std::endl;
                    return_buf[0] = '0';
                }
                else if(cmd == "rt_control" && init_flag==1)
                {
                    /*
                    std::string control_type = args;
                    if(control_type == 'q'){
                        // start
                        robot->control([&](const franka::RobotState& robot_state,
                                      franka::Duration period) -> franka::Torques{
                            memset(buf, 0, BUFF_LEN);
                            memset(return_buf, 0, BUFF_LEN);
                            len = sizeof(clent_addr);
                            count = recvfrom(fd, buf, BUFF_LEN, 0, (struct sockaddr*)&clent_addr, &len);
                            if(count == -1){
                                printf("recieve data fail!\n");
                            }
                            
                        });

                    }else if(control_type == 'dq'){

                    }else if(control_type == 'tq'){

                    }else if(control_type == 'tq_q'){

                    }else if(control_type == 'tq_dq'){

                    }else if(control_type == 'tq_c'){

                    }else if(control_type == 'tq_dc'){

                    }else if(control_type == 'c'){

                    }else if(control_type == 'dc'){

                    }else if(control_type == 't'){

                    }
                    */
                }
                else
                {
                    std::cout << "Do not match any cmd!" <<std::endl;
                }
            }
            else
            {
                std::cout << "Bad massage!" <<std::endl;
            }
        }
        catch (franka::Exception const& e) 
        {
            std::cout << e.what() << std::endl;
        }
        sendto(fd, return_buf, BUFF_LEN, 0, (struct sockaddr*)&clent_addr, len);
        memset(buf, 0, BUFF_LEN);
        memset(return_buf, 0, BUFF_LEN);

    }
}

static int gripper_init_flag = 0;
static int init_flag = 0;
const std::array<double, 7> q_home = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};

int main()
{
    // udp -> init -> franka -> control mode(while(true))
    int server_fd, ret;
    struct sockaddr_in ser_addr; 

    server_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if(server_fd < 0)
    {
        printf("create socket fail!\n");
        return -1;
    }

    memset(&ser_addr, 0, sizeof(ser_addr));
    ser_addr.sin_family = AF_INET;
    ser_addr.sin_addr.s_addr = inet_addr(SERVER_IP);
    ser_addr.sin_port = htons(SERVER_PORT);

    ret = bind(server_fd, (struct sockaddr*)&ser_addr, sizeof(ser_addr));
    if(ret < 0)
    {
        printf("socket bind fail!\n");
        return 0;
    }

    handle_udp_msg(server_fd);

    close(server_fd);
    return 0;
}
