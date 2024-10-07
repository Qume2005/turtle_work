#ifndef TURTLE_CONTROLLER_HPP
#define TURTLE_CONTROLLER_HPP

#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include "target_manager.hpp"
#include "turtle_mover.hpp"
#include <turtlesim/msg/pose.hpp>
#include <turtlesim/srv/set_pen.hpp>
#include <chrono>
#include <geometry_msgs/msg/twist.hpp>
#include "config.hpp"
#include "turtle_spawn.hpp"
#include "turtle_state.hpp"
#include "turtle_pen.hpp"

class TurtleController {
public:
    TurtleController(
        const rclcpp::Node::SharedPtr nh,
        const int id
    ) :
        has_init(false),
        has_reached(true),
        nh_(nh),
        id_(id),
        wait_timer_(nullptr),
        state_manager_(),
        turtle_mover_(),
        turtle_spawn_(nh_),
        turtle_pen_(nh_, id_)
    {
        pose_subscription_ = nh_->create_subscription<turtlesim::msg::Pose>(
            "turtle" + std::to_string(id_) + "/pose",
            10,
            std::bind(&TurtleController::pose_callback, this, std::placeholders::_1)
        );
        cmd_vel_publisher_ = nh_->create_publisher<geometry_msgs::msg::Twist>(
            "turtle" + std::to_string(id_) + "/cmd_vel",
            10
        );
        timer_ = nh_->create_wall_timer(
            std::chrono::milliseconds(1),
            std::bind(&TurtleController::timer_callback, this)
        );
    }

    std::atomic<bool> has_init; // 是否被创建
    std::atomic<bool> has_reached;
    turtlesim::msg::Pose current_pose_; // 当前坐标
    void set_target(Target target);
    void set_spawn(const float x, const float y, const float theta, const int id);
    void set_pen(bool is_enabled, int r, int g, int b, int widt);

private:
    rclcpp::Node::SharedPtr nh_;
    int id_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr wait_timer_;
    Target target_;
    TurtleStateManager state_manager_;
    TurtleMover turtle_mover_;
    TurtleSpawn turtle_spawn_;
    TurtlePen turtle_pen_;

    void wait_for_services();
    void pose_callback(const turtlesim::msg::Pose::SharedPtr msg);
    void timer_callback();
    void wait_callback();
    inline double target_x() const { return std::get<0>(target_); }
    inline double target_y() const { return std::get<1>(target_); }
};

void TurtleController::wait_for_services() {
    RCLCPP_INFO(nh_->get_logger(), "Waiting for turtlesim services to become available...");
    // 等待 turtlesim 相关服务连接
    while (!turtle_pen_.wait_for_service()) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(nh_->get_logger(), "Interrupted while waiting for the services. Exiting.");
            return;
        }
        if (has_init) {
            RCLCPP_INFO(nh_->get_logger(), "Still waiting for turtlesim services...");
        }
    }

    RCLCPP_INFO(nh_->get_logger(), "TurtleController instance created");
}

void TurtleController::set_target(Target target) {
    target_ = target; // 初始化目标点
    has_reached = false;
}

void TurtleController::set_spawn(const float x, const float y, const float theta, const int id) {
    turtle_spawn_.spawn_turtle(x, y, theta, "turtle" + std::to_string(id));
}

void TurtleController::set_pen(bool is_enabled, int r, int g, int b, int width) {
    turtle_pen_.set_pen(is_enabled, r, g, b, width);
}

void TurtleController::pose_callback(const turtlesim::msg::Pose::SharedPtr msg) {
    if (!has_init) {
        wait_for_services(); // 等待 turtlesim 相关服务连接
        turtle_pen_.set_pen(false, 0, 0, 0, 0); // 默认关闭画笔
        state_manager_.set_state(TurtleState::MOVING);
        has_init = true;
    }
    current_pose_ = *msg;
}

void TurtleController::timer_callback() {
    if (has_reached) return;
    if (state_manager_.get_state() == TurtleState::WAITING) return;

    geometry_msgs::msg::Twist move_cmd = turtle_mover_.calculate_movement(current_pose_, target_);
    cmd_vel_publisher_->publish(move_cmd);

    if (move_cmd.linear.x == 0 && move_cmd.angular.z == 0) {
        wait_timer_ = nh_->create_wall_timer(std::get<3>(target_), std::bind(&TurtleController::wait_callback, this));
        state_manager_.set_state(TurtleState::WAITING); 
        wait_timer_->reset(); 
    }
}

void TurtleController::wait_callback() {
    state_manager_.set_state(TurtleState::MOVING); 
    has_reached = true;
    wait_timer_->cancel();
    RCLCPP_INFO(nh_->get_logger(), "Waiting time finished, moving to the next target.");
}

#endif // TURTLE_CONTROLLER_HPP
