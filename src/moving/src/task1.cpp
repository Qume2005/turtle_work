#include "target_manager.hpp"
#include "turtle_controller.hpp"
#include <array>
#include <chrono>
#include <functional>
#include <memory>
#include <rclcpp/executors.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/logging.hpp>
#include <tuple>
#include "config.hpp"

// 设置目标速度
const double SPEED = 0.4;

// 定义乌龟1目标数组，包含位置，速度和等待时间
std::array<Target, 11> turtle1_targets = {
    std::make_tuple(8.88, 6.88, SPEED, std::chrono::seconds(0)),
    std::make_tuple(10.294, 7.466, SPEED, std::chrono::seconds(0)),
    std::make_tuple(10.88, 8.88, SPEED, std::chrono::seconds(0)),
    std::make_tuple(10.294, 10.294, SPEED, std::chrono::seconds(0)),
    std::make_tuple(8.88, 10.88, SPEED, std::chrono::seconds(0)),
    std::make_tuple(9.466, 12.294, SPEED, std::chrono::seconds(0)),
    std::make_tuple(10.88, 12.88, SPEED, std::chrono::seconds(0)),
    std::make_tuple(12.294, 12.294, SPEED, std::chrono::seconds(0)),
    std::make_tuple(12.88, 10.88, SPEED, std::chrono::seconds(0)),
    std::make_tuple(11.88, 8.88, SPEED, std::chrono::seconds(0)),
    std::make_tuple(8.88, 4.88, SPEED, std::chrono::seconds(0))
};

// 定义乌龟2目标数组，包含位置，速度和等待时间
std::array<Target, 6> turtle2_targets = {
    std::make_tuple(8.294, 12.294, SPEED, std::chrono::seconds(0)),
    std::make_tuple(6.88, 12.88, SPEED, std::chrono::seconds(0)),
    std::make_tuple(5.466, 12.294, SPEED, std::chrono::seconds(0)),
    std::make_tuple(4.88, 10.88, SPEED, std::chrono::seconds(0)),
    std::make_tuple(5.88, 8.88, SPEED, std::chrono::seconds(0)),
    std::make_tuple(8.88, 4.88, SPEED, std::chrono::seconds(0))
};

class Task : public rclcpp::Node{
public:
    // 启动ROS节点
    Task() : Node("task1") {}
    // 使用析构函数管理线程
    ~Task() {
        if (start_thread.joinable()) {
            start_thread.join();
        }
    }
    // 启动函数
    void start() {
        int turtle1_task_idx = 0, turtle2_task_idx = 0; // 定义两个乌龟的任务索引
        auto turtle1_controller = std::make_shared<TurtleController>(Node::shared_from_this(), 1); // 实例化第一个乌龟控制器
        auto turtle2_controller = std::make_shared<TurtleController>(Node::shared_from_this(), 2); // 实例化第二个乌龟控制器

        // 任务包括set_pen，set_target，se_spawn，用法参考下面第一个乌龟的任务数组。

        // 定义第一个乌龟的任务数组
        std::array<std::function<void()>, 14> turtle1_tasks = {
            std::bind(&TurtleController::set_pen, turtle1_controller, false, 0, 0, 0, 0), // 关闭笔迹
            std::bind(&TurtleController::set_target, turtle1_controller, turtle1_targets[0]),
            std::bind(&TurtleController::set_target, turtle1_controller, turtle1_targets[1]),
            std::bind(&TurtleController::set_target, turtle1_controller, turtle1_targets[2]),
            std::bind(&TurtleController::set_target, turtle1_controller, turtle1_targets[3]),
            std::bind(&TurtleController::set_target, turtle1_controller, turtle1_targets[4]),
            std::bind(&TurtleController::set_spawn, turtle1_controller, 8.88, 10.88, 0, 2), // 创建乌龟2
            std::bind(&TurtleController::set_pen, turtle1_controller, true, 255, 192, 203, 2), // 开启笔迹，并设置笔的颜色为粉色，宽度为2
            std::bind(&TurtleController::set_target, turtle1_controller, turtle1_targets[5]),
            std::bind(&TurtleController::set_target, turtle1_controller, turtle1_targets[6]),
            std::bind(&TurtleController::set_target, turtle1_controller, turtle1_targets[7]),
            std::bind(&TurtleController::set_target, turtle1_controller, turtle1_targets[8]),
            std::bind(&TurtleController::set_target, turtle1_controller, turtle1_targets[9]),
            std::bind(&TurtleController::set_target, turtle1_controller, turtle1_targets[10])
        };

        // 定义第二个乌龟的任务数组（只有当第二个乌龟被创建后才开始执行）
        std::array<std::function<void()>, 7> turtle2_tasks = {
            std::bind(&TurtleController::set_pen, turtle2_controller, true, 255, 192, 203, 2), // 开启笔迹，并设置笔的颜色为粉色，宽度为2
            std::bind(&TurtleController::set_target, turtle2_controller, turtle2_targets[0]),
            std::bind(&TurtleController::set_target, turtle2_controller, turtle2_targets[1]),
            std::bind(&TurtleController::set_target, turtle2_controller, turtle2_targets[2]),
            std::bind(&TurtleController::set_target, turtle2_controller, turtle2_targets[3]),
            std::bind(&TurtleController::set_target, turtle2_controller, turtle2_targets[4]),
            std::bind(&TurtleController::set_target, turtle2_controller, turtle2_targets[5])
        };


        // 任务执行器（不需要改动）
        while (rclcpp::ok()) {
            if (turtle1_controller->has_init && turtle1_controller->has_reached && turtle1_task_idx < turtle1_tasks.size()) {
                turtle1_tasks[turtle1_task_idx++]();
            }
            if (turtle2_controller->has_init && turtle2_controller->has_reached && turtle2_task_idx < turtle2_tasks.size()) {
                turtle2_tasks[turtle2_task_idx++]();
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    std::thread start_thread;
private:
    
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    try {
        auto turtle_controller_node = std::make_shared<Task>();
        turtle_controller_node->start_thread = std::thread(&Task::start, turtle_controller_node); // 启动乌龟任务线程，与ROS主线程分开，避免互相阻塞
        RCLCPP_INFO(rclcpp::get_logger("Task1"), "Turtle Controller Node is running...");
        rclcpp::spin(turtle_controller_node);
    } catch (const std::exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("Task1"), "Exception thrown: %s", e.what());
    } catch (...) {
        RCLCPP_ERROR(rclcpp::get_logger("Task1"), "Unknown exception thrown.");
    }
    rclcpp::shutdown();
    return 0;
}
