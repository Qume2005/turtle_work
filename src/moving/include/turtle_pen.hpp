#ifndef TURTLE_PEN_HPP
#define TURTLE_PEN_HPP

#include <rclcpp/rclcpp.hpp>
#include <turtlesim/srv/set_pen.hpp>
#include <chrono>

class TurtlePen {
public:
    TurtlePen(rclcpp::Node::SharedPtr &node, int id);
    bool wait_for_service();
    void set_pen(bool is_enabled, int r, int g, int b, int width);

private:
    int id_;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<turtlesim::srv::SetPen>::SharedPtr set_pen_client_;
};

inline TurtlePen::TurtlePen(rclcpp::Node::SharedPtr &node, int id) : 
    id_(id), node_(node) {
    // 创建客户端
    set_pen_client_ = node_->create_client<turtlesim::srv::SetPen>("turtle" + std::to_string(id_) + "/set_pen");
}

inline bool TurtlePen::wait_for_service() {
    return set_pen_client_->wait_for_service(std::chrono::seconds(1));
}

inline void TurtlePen::set_pen(bool is_enabled, int r, int g, int b, int width) {
    if (!wait_for_service()) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(node_->get_logger(), "Waiting for the service turtle1/set_pen to become available...");
        return; // 确保如果服务不可用则返回
    }

    auto request = std::make_shared<turtlesim::srv::SetPen::Request>();
    request->set__r(r);
    request->set__g(g);
    request->set__b(b);
    request->set__width(width);
    request->set__off(!is_enabled);

    // 发送请求
    if (!set_pen_client_->service_is_ready()) {
        RCLCPP_WARN(node_->get_logger(), "SetPen service is not ready.");
        return;
    }

    auto future = set_pen_client_->async_send_request(request,
        [this](rclcpp::Client<turtlesim::srv::SetPen>::SharedFuture future) {
            try {
                if (rclcpp::ok()) { // 检查节点是否仍在运行
                    future.get(); // 确保成功获取响应
                    RCLCPP_INFO(node_->get_logger(), "Pen set successfully.");
                } else {
                    RCLCPP_WARN(node_->get_logger(), "Ignoring SetPen request, node is shutting down.");
                }
            } catch (const std::exception &e) {
                RCLCPP_ERROR(node_->get_logger(), "Failed to set pen: %s", e.what());
            }
        });
}

#endif // TURTLE_PEN_HPP
