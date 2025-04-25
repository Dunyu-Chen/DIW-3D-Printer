//
// Created by dunyu on 25-4-24.
//

#include "std_msgs/msg/float64.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include <functional>
#include <chrono>
#include <memory>

class PID_incremental
{
public:
    double k_p,k_i,k_d;
    double error_k, error_k1,error_k2;
    double control_output;

    PID_incremental (double K_p,double K_i, double K_d) :
    k_p(K_p),k_i(K_i),k_d(K_d),
    error_k(0.0),error_k1(0.0),error_k2(0.0),control_output(0.0)
    {}

    double step(double Target, double State)
    {
        error_k2 = error_k1;
        error_k1 = error_k;
        error_k = Target - State;
        control_output += k_p*(error_k-error_k1)+k_i*(error_k)+k_d*(error_k-2*error_k1+error_k2);
        return control_output;
    }
};
class PID_node : public rclcpp::Node
{
public:
    PID_incremental controller;
    PID_node(double K_p,double K_i, double K_d,double Sample_time) :
    rclcpp::Node("pid_controller"),
    controller(K_p,K_i,K_d),
    timer_(
            this->create_wall_timer(
                std::chrono::milliseconds(static_cast<int>(Sample_time * 1e3)),
                std::bind(&PID_node::timer_callback,this)
            )
        ),
    publisher_force(this->create_publisher<std_msgs::msg::Float64>("force", 10)),
    subscription_state(
        this->create_subscription<std_msgs::msg::Float64>(
            "position",10,
            std::bind(
                &PID_node::sub_pos_callback,this,std::placeholders::_1
            )
        )
    ),
    subscription_target(
    this->create_subscription<std_msgs::msg::Float64>(
        "target",10,
            std::bind(
                &PID_node::sub_target_callback,this,std::placeholders::_1
            )
        )
    ),
    target_data(0.0),state_data(0.0)
    {}
private:
    void timer_callback()
    {
        std_msgs::msg::Float64 publish_msg;
        double force =controller.step(target_data,state_data);
        publish_msg.data = force;
        publisher_force->publish(publish_msg);
    }

    void sub_pos_callback(const std_msgs::msg::Float64& sub_msg)
    {
        state_data = sub_msg.data;
    }
    void sub_target_callback(const std_msgs::msg::Float64& sub_msg)
    {
        target_data = sub_msg.data;
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_force;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_state;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_target;
    double target_data;
    double state_data;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    double sample_time = 0.01; // 10ms = 0.01s
    double k_p =0.05,k_i=1,k_d=1000;
    auto node_ = std::make_shared<PID_node>(
        k_p,k_i,k_d,sample_time
    );
    rclcpp::spin(node_);
    rclcpp::shutdown();
    return 0;
}