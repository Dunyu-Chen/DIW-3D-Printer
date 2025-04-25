//
// Created by dunyu on 25-4-24.
//
#include "std_msgs/msg/float64.hpp"
#include "rclcpp/rclcpp.hpp"

#include <functional>
#include <chrono>
#include <memory>
class MSD_simulator
{
public:
    double m,k,b,sample_time;
    double pos,vel,ace;
    MSD_simulator(double M,double K,double B,double Sample_time):
        m(M),k(K),b(B), sample_time(Sample_time),
        pos(0.0),vel(0.0),ace(0.0)
    {}
    void reset(double Init_pos, double Init_vec)
    {
        this->pos=Init_pos;
        this->vel=Init_vec;
    }
    double step(double Force)
    {
        ace = (1/m)*(-k*pos-b*vel+Force);
        pos += vel*sample_time;
        vel += ace * sample_time;
        return pos;
    }
};



class MSD_sim_node : public rclcpp::Node
{
public:
    MSD_simulator simulator;
    MSD_sim_node(double M,double K,double B,double Sample_time) :
        rclcpp::Node("msd_simulator_node"),
        simulator(M, K, B, Sample_time),
        timer_(
            this->create_wall_timer(
                std::chrono::milliseconds(static_cast<int>(Sample_time * 1e3)),
                std::bind(&MSD_sim_node::timer_callback, this)
            )
        ),
        publisher_position(this->create_publisher<std_msgs::msg::Float64>("position", 10)),
        subscription_force(
            this->create_subscription<std_msgs::msg::Float64>(
                "force", 10,
                std::bind(
                    &MSD_sim_node::subscription_callback, this, std::placeholders::_1
                )
            )
        ),
        force_data(0.0)
    {
    }

private:
    void timer_callback()
    {
        std_msgs::msg::Float64 publish_msg;
        publish_msg.data = simulator.step(force_data);
        publisher_position->publish(publish_msg);
    }
    void subscription_callback(const std_msgs::msg::Float64& sub_msg)
    {
        force_data = sub_msg.data;
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_position;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_force;
    double force_data;
};



int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    double sample_time = 0.01; // 10ms = 0.01s
    double m =1.0,b=1.0,k=1.0;
    auto node_ = std::make_shared<MSD_sim_node>(
        m,k,b,sample_time
    );
    double init_pos =0.0,init_vec=0.0;
    node_->simulator.reset(init_pos,init_vec);
    rclcpp::spin(node_);
    rclcpp::shutdown();
    return 0;
}
