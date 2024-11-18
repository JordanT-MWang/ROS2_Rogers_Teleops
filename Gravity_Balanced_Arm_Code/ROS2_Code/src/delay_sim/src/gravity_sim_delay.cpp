#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/int64.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include <chrono>
#include <cmath>
#include <iostream>
#include <queue> 
using namespace std::chrono_literals;
using std::placeholders::_1;
/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class SimDelay : public rclcpp::Node
{
public:
  SimDelay()
  : Node("sim_delay_gravity")
  {
    //pub enocder values 
    delay_publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>("encoder_counter_gravity_delay", 10);

    //init subscripber to encoder values
    data_subscription_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "encoder_counter_gravity", 10, std::bind(&SimDelay::sensor_topic_callback, this, _1));
    //init subscriber to delay amount
    delay_subscription_ = this->create_subscription<std_msgs::msg::Int64>(
      "delay_sim_value_gravity", 10, std::bind(&SimDelay::delay_value_callback, this, _1));
  }

private:
  void delay_value_callback(const std_msgs::msg::Int64::SharedPtr msg){
    sim_delay = (unsigned long int)msg->data;
  }
  //will get change of encoder data when occers  and record them in encoder_angles array. notation. x = base, y = shoulder, z = elbow)
  void sensor_topic_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    input_buffer.push(msg);

    //RCLCPP_INFO_STREAM(this->get_logger(),"test");

    while(((static_cast<unsigned long int>(std::chrono::system_clock::now().time_since_epoch().count())) -
          (unsigned long int)(input_buffer.front()->header.stamp.sec * std::pow(10,9) + input_buffer.front()->header.stamp.nanosec))
            > sim_delay){
        //RCLCPP_INFO_STREAM(this->get_logger(),"test2   " << ( (unsigned long int)(input_buffer.front()->header.stamp.sec * std::pow(10,9) + input_buffer.front()->header.stamp.nanosec)));
        //RCLCPP_INFO_STREAM(this->get_logger(),"test2.1 " << ((static_cast<unsigned long int>(std::chrono::high_resolution_clock::now().time_since_epoch().count()))));
        auto output_msg = geometry_msgs::msg::PointStamped();
        output_msg.header = input_buffer.front()->header;
        output_msg.point = input_buffer.front()->point;
        //RCLCPP_INFO_STREAM(this->get_logger(),"test3   " << sim_delay);
        //RCLCPP_INFO_STREAM(this->get_logger(),"test3.1 " << ((unsigned long int)(input_buffer.front()->header.stamp.sec * std::pow(10,9) + input_buffer.front()->header.stamp.nanosec)) -
        //                    (unsigned long int)(std::chrono::high_resolution_clock::now().time_since_epoch().count()));
        
        //output_msg.header = input_buffer.front()->header;
        //output_msg.point = input_buffer.front()->point;
        
        delay_publisher_->publish(output_msg);    
        //RCLCPP_INFO_STREAM(this->get_logger(),"test4");
        input_buffer.pop();
        //RCLCPP_INFO_STREAM(this->get_logger(),"test5");
        if(input_buffer.empty()){
          break;
        }
        

    }
    

  }
  
  //shared pointer for the timer, publisher, and subscriber
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr delay_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr data_subscription_;
  rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr delay_subscription_;
  unsigned long int sim_delay = 0;

  std::queue<geometry_msgs::msg::PointStamped::SharedPtr> input_buffer;
  
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimDelay>());
  rclcpp::shutdown();
  return 0;
}