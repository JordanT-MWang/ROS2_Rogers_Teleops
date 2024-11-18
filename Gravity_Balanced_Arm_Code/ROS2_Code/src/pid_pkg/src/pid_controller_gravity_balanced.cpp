#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "std_msgs/msg/int64.hpp"
#include <chrono>
#include <cmath>
using namespace std::chrono_literals;
using std::placeholders::_1;
/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class PDController : public rclcpp::Node
{
public:
  PDController()
  : Node("pd_controller_node_gravity_balanced")
  {
    //pub test values outputed by controller. delete or comment when not testing
    controller_publisher_ = this->create_publisher<geometry_msgs::msg::Point>("controller_output_gravity_balanced", 10);


    //init publisher to post pwmvalues and timer for publisher
    pwm_publisher_ = this->create_publisher<geometry_msgs::msg::Point>("pwm_values_gravity", 10);
    timer_ = this->create_wall_timer(0.500ms, std::bind(&PDController::timer_callback, this));//change the ms seconds to match on the ardino code for best posting speeds
    //init subscripber to encoder values
    data_subscription_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "encoder_counter_gravity", 10, std::bind(&PDController::sensor_topic_callback, this, _1));
    //init subcriber to reference angle
    reference_subscription_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "encoder_counter_spring_delay", 10, std::bind(&PDController::reference_angles_topic_callback, this, _1));
    //init subscriber to control mode 
    calc_delay_publisher_ = this->create_publisher<std_msgs::msg::Int64>("calculated_delay_spring_to_gravity",10);
    //init subscriber to tuning p variables
    pid_p_tune_subscription_ = this->create_subscription<geometry_msgs::msg::Point>(
      "pid_tune_p_vars_gravity", 10, std::bind(&PDController::pid_tune_p_vars_topic_callback, this, _1));
    //init subscriber to tuning i variables
    pid_i_tune_subscription_ = this->create_subscription<geometry_msgs::msg::Point>(
      "pid_tune_i_vars_gravity", 10, std::bind(&PDController::pid_tune_i_vars_topic_callback, this, _1));
    //init subscriber to tuning d variables
    pid_d_tune_subscription_ = this->create_subscription<geometry_msgs::msg::Point>(
      "pid_tune_d_vars_gravity", 10, std::bind(&PDController::pid_tune_d_vars_topic_callback, this, _1));
    //init values for stable arm
    
    kp_values[0] =-1;
    kp_values[1] =-1;
    kp_values[2] =-4.5;
    
    kd_values[0] = 0.2;
    kd_values[1] = 0.25;
    kd_values[2] = 0.25;
    
    
    sampling_time = 500/(pow(10.0,6.0));
  }

private:
  //will get change of turning  p variable and record them in kp_values array. notation. x = base, y = shoulder, z = elbow)
  void pid_tune_p_vars_topic_callback(const geometry_msgs::msg::Point::SharedPtr p_msg)
  {
    kp_values[0]= p_msg->x;
    kp_values[1]= p_msg->y;
    kp_values[2]= p_msg->z;
  }
  //will get change of turning  i variable and record them in ki_values array. notation. x = base, y = shoulder, z = elbow)
  void pid_tune_i_vars_topic_callback(const geometry_msgs::msg::Point::SharedPtr i_msg)
  {
    ki_values[0]= i_msg->x;
    ki_values[1]= i_msg->y;
    ki_values[2]= i_msg->z;
  }
  //will get change of turning  d variable and record them in kd_values array. notation. x = base, y = shoulder, z = elbow)
  void pid_tune_d_vars_topic_callback(const geometry_msgs::msg::Point::SharedPtr d_msg)
  {
    kd_values[0]= d_msg->x;
    kd_values[1]= d_msg->y;
    kd_values[2]= d_msg->z;
  }
  //will get change of reference angles and record them in referenace_angles array. notation. x = base, y = shoulder, z = elbow)
  void reference_angles_topic_callback(const geometry_msgs::msg::PointStamped::SharedPtr ref_msg)
  {
    //exponental weighting for the reference anlges
    alpha[0] = 1 - exp((-1*sampling_time)*filter_const[0]);
    alpha[1] = 1 - exp((-1*sampling_time)*filter_const[1]);
    alpha[2] = 1 - exp((-1*sampling_time)*filter_const[2]);
    reference_angles[0]= reference_angles_prev[0]*(1-alpha[0]) + ref_msg->point.x*(270.0/3030.0)*alpha[0];
    reference_angles[1]= reference_angles_prev[1]*(1-alpha[1]) + ref_msg->point.y*(150.0/2600.0)*alpha[1];
    reference_angles[2]= reference_angles_prev[2]*(1-alpha[2]) +ref_msg->point.z*(90.0/1075.0)*alpha[2];

    reference_angles_prev[0]= ref_msg->point.x*(270.0/3030.0);
    reference_angles_prev[1]= ref_msg->point.y*(150.0/2600.0);
    reference_angles_prev[2]= ref_msg->point.z*(90.0/1075.0);
    /*working
    reference_angles[0]= ref_msg->point.x*(270.0/3030.0);
    reference_angles[1]= ref_msg->point.y*(150.0/2600.0);
    reference_angles[2]= ref_msg->point.z*(90.0/1075.0);
    */
    auto delay_msg = std_msgs::msg::Int64();
    delay_msg.data = static_cast<unsigned long int>(std::chrono::system_clock::now().time_since_epoch().count()) -
          (unsigned long int)(ref_msg->header.stamp.sec * std::pow(10,9) + ref_msg->header.stamp.nanosec);
    calc_delay_publisher_->publish(delay_msg);
  }
  //will get change of encoder data when occers  and record them in encoder_angles array. notation. x = base, y = shoulder, z = elbow)
  void sensor_topic_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    //calculation to turn the encoder counts to angles for small arm
    /*
    current_angles[0]=msg->x*(270.0/3030.0);
    current_angles[1]=msg->y*(150.0/2600.0);
    current_angles[2]=msg->z*(90.0/1075.0);
    */
    current_angles[0]=msg->point.x*(270.0/199500.0);
    current_angles[1]=msg->point.y*(175.0/118550.0);
    current_angles[2]=msg->point.z*(190.0/141500.0);
    
    // for testing angles
    //auto pwm_values = geometry_msgs::msg::Point();
    //pwm_values.x = current_angles[0];
    //pwm_values.y = current_angles[1];
    //pwm_values.z = current_angles[2];
    //pwm_publisher_->publish(pwm_values);
    //controller_publisher_->publish(pwm_values);

  }
  //calculates pwmvalues. PID Controller
  void timer_callback()
  {
    
    //calculate error based on current and reference angles and output 
    
    for(int joint_in_arm=0; joint_in_arm < num_joints; joint_in_arm++){
      //error for each joint
      error[joint_in_arm] = reference_angles[joint_in_arm] - current_angles[joint_in_arm];
      //cumulative error for each joint. not used
      com_error[joint_in_arm] += (error[joint_in_arm]*sampling_time);
      //change in error
      //d_error[joint_in_arm] = (error[joint_in_arm] - prevous_input[joint_in_arm])/sampling_time;
      //set prvouse_error to current error
      
      //calculat output from controller
      controller_output[joint_in_arm] = kp_values[joint_in_arm]*error[joint_in_arm] + ki_values[joint_in_arm]*com_error[joint_in_arm]
                                        +kd_values[joint_in_arm]*(current_angles[joint_in_arm] - prevous_input[joint_in_arm])/sampling_time;
      prevous_input[joint_in_arm] = current_angles[joint_in_arm];
      //scale down the rand and map it to 0-255 for pwm output
      if(controller_output[joint_in_arm] > 240.0){
        controller_output[joint_in_arm] = 240.0;
      }else if (controller_output[joint_in_arm] < -240.0){
        controller_output[joint_in_arm] = -240.0;
      }
      pwm_output[joint_in_arm] = (0.5* controller_output[joint_in_arm] + 120.0)/1;
      if ((pwm_output[joint_in_arm] < float(121.0)) && (pwm_output[joint_in_arm] > float(119.0))){
        pwm_output[joint_in_arm] = 120.0;
      }
      
    }
    //create message to post to the pwm topic to be sent to the arm motors and set the values
    /*for testing controller output*/
    auto pwm_values = geometry_msgs::msg::Point();
    //pwm_values.x = controller_output[0];
    //pwm_values.y = controller_output[1];
    //pwm_values.z = controller_output[2];
    //publish values to topic
    //controller_publisher_->publish(pwm_values);
    //create message to post to the pwm topic to be sent to the arm motors and set the values
    
    pwm_values.x = pwm_output[0];
    pwm_values.y = pwm_output[1];
    pwm_values.z = pwm_output[2];
    //publish values to topic
    pwm_publisher_->publish(pwm_values);
  }
  //shared pointer for the timer, publisher, and subscriber
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr calc_delay_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pwm_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr controller_publisher_;//test publisher. comment or delete when not in use
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr data_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr reference_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr pid_p_tune_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr pid_i_tune_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr pid_d_tune_subscription_;
  //var to stor the reference angles and current encoder values. float for topic values
  float reference_angles[3]={0,0,0};
  float reference_angles_prev[3]={0,0,0};
  float current_angles[3] ={0,0,0};
  //vars to store the pid controller error and com error
  double error[3] = {0,0,0};
  double com_error[3] = {0,0,0};
  double d_error[3]  = {0,0,0};
  double prevous_input[3]  = {0,0,0};
  //var to store pid values
  float kp_values[3] ={0,0,0};
  float ki_values[3] ={0,0,0};
  float kd_values[3] ={0,0,0};
  //change to match timer callback.....!!!!!!!!!!!!!!!!!!!!!!!!!!
  double sampling_time = 500/(pow(10.0,6.0));//in ms
  float angle_error[3] = {0,0,0};
  float angle_error_sum[3] = {0,0,0};
  float controller_output[3]= {0,0,0};
  float pwm_output[3]= {0,0,0};
  int num_joints =3;
  float filter_const[3] = {0.02,0.02,0.01};
  float alpha[3];
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PDController>());
  rclcpp::shutdown();
  return 0;
}