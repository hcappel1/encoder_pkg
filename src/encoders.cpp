#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <chrono>
#include <math.h>
#include <cmath>
#include <vector>

#define m1_pin 0
#define m2_pin 1 

using namespace std::chrono_literals;

class EncoderTrigger : public rclcpp::Node
{
public:
	//Encoder 1 values
	int m1_time_diff_ = 0;
	float m1_time_diff_secs_ = 0.1;
	float m1_ang_vel_ = 0.0;
	float m1_ang_vel_filtered_ = 0.0;
	int m1_init_read_;
	int m1_start_time_;
	int m1_end_time_;
	int m1_updated_ = false;
	std::vector<float> m1_ang_vel_vec_;

	//Encoder 2 values
	int m2_time_diff_ = 0;
	float m2_time_diff_secs_ = 0.1;
	float m2_ang_vel_ = 0.0;
	float m2_ang_vel_filtered_ = 0.0;
	int m2_init_read_;
	int m2_start_time_;
	int m2_end_time_;
	int m2_updated_ = false;
	std::vector<float> m2_ang_vel_vec_;

	//Filtering params 
	float xn_e1 = 0;
	float yn_e1 = 0;
	float xn1_e1 = 0;
	float yn1_e1 = 0;
	int k_e1 = 3;

	//Filtering params 
	float xn_e2 = 0;
	float yn_e2 = 0;
	float xn1_e2 = 0;
	float yn1_e2 = 0;
	int k_e2 = 3;

	EncoderTrigger() : Node("encoder_trigger")
	{
		//Initialize callback groups reentrant callback group object
		callback_group_1_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
		callback_group_2_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
		callback_group_3_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
		callback_group_4_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
		callback_group_5_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

		//Publishers and Subscribers
		ang_vel_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("wheel_omegas",10);

		//Timers
		encoder1_timer_ = this->create_wall_timer(1ms, std::bind(&EncoderTrigger::encoder1Callback, this), callback_group_1_);
		encoder2_timer_ = this->create_wall_timer(1ms, std::bind(&EncoderTrigger::encoder2Callback, this), callback_group_2_);
		ang_vel_publisher_timer_ = this->create_wall_timer(10ms, std::bind(&EncoderTrigger::publisherCallback, this), callback_group_3_);
		encoder1_zero_check_timer_ = this->create_wall_timer(500ms, std::bind(&EncoderTrigger::encoder1ZeroCheckCallback, this), callback_group_4_);
		encoder2_zero_check_timer_ = this->create_wall_timer(500ms, std::bind(&EncoderTrigger::encoder2ZeroCheckCallback, this), callback_group_5_);
	}

private:
	////////////////////////////////////////////////////////
	//Callbacks for reading encoder data. Will run every 1ms
	void encoder1Callback(void) {
		m1_init_read_ = digitalRead(0);
		while(true) {
			if(digitalRead(0) != m1_init_read_) {
				m1_start_time_ = this->now().nanoseconds();
				m1_init_read_ = digitalRead(0);
				break;
			}
		}

		while(true) {
			if(digitalRead(0) != m1_init_read_) {
				m1_end_time_ = this->now().nanoseconds();
				m1_updated_ = true;
				break;
			}
		}
		if (m1_end_time_ > m1_start_time_){
			m1_time_diff_ = m1_end_time_ - m1_start_time_;
		}
		//RCLCPP_INFO(this->get_logger(), "'%d'", m1_time_diff_);
		m1_time_diff_secs_ = float(m1_time_diff_*pow(10,-9));
		m1_ang_vel_ = (1/m1_time_diff_secs_)*(M_PI/20);
		if (m1_ang_vel_ < 60) {
			m1_ang_vel_filtered_ = LowPassFilterE1();
		}
			
		// RCLCPP_INFO(this->get_logger(), "'%f'", m1_ang_vel_filtered_);

	}

	void encoder2Callback(void) {
		m2_init_read_ = digitalRead(1);
		while(true) {
			if(digitalRead(1) != m2_init_read_) {
				m2_start_time_ = this->now().nanoseconds();
				m2_init_read_ = digitalRead(1);
				break;
			}
		}

		while(true) {
			if(digitalRead(1) != m2_init_read_) {
				m2_end_time_ = this->now().nanoseconds();
				m2_updated_ = true;
				break;
			}
		}
		if (m2_end_time_ > m2_start_time_){
			m2_time_diff_ = m2_end_time_ - m2_start_time_;
		}
		//RCLCPP_INFO(this->get_logger(), "'%d'", m1_time_diff_);
		m2_time_diff_secs_ = float(m2_time_diff_*pow(10,-9));
		m2_ang_vel_ = (1/m2_time_diff_secs_)*(M_PI/20);
		if (m2_ang_vel_ < 60) {
			m2_ang_vel_filtered_ = LowPassFilterE2();
		}
			
		RCLCPP_INFO(this->get_logger(), "'%f'", m2_ang_vel_filtered_);

	}

	///////////////////////////////////////////////////////////////////////////////
	//Functions to check if the wheel has stopped spinning and set the ang vel to 0

	void encoder1ZeroCheckCallback(void) {
		if(m1_updated_ == false) {
			m1_ang_vel_filtered_ = 0.0;
		}
		m1_updated_ = false;
	}

	void encoder2ZeroCheckCallback(void) {
		if(m2_updated_ == false) {
			m2_ang_vel_filtered_ = 0.0;
		}
		m2_updated_ = false;
	}

	//////////////////////////////////////////////////////////////////////////////////

	float LowPassFilterE1() {
		m1_ang_vel_vec_.push_back(m1_ang_vel_);
		if (m1_ang_vel_vec_.size() == 3) {
			for (int i=0; i < 4; i++) {
				xn_e1 = m1_ang_vel_vec_[i];
				yn_e1 = 0.990*yn1_e1 + 0.005*xn_e1 + 0.005*xn1_e1;
				yn1_e1 = yn_e1;
				xn1_e1 = xn_e1;
			}
			m1_ang_vel_vec_.erase(m1_ang_vel_vec_.begin());
		}
		
		return yn_e1;
	}

	float LowPassFilterE2() {
		m2_ang_vel_vec_.push_back(m2_ang_vel_);
		if (m2_ang_vel_vec_.size() == 3) {
			for (int i=0; i < 4; i++) {
				xn_e2 = m2_ang_vel_vec_[i];
				yn_e2 = 0.990*yn1_e2 + 0.005*xn_e2 + 0.005*xn1_e2;
				yn1_e2 = yn_e2;
				xn1_e2 = xn_e2;
			}
			m2_ang_vel_vec_.erase(m2_ang_vel_vec_.begin());
		}
		
		return yn_e2;
	}

	void publisherCallback(void) {
		auto message = std_msgs::msg::Float32MultiArray();
		message.data.push_back(m1_ang_vel_filtered_);
		message.data.push_back(m2_ang_vel_filtered_);
		ang_vel_publisher_->publish(message);
	}
	
	rclcpp::TimerBase::SharedPtr encoder1_timer_;
	rclcpp::TimerBase::SharedPtr encoder2_timer_;
	rclcpp::TimerBase::SharedPtr ang_vel_publisher_timer_;
	rclcpp::TimerBase::SharedPtr encoder1_zero_check_timer_;
	rclcpp::TimerBase::SharedPtr encoder2_zero_check_timer_;
	rclcpp::CallbackGroup::SharedPtr callback_group_1_;
	rclcpp::CallbackGroup::SharedPtr callback_group_2_;
	rclcpp::CallbackGroup::SharedPtr callback_group_3_;
	rclcpp::CallbackGroup::SharedPtr callback_group_4_;
	rclcpp::CallbackGroup::SharedPtr callback_group_5_;
	rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr ang_vel_publisher_;
};

int main(int argc, char * argv[])
{
	//Setup GPIO pins
	wiringPiSetup();
	pinMode(0, OUTPUT);
	pinMode(1, OUTPUT);
	pinMode(4, OUTPUT);
	pinMode(3, OUTPUT);

	//Initialize node
	rclcpp::init(argc,argv);
	std::shared_ptr<EncoderTrigger> encoder_trigger_node = std::make_shared<EncoderTrigger>();

	//Initialize one multithreaded executor object
	rclcpp::executors::MultiThreadedExecutor executor;
	executor.add_node(encoder_trigger_node);
	executor.spin();

	rclcpp::shutdown();
	return 0;
}