#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <chrono>
#include <math.h>
#include <cmath>
#include <vector>

using namespace std::chrono_literals;

class EncoderTrigger : public rclcpp::Node
{
public:
	int m1_last_ = 0;
	int m1_curr_time_ = 0;
	int m1_prev_time_ = 0;
	int m1_time_diff_ = 0;
	float m1_time_diff_secs_ = 0.1;
	float m1_ang_vel_ = 0.0;
	float m1_ang_vel_filtered_ = 0.0;
	int m1_init_read_;
	int m1_start_read_;
	int m1_start_time_;
	int m1_end_time_;
	std::vector<float> m1_ang_vel_vec_;
	int m1_updated_ = false;

	//Filtering params 2
	float xn = 0;
	float yn = 0;
	float xn1 = 0;
	float yn1 = 0;
	int k = 3;

	EncoderTrigger() : Node("encoder_trigger")
	{
		// wiringPiISR (17, INT_EDGE_FALLING, &EncoderTrigger::interruptCallback);
		//Initialize one reentrant callback group object
		callback_group_1_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
		callback_group_2_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
		callback_group_3_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
		ang_vel_publisher_ = this->create_publisher<std_msgs::msg::Float32>("wheel_velocities",10);
		encoder1_timer_ = this->create_wall_timer(1ms, std::bind(&EncoderTrigger::encoder1Callbacknew, this), callback_group_1_);
		ang_vel_timer_ = this->create_wall_timer(10ms, std::bind(&EncoderTrigger::publisherCallback, this), callback_group_2_);
		encoder1_zero_check_timer_ = this->create_wall_timer(500ms, std::bind(&EncoderTrigger::encoderZeroCheckCallback, this), callback_group_3_);
	}

private:
	void encoder1Callback(void) {
		if (digitalRead(0) == 1){
			if (m1_last_ == 0) {
				// RCLCPP_INFO(this->get_logger(), "HIGH");
				m1_curr_time_ = this->now().nanoseconds();
				if (m1_curr_time_ > m1_prev_time_) {
					m1_time_diff_ = m1_curr_time_ - m1_prev_time_;
					m1_time_diff_secs_ = float(m1_time_diff_*pow(10,-9));
					m1_ang_vel_ = 1/m1_time_diff_secs_*M_PI/20;
					RCLCPP_INFO(this->get_logger(), "'%f'", m1_ang_vel_);
				}
			}
			m1_last_ = 1;
			m1_prev_time_ = m1_curr_time_;
		}
		else {
			if (m1_last_ == 1) {
				// RCLCPP_INFO(this->get_logger(), "LOW");
				m1_curr_time_ = this->now().nanoseconds();
				if (m1_curr_time_ > m1_prev_time_) {
					m1_time_diff_ = m1_curr_time_ - m1_prev_time_;
					m1_time_diff_secs_ = float(m1_time_diff_*pow(10,-9));
					m1_ang_vel_ = 1/m1_time_diff_secs_*M_PI/20;
					RCLCPP_INFO(this->get_logger(), "'%f'", m1_ang_vel_);
				}
			}
			m1_last_ = 0;
			m1_prev_time_ = m1_curr_time_;
		}

	}

	void encoder1Callbacknew(void) {
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
			m1_ang_vel_filtered_ = LowPassFilter(m1_ang_vel_);
		}
			
		RCLCPP_INFO(this->get_logger(), "'%f'", m1_ang_vel_filtered_);

	}

	void encoderZeroCheckCallback(void) {
		if(m1_updated_ == false) {
			m1_ang_vel_filtered_ = 0.0;
		}
		m1_updated_ = false;
	}

	float LowPassFilter(float ang_vel) {
		m1_ang_vel_vec_.push_back(ang_vel);
		if (m1_ang_vel_vec_.size() == 3) {
			for (int i=0; i < 4; i++) {
				xn = m1_ang_vel_vec_[i];
				yn = 0.990*yn1 + 0.005*xn + 0.005*xn1;
				yn1 = yn;
				xn1 = xn;
			}
			m1_ang_vel_vec_.erase(m1_ang_vel_vec_.begin());
		}
		
		return yn;
	}

	void publisherCallback(void) {
		auto message = std_msgs::msg::Float32();
		message.data = m1_ang_vel_filtered_;
		if (m1_ang_vel_filtered_ < 60) {
			ang_vel_publisher_->publish(message);
		}
	}
	rclcpp::TimerBase::SharedPtr encoder1_timer_;
	rclcpp::TimerBase::SharedPtr ang_vel_timer_;
	rclcpp::TimerBase::SharedPtr encoder1_zero_check_timer_;
	rclcpp::CallbackGroup::SharedPtr callback_group_1_;
	rclcpp::CallbackGroup::SharedPtr callback_group_2_;
	rclcpp::CallbackGroup::SharedPtr callback_group_3_;
	rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr ang_vel_publisher_;
};

int main(int argc, char * argv[])
{
	wiringPiSetup();
	pinMode(0, OUTPUT);
	rclcpp::init(argc,argv);
	std::shared_ptr<EncoderTrigger> encoder_trigger_node = std::make_shared<EncoderTrigger>();

	//Initialize one multithreaded executor object
	rclcpp::executors::MultiThreadedExecutor executor;
	executor.add_node(encoder_trigger_node);
	executor.spin();

	rclcpp::shutdown();
	return 0;
}