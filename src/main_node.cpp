#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class MainClass : public rclcpp::Node {
	rclcpp::TimerBase::SharedPtr timer_;
	
	public : MainClass() : Node("main_node") {
		RCLCPP_INFO(this->get_logger(), "Running main_node...");
		timer_ = this->create_wall_timer(500ms, std::bind(&MainClass::main_function, this));
	}
	
	private : void main_function() {
		RCLCPP_INFO(this->get_logger(), "Executing main_function...");
	}
};

int main(int argc, char * argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<MainClass>());
	rclcpp::shutdown();
	return 0;
}
