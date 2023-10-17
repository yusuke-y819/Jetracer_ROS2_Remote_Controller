/*!
  @file
  @brief : publish command from zerotech controller
  @brief : publish navigation order based on zerotech command
  @author : Tetsuo TOMIZAWA, Kiyoshi MATSUO Masataka Hirai, Naoki AKAI
*/

// #include <ros/ros.h>
#include "rclcpp/rclcpp.hpp"
#include <stdio.h>
#include <time.h>
#include <sys/time.h>
#include <unistd.h>
#include <math.h>
#include <pthread.h>
#include "control_msgs/msg/jetracer_order.hpp"

#include "controller.hpp"

using namespace std::chrono_literals;

int g_hundle = 0;
int g_pedal[3];
char g_cross[2];
char g_button[24];
int tm = 0;

void *sleeped(void *arg)
{
	while (rclcpp::ok())
	{
		readJoystick(&tm, &g_hundle, g_pedal, g_cross, g_button);
	}
	exit(0);
}

class LogicoolNode : public rclcpp::Node
{
	public:
	LogicoolNode() : Node("jetracer_controller_logciool"), count_(0)
	{
		publisher_ = this->create_publisher<control_msgs::msg::JetracerOrder>("/jetracer_order_logi", 10);
		timer_ = this->create_wall_timer(50ms, std::bind(&LogicoolNode::timer_callback, this));
		int k = 0;
        hundle = g_hundle;
		for (k = 0; k < 3; k++) {
			pedal[k] = g_pedal[k];
		}
		for (k = 0; k < 2; k++) {
			cross[k] = g_cross[k];
		}
		for (k = 0; k < 24; k++) {
			button[k] = g_button[k];
		}
	}
	private:
	rclcpp::Publisher<control_msgs::msg::JetracerOrder>::SharedPtr publisher_;
	rclcpp::TimerBase::SharedPtr timer_;
	size_t count_;

	//USBコントローラ用配列・変数
    int hundle = 0;
	int pedal[3];
	char cross[2];
	char button[24];

	// ZeroTech用変数等
	double measured_time; 
	double input_offset=0.0;
	char manual_mode = 0;	// 初期状態は自動モード

	void timer_callback()
	{
		int k = 0;
        hundle = g_hundle;
		for (k = 0; k < 3; k++) {
			pedal[k] = g_pedal[k];
		}
		for (k = 0; k < 2; k++) {
			cross[k] = g_cross[k];
		}
		for (k = 0; k < 24; k++) {
			button[k] = g_button[k];
		}
		control_msgs::msg::JetracerOrder jetracer_order_logi;
		//ここからLogicoolから車両への命令処理
		if (button[3] == 1) {
			fprintf(stderr, "change manual mode\n");
			manual_mode = 1;
		}
		if (button[1] == 1) {
			fprintf(stderr, "change auto mode\n");	
			manual_mode = 0;
		}
		jetracer_order_logi.enable = 0;
		
		if (manual_mode == 1) {
			fprintf(stderr, "Manual_mode %3d %3d %3d %3d \n",hundle,pedal[0],pedal[1],pedal[2]);
			/*ジョイスティックからの操作をセニアカーの指令に*/
			// 入力範囲（-32<pedal[2]<32）  出力範囲(-1.0<set_vel<1.0)
			if (pedal[1] + 32 == 0 && hundle + 32 == 0) {
				jetracer_order_logi.set_vel = 0.0;
			} else if (pedal[1] != -32) {	// 前進用スティックが握られているとき
				jetracer_order_logi.set_vel =  (double)(pedal[1] + 32) * 1.0  / 64.0;
			} else {
				jetracer_order_logi.set_vel = -(double)(hundle + 32) * 0.55 / 64.0;
			}
			// 入力範囲（-32<control.hundle<32）  出力範囲(-pi/3<str_angle<pi/3) 		 
			jetracer_order_logi.str_angle = (double)(pedal[2])*M_PI/3.0/64.0+input_offset;    
			// ステアリングオフセット指令		
			input_offset -= cross[0]*M_PI/360.0;
			if (input_offset >= M_PI/36.0) {input_offset=M_PI/36.0;}
			if (input_offset <= -M_PI/36.0) {input_offset=-M_PI/36.0;}
			jetracer_order_logi.str_offset = input_offset; 
		    if (button[9] == 1) {input_offset=0;}
		    jetracer_order_logi.enable = 1;
		} else {
		    jetracer_order_logi.enable = 0;
			jetracer_order_logi.set_vel =  0.0;
			jetracer_order_logi.str_angle = 0.0;
		}
		fprintf(stderr, "str: %lf\n", jetracer_order_logi.str_angle*180.0/M_PI );
		publisher_->publish(jetracer_order_logi);
	}
};


int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	if (openJoystick() == 1) {
		fprintf(stderr, "Cannot open controller¥n");
		return 1;
	}
	pthread_t tid;
	pthread_create(&tid, NULL, sleeped, NULL);
	initializeButton(&g_hundle, g_pedal, g_cross, g_button);
	fprintf(stderr, "str: %lf\n L LstUD | RstRL RstUD | +RL +UD |  A  B  X  Y  z  w | L1 L2 R1 R2 |\n");
	fprintf(stderr, "|   %3d   %3d |   %3d   %3d | %3d %3d |  %d  %d  %d  %d  %d  %d |  %d  %d  %d  %d |\n",
				g_hundle, g_pedal[0], g_pedal[1], g_pedal[2], g_cross[0], g_cross[1],
				g_button[0], g_button[1], g_button[2], g_button[3],
				g_button[4], g_button[5], g_button[6], g_button[7],
				g_button[8], g_button[9]);
	int sum = 0;
	int i;

	while (1) {
		if (readJoystick(&tm, &g_hundle, g_pedal, g_cross, g_button) == 1) {
			fprintf(stderr,"READ ERRO");
		}
		fprintf(stderr, "|   %3d   %3d |   %3d   %3d | %3d %3d |  %d  %d  %d  %d  %d  %d |  %d  %d  %d  %d |\n",
			   g_hundle, g_pedal[0], g_pedal[1], g_pedal[2], g_cross[0], g_cross[1],
			   g_button[0], g_button[1], g_button[2], g_button[3],
			   g_button[4], g_button[5], g_button[6], g_button[7],
			   g_button[8], g_button[9]);
		for (i = 0; i < 24; ++i) {sum += g_button[i];}
		if (sum > 3) {break;}
		sum = 0;
		if(g_hundle == -32 && g_pedal[1] == -32){break;}	
	}
	fprintf(stderr, "Logicool Control OK \n");
	rclcpp::spin(std::make_shared<LogicoolNode>());
	rclcpp::shutdown();
	return 0;
}
