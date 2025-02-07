#include "chrono"
// ROS
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
// keybow input
#include <iostream>
#include <ncurses.h>
#include <termios.h>
#include <unistd.h>
#include <math.h>

// int main(int argc, char** argv) {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<rclcpp::Node>("teleop");

//     char teleop_command_;
//     int i = 0;

//     initscr();              // 初始化 ncurses
//     cbreak();               // 立即讀取鍵盤輸入
//     noecho();               // 禁止輸入回顯
//     nodelay(stdscr, TRUE);  // 設為非阻塞模式
//     keypad(stdscr, TRUE);   // 允許讀取特殊鍵 (如方向鍵)

//     std::cout << "Press two keys simultaneously (press 'q' to quit):\n";

//     while (true) {
//         // clear();
//         teleop_command_ = getch();  // 讀取第一個鍵
//         if (teleop_command_ != ERR) {   // 如果有輸入
//             mvprintw(i++, 0, "You pressed: %c\n", (char)teleop_command_);
//             if (i > 5) i = 0;
//             if (teleop_command_ == 'q') break; // 按 'q' 退出
//             switch ((char)teleop_command_) {
//             case 'w':
//                 break;
//             case 'a':
//                 break;
//             case 's':
//                 break;
//             case 'd':
//                 break;
//             default:
//                 break;
//             }
//         }
//         usleep(10000); // 避免過度消耗 CPU
//     }

//     endwin(); // 關閉 ncurses
//     return 0;
// }

class RivalSimPub : public rclcpp::Node {
    public:
        RivalSimPub() : Node("rival_sim_pub") {
            // Get the rival mode -> 0: Halted, 1: Wandering, 2: Moving, 3: Moving with noise
            declare_parameter("Rival_mode", rclcpp::ParameterValue(0));
            this->get_parameter("Rival_mode", rival_mode_);

            initscr();              // 初始化 ncurses
            cbreak();               // 立即讀取鍵盤輸入
            noecho();               // 禁止輸入回顯
            nodelay(stdscr, TRUE);  // 設為非阻塞模式
            keypad(stdscr, FALSE);   // 允許讀取特殊鍵 (如方向鍵)

            message = geometry_msgs::msg::PoseWithCovarianceStamped();
            rival_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/rival_pose", 100);
            rival_timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&RivalSimPub::timer_callback, this));
        }   

    private:
        void timer_callback() {
            // header
            message.header.stamp = this->now();
            message.header.frame_id = "map";

            trigger_once_cnt_--;
            if(trigger_once_cnt_ > 0) {
                // position 
                message.pose.pose.position.x = 1.5;
                message.pose.pose.position.y = 1.0;

                rival_pub_->publish(message);
                return;
            } 
            // position
            // Hardcoded moving rival
            if (is_paused_) {
                cooldown_++;
                
                // message.pose.pose.position.x = pause_pose_.pose.pose.position.x;
                // message.pose.pose.position.y = pause_pose_.pose.pose.position.y;
                message.pose.pose.position.x = pause_position_.x;
                message.pose.pose.position.y = pause_position_.y;

                if (cooldown_ > 50) {
                    is_paused_ = false;
                    cooldown_ = 0;
                }
            } else {
                if (rival_mode_ == 3) {
                    move_x_ += (0.01+float(rand()%5-2)/400.0) * toggle_x_;
                    move_y_ += (0.01+float(rand()%5-2)/400.0) * toggle_y_;
                }

                if (rival_mode_ == 2) {
                    move_x_ += 0.01 * toggle_x_;
                    move_y_ += 0.01 * toggle_y_;
                }
                collider(rival_mode_);
                message.pose.pose.position.x = move_x_;
                message.pose.pose.position.y = move_y_;
            }
            if (rival_mode_ == 4) {
                teleop_command_ = getch();  // 讀取第一個鍵
                // mvprintw(i++, 0, "You pressed: %c\n", (char)teleop_command_);
                if (i > 5) i = 0;
                // if (teleop_command_ == 'q') break; // 按 'q' 退出
                switch ((char)teleop_command_) {
                case 'w':
                    move_x_ += 0;
                    move_y_ += 0.01 * sqrt(2);
                    mvprintw(i++, 0, "You pressed: %c\n", (char)teleop_command_);
                    break;
                case 'a':
                    move_x_ -= 0.01 * sqrt(2);
                    move_y_ += 0;
                    mvprintw(i++, 0, "You pressed: %c\n", (char)teleop_command_);
                    break;
                case 's':
                    move_x_ += 0;
                    move_y_ -= 0.01 * sqrt(2);
                    mvprintw(i++, 0, "You pressed: %c\n", (char)teleop_command_);
                    break;
                case 'd':
                    move_x_ += 0.01 * sqrt(2);
                    move_y_ += 0;
                    mvprintw(i++, 0, "You pressed: %c\n", (char)teleop_command_);
                    break;
                case 'q':
                    move_x_ -= 0.01;
                    move_y_ += 0.01;
                    mvprintw(i++, 0, "You pressed: %c\n", (char)teleop_command_);
                    break;
                case 'e':
                    move_x_ += 0.01;
                    move_y_ += 0.01;
                    mvprintw(i++, 0, "You pressed: %c\n", (char)teleop_command_);
                    break;
                case 'z':
                    move_x_ -= 0.01;
                    move_y_ -= 0.01;
                    mvprintw(i++, 0, "You pressed: %c\n", (char)teleop_command_);
                    break;
                case 'c':
                    move_x_ += 0.01;
                    move_y_ -= 0.01;
                    mvprintw(i++, 0, "You pressed: %c\n", (char)teleop_command_);
                    break;
                default:
                    mvprintw(i++, 0, "Empty Input\n");
                    break;
                }
                if (collider(rival_mode_)) {
                    message.pose.pose.position.x = move_x_;
                    message.pose.pose.position.y = move_y_;
                }
            }
            // Hardcoded wandering rival
            if(rival_mode_ == 1) {
                message.pose.pose.position.x = float(rand()%51-25)/200.0 + 1.5;
                message.pose.pose.position.y = float(rand()%51-25)/200.0 + 1.0;
            }

            // Hardcoded halted rival
            if(rival_mode_ == 0) {
                message.pose.pose.position.x = float(rand()%5-2)/200.0 + 1.5;
                message.pose.pose.position.y = float(rand()%5-2)/200.0 + 1.0;
            }
            // RCLCPP_INFO(this->get_logger(), "Publishing: x=%f, y=%f\n", message.pose.pose.position.x, message.pose.pose.position.y);
            rival_pub_->publish(message);
        
        }

        bool collider(int rival_mode_) {
            if (move_x_ > 2.5 || move_x_ < 0.5) {
                if (rival_mode_ == 2 || rival_mode_ == 3) {
                    toggle_x_ *= -1;
                    is_paused_ = true;
                    pause_position_.x = move_x_;
                    pause_position_.y = move_y_;
                    return true;
                } else if (rival_mode_ == 4)
                    return false;
                else
                    return true;
            }
            if (move_y_ > 1.5 || move_y_ < 0.5) {
                if (rival_mode_ == 2 || rival_mode_ == 3) {
                    toggle_y_ *= -1;
                    is_paused_ = true;
                    pause_position_.x = move_x_;
                    pause_position_.y = move_y_;
                    return true;
                } else if (rival_mode_ == 4)
                    return false;
                else
                    return true;
            }
            return true;
        }

        rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr rival_pub_;
        rclcpp::TimerBase::SharedPtr rival_timer_;
        int rival_mode_ = 0;
        double move_x_ = 1.5;
        double move_y_ = 1.0;
        int toggle_x_ = 1;
        int toggle_y_ = 1;
        bool is_paused_ = false;
        int cooldown_ = 0;
        int trigger_once_cnt_ = 50;
        char teleop_command_;
        int i = 0;
        // geometry_msgs::msg::PoseWithCovarianceStamped pause_pose_;
        geometry_msgs::msg::Point pause_position_;
        geometry_msgs::msg::PoseWithCovarianceStamped message;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RivalSimPub>());
    rclcpp::shutdown();
    endwin();
    return 0;
}
