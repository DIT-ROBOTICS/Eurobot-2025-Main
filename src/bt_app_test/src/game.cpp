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
#include <stdio.h>

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

class Controller : public rclcpp::Node {
    public:
        Controller() : Node("rival_sim_pub") {
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
            rival_timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&Controller::timer_callback, this));
        }   

    private:
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
    rclcpp::spin(std::make_shared<Controller>());
    rclcpp::shutdown();
    endwin();
    return 0;
}

void draw_square(int x, int y, int size);
void clear_square(int x, int y, int size);

int main() {
    int x = 5, y = 5;  // 正方形左上角的起始位置 (行, 列)
    int size = 5;      // 正方形大小

    draw_square(x, y, size);  // 画正方形
    sleep(2);                 // 等待 2 秒
    clear_square(x, y, size); // 删除正方形

    return 0;
}

// 画一个正方形
void draw_square(int x, int y, int size) {
    for (int i = 0; i < size; i++) {
        printf("\033[%d;%dH", x + i, y); // 移动光标
        for (int j = 0; j < size; j++) {
            printf("█"); // 输出填充方块
        }
        printf("\n");
    }
    fflush(stdout);
}

// 删除正方形（用空格覆盖）
void clear_square(int x, int y, int size) {
    for (int i = 0; i < size; i++) {
        printf("\033[%d;%dH", x + i, y); // 移动光标
        for (int j = 0; j < size; j++) {
            printf(" "); // 用空格覆盖
        }
        printf("\n");
    }
    fflush(stdout);
}
