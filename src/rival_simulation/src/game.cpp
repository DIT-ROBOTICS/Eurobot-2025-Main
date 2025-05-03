#include "chrono"
// ROS
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
// keybow input
#include <iostream>
#include <ncurses.h>
#include <termios.h>
#include <unistd.h>
#include <math.h>
#include <string.h>

using namespace std;

class Player : public rclcpp::Node {
public:
    Player(const string& name): Node("player_node"), name_(name)
    {
        velocity = 0.3;
        delta_time = 0.01;
        x = 0;
        y = 0;
        publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("game/player", 10);
    }
    int Moving(double speed)
    {
        x += speed;
        return 0;
    }
    int Jump(double gravity)
    {
        y += velocity * delta_time - gravity * delta_time * delta_time;
        return 0;
    }
    int Display()
    {
        rclcpp::Time now = this->now();
        player_pose.header.stamp = now;
        player_pose.header.frame_id = "map";
        player_pose.pose.position.y = y;
        publisher_->publish(player_pose);
        return 0;
    }
private:
    string name_;
    double force;
    double x, y, z;
    double delta_time, velocity;
    geometry_msgs::msg::PoseStamped player_pose;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
};

class PlayGround : public rclcpp::Node {
public:
    PlayGround(const string& name): Node("playground_node"), name_(name)
    {
        gravity = 9.8;
        gameSpeed = 0.01;
        CreateMap();
    }
    int CreateMap()
    {
        high = 10;
        low = 10;
        return 0;
    }
    int FSM()
    {
        Player player1(name_);
        while (true) {
            input = getch();  // 讀取第一個鍵
            if (input != ERR) {   // 如果有輸入
                if (input == 'q') break; // 按 'q' 退出
                switch ((char)input) {
                case ' ':
                    player1.Jump(gravity);
                    break;
                default:
                    break;
                }
            }
            player1.Moving(gameSpeed);
            player1.Display();
            usleep(10000);
        }
        return 0;
    }
private:
    string name_;
    char input = '0';
    double gravity;
    double gameSpeed;
    double high, low;
};

// char getch() {
//     struct termios oldt, newt;
//     char ch;

//     tcgetattr(STDIN_FILENO, &oldt); // get current terminal settings
//     newt = oldt;
//     newt.c_lflag &= ~(ICANON | ECHO); // disable buffering and echo
//     tcsetattr(STDIN_FILENO, TCSANOW, &newt); // apply new settings

//     ch = getchar(); // get single character

//     tcsetattr(STDIN_FILENO, TCSANOW, &oldt); // restore original settings
//     return ch;
// }

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    char input = '0';
    bool startGame = false;
    string name;
    rclcpp::Rate rate(100);

    // initscr();              // 初始化 ncurses
    // cbreak();               // 立即讀取鍵盤輸入
    // noecho();               // 禁止輸入回顯
    // nodelay(stdscr, TRUE);  // 設為非阻塞模式
    // keypad(stdscr, FALSE);   // 允許讀取特殊鍵 (如方向鍵)

    while (true) {
        input = getch();  // 讀取第一個鍵
        if (input != ERR) {   // 如果有輸入
            // mvprintw(i++, 0, "You pressed: %c\n", (char)input);
            // if (i > 5) i = 0;
            if (input == 27) break; // 按 'esc' 退出
            switch ((char)input) {
            case 'e':
                if (!startGame) {
                    cout << "start game";
                    cout << "input name: ";
                    cin >> name;
                    startGame = true;
                    // Player player1(name);
                    PlayGround playGround(name);
                    playGround.FSM();
                }
                break;
            default:
                break;
            }
        }
        usleep(10000); // 避免過度消耗 CPU
    }

    endwin(); // 關閉 ncurses
    return 0;
}