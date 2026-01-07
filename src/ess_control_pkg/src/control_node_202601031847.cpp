#include <chrono>
#include <cmath>
#include <memory>
#include <vector>
#include <string>
#include <atomic>
#include <thread>
#include <deque>

// ROS2 Headers
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include <gpiod.h>
#include <poll.h>

using namespace std::chrono_literals;

// Lift Control Flags
#define STOP 0
#define TOFLOOR1 (1 << 0)
#define TOFLOOR2 (1 << 1)
#define TOFLOOR3 (1 << 2)
#define GOUP     (1 << 3)
#define GODOWN   (1 << 4)

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define START_TH M_PI/180.0 * 3.0  // 3 degrees in radians

struct point_t {
    double x;
    double y;
};

// Patrol Points
struct point_t points[] = {
    {0.51, 0.20},   
    {1.04, 0.19}, 
    {1.58, 0.18}  
};

// Zone Boundaries for Detection
struct point_t zonePoints[] = {
    {0.6, 0.5},   // Index 0 -> Zone 1
    {1.4, 0.5},   // Index 1 -> Zone 2
    {2.3, 0.5},   // Index 2 -> Zone 3
    {0.6, -0.3},  // Index 3 -> Zone 4
    {1.4, -0.3},  // Index 4 -> Zone 5
    {2.3, -0.3}   // Index 5 -> Zone 6
};

#define PUBLISH_INTERVAL_RATIO 4
#define USE_ARUCO_ALIGNMENT    1

class ControlNode : public rclcpp::Node {
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    ControlNode() 
    : Node("control_node"), 
      tf_buffer_(this->get_clock()), 
      tf_listener_(tf_buffer_) 
    {
        // 1. ROS Subscribers
        sub_home_ = create_subscription<std_msgs::msg::Bool>(
            "/ess/home", 10, std::bind(&ControlNode::on_home, this, std::placeholders::_1));
        sub_thermal = create_subscription<std_msgs::msg::Int32>(
            "/ess/thermal/ack", 10, std::bind(&ControlNode::thermal_callback, this, std::placeholders::_1));
        sub_priority_zone_ = create_subscription<std_msgs::msg::Int32>(
            "/ess/priority_zone", 10, std::bind(&ControlNode::priority_zone_callback, this, std::placeholders::_1));
        sub_aruco = create_subscription<std_msgs::msg::Int32>(
            "/ess/aruco/ack", 10, std::bind(&ControlNode::aruco_callback, this, std::placeholders::_1));
        
        // 2. ROS Publishers
        pub_cmd_vel_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        pub_capture = create_publisher<std_msgs::msg::Int32>("/ess/request/id", 10);
        pub_pose_mqtt_ = create_publisher<geometry_msgs::msg::Pose>("/ess/robot_pose", 10);
        pub_aruco_ = create_publisher<std_msgs::msg::Int32>("/ess/aruco/request", 10);
        pub_zone_status_ = create_publisher<std_msgs::msg::Int32>("/ess/zone_status", 10);

        // 3. Action Client
        nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

        // 4. GPIO Setup
        setup_gpio();
        
        // Initial Lift Reset
        lift_inflight_ = true;
        std::thread(&ControlNode::move_to_floor_blocking, this, TOFLOOR1 | GODOWN).detach();

        // 5. Timer (Main Loop 50ms)
        timer_ = create_wall_timer(50ms, std::bind(&ControlNode::tick, this));
        
        // Init Variables
        current_row_bit = TOFLOOR1;
        target_floor_bit_ = TOFLOOR1; 
        thermal_wait_count_ = 0;

        RCLCPP_INFO(get_logger(), "Integrated Control Node Started. Waiting for /ess/home signal...");
    }

    ~ControlNode() {
        cleanup_gpio();
    }

private:
    enum class State { IDLE, NAV_HOME, NAV_FIRST_POS, NAV_SECOND_POS, DRIVE_STRAIGHT, START_LIFT, WAIT_LIFT, WAIT_THERMAL, EMERGENCY ,NAV_BACK_HOME,NAV_FINAL_APPROACH,  WAIT_ARUCO};
    
    std::string strState(State s) {
        switch(s) {
            case State::IDLE: return "IDLE";
            case State::NAV_HOME: return "NAV_HOME";
            case State::NAV_FIRST_POS: return "NAV_FIRST_POS";
            case State::NAV_SECOND_POS: return "NAV_SECOND_POS";
            case State::DRIVE_STRAIGHT: return "DRIVE_STRAIGHT";
            case State::START_LIFT: return "START_LIFT";
            case State::WAIT_LIFT: return "WAIT_LIFT";
            case State::WAIT_THERMAL: return "WAIT_THERMAL";
            case State::EMERGENCY: return "EMERGENCY";
            case State::NAV_BACK_HOME: return "NAV_BACK_HOME";
            case State::NAV_FINAL_APPROACH: return "NAV_FINAL_APPROACH";
            case State::WAIT_ARUCO: return "WAIT_ARUCO";
            default: return "UNKNOWN";
        }
    }

    State state_{State::IDLE};
    State preState_{State::IDLE};
    const int ONE_MINUTE_TICKS = (60 * 1000) / 50; 
    int idle_counter_ = 0;
    
    // --- Callbacks ---
    void on_home(const std_msgs::msg::Bool::SharedPtr msg) {
         if (msg->data) 
            home_requested_ = true; 
        RCLCPP_INFO(get_logger(), "on_home()");
    }
    
    void thermal_callback(const std_msgs::msg::Int32::SharedPtr msg) { 
        if (msg->data == 1) 
            thermal_done_ = true; 
        RCLCPP_INFO(get_logger(), "thermal_callback()");
    }

    void priority_zone_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        int zone = msg->data;
        int input_col = (zone - 1) % 3;

        if (zone >= 1 && zone <= 6) {
            // Prevent duplicates
            if (is_emergency_active_ && current_col_ == input_col) return;
            for (int q_col : emergency_queue_) {
                if (q_col == input_col) return;
            }

            emergency_queue_.push_back(input_col);
            RCLCPP_WARN(get_logger(), "!!! Emergency Added to Queue: Zone %d !!!", zone);

            // Interrupt if not already handling emergency
            if (!is_emergency_active_) {
                
                // If checking Aruco, stop it
                if(!aruco_done_ && state_ == State::WAIT_ARUCO) {
                    aruco_done_ = true;
                    stop_cmd_vel();
                    std_msgs::msg::Int32 m; m.data = 0;
                    pub_aruco_->publish(m);
                    RCLCPP_WARN(get_logger(), "Aruco Interrupted by Emergency!");
                }

                if (saved_patrol_col_ == -1) {
                    saved_patrol_col_ = current_col_;
                }

                RCLCPP_WARN(get_logger(), "!!! INTERRUPTING -> SWITCHING TO EMERGENCY !!!");

                nav_client_->async_cancel_all_goals();
                stop_cmd_vel(); 
                
                // Reset flags so we don't trigger "Fail" logic immediately
                nav_done_ = false; 
                nav_ok_ = true; 

                is_emergency_active_ = true;
                start_next_emergency_task();
            }
        } else {
            RCLCPP_WARN(get_logger(), "Received invalid priority zone: %d", zone);
        }
    }

    void aruco_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        int aruco_id = msg->data;
        RCLCPP_INFO(get_logger(), "Aruco ACK received: %d", aruco_id);
        aruco_done_ = true;
    }

    void start_next_emergency_task() {
    
        if (emergency_queue_.empty()) {
            is_emergency_active_ = false; 
            return;
        }

        int next_col = emergency_queue_.front();
        emergency_queue_.pop_front();

        current_col_ = next_col;
        target_floor_bit_ = TOFLOOR1; 
        
        thermal_done_ = true;   
        home_requested_ = false;
        
        // [중요] 비상 시작 시 네비게이션 플래그 초기화
        emergency_goal_sent_ = false;
        nav_done_ = false;
        nav_ok_ = true; // 실패 아님으로 초기화
        
        is_emergency_active_ = true; 

        RCLCPP_WARN(get_logger(), ">>> Starting Emergency Task for Zone %d <<<", current_col_ + 1);

        // [중요] 리프트가 이미 이동 중이어도 1층으로 가야 하므로 체크 강화
        if (!lift_inflight_) {
            lift_inflight_ = true;
            std::thread(&ControlNode::move_to_floor_blocking, this, TOFLOOR1 | GODOWN).detach();
        } else {
            // 이미 움직이고 있다면 다음 로직에서 1층 확인하도록 유도
            RCLCPP_INFO(get_logger(), "Lift inflight. Target set to Floor 1.");
        }

        state_ = State::EMERGENCY;
    }

    // --- Main Loop ---
    void tick() {
        if(state_ != preState_) {
            RCLCPP_INFO(get_logger(), "State changed: %s -> %s", strState(preState_).c_str(), strState(state_).c_str());
            preState_ = state_;
        }

        publish_current_pose();

        switch (state_) {
            case State::IDLE:
                // [비상 체크] IDLE 상태에서도 비상 큐 확인
                if (!emergency_queue_.empty()) {
                    RCLCPP_WARN(get_logger(), "Emergency detected during IDLE!");
                    idle_counter_ = 0; 
                    
                    if (!lift_inflight_) {
                        lift_inflight_ = true;
                        std::thread(&ControlNode::move_to_floor_blocking, this, TOFLOOR1 | GODOWN).detach();
                    }
                    
                    start_next_emergency_task(); 
                    break; 
                }

                if (++idle_counter_ >= ONE_MINUTE_TICKS) {
                    RCLCPP_INFO(get_logger(), "1 minute wait finished. Restarting Patrol...");
                    idle_counter_ = 0;
                    
                    if (!lift_inflight_) {
                        lift_inflight_ = true;
                        std::thread(&ControlNode::move_to_floor_blocking, this, TOFLOOR1 | GODOWN).detach();
                    }

                    nav_done_ = true; 
                    nav_ok_ = true;
                    current_col_ = 0;
                    target_floor_bit_ = TOFLOOR1;

                    state_ = State::NAV_FIRST_POS; 
                }
                
                if (home_requested_) {
                    RCLCPP_INFO(get_logger(), "Manual home signal received.");
                    idle_counter_ = 0;
                    
                    if (!lift_inflight_) {
                        lift_inflight_ = true;
                        std::thread(&ControlNode::move_to_floor_blocking, this, TOFLOOR1 | GODOWN).detach();
                    }
                    
                    state_ = State::NAV_HOME;
                }
                break;

            case State::NAV_HOME:
                if (home_requested_.exchange(false)) {
                    RCLCPP_INFO(get_logger(), "Moving to Home(0,0)");
                    send_nav_goal(0.0, 0.0, START_TH);
                    state_ = State::NAV_FIRST_POS;
                }
                break;

            case State::NAV_FIRST_POS:
                if (nav_done_) {
                    if (!nav_ok_) { fail_and_stop("First Pos Failed"); return; }
                    RCLCPP_INFO(get_logger(), "Arrived Home... Going to First Point");
                    send_nav_goal(points[0].x, 0.2, M_PI/2.0 + START_TH);
                    state_ = State::WAIT_LIFT;
                }
                break;
                
            case State::NAV_SECOND_POS:
                if (nav_done_) {
                    if (!nav_ok_) { fail_and_stop("Second Pos Failed"); return; }
                    send_nav_goal(1.0, 1.113, M_PI + START_TH);
                    state_ = State::NAV_BACK_HOME;
                }
                break;

            case State::DRIVE_STRAIGHT:
                if (lift_inflight_) {
                    stop_cmd_vel(); 
                    break;
                }
                if (nav_done_) {
                    send_nav_goal(points[current_col_].x, points[current_col_].y, M_PI/2.0 + START_TH);
                    state_ = State::WAIT_LIFT; 
                }
                break;

            case State::START_LIFT:
                if (!lift_inflight_) {
                    lift_inflight_ = true;
                    RCLCPP_INFO(get_logger(), "Moving Lift: target %d", target_floor_bit_);
                    std::thread(&ControlNode::move_to_floor_blocking, this, target_floor_bit_ | GOUP).detach();
                    state_ = State::WAIT_LIFT;
                }
                break;

            case State::WAIT_LIFT:
                if (!lift_inflight_ && nav_done_) {
                    RCLCPP_INFO(get_logger(), "Condition met. Starting Thermal Capture...");
                    
                    int floor_num = 1;
                    if (target_floor_bit_ & TOFLOOR2) floor_num = 2;
                    else if (target_floor_bit_ & TOFLOOR3) floor_num = 3;

                    thermal_done_ = false; 
                    send_capture_cmd(current_col_ * 3 + floor_num);
                    
                    RCLCPP_INFO(get_logger(), "Request ID: %d (Zone %d, Floor %d)", 
                                current_col_ * 3 + floor_num, current_col_ + 1, floor_num);

                    state_ = State::WAIT_THERMAL;
                }
                break;

            case State::WAIT_THERMAL:
            {
                if (thermal_done_) {
                    if (++thermal_wait_count_ < 2) break;
                    thermal_wait_count_ = 0;

                    // 3층 작업 완료 (한 구역 끝)
                    if (target_floor_bit_ == TOFLOOR3) {
                        
                        RCLCPP_INFO(get_logger(), "[DEBUG] Zone %d Finished. CurrentCol: %d", current_col_ + 1, current_col_);

                        // [비상 상황 처리 후 로직]
                        if (is_emergency_active_) {
                            // ... (기존 비상 처리 로직) ...
                             if (!emergency_queue_.empty()) {
                                start_next_emergency_task(); 
                                break; 
                            }
                            else {
                                RCLCPP_WARN(get_logger(), "Emergency Done. Resuming Patrol Sequence...");
                                is_emergency_active_ = false;
                                saved_patrol_col_ = -1; 
                            }
                        }

                        // [다음 구역 이동 로직]
                        // 0(Zone1) -> 1(Zone2) -> 2(Zone3)
                        
                        // 현재가 2(Zone 3)였다면, 이제 다 끝난 거니까 집(경유지)으로
                        if (current_col_ >= 2) {
                            RCLCPP_INFO(get_logger(), "All Zones complete (Index %d). Moving to Waypoint.", current_col_);
                            send_nav_goal(2.0, 1.2, M_PI);
                            
                            lift_inflight_ = true;
                            std::thread(&ControlNode::move_to_floor_blocking, this, TOFLOOR1 | GODOWN).detach();
                            state_ = State::NAV_SECOND_POS;
                        } 
                        // 아직 0(Zone1)이나 1(Zone2)이라면 -> 다음 구역으로
                        else {
                            current_col_++; // 여기서 증가시킴! (0->1, 1->2)
                            target_floor_bit_ = TOFLOOR1; 
                            
                            lift_inflight_ = true;
                            std::thread(&ControlNode::move_to_floor_blocking, this, TOFLOOR1 | GODOWN).detach();
                            
                            RCLCPP_INFO(get_logger(), "Moving to Next Zone %d (Index %d)", current_col_ + 1, current_col_);
                            send_nav_goal(points[current_col_].x, points[current_col_].y, M_PI/2.0 + START_TH);
                            
                            state_ = State::DRIVE_STRAIGHT;
                        }
                    } else {
                        // 층 이동
                        target_floor_bit_ <<= 1;
                        state_ = State::START_LIFT;
                    }
                    thermal_done_ = false; 
                }
                break;
            }

            case State::NAV_BACK_HOME:
                if (nav_done_) {
                    RCLCPP_INFO(get_logger(), "All Sequence Complete. Going Home.");
                    send_nav_goal(0.0, 0.0, START_TH);
                    state_ = State::NAV_FINAL_APPROACH;
                }
                break;

            case State::NAV_FINAL_APPROACH:
                if (nav_done_) {
                    if (!nav_ok_) { fail_and_stop("Final Docking Failed"); return; }
                    RCLCPP_INFO(get_logger(), "Arrived Home. Requesting Aruco Alignment...");

#if USE_ARUCO_ALIGNMENT
                    aruco_done_ = false;
                    stop_cmd_vel();
                    std_msgs::msg::Int32 msg;
                    msg.data = 1;
                    pub_aruco_->publish(msg);
                    aruco_wait_count_ = 0;
                    state_ = State::WAIT_ARUCO;
#else
                    finish_patrol_and_idle();
#endif
                }
                break;

            case State::EMERGENCY:
                if (lift_inflight_) break; 

                if (!emergency_goal_sent_) {
                    RCLCPP_INFO(get_logger(), "Emergency: Sending Nav Zone_%d", current_col_ % 3 + 1);
                    send_nav_goal(points[current_col_].x, points[current_col_].y, START_TH + M_PI / 2.0);
                    emergency_goal_sent_ = true;
                    nav_done_ = false;
                    break;
                }

                if (nav_done_) {
                    emergency_goal_sent_ = false; 
                    if (nav_ok_) {
                        state_ = State::WAIT_LIFT;
                    } else {
                        // [중요] 네비게이션 취소 등으로 인한 일시적 실패일 수 있으므로
                        // 바로 죽지 않고 로그만 띄우거나, 재시도 로직을 넣을 수 있음.
                        // 여기서는 일단 fail 처리하되, 취소(Cancel) 로직과 겹치지 않게 주의.
                        fail_and_stop("Emergency Navigation Failed");
                    }
                }

            break;

            case State::WAIT_ARUCO:
                if (aruco_done_) {
                    RCLCPP_INFO(get_logger(), "Aruco Alignment Success. Patrol Complete.");
                    finish_patrol_and_idle(); 
                }
                else if (++aruco_wait_count_ >= 200) {
                    aruco_done_ = true; 
                    stop_cmd_vel();
                    std_msgs::msg::Int32 msg;
                    msg.data = 0;
                    pub_aruco_->publish(msg);
                    RCLCPP_WARN(get_logger(), "Aruco Timeout (10s)! Forcing IDLE state.");
                    finish_patrol_and_idle(); 
                }
            break;
        }
    }

    void publish_current_pose() {
        double x, y;
        if (get_robot_xy(x, y)) {
            geometry_msgs::msg::Pose pose_msg;
            pose_msg.position.x = x;
            pose_msg.position.y = y;
            // pub_pose_mqtt_->publish(pose_msg);

            if (x < zonePoints[0].x && y > zonePoints[0].y) zone_Num = 1; 
            else if (x < zonePoints[1].x && x >= zonePoints[0].x && y > zonePoints[1].y) zone_Num = 2; 
            else if (x < zonePoints[2].x && x >= zonePoints[1].x && y > zonePoints[2].y) zone_Num = 3; 
            else if (x < zonePoints[0].x && y <= zonePoints[0].y) zone_Num = 4; 
            else if (x < zonePoints[1].x && x >= zonePoints[0].x && y <= zonePoints[1].y) zone_Num = 5; 
            else if (x < zonePoints[2].x && x >= zonePoints[1].x && y <= zonePoints[2].y) zone_Num = 6; 
            
            std_msgs::msg::Int32 zone_msg;
            zone_msg.data = zone_Num;
            pub_zone_status_->publish(zone_msg); 
        }
    }

    void finish_patrol_and_idle() {
        current_col_ = -1; 
        target_floor_bit_ = TOFLOOR1;
        home_requested_ = false;
        idle_counter_ = 0; 
        
        RCLCPP_INFO(get_logger(), "Resting for 1 min.");
        state_ = State::IDLE;
    }

    // --- GPIO ---
    void setup_gpio() {
        chip_ = gpiod_chip_open_by_name("gpiochip0");
        line_ena_ = gpiod_chip_get_line(chip_, 13);
        line_in1_ = gpiod_chip_get_line(chip_, 5);
        line_in2_ = gpiod_chip_get_line(chip_, 6);
        gpiod_line_request_output(line_ena_, "ena", 0);
        gpiod_line_request_output(line_in1_, "in1", 0);
        gpiod_line_request_output(line_in2_, "in2", 0);

        std::vector<int> pins = {17, 27, 22};
        for (int p : pins) {
            gpiod_line* l = gpiod_chip_get_line(chip_, p);
            gpiod_line_request_both_edges_events(l, "sensor");
            sensor_lines_.push_back(l);
            pollfd pfd{}; pfd.fd = gpiod_line_event_get_fd(l); pfd.events = POLLIN;
            pfds_.push_back(pfd);
        }
    }

    void move_to_floor_blocking(int cmd) {
        int target_bit = cmd & (TOFLOOR1 | TOFLOOR2 | TOFLOOR3);
        
        if ((check_current_floor() & target_bit) && (check_current_floor() & target_bit)) {
            RCLCPP_INFO(get_logger(), "Lift already at target floor. Skip moving.");
            apply_motor_state(STOP);
            lift_inflight_ = false;
            return;
        }

        apply_motor_state(cmd);
        
        auto start_time = std::chrono::steady_clock::now();
        const double TIMEOUT_SEC = 10.0; 

        while (rclcpp::ok()) {
            if (::poll(pfds_.data(), pfds_.size(), 50) > 0) {
                for (size_t i = 0; i < pfds_.size(); ++i) {
                    if (pfds_[i].revents & POLLIN) {
                        gpiod_line_event ev; 
                        gpiod_line_event_read(sensor_lines_[i], &ev);
                    }
                }
            }
            
            int current_f = check_current_floor();

            if (current_f & target_bit) break;

            if ((cmd & GODOWN) && (current_f & TOFLOOR1)) {
                RCLCPP_WARN(get_logger(), "Safety Stop: Hit Floor 1 while going down!");
                break;
            }
            if ((cmd & GOUP) && (current_f & TOFLOOR3)) {
                RCLCPP_WARN(get_logger(), "Safety Stop: Hit Floor 3 while going up!");
                break;
            }
            auto current_time = std::chrono::steady_clock::now();
            if ((current_time - start_time).count() / 1e9 > TIMEOUT_SEC) {
                RCLCPP_ERROR(get_logger(), "LIFT TIMEOUT! Safety Stop.");
                break;
            }
            
            std::this_thread::sleep_for(10ms);
        }

        apply_motor_state(STOP);
        lift_inflight_ = false;
    }

    int check_current_floor() {
        for (size_t i = 0; i < sensor_lines_.size(); ++i) {
            if (gpiod_line_get_value(sensor_lines_[i]) == 0) return (1 << i);
        }
        return 0;
    }

    void apply_motor_state(int cmd) {
        int in1 = (cmd & GOUP) ? 1 : 0;
        int in2 = (cmd & GODOWN) ? 1 : 0;
        int ena = (in1 || in2) ? 1 : 0;

        gpiod_line_set_value(line_in1_, in1);
        gpiod_line_set_value(line_in2_, in2);
        gpiod_line_set_value(line_ena_, ena);
    }

    // --- Navigation ---
    void send_nav_goal(double x, double y, double yaw) {
        if (!nav_client_->wait_for_action_server(1s)) {
            fail_and_stop("Nav2 Server Not Found");
            return;
        }
        auto goal = NavigateToPose::Goal();
        goal.pose.header.frame_id = "map";
        goal.pose.header.stamp = now();

        goal.pose.pose.position.x = x; 
        goal.pose.pose.position.y = y;
        goal.pose.pose.position.z = 0.0;

        tf2::Quaternion q; 
        q.setRPY(0, 0, yaw);
        goal.pose.pose.orientation = tf2::toMsg(q);

        nav_done_ = false;
        auto options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        options.result_callback = [this](const GoalHandleNav::WrappedResult &res) {
            nav_ok_ = (res.code == rclcpp_action::ResultCode::SUCCEEDED);
            nav_done_ = true;
        };
        nav_client_->async_send_goal(goal, options);
    }

    bool get_robot_xy(double &x, double &y) {
        try {
            auto t = tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);
            x = t.transform.translation.x; y = t.transform.translation.y;
            return true;
        } catch (...) { return false; }
    }
    
    void stop_cmd_vel() { pub_cmd_vel_->publish(geometry_msgs::msg::Twist()); }
    void send_capture_cmd(int id) { std_msgs::msg::Int32 m; m.data = id; pub_capture->publish(m); }
    
    void fail_and_stop(std::string m) { RCLCPP_ERROR(get_logger(), "%s", m.c_str()); state_ = State::IDLE; }
    void cleanup_gpio() { apply_motor_state(STOP); for(auto* l : sensor_lines_) gpiod_line_release(l); gpiod_chip_close(chip_); }

    // Members
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_home_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_thermal;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_priority_zone_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_aruco; 

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_capture;
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pub_pose_mqtt_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_aruco_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_zone_status_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    gpiod_chip* chip_;
    gpiod_line *line_ena_, *line_in1_, *line_in2_;
    std::vector<gpiod_line*> sensor_lines_;
    std::vector<pollfd> pfds_;

    std::atomic<bool> home_requested_{false}, nav_done_{false}, nav_ok_{false}, lift_inflight_{false}, thermal_done_{true};
    bool emergency_goal_sent_ = false;

    uint8_t target_floor_bit_; 
    int current_col_; 
    uint8_t current_row_bit; 
    int thermal_wait_count_;
    
    std::deque<int> emergency_queue_; 
    bool is_emergency_active_ = false; 

    std::atomic<bool> aruco_done_{false}; 
    int aruco_wait_count_ = 0; 

    int saved_patrol_col_ = -1; 
    int zone_Num = 0;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControlNode>());
    rclcpp::shutdown();
    return 0;
}