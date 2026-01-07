#include <chrono>
#include <cmath>
#include <memory>
#include <vector>
#include <string>
#include <atomic>
#include <thread>
#include <deque>

// 필요한 타입 발행하는 publisher 추가들
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

// 리프터 제어 비트 플래그
#define STOP 0
#define TOFLOOR1 (1 << 0)
#define TOFLOOR2 (1 << 1)
#define TOFLOOR3 (1 << 2)
#define GOUP     (1 << 3)
#define GODOWN   (1 << 4)

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define START_TH M_PI/180.0 * 3.0  // 15도

typedef struct point_t{
    double x;
    double y;
}point;

// struct point_t points[] = {
//     {0.48, 0.2},   // 첫 번째 위치
//     {1.06, 0.2},   // 두 번째 위치
//     {1.67, 0.2}    // 최종 위ㄷ
// };

//  변환 해줘야함 ㄷㄷ
struct point_t points[] = {
    {0.51, 0.24},   
    {1.04, 0.19}, 
    {1.58, 0.18}  
};


struct point_t zonePoints[] = {
    {0.6, 0.5},   // Index 0 -> Zone 1
    {1.4, 0.5},   // Index 1 -> Zone 2
    {2.3, 0.5},   // Index 2 -> Zone 3
    {0.6, -0.3},  // Index 3 -> Zone 4
    {1.4, -0.3},  // Index 4 -> Zone 5
    {2.3, -0.3}   // Index 5 -> Zone 6
};





#define PUBLISH_INTERVAL_RATIO 4  // 50ms 타이머 중 4번째마다 좌표 발행
#define USE_ARUCO_ALIGNMENT    1// 아루코 정렬 사용 여부


class ControlNode : public rclcpp::Node {
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    ControlNode() 
    : Node("control_node"), 
      tf_buffer_(this->get_clock()), 
      tf_listener_(tf_buffer_) 
    {


        // 1. ROS 통신 설정
        sub_home_ = create_subscription<std_msgs::msg::Bool>(
            "/ess/home", 10, std::bind(&ControlNode::on_home, this, std::placeholders::_1));
        sub_thermal = create_subscription<std_msgs::msg::Int32>(
            "/ess/thermal/ack", 10, std::bind(&ControlNode::thermal_callback, this, std::placeholders::_1));

        sub_priority_zone_ = create_subscription<std_msgs::msg::Int32>(
            "/ess/priority_zone", 10, std::bind(&ControlNode::priority_zone_callback, this, std::placeholders::_1));

        sub_aruco = create_subscription<std_msgs::msg::Int32>(
            "/ess/aruco/ack", 10, std::bind(&ControlNode::aruco_callback, this, std::placeholders::_1));

        
        pub_cmd_vel_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        pub_capture = create_publisher<std_msgs::msg::Int32>("/ess/request/id", 10);
        pub_pose_mqtt_ = create_publisher<geometry_msgs::msg::Pose>("/ess/robot_pose", 10);
        pub_aruco_ = create_publisher<std_msgs::msg::Int32>("/ess/aruco/request", 10);
        pub_zone_status_ = create_publisher<std_msgs::msg::Int32>("/ess/zone_status", 10);


        // 50ms 마다 실행되는 tick 타이머에서 좌표를 계속 발행할거임
        nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

        // 2. GPIO 초기화
        setup_gpio();
        
        lift_inflight_ = true;
        std::thread(&ControlNode::move_to_floor_blocking, this, TOFLOOR1 | GODOWN).detach();
        // 3. 메인 루프 타이머 (FSM)
        timer_ = create_wall_timer(50ms, std::bind(&ControlNode::tick, this));
        current_row_bit = TOFLOOR1;
        target_floor_bit_ = TOFLOOR1; // 초기화 안해줘서 죽을뻔함
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
    const int ONE_MINUTE_TICKS = (60 * 1000) / 50; // 60초 / 50ms = 1200 ticks
    int idle_counter_ = 0;
    // --- 콜백 함수 ---
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
        int input_col = (zone - 1) % 3; // Zone 1,4 -> 0 / Zone 2,5 -> 1 / Zone 3,6 -> 2


        if(!aruco_done_)
        {
            aruco_done_ = true; // 강제 성공 플래그 세팅
    
            stop_cmd_vel();
    
            std_msgs::msg::Int32 msg;
            msg.data = 0;
            pub_aruco_->publish(msg);
            RCLCPP_WARN(get_logger(), "Aruco Interrupted by Emergency!");
        }


        if (zone >= 1 && zone <= 6) {
            // [중복 방지 1] 이미 비상 모드이고, 목표 구역이 같으면 무시
            if (is_emergency_active_ && current_col_ == input_col) return;
            
            // [중복 방지 2] 큐에 이미 같은 구역이 대기 중이면 무시
            for (int q_col : emergency_queue_) {
                if (q_col == input_col) return;
            }

            // 1. 큐에 추가
            emergency_queue_.push_back(input_col);
            RCLCPP_WARN(get_logger(), "!!! Emergency Added to Queue: Zone %d (Queue Size: %ld) !!!", 
                        zone, emergency_queue_.size());

            // =================================================================================
            // [핵심 수정] 패트롤 중이든 뭐든, 비상 모드가 아니라면 "즉시 중단하고 출동"
            // =================================================================================
            if (!is_emergency_active_) {
                
                // 1) 현재 하고 있는 패트롤 위치 저장 (나중에 돌아오기 위함, 선택사항)
                if (saved_patrol_col_ == -1) {
                    saved_patrol_col_ = current_col_;
                }

                RCLCPP_WARN(get_logger(), "!!! INTERRUPTING PATROL -> SWITCHING TO EMERGENCY !!!");

                // 2) 현재 이동 중인 네비게이션 강제 취소
                nav_client_->async_cancel_all_goals();
                stop_cmd_vel(); // 로봇 정지 명령
                
                // 3) 이동 완료 플래그 강제 리셋 (기존 이동이 끝난 것처럼 처리하거나, 무시하게)
                nav_done_ = false; 
                nav_ok_ = false;

                // 4) 비상 모드 플래그 활성화
                is_emergency_active_ = true;

                // 5) 즉시 비상 태스크 시작 함수 호출
                start_next_emergency_task();
            }
            // 이미 비상 모드(is_emergency_active_ == true)라면 큐에 쌓였으니, 
            // 현재 비상 작업 끝난 후 다음 비상 작업으로 이어짐.

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

        // 1. 큐에서 꺼내기
        int next_col = emergency_queue_.front();
        emergency_queue_.pop_front();

        // 2. 변수 세팅
        current_col_ = next_col;
        target_floor_bit_ = TOFLOOR1; // 비상시는 무조건 1층 탐색부터
        
        thermal_done_ = true;   // 기존 작업 플래그 리셋
        home_requested_ = false;
        emergency_goal_sent_ = false;
        nav_done_ = false;
        
        is_emergency_active_ = true; 

        RCLCPP_WARN(get_logger(), ">>> Starting Emergency Task for Zone %d <<<", current_col_ + 1);

        if (!lift_inflight_) {
            lift_inflight_ = true;
            std::thread(&ControlNode::move_to_floor_blocking, this, TOFLOOR1 | GODOWN).detach();
        } 

        // 4. 상태 전환 -> 바로 비상 모드로!
        state_ = State::EMERGENCY;
    }


    // --- 상태 머신 (FSM) ---
    void tick() {
        // 매 tick 루프중 4번째마다 현재 좌표를 읽어서 MQTT 전송용 토픽으로 발행 해야징~


        if(state_ != preState_) {
            RCLCPP_INFO(get_logger(), "State changed: %s -> %s", strState(preState_).c_str(), strState(state_).c_str());
            preState_ = state_;
        }

        publish_current_pose();
        

        // 아 함수화 마렵다 ㄹㅇ로다가
        switch (state_) {
            case State::IDLE:

                if (++idle_counter_ >= ONE_MINUTE_TICKS) {
                        RCLCPP_INFO(get_logger(), "1 minute wait finished. Restarting Patrol...");
                        idle_counter_ = 0;
                        
                        // [수정] 자동 시작 시 첫 이동을 위해 nav_done_을 true로
                        nav_done_ = true; 
                        nav_ok_ = true;
                        current_col_ = 0;
                        target_floor_bit_ = TOFLOOR1;

                        state_ = State::NAV_FIRST_POS; 
                }
                // 만약 대기 중에 외부에서 /ess/home 신호가 오면 즉시 시작
                if (home_requested_) {
                    RCLCPP_INFO(get_logger(), "Manual home signal received. Starting Sequence");
                    idle_counter_ = 0;
                    state_ = State::NAV_HOME;
                }

                if (!emergency_queue_.empty()) {
                    RCLCPP_WARN(get_logger(), "Emergency detected during IDLE! Waking up immediately.");
                    idle_counter_ = 0; // 카운터 초기화
                    start_next_emergency_task(); // 비상 작업 시작 함수 호출
                    break; // 스위치문 탈출
                }

                break;
            case State::NAV_HOME:
                if (home_requested_.exchange(false)) {
                    RCLCPP_INFO(get_logger(), "Sequence Start: Moving to Home(0,0)");
                    send_nav_goal(0.1, 0.0, START_TH);
                    state_ = State::NAV_FIRST_POS;
                }
                break;

            case State::NAV_FIRST_POS:
                if (nav_done_) {
                    if (!nav_ok_) { fail_and_stop("First Pos Failed"); return; }
                    RCLCPP_INFO(get_logger(), "Arrived Home...");
                    send_nav_goal(points[0].x, 0.2, M_PI/2.0 + START_TH);
        
                    state_ = State::WAIT_LIFT;
                }
                break;
                
            case State::NAV_SECOND_POS:
                if (nav_done_) {
                    if (!nav_ok_) { fail_and_stop("Second Pos Failed"); return; }
                    send_nav_goal(1.0, 1.113, M_PI + START_TH);
                    //state_ = State::DRIVE_STRAIGHT; // 이때는 바로 Wait lift로 가는게 맞는거같음
                    state_ = State::NAV_BACK_HOME;
                }
                break;

            case State::DRIVE_STRAIGHT:
            
                if (lift_inflight_) {// 리프트 이동 중이 아닐 때만 실행
                    stop_cmd_vel(); // 안전을 위해 정지 명령 유지
                    break;
                }
                if (nav_done_) {

                    send_nav_goal(points[current_col_].x, points[current_col_].y, M_PI/2.0 + START_TH);
                    
                    // Nav2가 각도를 맞출 때까지 기다리기 위해 상태 변경
                    state_ = State::WAIT_LIFT; 
                }
                break;

            case State::START_LIFT:
                if (!lift_inflight_) {
                    lift_inflight_ = true;
                    // 예시: 2층으로 올라가기

                    RCLCPP_INFO(get_logger(), "Moving Lift: target %d", target_floor_bit_);
                    std::thread(&ControlNode::move_to_floor_blocking, this, target_floor_bit_ | GOUP).detach();
                    state_ = State::WAIT_LIFT;
                }
                break;
            case State::WAIT_LIFT:
            {
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
            }

            case State::WAIT_THERMAL:
            {
                if (thermal_done_) {
                    if (++thermal_wait_count_ < 2) break;
                    thermal_wait_count_ = 0;

                    // 3층까지 작업 완료 시 (한 구역 검사 끝)
                    if (target_floor_bit_ == TOFLOOR3) {
                        
                        // [비상 상황 처리 로직]
                        if (is_emergency_active_) {
                            RCLCPP_INFO(get_logger(), "Emergency Zone %d Processed.", current_col_ + 1);
                            
                            // 1. 큐에 또 다른 비상이 남았으면 -> 그리로 바로 이동
                            if (!emergency_queue_.empty()) {
                                start_next_emergency_task(); 
                                break; // 상태 전환했으니 여기서 탈출
                            }
                            // 2. 비상 상황 모두 종료 -> "여기서부터 계속 순찰 진행"
                            else {
                                RCLCPP_WARN(get_logger(), "Emergency Done. Continuing Patrol Sequence from here...");
                                is_emergency_active_ = false; 
                                saved_patrol_col_ = -1; 
                                // break 없이 아래로 흘려보내서(Fall-through) 자연스럽게 다음 단계 진행
                            }
                        }
                        
                        // 현재 구역이 마지막 구역(Index 2, Zone 3)이었으면
                        // -> 바로 집(FINAL)이 아니라, 지정된 경유지(SECOND_POS)로 이동!
                        if (current_col_ >= 2) {
                            RCLCPP_INFO(get_logger(), "All Zones complete. Moving to Waypoint 2.");
                            
                            // [수정 포인트] 집(0,0)으로 바로 안 가고, Waypoint(2.0, 1.2)로 먼저 감
                            send_nav_goal(2.0, 1.2, M_PI);
                            
                            // 리프트 내리면서 이동
                            lift_inflight_ = true;
                            std::thread(&ControlNode::move_to_floor_blocking, this, TOFLOOR1 | GODOWN).detach();
                            
                            // 상태를 NAV_SECOND_POS로 변경 (그래야 이후 BACK_HOME -> FINAL 순서로 감)
                            state_ = State::NAV_SECOND_POS;
                        } 
                        // 아직 다음 구역이 남았으면 (Zone 1 or 2) -> 다음 구역으로
                        else {
                            current_col_++; 
                            target_floor_bit_ = TOFLOOR1; 
                            
                            // 이동하면서 리프트 내리기
                            lift_inflight_ = true;
                            std::thread(&ControlNode::move_to_floor_blocking, this, TOFLOOR1 | GODOWN).detach();
                            
                            RCLCPP_INFO(get_logger(), "Moving to Next Zone %d", current_col_ + 1);
                            send_nav_goal(points[current_col_].x, points[current_col_].y, M_PI/2.0 + START_TH);
                            
                            state_ = State::DRIVE_STRAIGHT;
                        }

                    } else {
                        // (층 이동 로직) 1층 -> 2층, 2층 -> 3층
                        target_floor_bit_ <<= 1;
                        state_ = State::START_LIFT;
                    }
                    thermal_done_ = false; 
                }
                break;
            }
            case State::NAV_BACK_HOME:
                if (nav_done_) {
                    RCLCPP_INFO(get_logger(), "All Sequence Complete.");
                    send_nav_goal(0.1, 0.0, START_TH);
                    state_ = State::NAV_FINAL_APPROACH;
                }
                break;

            case State::NAV_FINAL_APPROACH:
                if (nav_done_) {
                    // 홈 도착 주행 실패 시 정지
                    if (!nav_ok_) { fail_and_stop("Final Docking Failed"); return; }
                    
                    RCLCPP_INFO(get_logger(), "Arrived Home. Requesting Aruco Alignment...");

#if USE_ARUCO_ALIGNMENT

                    aruco_done_ = false;

                    stop_cmd_vel();
                    // 1. Aruco 정렬 요청 발행 (Topic: /ess/aruco/request, Data: 1)
                    std_msgs::msg::Int32 msg;
                    msg.data = 1;
                    pub_aruco_->publish(msg);

                    // 2. Aruco 대기 상태로 전환 변수 설정
                    aruco_wait_count_ = 0;
                    
                    // 3. WAIT_ARUCO 상태로 전환 
                    state_ = State::WAIT_ARUCO;
#else
                    finish_patrol_and_idle(); // 아루코 사용 안하면 바로 IDLE로
#endif


                }
                break;

            case State::EMERGENCY:
                if (lift_inflight_) break; 

                if (!emergency_goal_sent_) {
                    RCLCPP_INFO(get_logger(), "Emergency: Sending Nav Zone_%d",current_col_ % 3 + 1);
                    
                    send_nav_goal(points[current_col_].x, points[current_col_].y, START_TH + M_PI / 2.0);
                    
                    emergency_goal_sent_ = true;
                    nav_done_ = false;
                    break;
                }

                if (nav_done_) {
                    emergency_goal_sent_ = false; // 다음을 위해 리셋
                    if (nav_ok_) {
                        state_ = State::WAIT_LIFT;
                    } else {
                        fail_and_stop("Emergency Navigation Failed");
                    }
                }
            break;

            case State::WAIT_ARUCO:
                // [성공 조건] 콜백 받음
                if (aruco_done_) {
                    RCLCPP_INFO(get_logger(), "Aruco Alignment Success. Patrol Complete.");
                    finish_patrol_and_idle(); // 초기화 및 IDLE 이동 함수 호출 
                }
                // [타임아웃 조건] 10초 경과
                // 50ms * 200 = 10000ms = 10초
                else if (++aruco_wait_count_ >= 200) {

                    aruco_done_ = true; // 강제 성공 플래그 세팅

                    stop_cmd_vel();

                    std_msgs::msg::Int32 msg;
                    msg.data = 0;
                    pub_aruco_->publish(msg);

                    RCLCPP_WARN(get_logger(), "Aruco Timeout (10s)! Forcing IDLE state.");
                    finish_patrol_and_idle(); // 실패해도 IDLE로 이동

                }
            break;
        }
    }

    // 로봇의 현재 좌표 발행 함수
    void publish_current_pose() {
        double x, y;
        if (get_robot_xy(x, y)) {
            geometry_msgs::msg::Pose pose_msg;
            pose_msg.position.x = x;
            pose_msg.position.y = y;
            // 필요하다면 orientation(방향) 정보도 추가 가능
            //pub_pose_mqtt_->publish(pose_msg);

            if (x < zonePoints[0].x  &&  y > zonePoints[0].y ) {
                zone_Num = 1; // Index 0 -> Zone 1, Index 3 -> Zone 4
            }
            else if (x < zonePoints[1].x && x >= zonePoints[0].x && y > zonePoints[1].y) {
                zone_Num = 2; // Index 1 -> Zone 2, Index 4 -> Zone 5
            }
            else if (x < zonePoints[2].x && x >= zonePoints[1].x && y > zonePoints[2].y) {
                zone_Num = 3; // Index 2 -> Zone 3, Index 5 -> Zone 6
            }
            else if (x < zonePoints[0].x && y <= zonePoints[0].y) {
                zone_Num = 4; // Index 3 -> Zone 4
            }
            else if (x < zonePoints[1].x && x >= zonePoints[0].x && y <= zonePoints[1].y) {
                zone_Num = 5; // Index 4 -> Zone 5
            }
            else if (x < zonePoints[2].x && x >= zonePoints[1].x && y <= zonePoints[2].y) {
                zone_Num = 6; // Index 5 -> Zone 6
            }
            std_msgs::msg::Int32 zone_msg;
                zone_msg.data = zone_Num;
            pub_zone_status_->publish(zone_msg); 
        }
    }

    void finish_patrol_and_idle() {
        current_col_ = 0;
        target_floor_bit_ = TOFLOOR1;
        home_requested_ = false;
        
        // 1분 대기 카운터 초기화
        idle_counter_ = 0; 
        
        RCLCPP_INFO(get_logger(), "Resting for 1 min.");
        state_ = State::IDLE;
    }


    // --- 하드웨어(GPIO) 제어 ---
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
        
        // 1. 시작 전 이미 해당 층인지 확인 (노이즈 방지를 위해 2번 체크)
        if ((check_current_floor() & target_bit) && (check_current_floor() & target_bit)) {
            RCLCPP_INFO(get_logger(), "Lift already at target floor. Skip moving.");
            apply_motor_state(STOP);
            lift_inflight_ = false;
            return;
        }

        // 2. 모터 구동 시작
        apply_motor_state(cmd);
        
        // 3. 타임아웃 설정을 위한 시작 시간 기록
        auto start_time = std::chrono::steady_clock::now();
        const double TIMEOUT_SEC = 10.0; // 7초 이상 작동하면 문제 있는 것으로 간주

       while (rclcpp::ok()) {
            // poll 로직 (기존과 동일)
            if (::poll(pfds_.data(), pfds_.size(), 50) > 0) {
                for (size_t i = 0; i < pfds_.size(); ++i) {
                    if (pfds_[i].revents & POLLIN) {
                        gpiod_line_event ev; 
                        gpiod_line_event_read(sensor_lines_[i], &ev);
                    }
                }
            }
            
            int current_f = check_current_floor();

            // [정상 종료] 목표 도달
            if (current_f & target_bit) break;

            // [안전장치 1] 하강 중 1층 센서 감지 시 무조건 정지 (추락 방지)
            if ((cmd & GODOWN) && (current_f & TOFLOOR1)) {
                RCLCPP_WARN(get_logger(), "Safety Stop: Hit Floor 1 while going down!");
                break;
            }
            // [안전장치 2] 상승 중 3층 센서 감지 시 무조건 정지
            if ((cmd & GOUP) && (current_f & TOFLOOR3)) {
                RCLCPP_WARN(get_logger(), "Safety Stop: Hit Floor 3 while going up!");
                break;
            }
            // [안전장치 3] 시간 초과
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

    // --- 주행 제어 (Nav2 & TF) ---
    void send_nav_goal(double x, double y, double yaw) {
        if (!nav_client_->wait_for_action_server(1s)) {
            fail_and_stop("Nav2 Server Not Found");
            return;
        }
        auto goal = NavigateToPose::Goal();
        goal.pose.header.frame_id = "map";
        goal.pose.header.stamp = now();

        // [중요 수정] goal.pose(PoseStamped) -> pose(Pose) -> position 순서입니다.
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


    // map으로 위치 줌
    bool get_robot_xy(double &x, double &y) {
        try {
            auto t = tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);
            x = t.transform.translation.x; y = t.transform.translation.y;
            return true;
        } catch (...) { return false; }
    }

    // odom 으로 위치좌표를 가져오면
    // 장거리시 오류 누적 가능
    // map이 아닌 바퀴와 imu 센서만 믿고 계산한 위치
    // bool get_robot_xy(double &x, double &y) {
    //     try {
    //         // "map" 대신 "odom"을 사용하여 지도 보정으로 인한 위치 튐 현상 방지
    //         auto t = tf_buffer_.lookupTransform("odom", "base_link", tf2::TimePointZero);
    //         x = t.transform.translation.x; 
    //         y = t.transform.translation.y;
    //         return true;
    //     } catch (tf2::TransformException &ex) {
    //         return false; 
    //     }
    // }
    
    // 남한테 던질떄 사용하는 함수
    void stop_cmd_vel() { pub_cmd_vel_->publish(geometry_msgs::msg::Twist()); }
    void send_capture_cmd(int id) { std_msgs::msg::Int32 m; m.data = id; pub_capture->publish(m); }
    
    
    // 
    void fail_and_stop(std::string m) { RCLCPP_ERROR(get_logger(), "%s", m.c_str()); state_ = State::IDLE; }
    void cleanup_gpio() { apply_motor_state(STOP); for(auto* l : sensor_lines_) gpiod_line_release(l); gpiod_chip_close(chip_); }

    // 멤버 변수 정의
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
    //긴급 상황 목표 전송 상태 관리용
    bool emergency_goal_sent_ = false;

    uint8_t target_floor_bit_; // 목표 층 비트 플래그 1, 2, 4 |연산으로 GOUP, GODOWN 을하여 제어함
  
    int current_col_; // 현재 로봇의 열 위치 0, 1, 2 즉 몇번째 구역인지
    uint8_t current_row_bit; // 현재 로봇의 행 위치 1, 2, 4 즉 몇층인지몇번째 비트인지 
    int thermal_wait_count_;
    
    std::deque<int> emergency_queue_; // 비상상황 큐 Zone 번호 저장용
    bool is_emergency_active_ = false; // 지금 비상 작업 중인지 확인하는 플래그

    std::atomic<bool> aruco_done_{false}; // 아루코 완료 플래그
    int aruco_wait_count_ = 0;            // 타임아웃 카운터

    int saved_patrol_col_ = -1; // emergency 직전 상태로 돌아가기 위한

    int zone_Num = 0;


};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControlNode>());
    rclcpp::shutdown();
    return 0;
}
