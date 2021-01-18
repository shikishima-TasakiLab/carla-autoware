#include <deque>
#include <string>
#include "ros/ros.h"
#include "cv_bridge/cv_bridge.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/Joy.h"
#include "carla_msgs/CarlaEgoVehicleControl.h"
#include "carla_msgs/CarlaEgoVehicleStatus.h"
#include "opencv2/opencv.hpp"

#define KEY_F11 200
#define WINDOW_NAME "CARLA G29"
#define GEAR_COUNT 5
#define CHATTERING_COUNT 5

class CV_Manual_Control
{
public:
    CV_Manual_Control(std::string role_name = "ego_vehicle", double_t hz = 20.0);
    ~CV_Manual_Control();
private:
    ros::NodeHandle nh_;
    ros::Timer timer_;

    ros::Subscriber view_sub_;
    std::deque<sensor_msgs::Image> view_queue_;

    ros::Subscriber vehicle_status_sub_;
    std::deque<carla_msgs::CarlaEgoVehicleStatus> vehicle_status_queue_;
    ros::Subscriber g29_sub_;

    ros::Publisher vehicle_cmd_pub_;
    ros::Publisher manual_override_pub_;

    bool reverse_ = false;
    bool hand_brake_ = true;
    float hand_brake_pedal_ = -1.0f;
    int gear_ = 0;
    bool manual_gear_shift_ = false;
    int chattering = 0;
    bool manual_override_ = false;
    int velocity_kmh_ = 0;

    cv::Size textsize_kmh = cv::getTextSize(" km/h", cv::FONT_HERSHEY_SIMPLEX, 1.0, 2, 0);
    cv::Size textsize_num[10] = {
        cv::getTextSize("0", cv::FONT_HERSHEY_SIMPLEX, 1.0, 2, 0),
        cv::getTextSize("1", cv::FONT_HERSHEY_SIMPLEX, 1.0, 2, 0),
        cv::getTextSize("2", cv::FONT_HERSHEY_SIMPLEX, 1.0, 2, 0),
        cv::getTextSize("3", cv::FONT_HERSHEY_SIMPLEX, 1.0, 2, 0),
        cv::getTextSize("4", cv::FONT_HERSHEY_SIMPLEX, 1.0, 2, 0),
        cv::getTextSize("5", cv::FONT_HERSHEY_SIMPLEX, 1.0, 2, 0),
        cv::getTextSize("6", cv::FONT_HERSHEY_SIMPLEX, 1.0, 2, 0),
        cv::getTextSize("7", cv::FONT_HERSHEY_SIMPLEX, 1.0, 2, 0),
        cv::getTextSize("8", cv::FONT_HERSHEY_SIMPLEX, 1.0, 2, 0),
        cv::getTextSize("9", cv::FONT_HERSHEY_SIMPLEX, 1.0, 2, 0)
    };

    void vehicle_status_callback(const carla_msgs::CarlaEgoVehicleStatusConstPtr &vehicle_status);
    void g29_sub_callback(const sensor_msgs::JoyConstPtr &g29_msg);
    void view_sub_callback(const sensor_msgs::ImageConstPtr &view_msg);
    void timer_callback(const ros::TimerEvent &e);
};

CV_Manual_Control::CV_Manual_Control(std::string role_name, double_t hz)
{
    std::cout << "km/h : " << this->textsize_kmh.height << " : " << this->textsize_kmh.width << std::endl;
    for (int i = 0; i < 10; i++) {
        std::cout << std::to_string(i) << "    : " << this->textsize_num[i].height << " : " << this->textsize_num[i].width << std::endl;
    }

    cv::Mat img;
    cv::namedWindow(WINDOW_NAME, cv::WINDOW_FREERATIO);
    cv::setWindowProperty(WINDOW_NAME, cv::WND_PROP_FULLSCREEN, cv::WINDOW_NORMAL);
    cv::waitKey(1);
    this->manual_override_pub_ = this->nh_.advertise<std_msgs::Bool>("/carla/" + role_name + "/vehicle_control_manual_override", 10);
    this->vehicle_cmd_pub_ = this->nh_.advertise<carla_msgs::CarlaEgoVehicleControl>("/carla/" + role_name + "/vehicle_control_cmd_manual", 10);

    this->vehicle_status_sub_ = this->nh_.subscribe<carla_msgs::CarlaEgoVehicleStatus>("/carla/" + role_name + "/vehicle_status", 10, &CV_Manual_Control::vehicle_status_callback, this);
    this->g29_sub_ = this->nh_.subscribe<sensor_msgs::Joy>("/g29/joy", 10, &CV_Manual_Control::g29_sub_callback, this);
    this->view_sub_ = this->nh_.subscribe<sensor_msgs::Image>("/carla/" + role_name + "/camera/rgb/view/image_color", 10, &CV_Manual_Control::view_sub_callback, this);
    this->timer_ = this->nh_.createTimer(ros::Duration(1.0 / hz), &CV_Manual_Control::timer_callback, this);
}

CV_Manual_Control::~CV_Manual_Control()
{
    cv::destroyAllWindows();
}

void CV_Manual_Control::vehicle_status_callback(const carla_msgs::CarlaEgoVehicleStatusConstPtr &vehicle_status)
{
    this->vehicle_status_queue_.push_back(*vehicle_status);
}

void CV_Manual_Control::g29_sub_callback(const sensor_msgs::JoyConstPtr &g29_msg)
{
    carla_msgs::CarlaEgoVehicleControl cmd_msg;
    cmd_msg.steer = -g29_msg->axes[0];
    cmd_msg.throttle = (g29_msg->axes[2] + 1.0f) * 0.5f;
    cmd_msg.brake = (g29_msg->axes[3] + 1.0f) * 0.5f;

    if (this->hand_brake_pedal_ < g29_msg->axes[1] && 1.0f <= g29_msg->axes[1]) {
        this->hand_brake_ = !(this->hand_brake_);
    }
    this->hand_brake_pedal_ = g29_msg->axes[1];
    cmd_msg.hand_brake = this->hand_brake_;

    if (this->chattering < 1) {
        if (this->velocity_kmh_ == 0) {
            if (g29_msg->buttons[5] == 1) {
                if (this->gear_ > 1) {
                    this->gear_--;
                    this->manual_gear_shift_ = true;
                }
                else {
                    this->gear_ = 0;
                    this->manual_gear_shift_ = false;
                }
                this->chattering = CHATTERING_COUNT;
            }
            else if (g29_msg->buttons[4] == 1){
                if (this->gear_ < GEAR_COUNT) {
                    this->gear_++;
                }
                this->manual_gear_shift_ = true;
                this->chattering = CHATTERING_COUNT;
            }
        }

        if (g29_msg->buttons[19] == 1) {
            this->reverse_ = false;
            this->chattering = CHATTERING_COUNT;
        }
        else if (g29_msg->buttons[20] == 1) {
            this->reverse_ = true;
            this->gear_ = 0;
            this->manual_gear_shift_ = false;
            this->chattering = CHATTERING_COUNT;
        }
    }

    cmd_msg.gear = this->gear_;
    cmd_msg.manual_gear_shift = this->manual_gear_shift_;
    cmd_msg.reverse = this->reverse_;

    this->vehicle_cmd_pub_.publish(cmd_msg);
}

void CV_Manual_Control::view_sub_callback(const sensor_msgs::ImageConstPtr &view_msg)
{
    this->view_queue_.push_back(*view_msg);
}

void CV_Manual_Control::timer_callback(const ros::TimerEvent &e)
{
    if (this->chattering > 0) {
        this->chattering--;
    }
    if (this->view_queue_.size() < 1) {
        return;
    }

    cv_bridge::CvImagePtr cv_view(new cv_bridge::CvImage);
    cv_view = cv_bridge::toCvCopy(this->view_queue_.front(), this->view_queue_.front().encoding);

    this->velocity_kmh_ = static_cast<int>(roundf(this->vehicle_status_queue_.front().velocity * 3.6f));
    std::string velocity_kmh_str = std::to_string(this->velocity_kmh_);
    cv::rectangle(cv_view->image,
        cv::Point(0, static_cast<int>(cv_view->image.rows * 0.9)),
        cv::Point(cv_view->image.cols - 1, cv_view->image.rows - 1),
        cv::Scalar(0, 0, 0, 255),
        cv::FILLED
    );

    double_t text_scale_full = static_cast<double_t>(cv_view->image.rows) * 0.08 / static_cast<double_t>(this->textsize_kmh.height);
    double_t text_scale_half = text_scale_full * 0.5;

    int digit;
    if (this->velocity_kmh_ < 10) digit = 1;
    else if (this->velocity_kmh_ < 100) digit = 2;
    else digit = 3;
    cv::putText(
        cv_view->image,
        velocity_kmh_str,
        cv::Point(
            (cv_view->image.cols - static_cast<int>(static_cast<double_t>(this->textsize_num[0].width) * text_scale_full) * digit) / 2,
            static_cast<int>(static_cast<double_t>(cv_view->image.rows) * 0.99)
        ),
        cv::FONT_HERSHEY_SIMPLEX,
        text_scale_full,
        cv::Scalar(255, 255, 255, 255),
        2,
        CV_AA
    );
    cv::putText(
        cv_view->image, "km/h",
        cv::Point(
            (cv_view->image.cols + static_cast<int>(static_cast<double_t>(this->textsize_num[0].width) * text_scale_full) * 3) / 2,
            static_cast<int>(static_cast<double_t>(cv_view->image.rows) * 0.99)
        ),
        cv::FONT_HERSHEY_SIMPLEX,
        text_scale_half,
        cv::Scalar(255, 255, 255, 255),
        2,
        CV_AA
    );

    std_msgs::Bool override_msg;
    override_msg.data = this->manual_override_;
    this->manual_override_pub_.publish(override_msg);

    if (this->manual_override_ == true) {
        cv::putText(
            cv_view->image, "CTRL",
            cv::Point(
                0,
                static_cast<int>(static_cast<double_t>(cv_view->image.rows) * 0.99)
            ),
            cv::FONT_HERSHEY_SIMPLEX,
            text_scale_half,
            cv::Scalar(255, 255, 255, 255),
            2,
            CV_AA
        );
    }

    cv::imshow(WINDOW_NAME, cv_view->image);
    
    int key = cv::waitKey(1);
    if (key == static_cast<int>('q')) {
        ros::shutdown();
    }
    else if (key == static_cast<int>('c')) {
        this->manual_override_ = !(this->manual_override_);
    }
    else if (key == KEY_F11) {
        if (cv::getWindowProperty(WINDOW_NAME, cv::WND_PROP_FULLSCREEN) == cv::WINDOW_FULLSCREEN) {
            cv::setWindowProperty(WINDOW_NAME, cv::WND_PROP_FULLSCREEN, cv::WINDOW_NORMAL);
        }
        else {
            cv::setWindowProperty(WINDOW_NAME, cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
        }
    }

    for (; this->view_queue_.size() > 0;) {
        this->view_queue_.pop_front();
    }
    for (; this->vehicle_status_queue_.size() > 0;) {
        this->vehicle_status_queue_.pop_front();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cv_manual_control");
    ros::NodeHandle pnh("~");

    std::string role_name = "ego_vehicle";
    double_t hz = 20.0;
    pnh.getParam("role_name", role_name);
    pnh.getParam("hz", hz);

    CV_Manual_Control cmc(role_name, hz);
    ros::spin();
    return 0;
}