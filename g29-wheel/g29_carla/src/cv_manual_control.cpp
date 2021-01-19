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
    CV_Manual_Control(std::string img_dir, std::string role_name = "ego_vehicle", double_t hz = 20.0);
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
    ros::Publisher enable_autopilot_pub_;

    bool reverse_ = false;
    bool hand_brake_ = true;
    float hand_brake_pedal_ = -1.0f;
    int gear_ = 0;
    bool manual_gear_shift_ = false;
    int chattering = 0;
    bool manual_override_ = false;
    int velocity_kmh_ = 0;
    bool enable_autopilot_ = false;

    cv::Size screen_size_ = cv::Size(0, 0);
    cv::Size cockpit_size_full_ = cv::Size(0, 0);
    cv::Size cockpit_size_half_ = cv::Size(0, 0);
    
    std::vector<cv::Mat> cv_7seg_;
    std::vector<cv::Mat> cv_gear_;
    cv::Mat cv_reverse_;
    cv::Mat cv_kmh_;
    cv::Mat cv_steer_;
    cv::Mat cv_handbrake_;
    cv::Mat cv_autopilot_;

    void alpha_blend(cv::Mat &fg, cv::Mat &bg, cv::Point up_left, double_t alpha = 1.0);

    void vehicle_status_callback(const carla_msgs::CarlaEgoVehicleStatusConstPtr &vehicle_status);
    void g29_sub_callback(const sensor_msgs::JoyConstPtr &g29_msg);
    void view_sub_callback(const sensor_msgs::ImageConstPtr &view_msg);
    void timer_callback(const ros::TimerEvent &e);
};

CV_Manual_Control::CV_Manual_Control(std::string img_dir, std::string role_name, double_t hz)
{
    for (int i = 0; i < 10; i ++) {
        this->cv_7seg_.push_back(cv::imread(img_dir + "/7seg_" + std::to_string(i) + ".png", cv::IMREAD_UNCHANGED));
    }
    this->cv_7seg_.push_back(cv::imread(img_dir + "/7seg_null.png", cv::IMREAD_UNCHANGED));

    this->cv_gear_.push_back(cv::imread(img_dir + "/gear_d.png", cv::IMREAD_UNCHANGED));
    for (int i = 1; i <= 5; i ++) {
        this->cv_gear_.push_back(cv::imread(img_dir + "/gear_" + std::to_string(i) + ".png", cv::IMREAD_UNCHANGED));
    }
    this->cv_reverse_ = cv::imread(img_dir + "/gear_r.png", cv::IMREAD_UNCHANGED);
    this->cv_autopilot_ = cv::imread(img_dir + "/gear_a.png", cv::IMREAD_UNCHANGED);

    this->cv_kmh_ = cv::imread(img_dir + "/kmh.png", cv::IMREAD_UNCHANGED);
    this->cv_steer_ = cv::imread(img_dir + "/steer.png", cv::IMREAD_UNCHANGED);
    this->cv_handbrake_ = cv::imread(img_dir + "/handbrake.png", cv::IMREAD_UNCHANGED);

    cv::namedWindow(WINDOW_NAME, cv::WINDOW_FREERATIO);
    cv::setWindowProperty(WINDOW_NAME, cv::WND_PROP_FULLSCREEN, cv::WINDOW_NORMAL);
    cv::waitKey(1);
    this->manual_override_pub_ = this->nh_.advertise<std_msgs::Bool>("/carla/" + role_name + "/vehicle_control_manual_override", 1);
    this->vehicle_cmd_pub_ = this->nh_.advertise<carla_msgs::CarlaEgoVehicleControl>("/carla/" + role_name + "/vehicle_control_cmd_manual", 1);
    this->enable_autopilot_pub_ = this->nh_.advertise<std_msgs::Bool>("/carla/" + role_name + "/enable_autopilot", 1);

    this->vehicle_status_sub_ = this->nh_.subscribe<carla_msgs::CarlaEgoVehicleStatus>("/carla/" + role_name + "/vehicle_status", 1, &CV_Manual_Control::vehicle_status_callback, this);
    this->g29_sub_ = this->nh_.subscribe<sensor_msgs::Joy>("/g29/joy", 1, &CV_Manual_Control::g29_sub_callback, this);
    this->view_sub_ = this->nh_.subscribe<sensor_msgs::Image>("/carla/" + role_name + "/camera/rgb/view/image_color", 1, &CV_Manual_Control::view_sub_callback, this);
    this->timer_ = this->nh_.createTimer(ros::Duration(1.0 / hz), &CV_Manual_Control::timer_callback, this);
}

CV_Manual_Control::~CV_Manual_Control()
{
    cv::destroyAllWindows();
}

void CV_Manual_Control::alpha_blend(cv::Mat &fg, cv::Mat &bg, cv::Point up_left, double_t alpha)
{
    cv::Size roi_size;
    roi_size.width = (up_left.x + fg.cols > bg.cols)? (bg.cols - up_left.x) : (fg.cols);
    roi_size.height = (up_left.y + fg.rows > bg.rows)? (bg.rows - up_left.y) : (fg.rows);

    cv::Rect s_roi_rect(cv::Point(0, 0), roi_size);
    cv::Rect d_roi_rect(up_left, roi_size);
    cv::Mat s_roi = fg(s_roi_rect);
    cv::Mat d_roi = bg(d_roi_rect);

    cv::Mat s_bgra[4];
    cv::Mat d_bgra[4];
    cv::split(s_roi, s_bgra);
    cv::split(d_roi, d_bgra);

    cv::Mat s_bgr, d_bgr;
    cv::merge(s_bgra, 3, s_bgr);
    cv::merge(d_bgra, 3, d_bgr);

    cv::Mat s_aaa[3] = {s_bgra[3], s_bgra[3], s_bgra[3]};
    cv::Mat d_aaa[3] = {d_bgra[3], d_bgra[3], d_bgra[3]};
    cv::Mat s_alpha, d_alpha;
    cv::merge(s_aaa, 3, s_alpha);
    cv::merge(d_aaa, 3, d_alpha);

    cv::Mat s_bgr_f, s_alpha_f, d_bgr_f, d_alpha_f;
    s_bgr.convertTo(s_bgr_f, CV_32FC3);
    d_bgr.convertTo(d_bgr_f, CV_32FC3);
    s_alpha.convertTo(s_alpha_f, CV_32FC3, 1.0f / 255.0f);
    d_alpha.convertTo(d_alpha_f, CV_32FC3, 1.0f / 255.0f);
    
    cv::Mat tmp_f = s_bgr_f.mul(s_alpha_f, alpha) + d_bgr_f.mul(d_alpha_f, 1.0 - alpha);
    cv::Mat tmp_i;
    tmp_f.convertTo(tmp_i, CV_8UC4);
    
    cv::cvtColor(tmp_i, d_roi, CV_BGR2BGRA);
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
        if (this->manual_gear_shift_ == true) {
            if (g29_msg->buttons[5] == 1) {
                if (this->gear_ > 1) {
                    this->gear_--;
                    this->manual_gear_shift_ = true;
                }
                else if (this->velocity_kmh_ == 0) {
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
        else if (this->velocity_kmh_ == 0) {
            if (g29_msg->buttons[4] == 1) {
                this->manual_gear_shift_ = true;
                this->gear_ = 1;
                this->chattering = CHATTERING_COUNT;
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

    if (cv_view->image.cols != this->screen_size_.width || cv_view->image.rows != this->screen_size_.height) {
        this->screen_size_.width = cv_view->image.cols;
        this->screen_size_.height = cv_view->image.rows;
        this->cockpit_size_full_.width = cv_view->image.cols;
        this->cockpit_size_full_.height = cv_view->image.rows / 10;
        this->cockpit_size_half_.width = cv_view->image.cols;
        this->cockpit_size_half_.height = cv_view->image.rows / 20;

        double_t scale;
        for (int i = 0; i < this->cv_7seg_.size(); i++) {
            scale = static_cast<double_t>(this->cockpit_size_full_.height) / static_cast<double_t>(this->cv_7seg_[i].rows);
            cv::resize(this->cv_7seg_[i], this->cv_7seg_[i], cv::Size(
                static_cast<int>(static_cast<double_t>(this->cv_7seg_[0].cols) * scale),
                this->cockpit_size_full_.height
            ));
        }
        for (int i = 0; i < this->cv_gear_.size(); i++) {
            scale = static_cast<double_t>(this->cockpit_size_full_.height) / static_cast<double_t>(this->cv_gear_[i].rows);
            cv::resize(this->cv_gear_[i], this->cv_gear_[i], cv::Size(
                static_cast<int>(static_cast<double_t>(this->cv_gear_[0].cols) * scale),
                this->cockpit_size_full_.height
            ));
        }
        scale = static_cast<double_t>(this->cockpit_size_full_.height) / static_cast<double_t>(this->cv_reverse_.rows);
        cv::resize(this->cv_reverse_, this->cv_reverse_, cv::Size(
            static_cast<int>(static_cast<double_t>(this->cv_reverse_.cols) * scale),
            this->cockpit_size_full_.height
        ));
        scale = static_cast<double_t>(this->cockpit_size_full_.height) / static_cast<double_t>(this->cv_autopilot_.rows);
        cv::resize(this->cv_autopilot_, this->cv_autopilot_, cv::Size(
            static_cast<int>(static_cast<double_t>(this->cv_autopilot_.cols) * scale),
            this->cockpit_size_full_.height
        ));
        scale = static_cast<double_t>(this->cockpit_size_full_.height) / static_cast<double_t>(this->cv_steer_.rows) * 0.8;
        cv::resize(this->cv_steer_, this->cv_steer_, cv::Size(
            static_cast<int>(static_cast<double_t>(this->cv_steer_.cols) * scale),
            this->cockpit_size_full_.height * 4 / 5
        ));
        scale = static_cast<double_t>(this->cockpit_size_half_.height) / static_cast<double_t>(this->cv_kmh_.rows);
        cv::resize(this->cv_kmh_, this->cv_kmh_, cv::Size(
            static_cast<int>(static_cast<double_t>(this->cv_kmh_.cols) * scale),
            this->cockpit_size_half_.height
        ));
        scale = static_cast<double_t>(this->cockpit_size_half_.height) / static_cast<double_t>(this->cv_handbrake_.rows);
        cv::resize(this->cv_handbrake_, this->cv_handbrake_, cv::Size(
            static_cast<int>(static_cast<double_t>(this->cv_handbrake_.cols) * scale),
            this->cockpit_size_half_.height
        ));
    }

    this->velocity_kmh_ = static_cast<int>(roundf(this->vehicle_status_queue_.front().velocity * 3.6f));
    std::string velocity_kmh_str = std::to_string(this->velocity_kmh_);
    cv::rectangle(cv_view->image,
        cv::Point(0, static_cast<int>(cv_view->image.rows * 0.9)),
        cv::Point(cv_view->image.cols - 1, cv_view->image.rows - 1),
        cv::Scalar(0, 0, 0, 255),
        cv::FILLED
    );

    int digit[3];
    digit[0] = this->velocity_kmh_ / 100;
    if (digit[0] == 0 && this->velocity_kmh_ < 100) digit[0] = 10;
    digit[2] = this->velocity_kmh_ % 100;

    digit[1] = digit[2] / 10;
    if (digit[1] == 0 && this->velocity_kmh_ < 10) digit[1] = 10;
    digit[2] %= 10;

    for (int i = 0; i < 3; i++) {
        this->alpha_blend(
            this->cv_7seg_[digit[i]],
            cv_view->image,
            cv::Point(
                (this->screen_size_.width - this->cv_7seg_[0].cols * 3) / 2 + this->cv_7seg_[0].cols * i,
                this->screen_size_.height - this->cv_7seg_[0].rows
            )
        );
    }

    this->alpha_blend(
        this->cv_kmh_,
        cv_view->image,
        cv::Point(
            (this->screen_size_.width + this->cv_7seg_[0].cols * 3) / 2,
            this->screen_size_.height - this->cv_kmh_.rows
        )
    );

    if (this->manual_override_ == true) {
        this->alpha_blend(
            this->cv_steer_,
            cv_view->image,
            cv::Point(this->cv_steer_.cols / 8, this->screen_size_.height - this->cv_steer_.rows * 9 / 8)
        );
    }

    if (this->vehicle_status_queue_.front().control.hand_brake == true) {
        this->alpha_blend(
            this->cv_handbrake_,
            cv_view->image,
            cv::Point(this->screen_size_.width * 5 / 8, this->screen_size_.height - this->cockpit_size_full_.height)
        );
    }

    if (this->enable_autopilot_ == true) {
        this->alpha_blend(
            this->cv_autopilot_,
            cv_view->image,
            cv::Point(this->screen_size_.width * 3 / 8 - this->cv_autopilot_.cols, this->screen_size_.height - this->cockpit_size_full_.height)
        );
    }
    if (this->vehicle_status_queue_.front().control.reverse == true) {
        this->alpha_blend(
            this->cv_reverse_,
            cv_view->image,
            cv::Point(this->screen_size_.width * 3 / 8, this->screen_size_.height - this->cockpit_size_full_.height)
        );
    }
    else if (this->vehicle_status_queue_.front().control.manual_gear_shift == true) {
        this->alpha_blend(
            this->cv_gear_[this->vehicle_status_queue_.front().control.gear],
            cv_view->image,
            cv::Point(this->screen_size_.width * 3 / 8, this->screen_size_.height - this->cockpit_size_full_.height)
        );
    }
    else {
        this->alpha_blend(
            this->cv_gear_[0],
            cv_view->image,
            cv::Point(this->screen_size_.width * 3 / 8, this->screen_size_.height - this->cockpit_size_full_.height)
        );
    }

    cv::imshow(WINDOW_NAME, cv_view->image);
    
    int key = cv::waitKey(1);
    if (key == static_cast<int>('q')) {
        ros::shutdown();
    }
    else if (key == static_cast<int>('b')) {
        this->manual_override_ = !(this->manual_override_);
        std_msgs::Bool override_msg, enable_autopilot_msg;
        override_msg.data = this->manual_override_;
        this->manual_override_pub_.publish(override_msg);
    }
    else if (key == static_cast<int>('p')) {
        this->enable_autopilot_ = !(this->enable_autopilot_);
        std_msgs::Bool override_msg, enable_autopilot_msg;
        enable_autopilot_msg.data = this->enable_autopilot_;
        this->enable_autopilot_pub_.publish(enable_autopilot_msg);
    }
    else if (key == KEY_F11) {
        if (cv::getWindowProperty(WINDOW_NAME, cv::WND_PROP_FULLSCREEN) == cv::WINDOW_FULLSCREEN) {
            cv::setWindowProperty(WINDOW_NAME, cv::WND_PROP_FULLSCREEN, cv::WINDOW_NORMAL);
        }
        else {
            cv::setWindowProperty(WINDOW_NAME, cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
        }
    }

    for (; this->view_queue_.size() > 1;) {
        this->view_queue_.pop_front();
    }
    for (; this->vehicle_status_queue_.size() > 1;) {
        this->vehicle_status_queue_.pop_front();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cv_manual_control");
    ros::NodeHandle pnh("~");

    std::string role_name = "ego_vehicle";
    std::string img_dir = "img";
    double_t hz = 20.0;
    pnh.getParam("role_name", role_name);
    pnh.getParam("img_dir", img_dir);
    pnh.getParam("hz", hz);

    CV_Manual_Control cmc(img_dir, role_name, hz);
    ros::spin();
    return 0;
}