#include <deque>
#include <string>
#include <thread>
#include <tuple>
#include <deque>
#include <fstream>
#include <iostream>
#include <random>

#include "opencv2/opencv.hpp"

#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "carla_msgs/CarlaEgoVehicleInfo.h"

#include "rapidjson/document.h"
#include "rapidjson/istreamwrapper.h"

#include "carla/client/detail/Client.h"
#include "carla/client/ActorBlueprint.h"
#include "carla/client/BlueprintLibrary.h"
#include "carla/client/Client.h"
#include "carla/client/Sensor.h"
#include "carla/client/TimeoutException.h"
#include "carla/client/Map.h"
#include "carla/client/World.h"
#include "carla/image/ImageIO.h"
#include "carla/image/ImageView.h"
#include "carla/sensor/data/Image.h"

#include "g29.hpp"

#define CV_KEY_F11 200
#define WINDOW_NAME "CARLA G29"
#define GEAR_COUNT 5
#define CHATTERING_COUNT 5

#define SIG_RED 0
#define SIG_YELLOW 1
#define SIG_GREEN 2
#define SIG_NULL 3

#define AUTOLIGHT_POSITION_TH 110
#define AUTOLIGHT_BEAM_TH 40

class CV_Manual_Control
{
public:
    CV_Manual_Control(
        std::string img_dir, int width, int height,
        std::string sensor_config_path = "",
        std::string spawn_point = "",
        std::string vehicle_filter = "vehicle.*",
        std::string role_name = "ego_vehicle",
        std::string event_path = "/dev/input/event-g29",
        double_t hz = 20.0,
        std::string host = "localhost", u_int16_t port = 2000
    );
    ~CV_Manual_Control();

    template <typename RangeT, typename RNG>
    static auto &RandomChoice(const RangeT &range, RNG &&generator) {
        std::uniform_int_distribution<size_t> dist{0u, range.size() - 1u};
        return range[dist(std::forward<RNG>(generator))];
    }

private:
    float hand_brake_pedal_ = -1.0f;
    int velocity_kmh_ = 0;
    bool enable_autopilot_ = false;
    bool manual_override_ = false;

    u_int32_t vehicle_id_ = 0;
    bool vehicle_id_usable_ = false;

    int view_id_ = 0;    //  0: BirdFront, 1: BirdBack, 2: Front, 3:Back

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
    cv::Mat cv_sig_frame_;
    std::vector<cv::Mat> cv_sig_;

    carla::client::Client *carla_client_;
    carla::client::World *carla_world_;
    boost::shared_ptr<carla::client::BlueprintLibrary> carla_bplib_;
    carla::geom::Transform carla_camtf_[4];
    carla::geom::Transform carla_bright_tf_;
    boost::shared_ptr<carla::client::Vehicle> carla_ego_vehicle_;
    boost::shared_ptr<carla::client::Sensor> carla_camera_ = nullptr;
    boost::shared_ptr<carla::client::Sensor> carla_bright_ = nullptr;
    std::vector<carla::traffic_manager::ActorPtr> carla_sensors_;
    std::deque<int> bright_deque_;

    std::deque<cv::Mat> img_deque_;

    ros::NodeHandle nh_;
    ros::Timer timer_, js_timer_;
    ros::Publisher manual_override_pub_;

    G29FFB *g29ffb_;

    void load_img(std::string img_dir);
    void change_view(int view_id = 0);
    int get_brightness(cv::Mat &src);
    void attach_sensors(std::string json_path);
    void draw_velocity(cv::Mat &bg);
    void draw_traffic_light(cv::Mat &bg);
    void draw_gear_status(bool reverse, int32_t gear, bool use_gear_autobox, cv::Mat &bg);
    void alpha_blend(cv::Mat &fg, cv::Mat &bg, cv::Point up_left = cv::Point(0, 0), double_t alpha = 1.0);
    void js_timer_callback(const ros::TimerEvent &e);
    void timer_callback(const ros::TimerEvent &e);
};

CV_Manual_Control::CV_Manual_Control(
    std::string img_dir, int width, int height,
    std::string sensor_config_path, std::string spawn_point,
    std::string vehicle_filter, std::string role_name, std::string event_path,
    double_t hz, std::string host, u_int16_t port
)
{
    std::mt19937_64 rng((std::random_device())());

    this->cockpit_size_full_.width = width;
    this->cockpit_size_full_.height = height / 10;
    this->cockpit_size_half_.width = width;
    this->cockpit_size_half_.height = height / 20;
    this->screen_size_.height = height - this->cockpit_size_full_.height;
    this->screen_size_.width = width;

    this->load_img(img_dir);

    this->carla_client_ = new carla::client::Client(host, port);
    this->carla_client_->SetTimeout(carla::time_duration::seconds(10));

    this->carla_world_ = new carla::client::World(this->carla_client_->GetWorld());
    this->carla_bplib_ = this->carla_world_->GetBlueprintLibrary();

    //  車の生成
    carla::SharedPtr<carla::client::BlueprintLibrary> vehicles = this->carla_bplib_->Filter(vehicle_filter);
    carla::client::ActorBlueprint vehicle_bp = RandomChoice(*vehicles, rng);
    if (vehicle_bp.ContainsAttribute("color")) {
        auto &attribute = vehicle_bp.GetAttribute("color");
        vehicle_bp.SetAttribute("color", RandomChoice(attribute.GetRecommendedValues(), rng));
    }
    vehicle_bp.SetAttribute("role_name", role_name);
    carla::SharedPtr<carla::client::Map> map = this->carla_world_->GetMap();
    carla::geom::Transform transform = RandomChoice(map->GetRecommendedSpawnPoints(), rng);

    this->carla_ego_vehicle_ = boost::static_pointer_cast<carla::client::Vehicle>(this->carla_world_->SpawnActor(vehicle_bp, transform));

    //  視点カメラの設定
    this->carla_camtf_[0] = carla::geom::Transform{
        carla::geom::Location{-4.5f, 0.0f, 2.8f},
        carla::geom::Rotation{-15.0f, 0.0f, 0.0f}};
    this->carla_camtf_[1] = carla::geom::Transform{
        carla::geom::Location{4.5f, 0.0f, 2.8f},
        carla::geom::Rotation{-15.0f, 180.0f, 0.0f}};
    this->carla_camtf_[2] = carla::geom::Transform{
        carla::geom::Location{2.0f, 0.0f, 2.0f},
        carla::geom::Rotation{0.0f, 0.0f, 0.0f}};
    this->carla_camtf_[3] = carla::geom::Transform{
        carla::geom::Location{-2.0f, 0.0f, 2.0f},
        carla::geom::Rotation{0.0f, 180.0f, 0.0f}};

    this->view_id_ = 0;
    this->change_view(this->view_id_);

    //  照度センサ(カメラ)の取り付け
    this->carla_bright_tf_ = carla::geom::Transform{
        carla::geom::Location{0.0f, 0.0f, 2.0f},
        carla::geom::Rotation{0.0f, -90.0f, 0.0f}};
    
    carla::client::ActorBlueprint bright_bp = *this->carla_bplib_->Find("sensor.camera.rgb");
    bright_bp.SetAttribute("image_size_x", std::to_string(3));
    bright_bp.SetAttribute("image_size_y", std::to_string(3));
    bright_bp.SetAttribute("role_name", "bright");

    carla::traffic_manager::ActorPtr carla_bright_actor = this->carla_world_->SpawnActor(bright_bp, this->carla_bright_tf_, this->carla_ego_vehicle_.get());

    this->carla_bright_ = boost::static_pointer_cast<carla::client::Sensor>(carla_bright_actor);
    
    this->carla_bright_->Listen([this](auto data) {
        boost::shared_ptr<carla::sensor::data::Image> image = boost::static_pointer_cast<carla::sensor::data::Image>(data);
        if (image == nullptr) return;
        boost::gil::bgra8c_view_t image_data = carla::image::ImageView::MakeView(*image);
        using pixel = decltype(image_data)::value_type;

        pixel raw_data[image_data.width() * image_data.height()];
        boost::gil::copy_pixels(image_data, boost::gil::interleaved_view(image_data.width(), image_data.height(), raw_data, image_data.width() * sizeof(pixel)));
        cv::Mat cv_image = cv::Mat(image->GetHeight(), image->GetWidth(), CV_8UC4, raw_data);
        cv::Mat cv_bright;
        cv::cvtColor(cv_image, cv_bright, CV_BGR2HLS_FULL);

        this->bright_deque_.push_back(static_cast<int>(cv::mean(cv_bright)[1]));
    });

    //  センサ設定ファイル読み込み
    this->attach_sensors(sensor_config_path);

    this->g29ffb_ = new G29FFB(event_path);

    cv::namedWindow(WINDOW_NAME, cv::WINDOW_FREERATIO);
    cv::setWindowProperty(WINDOW_NAME, cv::WND_PROP_FULLSCREEN, cv::WINDOW_NORMAL);
    cv::waitKey(1);

    this->manual_override_pub_ = this->nh_.advertise<std_msgs::Bool>("/carla/" + role_name + "/vehicle_control_manual_override", 1);
    this->timer_ = this->nh_.createTimer(ros::Duration(1.0 / hz), &CV_Manual_Control::timer_callback, this);
    this->js_timer_ = this->nh_.createTimer(ros::Duration(0.05), &CV_Manual_Control::js_timer_callback, this);
}

CV_Manual_Control::~CV_Manual_Control()
{
    for (; this->carla_sensors_.size() > 0;) {
        this->carla_sensors_.back()->Destroy();
        this->carla_sensors_.pop_back();
    }
    this->carla_bright_->Destroy();
    this->carla_camera_->Destroy();
    this->carla_ego_vehicle_->Destroy();
    cv::destroyAllWindows();
}

void CV_Manual_Control::attach_sensors(std::string json_path)
{
    std::ifstream ifs(json_path);
    if (ifs.is_open() == false) {
        std::cout << "no sensor_config file." << std::endl;
        return;
    }

    rapidjson::IStreamWrapper isw(ifs);
    rapidjson::Document doc;
    doc.ParseStream(isw);

    static const char* kTypeNames[] = { "Null", "False", "True", "Object", "Array", "String", "Number" };

    if (doc.HasMember("sensors")) {
        if (doc["sensors"].IsArray() == false) return;

        std::cout << "sensors" << std::endl;
        const rapidjson::Value &sensors = doc["sensors"].GetArray();

        for (auto& sensor : sensors.GetArray()) {
            if (sensor.HasMember("type") == false || sensor.HasMember("id") == false) continue;
            if (sensor.HasMember("x") == false || sensor.HasMember("y") == false || sensor.HasMember("z") == false) continue;
            if (sensor.HasMember("roll") == false || sensor.HasMember("pitch") == false || sensor.HasMember("yaw") == false) continue;

            if (sensor["type"].IsString() == false || sensor["id"].IsString() == false) continue;
            if (sensor["x"].IsNumber() == false || sensor["y"].IsNumber() == false || sensor["z"].IsNumber() == false) continue;
            if (sensor["roll"].IsNumber() == false || sensor["pitch"].IsNumber() == false || sensor["yaw"].IsNumber() == false) continue;

            carla::client::ActorBlueprint sensor_bp = *(this->carla_bplib_->Find(sensor["type"].GetString()));
            sensor_bp.SetAttribute("role_name", sensor["id"].GetString());

            carla::geom::Location sensor_location;
            if (sensor["x"].IsFloat() == true) sensor_location.x = sensor["x"].GetFloat();
            else sensor_location.x = static_cast<float_t>(sensor["x"].GetInt());
            if (sensor["y"].IsFloat() == true) sensor_location.y = sensor["y"].GetFloat();
            else sensor_location.y = static_cast<float_t>(sensor["y"].GetInt());
            if (sensor["z"].IsFloat() == true) sensor_location.z = sensor["z"].GetFloat();
            else sensor_location.z = static_cast<float_t>(sensor["z"].GetInt());

            carla::geom::Rotation sensor_rotation;
            if (sensor["roll"].IsFloat() == true) sensor_rotation.roll = sensor["roll"].GetFloat();
            else sensor_rotation.roll = static_cast<float_t>(sensor["roll"].GetInt());
            if (sensor["pitch"].IsFloat() == true) sensor_rotation.pitch = sensor["pitch"].GetFloat();
            else sensor_rotation.pitch = static_cast<float_t>(sensor["pitch"].GetInt());
            if (sensor["yaw"].IsFloat() == true) sensor_rotation.yaw = sensor["yaw"].GetFloat();
            else sensor_rotation.yaw = static_cast<float_t>(sensor["yaw"].GetInt());

            carla::geom::Transform sensor_transform(sensor_location, sensor_rotation);
            
            for (rapidjson::Value::ConstMemberIterator itr = sensor.MemberBegin(); itr != sensor.MemberEnd(); itr++) {
                std::string name = itr->name.GetString();
                if (name == "type" || name == "id" || name == "x" || name == "y" || name == "z" || name == "roll" || name == "pitch" || name == "yaw") continue;

                std::string value;
                if (itr->value.IsString()) value = itr->value.GetString();
                else if (itr->value.IsInt64()) value = std::to_string(itr->value.GetInt64());
                else if (itr->value.IsDouble()) value = std::to_string(itr->value.GetDouble());
                else if (itr->value.IsTrue()) value = "True";
                else if (itr->value.IsFalse()) value = "False";
                else continue;

                sensor_bp.SetAttribute(name, value);
            }

            this->carla_sensors_.push_back(this->carla_world_->SpawnActor(sensor_bp, sensor_transform, this->carla_ego_vehicle_.get()));
        }
    }
}

void CV_Manual_Control::load_img(std::string img_dir)
{
    double_t scale;
    for (int i = 0; i < 10; i ++) {
        this->cv_7seg_.push_back(cv::imread(img_dir + "/7seg_" + std::to_string(i) + ".png", cv::IMREAD_UNCHANGED));
    }
    this->cv_7seg_.push_back(cv::imread(img_dir + "/7seg_null.png", cv::IMREAD_UNCHANGED));
    for (int i = 0; i < this->cv_7seg_.size(); i++) {
        scale = static_cast<double_t>(this->cockpit_size_full_.height) / static_cast<double_t>(this->cv_7seg_[i].rows);
        cv::resize(this->cv_7seg_[i], this->cv_7seg_[i], cv::Size(
            static_cast<int>(static_cast<double_t>(this->cv_7seg_[i].cols) * scale),
            this->cockpit_size_full_.height
        ));
    }

    this->cv_gear_.push_back(cv::imread(img_dir + "/gear_d.png", cv::IMREAD_UNCHANGED));
    for (int i = 1; i <= 5; i ++) {
        this->cv_gear_.push_back(cv::imread(img_dir + "/gear_" + std::to_string(i) + ".png", cv::IMREAD_UNCHANGED));
    }
    for (int i = 0; i < this->cv_gear_.size(); i++) {
        scale = static_cast<double_t>(this->cockpit_size_full_.height) / static_cast<double_t>(this->cv_gear_[i].rows);
        cv::resize(this->cv_gear_[i], this->cv_gear_[i], cv::Size(
            static_cast<int>(static_cast<double_t>(this->cv_gear_[i].cols) * scale),
            this->cockpit_size_full_.height
        ));
    }

    this->cv_reverse_ = cv::imread(img_dir + "/gear_r.png", cv::IMREAD_UNCHANGED);
    scale = static_cast<double_t>(this->cockpit_size_full_.height) / static_cast<double_t>(this->cv_reverse_.rows);
    cv::resize(this->cv_reverse_, this->cv_reverse_, cv::Size(
        static_cast<int>(static_cast<double_t>(this->cv_reverse_.cols) * scale),
        this->cockpit_size_full_.height
    ));

    this->cv_autopilot_ = cv::imread(img_dir + "/gear_a.png", cv::IMREAD_UNCHANGED);
    scale = static_cast<double_t>(this->cockpit_size_full_.height) / static_cast<double_t>(this->cv_autopilot_.rows);
    cv::resize(this->cv_autopilot_, this->cv_autopilot_, cv::Size(
        static_cast<int>(static_cast<double_t>(this->cv_autopilot_.cols) * scale),
        this->cockpit_size_full_.height
    ));

    this->cv_kmh_ = cv::imread(img_dir + "/kmh.png", cv::IMREAD_UNCHANGED);
    scale = static_cast<double_t>(this->cockpit_size_half_.height) / static_cast<double_t>(this->cv_kmh_.rows);
    cv::resize(this->cv_kmh_, this->cv_kmh_, cv::Size(
        static_cast<int>(static_cast<double_t>(this->cv_kmh_.cols) * scale),
        this->cockpit_size_half_.height
    ));

    this->cv_steer_ = cv::imread(img_dir + "/steer.png", cv::IMREAD_UNCHANGED);
    scale = static_cast<double_t>(this->cockpit_size_full_.height) / static_cast<double_t>(this->cv_steer_.rows) * 0.8;
    cv::resize(this->cv_steer_, this->cv_steer_, cv::Size(
        static_cast<int>(static_cast<double_t>(this->cv_steer_.cols) * scale),
        this->cockpit_size_full_.height * 4 / 5
    ));

    this->cv_handbrake_ = cv::imread(img_dir + "/handbrake.png", cv::IMREAD_UNCHANGED);
    scale = static_cast<double_t>(this->cockpit_size_half_.height) / static_cast<double_t>(this->cv_handbrake_.rows);
    cv::resize(this->cv_handbrake_, this->cv_handbrake_, cv::Size(
        static_cast<int>(static_cast<double_t>(this->cv_handbrake_.cols) * scale),
        this->cockpit_size_half_.height
    ));

    this->cv_sig_frame_ = cv::imread(img_dir + "/sig_frame.png", cv::IMREAD_UNCHANGED);
    scale = static_cast<double_t>(this->cockpit_size_full_.height) / static_cast<double_t>(this->cv_sig_frame_.rows) * 0.8;
    cv::resize(this->cv_sig_frame_, this->cv_sig_frame_, cv::Size(
        static_cast<int>(static_cast<double_t>(this->cv_sig_frame_.cols) * scale),
        this->cockpit_size_full_.height * 4 / 5
    ));

    this->cv_sig_.push_back(cv::imread(img_dir + "/sig_red.png", cv::IMREAD_UNCHANGED));
    this->cv_sig_.push_back(cv::imread(img_dir + "/sig_yellow.png", cv::IMREAD_UNCHANGED));
    this->cv_sig_.push_back(cv::imread(img_dir + "/sig_green.png", cv::IMREAD_UNCHANGED));
    this->cv_sig_.push_back(cv::imread(img_dir + "/sig_null.png", cv::IMREAD_UNCHANGED));
    for (int i = 0; i < this->cv_sig_.size(); i++) {
        cv::resize(this->cv_sig_[i], this->cv_sig_[i], cv::Size(
            this->cockpit_size_full_.height * 2 / 3,
            this->cockpit_size_full_.height * 2 / 3
        ));
    }
}

int CV_Manual_Control::get_brightness(cv::Mat &src)
{
    cv::Mat dst_hls;
    cv::cvtColor(src, dst_hls, CV_BGR2HLS_FULL);
    return static_cast<int>(cv::mean(dst_hls)[1]);
}

void CV_Manual_Control::js_timer_callback(const ros::TimerEvent &e)
{
    carla::geom::Vector3D velocity = this->carla_ego_vehicle_->GetVelocity();
    carla::client::Vehicle::Control control = this->carla_ego_vehicle_->GetControl();

    g29ffb_->speed_.push_back(roundf(sqrtf(powf32(velocity.x, 2.0f) + powf32(velocity.y, 2.0f) + powf32(velocity.z, 2.0f)) * 3.6f));
    g29ffb_->target_.push_back(control.steer);

    g29_btn_axes cmd = this->g29ffb_->cmd_deque_.back();
    control.steer = cmd.steer;
    control.brake = cmd.brake;
    control.throttle = cmd.throttle;

    this->carla_ego_vehicle_->ApplyControl(control);

    while (this->g29ffb_->cmd_deque_.size() > 1) this->g29ffb_->cmd_deque_.pop_front();
}

void CV_Manual_Control::timer_callback(const ros::TimerEvent &e)
{
    if (this->img_deque_.size() < 1 || this->bright_deque_.size() < 1) {

    }
    else {
        cv::Mat cv_cockpit = cv::Mat(this->cockpit_size_full_, CV_8UC4, cv::Scalar(0, 0, 0, 255));

        carla::geom::Vector3D velocity = this->carla_ego_vehicle_->GetVelocity();
        carla::client::Vehicle::Control vehicle_state = this->carla_ego_vehicle_->GetControl();
        carla::client::Vehicle::PhysicsControl vehicle_info = this->carla_ego_vehicle_->GetPhysicsControl();
        u_int32_t light_state = static_cast<u_int32_t>(this->carla_ego_vehicle_->GetLightState());
        u_int32_t new_light_state = light_state;

        this->velocity_kmh_ = static_cast<int>(roundf(sqrtf(powf32(velocity.x, 2.0f) + powf32(velocity.y, 2.0f) + powf32(velocity.z, 2.0f)) * 3.6f));

        int brightness = std::accumulate(this->bright_deque_.begin(), this->bright_deque_.end(), 0) / this->bright_deque_.size();

        if (brightness < AUTOLIGHT_POSITION_TH) new_light_state |= static_cast<u_int32_t>(carla::client::Vehicle::LightState::Position);
        else new_light_state &= ~(static_cast<u_int32_t>(carla::client::Vehicle::LightState::Position));

        if (brightness < AUTOLIGHT_BEAM_TH) new_light_state |= static_cast<u_int32_t>(carla::client::Vehicle::LightState::LowBeam);
        else new_light_state &= ~(static_cast<u_int32_t>(carla::client::Vehicle::LightState::LowBeam) | static_cast<u_int32_t>(carla::client::Vehicle::LightState::HighBeam));

        if (vehicle_state.brake > 0.0f) new_light_state |= static_cast<u_int32_t>(carla::client::Vehicle::LightState::Brake);
        else new_light_state &= ~(static_cast<u_int32_t>(carla::client::Vehicle::LightState::Brake));
        
        if (vehicle_state.reverse == true) new_light_state |= static_cast<u_int32_t>(carla::client::Vehicle::LightState::Reverse);
        else new_light_state &= ~(static_cast<u_int32_t>(carla::client::Vehicle::LightState::Reverse));
        
        if (new_light_state != light_state) this->carla_ego_vehicle_->SetLightState(static_cast<carla::client::Vehicle::LightState>(new_light_state));

        this->draw_velocity(cv_cockpit);

        if (vehicle_state.hand_brake == true) {
            this->alpha_blend(
                this->cv_handbrake_,
                cv_cockpit,
                cv::Point(this->cockpit_size_full_.width * 5 / 8, 0)
            );
        }

        this->draw_traffic_light(cv_cockpit);

        this->draw_gear_status(vehicle_state.reverse, vehicle_state.gear, vehicle_info.use_gear_autobox, cv_cockpit);

        if (this->enable_autopilot_ == true) {
            this->alpha_blend(
                this->cv_autopilot_,
                cv_cockpit,
                cv::Point(this->cockpit_size_full_.width * 3 / 8 - this->cv_autopilot_.cols, 0)
            );
        }

        if (this->manual_override_ == true) {
            this->alpha_blend(
                this->cv_steer_,
                cv_cockpit,
                cv::Point(this->cv_steer_.cols / 8, this->cv_steer_.rows / 8)
            );
        }

        cv::Mat cv_display;
        cv::vconcat(this->img_deque_.front(), cv_cockpit, cv_display);

        cv::imshow(WINDOW_NAME, cv_display);
    }

    int key = cv::waitKey(1);

    if (key == static_cast<int>('q')) {
        ros::shutdown();
    }
    else if (key == static_cast<int>('b')) {
        this->manual_override_ = !(this->manual_override_);
        std_msgs::Bool override_msg, enable_autopilot_msg;
        override_msg.data = this->manual_override_;
        this->manual_override_pub_.publish(override_msg);
        this->g29ffb_->manual_control_ = this->manual_override_;
    }
    else if (key == static_cast<int>('v')) {
        this->view_id_ += 2;
        this->view_id_ %= 4;
        this->change_view(this->view_id_);
    }
    else if (key == static_cast<int>('p')) {
        this->enable_autopilot_ = !(this->enable_autopilot_);
        this->carla_ego_vehicle_->SetAutopilot(this->enable_autopilot_);
    }
    else if (key == CV_KEY_F11) {
        if (cv::getWindowProperty(WINDOW_NAME, cv::WND_PROP_FULLSCREEN) == cv::WINDOW_FULLSCREEN) {
            cv::setWindowProperty(WINDOW_NAME, cv::WND_PROP_FULLSCREEN, cv::WINDOW_NORMAL);
        }
        else {
            cv::setWindowProperty(WINDOW_NAME, cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
        }
    }

    for (; this->img_deque_.size() > 1;) {
        this->img_deque_.pop_front();
    }
    for (; this->bright_deque_.size() > 40;) {
        this->bright_deque_.pop_front();
    }
}

void CV_Manual_Control::draw_gear_status(bool reverse, int32_t gear, bool use_gear_autobox, cv::Mat &bg)
{
    cv::Mat gear_info;
    if (reverse == true) {
        gear_info = this->cv_reverse_;

        if (this->view_id_ % 2 == 0) {
            this->view_id_ += 1;
            this->view_id_ %= 4;
            this->change_view(this->view_id_);
        }
    }
    else {
        if (this->view_id_ % 2 == 1) {
            this->view_id_ -= 1;
            this->view_id_ %= 4;
            this->change_view(this->view_id_);
        }
        if (use_gear_autobox == true) gear_info = this->cv_gear_[0];
        else {
            int gear_num = std::min(std::max(1, gear), 5);
            gear_info = this->cv_gear_[gear_num];
        }
    }
    this->alpha_blend(
        gear_info,
        bg,
        cv::Point(this->screen_size_.width * 3 / 8, 0)
    );
}

void CV_Manual_Control::draw_velocity(cv::Mat &bg)
{
    int velocity_digit[3];
    velocity_digit[0] = this->velocity_kmh_ / 100;
    if (velocity_digit[0] == 0 && this->velocity_kmh_ < 100) velocity_digit[0] = 10;
    velocity_digit[2] = this->velocity_kmh_ % 100;
    velocity_digit[1] = velocity_digit[2] / 10;
    if (velocity_digit[1] == 0 && this->velocity_kmh_ < 10) velocity_digit[1] = 10;
    velocity_digit[2] %= 10;

    for (size_t i = 0; i < 3; i++) {
        this->alpha_blend(
            this->cv_7seg_[velocity_digit[i]], bg,
            cv::Point((this->cockpit_size_full_.width - this->cv_7seg_[0].cols * 3) / 2 + this->cv_7seg_[0].cols * i, 0)
        );
    }

    this->alpha_blend(
        this->cv_kmh_, bg,
        cv::Point(
            (this->cockpit_size_full_.width + this->cv_7seg_[0].cols * 3) / 2,
            this->cockpit_size_full_.height - this->cv_kmh_.rows
        )
    );
}

void CV_Manual_Control::draw_traffic_light(cv::Mat &bg)
{
    u_int8_t traffic_light_state = static_cast<u_int8_t>(this->carla_ego_vehicle_->GetTrafficLightState());

    this->alpha_blend(this->cv_sig_frame_, bg, cv::Point(this->cockpit_size_full_.width / 8, this->cv_sig_frame_.rows / 8));

    for (int i = 0; i < 3; i++) {
        cv::Point point = cv::Point(
            this->cockpit_size_full_.width / 8 + this->cv_sig_frame_.cols * (1 + i * 2) / 6 - this->cv_sig_[0].cols / 2,
            (this->cockpit_size_full_.height - this->cv_sig_[0].rows) / 2
        );
        size_t sig_index;
        if (i == 0 && traffic_light_state == static_cast<u_int8_t>(carla::rpc::TrafficLightState::Green)) sig_index = SIG_GREEN;
        else if (i == 1 && traffic_light_state == static_cast<u_int8_t>(carla::rpc::TrafficLightState::Yellow)) sig_index = SIG_YELLOW;
        else if (i == 2 && traffic_light_state == static_cast<u_int8_t>(carla::rpc::TrafficLightState::Red)) sig_index = SIG_RED;
        else sig_index = SIG_NULL;
        this->alpha_blend(this->cv_sig_[sig_index], bg, point);
    }
}

void CV_Manual_Control::change_view(int view_id)
{
    if (this->carla_camera_ != nullptr) {
        this->carla_camera_->Destroy();
    }
    carla::client::ActorBlueprint camera_bp = *this->carla_bplib_->Find("sensor.camera.rgb");
    camera_bp.SetAttribute("image_size_x", std::to_string(this->screen_size_.width));
    camera_bp.SetAttribute("image_size_y", std::to_string(this->screen_size_.height));
    camera_bp.SetAttribute("role_name", "view");

    auto carla_camera_actor = this->carla_world_->SpawnActor(camera_bp, this->carla_camtf_[view_id], this->carla_ego_vehicle_.get());

    this->carla_camera_ = boost::static_pointer_cast<carla::client::Sensor>(carla_camera_actor);
    
    this->carla_camera_->Listen([this](auto data) {
        boost::shared_ptr<carla::sensor::data::Image> image = boost::static_pointer_cast<carla::sensor::data::Image>(data);
        if (image == nullptr) return;
        boost::gil::bgra8c_view_t image_data = carla::image::ImageView::MakeView(*image);
        using pixel = decltype(image_data)::value_type;

        pixel raw_data[image_data.width() * image_data.height()];
        boost::gil::copy_pixels(image_data, boost::gil::interleaved_view(image_data.width(), image_data.height(), raw_data, image_data.width() * sizeof(pixel)));
        cv::Mat cv_image = cv::Mat(image->GetHeight(), image->GetWidth(), CV_8UC4, raw_data);

        this->img_deque_.push_back(cv_image);
    });
}

void CV_Manual_Control::alpha_blend(cv::Mat &fg, cv::Mat &bg, cv::Point up_left, double_t alpha)
{
    cv::Size roi_size;
    roi_size.width = (up_left.x + fg.cols > bg.cols)? (bg.cols - up_left.x) : (fg.cols);
    roi_size.height = (up_left.y + fg.rows > bg.rows)? (bg.rows - up_left.y) : (fg.rows);

    roi_size.width = (up_left.x < 0)? (roi_size.width + up_left.x) : roi_size.width;
    roi_size.height = (up_left.y < 0)? (roi_size.height + up_left.y) : roi_size.height;

    cv::Rect s_roi_rect(
        cv::Point(
            (up_left.x < 0)? (-up_left.x) : 0,
            (up_left.y < 0)? (-up_left.y) : 0
        ),
        roi_size
    );
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
    cv::Mat s_alpha;
    cv::merge(s_aaa, 3, s_alpha);

    cv::Mat s_bgr_f, s_alpha_f, d_bgr_f;
    s_bgr.convertTo(s_bgr_f, CV_32FC3);
    d_bgr.convertTo(d_bgr_f, CV_32FC3);
    s_alpha.convertTo(s_alpha_f, CV_32FC3, 1.0f / 255.0f);
    
    cv::Mat tmp_f = d_bgr_f + (s_bgr_f - d_bgr_f).mul(s_alpha_f, alpha);
    cv::Mat tmp_i;
    tmp_f.convertTo(tmp_i, CV_8UC3);
    
    cv::cvtColor(tmp_i, d_roi, CV_BGR2BGRA);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cv_manual_control");
    ros::NodeHandle pnh("~");

    std::string role_name = "ego_vehicle";
    std::string img_dir = "img";
    double_t hz = 20.0;
    int width = 1920;
    int height = 1080;
    std::string sensor_config_path = "";
    std::string spawn_point = "";
    std::string vehicle_filter = "vehicle.*";
    std::string event_path = "/dev/input/event-g29";
    std::string host = "localhost";
    int port = 2000u;

    pnh.getParam("role_name", role_name);
    pnh.getParam("img_dir", img_dir);
    pnh.getParam("hz", hz);
    pnh.getParam("width", width);
    pnh.getParam("height", height);
    pnh.getParam("sensor_config_path", sensor_config_path);
    pnh.getParam("spawn_point", spawn_point);
    pnh.getParam("vehicle_filter", vehicle_filter);
    pnh.getParam("event_path", event_path);
    pnh.getParam("host", host);
    pnh.getParam("port", port);

    try {
        CV_Manual_Control cmc(
            img_dir, width, height,
            sensor_config_path, spawn_point, vehicle_filter,
            role_name, event_path, hz,
            host, static_cast<uint16_t>(port)
        );
        ros::spin();
    }
    catch (const carla::client::TimeoutException &e) {
        std::cout << std::endl << "Exception: " << e.what() << std::endl;
        return 1;
    }
    catch (const std::exception &e) {
        std::cout << std::endl << "Exception: " << e.what() << std::endl;
        return -1;
    }
}