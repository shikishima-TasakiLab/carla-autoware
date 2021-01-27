#include <linux/input.h>
#include <sys/ioctl.h>
#include <iostream>
#include <unistd.h>
#include <memory.h>
#include <fcntl.h>
#include <string>
#include <thread>
#include <deque>
#include <mutex>
#include <cmath>

#define GEAR_COUNT 6

#define G29_ABS_CODE_STEER        ABS_X
#define G29_ABS_CODE_CLUTCH       ABS_Y
#define G29_ABS_CODE_THROTTLE     ABS_Z
#define G29_ABS_CODE_BRAKE        ABS_RZ
#define G29_ABS_CODE_DPAD_X       ABS_HAT0X
#define G29_ABS_CODE_DPAD_Y       ABS_HAT0Y

#define G29_BTN_CODE_CROSS        BTN_TRIGGER
#define G29_BTN_CODE_SQUARE       BTN_THUMB
#define G29_BTN_CODE_CIRCLE       BTN_THUMB2
#define G29_BTN_CODE_TRIANGLE     BTN_TOP
#define G29_BTN_CODE_PADDLE_PLUS  BTN_TOP2
#define G29_BTN_CODE_PADDLE_MINUS BTN_PINKIE
#define G29_BTN_CODE_R2           BTN_BASE
#define G29_BTN_CODE_L2           BTN_BASE2
#define G29_BTN_CODE_SHARE        BTN_BASE3
#define G29_BTN_CODE_OPTIONS      BTN_BASE4
#define G29_BTN_CODE_R3           BTN_BASE5
#define G29_BTN_CODE_L3           BTN_BASE6
#define G29_BTN_CODE_GEAR_1       300u
#define G29_BTN_CODE_GEAR_2       301u
#define G29_BTN_CODE_GEAR_3       302u
#define G29_BTN_CODE_GEAR_4       BTN_DEAD
#define G29_BTN_CODE_GEAR_5       BTN_TRIGGER_HAPPY1
#define G29_BTN_CODE_GEAR_6       BTN_TRIGGER_HAPPY2
#define G29_BTN_CODE_GEAR_R       BTN_TRIGGER_HAPPY3
#define G29_BTN_CODE_PLUS         BTN_TRIGGER_HAPPY4
#define G29_BTN_CODE_MINUS        BTN_TRIGGER_HAPPY5
#define G29_BTN_CODE_DIAL_CW      BTN_TRIGGER_HAPPY6
#define G29_BTN_CODE_DIAL_CCW     BTN_TRIGGER_HAPPY7
#define G29_BTN_CODE_ENTER        BTN_TRIGGER_HAPPY8
#define G29_BTN_CODE_PLAYSTATION  BTN_TRIGGER_HAPPY9

struct g29_btn_axes
{
    float steer = 0.0f;         //  -1.0f <= steer <= 1.0f
    float clutch = 0.0f;        //  0.0f <= clutch <= 1.0f
    float throttle = 0.0f;      //  0.0f <= throttle <= 1.0f
    float brake = 0.0f;         //  0.0f <= brake <= 1.0f
    bool d_pad_left = false;    //  Press: true, Release: false
    bool d_pad_right = false;   //  Press: true, Release: false
    bool d_pad_up = false;      //  Press: true, Release: false
    bool d_pad_down = false;    //  Press: true, Release: false
    bool cross = false;         //  Press: true, Release: false
    bool square = false;        //  Press: true, Release: false
    bool circle = false;        //  Press: true, Release: false
    bool triangle = false;      //  Press: true, Release: false
    bool paddle_plus = false;   //  Press: true, Release: false
    bool paddle_minus = false;  //  Press: true, Release: false
    bool r2 = false;            //  Press: true, Release: false
    bool l2 = false;            //  Press: true, Release: false
    bool share = false;         //  Press: true, Release: false
    bool options = false;       //  Press: true, Release: false
    bool r3 = false;            //  Press: true, Release: false
    bool l3 = false;            //  Press: true, Release: false
    char gear = 0;              //  N:0, 1:1, 2:2, 3:3, 4:4, 5:5, 6:6, R:-1
    bool plus = false;          //  Press: true, Release: false
    bool minus = false;         //  Press: true, Release: false
    bool dial_cw = false;       //  Turn Clockwise: false->true->false
    bool dial_ccw = false;      //  Turn Counter-Clockwise: false->true->false
    bool enter = false;         //  Press: true, Release: false
    bool play_station = false;  //  Press: true, Release: false
};

struct pid_params {
    float kp = 1.0f;
    float ki = 0.0f;
    float kd = 0.0f;
    float force_min = 0.2f;
    float force_max = 1.0f;
};

class G29FFB
{
public:
    G29FFB(std::string event_path, pid_params pid = {1.0f, 0.0f, 0.0f, 0.2f, 1.0f}, bool manual_control = false);
    ~G29FFB();

    void set_manual_control(bool manual_control);
    void set_speed(float speed);
    void set_target(float target);
    g29_btn_axes get_cmd();

private:
    std::mutex mtx_;

    int32_t event_handle_;
    int32_t axis_max_;
    int32_t axis_min_;
    struct ff_effect effect_spring_;
    struct ff_effect effect_damper_;
    struct ff_effect effect_constant_;

    bool manual_control_ = false;
    float target_ = 0.0f;
    float speed_ = 0.0f;

    pid_params pid_;

    std::thread *loop_thread;
    bool terminate_ = false;
    g29_btn_axes cmd_;

    int32_t initDevice(std::string event_path);
    int32_t testBit(int32_t bit, u_int8_t *array);
    void loop();
};

G29FFB::G29FFB(std::string event_path, pid_params pid, bool manual_control)
{
    this->manual_control_ = manual_control;
    this->pid_ = pid;
    if (this->initDevice(event_path) != EXIT_SUCCESS) exit(EXIT_FAILURE);
    this->loop_thread = new std::thread(&G29FFB::loop, this);
    this->loop_thread->detach();
}

G29FFB::~G29FFB()
{
    this->terminate_ = true;
    this->loop_thread->join();
}

void G29FFB::set_manual_control(bool manual_control)
{
    std::lock_guard<std::mutex> lock(this->mtx_);
    this->manual_control_ = manual_control;
}

void G29FFB::set_speed(float speed)
{
    std::lock_guard<std::mutex> lock(this->mtx_);
    this->speed_ = speed;
}

void G29FFB::set_target(float target)
{
    std::lock_guard<std::mutex> lock(this->mtx_);
    this->target_ = target;
}

g29_btn_axes G29FFB::get_cmd()
{
    return this->cmd_;
}

int32_t G29FFB::initDevice(std::string event_path)
{
    u_int8_t key_bits[1 + KEY_MAX / 8 / sizeof(u_int8_t)];
    u_int8_t abs_bits[1 + ABS_MAX / 8 / sizeof(u_int8_t)];
    u_int8_t ff_bits[1 + FF_MAX / 8 / sizeof(u_int8_t)];

    struct input_event i_event;
    struct input_absinfo i_absinfo;

    this->event_handle_ = open(event_path.c_str(), O_RDWR | O_NONBLOCK);
    if (this->event_handle_ < 0) {
        std::cout << "ERROR: " << event_path << " : cannot open device" << std::endl;
        return EXIT_FAILURE;
    }

    memset(abs_bits, 0, sizeof(abs_bits));
    if (ioctl(this->event_handle_, EVIOCGBIT(EV_ABS, sizeof(abs_bits)), abs_bits) < 0) {
        std::cout << "ERROR: " << event_path << " : cannot get abs bits" << std::endl;
        return EXIT_FAILURE;
    }

    memset(ff_bits, 0, sizeof(ff_bits));
    if (ioctl(this->event_handle_, EVIOCGBIT(EV_FF, sizeof(ff_bits)), ff_bits) < 0) {
        std::cout << "ERROR: " << event_path << " : cannot get ff bits" << std::endl;
        return EXIT_FAILURE;
    }

    if (ioctl(this->event_handle_, EVIOCGABS(ABS_X), &i_absinfo) < 0) {
        std::cout << "ERROR: " << event_path << " : cannot get axis range" << std::endl;
        return EXIT_FAILURE;
    }
    this->axis_max_ = i_absinfo.maximum;
    this->axis_min_ = i_absinfo.minimum;
    if (this->axis_min_ >= this->axis_max_) {
        std::cout << "ERROR: " << event_path << " : axis range has bad value" << std::endl;
        return EXIT_FAILURE;
    }

    if (!testBit(FF_CONSTANT, ff_bits)) {
        std::cout << "ERROR: " << event_path << " : FFB is not supported" << std::endl;
        return EXIT_FAILURE;
    }

    memset(&i_event, 0, sizeof(i_event));
    i_event.type = EV_FF;
    i_event.code = FF_AUTOCENTER;
    i_event.value = 0;
    if (write(this->event_handle_, &i_event, sizeof(i_event)) != sizeof(i_event)) {
        std::cout << "ERROR: " << event_path << " : failed to disable auto centering" << std::endl;
        return EXIT_FAILURE;
    }

    memset(&this->effect_damper_, 0, sizeof(this->effect_damper_));
    this->effect_damper_.type = FF_DAMPER;
    this->effect_damper_.id = -1;
    this->effect_damper_.u.condition[0].right_saturation = 0;
    this->effect_damper_.u.condition[0].left_saturation = 0;
    this->effect_damper_.u.condition[0].right_coeff = 0x6000;
    this->effect_damper_.u.condition[0].left_coeff = 0x6000;
    this->effect_damper_.u.condition[0].center = 0;
    this->effect_damper_.u.condition[0].deadband = 0;
    this->effect_damper_.u.condition[1] = this->effect_damper_.u.condition[0];
    this->effect_damper_.trigger.button = 0;
    this->effect_damper_.trigger.interval = 0;
    this->effect_damper_.replay.length = 0;
    this->effect_damper_.replay.delay = 0;
    if (ioctl(this->event_handle_, EVIOCSFF, &this->effect_damper_) < 0) {
        std::cout << "ERROR: " << event_path << " : failed to upload effect" << std::endl;
        return EXIT_FAILURE;
    }

    memset(&i_event, 0, sizeof(i_event));
    i_event.type = EV_FF;
    i_event.code = this->effect_damper_.id;
    i_event.value = 1;
    if (write(this->event_handle_, &i_event, sizeof(i_event)) != sizeof(i_event)){
        std::cout << "ERROR: " << event_path << " : failed to start event" << std::endl;
        return EXIT_FAILURE;
    }

    memset(&this->effect_spring_, 0, sizeof(this->effect_spring_));
    this->effect_spring_.type = FF_SPRING;
    this->effect_spring_.id = -1;
    this->effect_spring_.u.condition[0].right_saturation = 0;
    this->effect_spring_.u.condition[0].left_saturation = 0;
    this->effect_spring_.u.condition[0].right_coeff = 0x4000;
    this->effect_spring_.u.condition[0].left_coeff = 0x4000;
    this->effect_spring_.u.condition[0].center = 0;
    this->effect_spring_.u.condition[0].deadband = 0;
    this->effect_spring_.u.condition[1] = this->effect_spring_.u.condition[0];
    this->effect_spring_.trigger.button = 0;
    this->effect_spring_.trigger.interval = 0;
    this->effect_spring_.replay.length = 0;
    this->effect_spring_.replay.delay = 0;
    if (ioctl(this->event_handle_, EVIOCSFF, &this->effect_spring_) < 0) {
        std::cout << "ERROR: " << event_path << " : failed to upload effect" << std::endl;
        return EXIT_FAILURE;
    }

    memset(&i_event, 0, sizeof(i_event));
    i_event.type = EV_FF;
    i_event.code = this->effect_spring_.id;
    i_event.value = 1;
    if (write(this->event_handle_, &i_event, sizeof(i_event)) != sizeof(i_event)){
        std::cout << "ERROR: " << event_path << " : failed to start event" << std::endl;
        return EXIT_FAILURE;
    }

    memset(&this->effect_constant_, 0, sizeof(this->effect_constant_));
    this->effect_constant_.type = FF_CONSTANT;
    this->effect_constant_.id = -1;
    this->effect_constant_.u.constant.level = 0;
    this->effect_constant_.u.constant.envelope.attack_length = 0;
    this->effect_constant_.u.constant.envelope.attack_level = 0;
    this->effect_constant_.u.constant.envelope.fade_length = 0;
    this->effect_constant_.u.constant.envelope.fade_level = 0;
    this->effect_constant_.direction = 0xc000;
    this->effect_constant_.trigger.button = 0;
    this->effect_constant_.trigger.interval = 0;
    this->effect_constant_.replay.delay = 0;
    this->effect_constant_.replay.length = 0;
    if (ioctl(this->event_handle_, EVIOCSFF, &this->effect_constant_) < 0) {
        std::cout << "ERROR: " << event_path << " : failed to upload effect" << std::endl;
        return EXIT_FAILURE;
    }

    memset(&i_event, 0, sizeof(i_event));
    i_event.type = EV_FF;
    i_event.code = this->effect_constant_.id;
    i_event.value = 1;
    if (write(this->event_handle_, &i_event, sizeof(i_event)) != sizeof(i_event)){
        std::cout << "ERROR: " << event_path << " : failed to start event" << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}

void G29FFB::loop()
{
    struct input_event i_event;
    float tmp_target = 2.0f;
    float tmp_speed = -1.0f;
    float diff = 0.0f;
    float diff_i = 0.0f;
    while(!this->terminate_) {
        {
            std::lock_guard<std::mutex> lock(this->mtx_);
            
            while (read(this->event_handle_, &i_event, sizeof(i_event)) == sizeof(i_event)) {
                if (i_event.type == EV_ABS) {
                    if (i_event.code == G29_ABS_CODE_STEER)
                        this->cmd_.steer = static_cast<float>(i_event.value) / static_cast<float>(UINT16_MAX) * 2.0f - 1.0f;
                    else if (i_event.code == G29_ABS_CODE_CLUTCH)
                        this->cmd_.clutch = 1.0f - static_cast<float>(i_event.value) / 255.0f;
                    else if (i_event.code == G29_ABS_CODE_THROTTLE)
                        this->cmd_.throttle = 1.0f - static_cast<float>(i_event.value) / 255.0f;
                    else if (i_event.code == G29_ABS_CODE_BRAKE)
                        this->cmd_.brake = 1.0f - static_cast<float>(i_event.value) / 255.0f;
                    else if (i_event.code == G29_ABS_CODE_DPAD_X) {
                        if (i_event.value > 0) {
                            this->cmd_.d_pad_right = true;
                            this->cmd_.d_pad_left = false;
                        }
                        else if (i_event.value < 0) {
                            this->cmd_.d_pad_right = false;
                            this->cmd_.d_pad_left = true;
                        }
                        else {
                            this->cmd_.d_pad_right = false;
                            this->cmd_.d_pad_left = false;
                        }
                    }
                    else if (i_event.code == G29_ABS_CODE_DPAD_Y) {
                        if (i_event.value > 0) {
                            this->cmd_.d_pad_down = true;
                            this->cmd_.d_pad_up = false;
                        }
                        else if (i_event.value < 0) {
                            this->cmd_.d_pad_down = false;
                            this->cmd_.d_pad_up = true;
                        }
                        else {
                            this->cmd_.d_pad_down = false;
                            this->cmd_.d_pad_up = false;
                        }
                    }
                }
                else if (i_event.type == EV_KEY) {
                    if (i_event.value > 0) {
                        if (i_event.code == G29_BTN_CODE_CROSS) this->cmd_.cross = true;
                        else if (i_event.code == G29_BTN_CODE_SQUARE) this->cmd_.square = true;
                        else if (i_event.code == G29_BTN_CODE_CIRCLE) this->cmd_.circle = true;
                        else if (i_event.code == G29_BTN_CODE_TRIANGLE) this->cmd_.triangle = true;
                        else if (i_event.code == G29_BTN_CODE_PADDLE_PLUS) this->cmd_.paddle_plus = true;
                        else if (i_event.code == G29_BTN_CODE_PADDLE_MINUS) this->cmd_.paddle_minus = true;
                        else if (i_event.code == G29_BTN_CODE_R2) this->cmd_.r2 = true;
                        else if (i_event.code == G29_BTN_CODE_L2) this->cmd_.l2 = true;
                        else if (i_event.code == G29_BTN_CODE_SHARE) this->cmd_.share = true;
                        else if (i_event.code == G29_BTN_CODE_OPTIONS) this->cmd_.options = true;
                        else if (i_event.code == G29_BTN_CODE_R3) this->cmd_.r3 = true;
                        else if (i_event.code == G29_BTN_CODE_L3) this->cmd_.l3 = true;
                        else if (i_event.code == G29_BTN_CODE_PLUS) this->cmd_.plus = true;
                        else if (i_event.code == G29_BTN_CODE_MINUS) this->cmd_.minus = true;
                        else if (i_event.code == G29_BTN_CODE_DIAL_CW) this->cmd_.dial_cw = true;
                        else if (i_event.code == G29_BTN_CODE_DIAL_CCW) this->cmd_.dial_ccw = true;
                        else if (i_event.code == G29_BTN_CODE_ENTER) this->cmd_.enter = true;
                        else if (i_event.code == G29_BTN_CODE_PLAYSTATION) this->cmd_.play_station = true;
                        else if (i_event.code == G29_BTN_CODE_GEAR_1) this->cmd_.gear = 1;
                        else if (i_event.code == G29_BTN_CODE_GEAR_2) this->cmd_.gear = 2;
                        else if (i_event.code == G29_BTN_CODE_GEAR_3) this->cmd_.gear = 3;
                        else if (i_event.code == G29_BTN_CODE_GEAR_4) this->cmd_.gear = 4;
                        else if (i_event.code == G29_BTN_CODE_GEAR_5) this->cmd_.gear = 5;
                        else if (i_event.code == G29_BTN_CODE_GEAR_6) this->cmd_.gear = 6;
                        else if (i_event.code == G29_BTN_CODE_GEAR_R) this->cmd_.gear = -1;
                    }
                    else {
                        if (i_event.code == G29_BTN_CODE_CROSS) this->cmd_.cross = false;
                        else if (i_event.code == G29_BTN_CODE_SQUARE) this->cmd_.square = false;
                        else if (i_event.code == G29_BTN_CODE_CIRCLE) this->cmd_.circle = false;
                        else if (i_event.code == G29_BTN_CODE_TRIANGLE) this->cmd_.triangle = false;
                        else if (i_event.code == G29_BTN_CODE_PADDLE_PLUS) this->cmd_.paddle_plus = false;
                        else if (i_event.code == G29_BTN_CODE_PADDLE_MINUS) this->cmd_.paddle_minus = false;
                        else if (i_event.code == G29_BTN_CODE_R2) this->cmd_.r2 = false;
                        else if (i_event.code == G29_BTN_CODE_L2) this->cmd_.l2 = false;
                        else if (i_event.code == G29_BTN_CODE_SHARE) this->cmd_.share = false;
                        else if (i_event.code == G29_BTN_CODE_OPTIONS) this->cmd_.options = false;
                        else if (i_event.code == G29_BTN_CODE_R3) this->cmd_.r3 = false;
                        else if (i_event.code == G29_BTN_CODE_L3) this->cmd_.l3 = false;
                        else if (i_event.code == G29_BTN_CODE_PLUS) this->cmd_.plus = false;
                        else if (i_event.code == G29_BTN_CODE_MINUS) this->cmd_.minus = false;
                        else if (i_event.code == G29_BTN_CODE_DIAL_CW) this->cmd_.dial_cw = false;
                        else if (i_event.code == G29_BTN_CODE_DIAL_CCW) this->cmd_.dial_ccw = false;
                        else if (i_event.code == G29_BTN_CODE_ENTER) this->cmd_.enter = false;
                        else if (i_event.code == G29_BTN_CODE_PLAYSTATION) this->cmd_.play_station = false;
                        else if (i_event.code == G29_BTN_CODE_GEAR_1 && this->cmd_.gear == 1) this->cmd_.gear = 0;
                        else if (i_event.code == G29_BTN_CODE_GEAR_2 && this->cmd_.gear == 2) this->cmd_.gear = 0;
                        else if (i_event.code == G29_BTN_CODE_GEAR_3 && this->cmd_.gear == 3) this->cmd_.gear = 0;
                        else if (i_event.code == G29_BTN_CODE_GEAR_4 && this->cmd_.gear == 4) this->cmd_.gear = 0;
                        else if (i_event.code == G29_BTN_CODE_GEAR_5 && this->cmd_.gear == 5) this->cmd_.gear = 0;
                        else if (i_event.code == G29_BTN_CODE_GEAR_6 && this->cmd_.gear == 6) this->cmd_.gear = 0;
                        else if (i_event.code == G29_BTN_CODE_GEAR_R && this->cmd_.gear == -1) this->cmd_.gear = 0;
                    }
                }
            }
        }

        if (this->manual_control_ == true) {
            float speed = this->speed_;
            speed = std::max(0.0f, speed);

            int16_t force = 0x0000;

            this->effect_constant_.u.constant.level = 0x0000;
            this->effect_constant_.u.constant.envelope.attack_level = 0x0000;
            this->effect_constant_.u.constant.envelope.fade_level = 0x0000;
            if (ioctl(this->event_handle_, EVIOCSFF, &this->effect_constant_) < 0) {
                std::cerr << "ERROR: failed to upload effect" << std::endl;
            }

            this->effect_damper_.u.condition[0].right_saturation = 0xc000;
            this->effect_damper_.u.condition[0].left_saturation = 0xc000;
            this->effect_damper_.u.condition[0].right_coeff = 0x7000;
            this->effect_damper_.u.condition[0].left_coeff = 0x7000;
            this->effect_damper_.u.condition[0].center = 0;
            this->effect_damper_.u.condition[0].deadband = 0;
            this->effect_damper_.u.condition[1] = this->effect_damper_.u.condition[0];
            if (ioctl(this->event_handle_, EVIOCSFF, &this->effect_damper_) < 0) {
                std::cerr << "ERROR: failed to upload effect" << std::endl;
            }

            this->effect_spring_.u.condition[0].right_saturation = static_cast<uint16_t>(std::min(speed * 4000.0f, static_cast<float>(UINT16_MAX)));
            this->effect_spring_.u.condition[0].left_saturation = static_cast<uint16_t>(std::min(speed * 4000.0f, static_cast<float>(UINT16_MAX)));
            this->effect_spring_.u.condition[0].right_coeff = 0x4000;
            this->effect_spring_.u.condition[0].left_coeff = 0x4000;
            this->effect_spring_.u.condition[0].center = static_cast<int16_t>(-(this->cmd_.steer) * static_cast<float>(INT16_MAX));
            this->effect_spring_.u.condition[1] = this->effect_spring_.u.condition[0];
            if (ioctl(this->event_handle_, EVIOCSFF, &this->effect_spring_) < 0) {
                std::cerr << "ERROR: failed to upload effect" << std::endl;
            }
        }
        else {
            float target = this->target_;
            target = std::max(-1.0f, std::min(1.0f, target));

            float buf = diff;
            diff = target - this->cmd_.steer;
            diff_i += diff;
            float diff_d = diff - buf;

            float force_f = this->pid_.kp * diff + this->pid_.ki * diff_i + this->pid_.kd * diff_d;
            float force_abs = std::fabs(force_f);
            float force_pn = (force_f >= 0.0f)? 1.0f : -1.0f;

            force_abs = std::max(this->pid_.force_min, std::min(this->pid_.force_max, force_abs));

            int16_t force = static_cast<int16_t>(force_abs * force_pn * static_cast<float>(INT16_MAX));

            this->effect_constant_.u.constant.level = force;
            this->effect_constant_.u.constant.envelope.attack_level = force;
            this->effect_constant_.u.constant.envelope.fade_level = 0x0000;
            if (ioctl(this->event_handle_, EVIOCSFF, &this->effect_constant_) < 0) {
                std::cerr << "ERROR: failed to upload effect" << std::endl;
            }

            this->effect_damper_.u.condition[0].right_saturation = 0x0000;
            this->effect_damper_.u.condition[0].left_saturation = 0x0000;
            this->effect_damper_.u.condition[0].right_coeff = 0x0000;
            this->effect_damper_.u.condition[0].left_coeff = 0x0000;
            this->effect_damper_.u.condition[0].center = 0;
            this->effect_damper_.u.condition[0].deadband = 0;
            this->effect_damper_.u.condition[1] = this->effect_damper_.u.condition[0];
            if (ioctl(this->event_handle_, EVIOCSFF, &this->effect_damper_) < 0) {
                std::cerr << "ERROR: failed to upload effect" << std::endl;
            }

            this->effect_spring_.u.condition[0].right_saturation = INT16_MAX;
            this->effect_spring_.u.condition[0].left_saturation = INT16_MAX;
            this->effect_spring_.u.condition[0].right_coeff = 0x0000;
            this->effect_spring_.u.condition[0].left_coeff = 0x0000;
            this->effect_spring_.u.condition[0].center = static_cast<int16_t>(target * static_cast<float>(INT16_MAX));
            this->effect_spring_.u.condition[1] = this->effect_spring_.u.condition[0];
            if (ioctl(this->event_handle_, EVIOCSFF, &this->effect_spring_) < 0) {
                std::cerr << "ERROR: failed to upload effect" << std::endl;
            }
            tmp_target = target;
        }
    }
}

int32_t G29FFB::testBit(int32_t bit, u_int8_t *array)
{
    return ((array[bit / (sizeof(u_int8_t) * 8)] >> (bit % (sizeof(u_int8_t) * 8))) & 1);
}
