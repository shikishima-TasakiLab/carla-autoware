#include <linux/input.h>
#include <sys/ioctl.h>
#include <iostream>
#include <unistd.h>
#include <memory.h>
#include <fcntl.h>
#include <string>
#include <thread>
#include <deque>

#define GEAR_COUNT 6

struct g29_btn_axes
{
    float steer = 0.0f;         //  type = EV_ABS, code = ABS_X
    float clutch = 0.0f;        //  type = EV_ABS, code = ABS_Y
    float throttle = 0.0f;      //  type = EV_ABS, code = ABS_Z
    float brake = 0.0f;         //  type = EV_ABS, code = ABS_RZ
    bool d_pad_left = false;    //  type = EV_ABS, code = ABS_HAT0X
    bool d_pad_right = false;   //  type = EV_ABS, code = ABS_HAT0X
    bool d_pad_up = false;      //  type = EV_ABS, code = ABS_HAT0Y
    bool d_pad_down = false;    //  type = EV_ABS, code = ABS_HAT0Y
    bool cross = false;         //  type = EV_KEY, code = BTN_TRIGGER
    bool square = false;        //  type = EV_KEY, code = BTN_THUMB
    bool circle = false;        //  type = EV_KEY, code = BTN_THUMB2
    bool triangle = false;      //  type = EV_KEY, code = BTN_TOP
    bool paddle_plus = false;   //  type = EV_KEY, code = BTN_TOP2
    bool paddle_minus = false;  //  type = EV_KEY, code = BTN_PINKIE
    bool r2 = false;            //  type = EV_KEY, code = BTN_BASE
    bool l2 = false;            //  type = EV_KEY, code = BTN_BASE2
    bool share = false;         //  type = EV_KEY, code = BTN_BASE3
    bool options = false;       //  type = EV_KEY, code = BTN_BASE4
    bool r3 = false;            //  type = EV_KEY, code = BTN_BASE5
    bool l3 = false;            //  type = EV_KEY, code = BTN_BASE6
    bool gear_1 = false;        //  type =       , code = 
    bool gear_2 = false;        //  type =       , code = 
    bool gear_3 = false;        //  type =       , code = 
    bool gear_4 = false;        //  type =       , code = 
    bool gear_5 = false;        //  type =       , code = 
    bool gear_6 = false;        //  type =       , code = 
    bool gear_r = false;        //  type =       , code = 
    bool plus = false;          //  type = EV_KEY, code = BTN_TRIGGER_HAPPY4
    bool minus = false;         //  type = EV_KEY, code = BTN_TRIGGER_HAPPY5
    bool encoder_cw = false;    //  type = EV_KEY, code = BTN_TRIGGER_HAPPY6
    bool encoder_ccw = false;   //  type = EV_KEY, code = BTN_TRIGGER_HAPPY7
    bool enter = false;         //  type = EV_KEY, code = BTN_TRIGGER_HAPPY8
    bool play_station = false;  //  type = EV_KEY, code = BTN_TRIGGER_HAPPY9
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
    bool manual_control_ = false;
    std::deque<float> target_ = {0.0f};
    std::deque<float> speed_ = {0.0f};

    std::deque<g29_btn_axes> cmd_deque_;

    void clear_cmd_deque();

private:
    int32_t event_handle_;
    int32_t axis_max_;
    int32_t axis_min_;
    struct ff_effect effect_spring_;
    struct ff_effect effect_damper_;
    struct ff_effect effect_constant_;

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
        bool update = false;
        while (read(this->event_handle_, &i_event, sizeof(i_event)) == sizeof(i_event)) {
            if (i_event.type == EV_ABS) {
                if (i_event.code == ABS_X)
                    this->cmd_.steer = static_cast<float>(i_event.value) / static_cast<float>(UINT16_MAX) * 2.0f - 1.0f;
                else if (i_event.code == ABS_Y)
                    this->cmd_.clutch = 1.0f - static_cast<float>(i_event.value) / 255.0f;
                else if (i_event.code == ABS_Z)
                    this->cmd_.throttle = 1.0f - static_cast<float>(i_event.value) / 255.0f;
                else if (i_event.code == ABS_RZ)
                    this->cmd_.brake = 1.0f - static_cast<float>(i_event.value) / 255.0f;
                else if (i_event.code == ABS_HAT0X) {
                    if (i_event.value > 0) this->cmd_.d_pad_right = true;
                    else if (i_event.value < 0) this->cmd_.d_pad_left = true;
                }
                else if (i_event.code == ABS_HAT0Y) {
                    if (i_event.value > 0) this->cmd_.d_pad_down = true;
                    else if (i_event.value < 0) this->cmd_.d_pad_up = true;
                }
                update = true;
            }
            else if (i_event.type == EV_KEY) {
                if (i_event.value > 0) {
                    if (i_event.code == BTN_TRIGGER) this->cmd_.cross = true;
                    else if (i_event.code == BTN_THUMB) this->cmd_.square = true;
                    else if (i_event.code == BTN_THUMB2) this->cmd_.circle = true;
                    else if (i_event.code == BTN_TOP) this->cmd_.triangle = true;
                    else if (i_event.code == BTN_TOP2) this->cmd_.paddle_plus = true;
                    else if (i_event.code == BTN_PINKIE) this->cmd_.paddle_minus = true;
                    else if (i_event.code == BTN_BASE) this->cmd_.r2 = true;
                    else if (i_event.code == BTN_BASE2) this->cmd_.l2 = true;
                    else if (i_event.code == BTN_BASE3) this->cmd_.share = true;
                    else if (i_event.code == BTN_BASE4) this->cmd_.options = true;
                    else if (i_event.code == BTN_BASE5) this->cmd_.r3 = true;
                    else if (i_event.code == BTN_BASE6) this->cmd_.l3 = true;
                    else if (i_event.code == BTN_TRIGGER_HAPPY4) this->cmd_.plus = true;
                    else if (i_event.code == BTN_TRIGGER_HAPPY5) this->cmd_.minus = true;
                    else if (i_event.code == BTN_TRIGGER_HAPPY6) this->cmd_.encoder_cw = true;
                    else if (i_event.code == BTN_TRIGGER_HAPPY7) this->cmd_.encoder_ccw = true;
                    else if (i_event.code == BTN_TRIGGER_HAPPY8) this->cmd_.enter = true;
                    else if (i_event.code == BTN_TRIGGER_HAPPY9) this->cmd_.play_station = true;
                }
                else {
                    if (i_event.code == BTN_TRIGGER) this->cmd_.cross = false;
                    else if (i_event.code == BTN_THUMB) this->cmd_.square = false;
                    else if (i_event.code == BTN_THUMB2) this->cmd_.circle = false;
                    else if (i_event.code == BTN_TOP) this->cmd_.triangle = false;
                    else if (i_event.code == BTN_TOP2) this->cmd_.paddle_plus = false;
                    else if (i_event.code == BTN_PINKIE) this->cmd_.paddle_minus = false;
                    else if (i_event.code == BTN_BASE) this->cmd_.r2 = false;
                    else if (i_event.code == BTN_BASE2) this->cmd_.l2 = false;
                    else if (i_event.code == BTN_BASE3) this->cmd_.share = false;
                    else if (i_event.code == BTN_BASE4) this->cmd_.options = false;
                    else if (i_event.code == BTN_BASE5) this->cmd_.r3 = false;
                    else if (i_event.code == BTN_BASE6) this->cmd_.l3 = false;
                    else if (i_event.code == BTN_TRIGGER_HAPPY4) this->cmd_.plus = false;
                    else if (i_event.code == BTN_TRIGGER_HAPPY5) this->cmd_.minus = false;
                    else if (i_event.code == BTN_TRIGGER_HAPPY6) this->cmd_.encoder_cw = false;
                    else if (i_event.code == BTN_TRIGGER_HAPPY7) this->cmd_.encoder_ccw = false;
                    else if (i_event.code == BTN_TRIGGER_HAPPY8) this->cmd_.enter = false;
                    else if (i_event.code == BTN_TRIGGER_HAPPY9) this->cmd_.play_station = false;
                }
                update = true;
            }
        }
        if (update == true) this->cmd_deque_.push_back(this->cmd_);

        if (this->manual_control_ == true) {
            float speed = this->speed_.back();
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
            this->effect_spring_.u.condition[0].right_coeff = 0x5000;
            this->effect_spring_.u.condition[0].left_coeff = 0x5000;
            this->effect_spring_.u.condition[0].center = static_cast<int16_t>(-(this->cmd_deque_.back().steer) * static_cast<float>(INT16_MAX));
            this->effect_spring_.u.condition[1] = this->effect_spring_.u.condition[0];
            if (ioctl(this->event_handle_, EVIOCSFF, &this->effect_spring_) < 0) {
                std::cerr << "ERROR: failed to upload effect" << std::endl;
            }
        }
        else {
            float target = this->target_.back();
            target = std::max(-1.0f, std::min(1.0f, target));

            float buf = diff;
            diff = target - this->cmd_deque_.back().steer;
            diff_i += diff;
            float diff_d = diff - buf;

            float force_f = this->pid_.kp * diff + this->pid_.ki * diff_i + this->pid_.kd * diff_d;
            float force_abs = fabs(force_f);
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

        while (this->target_.size() > 1) this->target_.pop_front();
        while (this->speed_.size() > 1) this->speed_.pop_front();
    }
}

int32_t G29FFB::testBit(int32_t bit, u_int8_t *array)
{
    return ((array[bit / (sizeof(u_int8_t) * 8)] >> (bit % (sizeof(u_int8_t) * 8))) & 1);
}

void G29FFB::clear_cmd_deque()
{
    while (this->cmd_deque_.size() > 1) this->cmd_deque_.pop_front();
}
