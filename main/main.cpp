#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "esp_timer.h"

#include <algorithm>
#include <string.h>
#include "driver/uart.h"

#include "math.hpp"
#include "mpu9250.hpp"
#include "receiver.hpp"
#include "imu_fusion.hpp"
#include "filter.hpp"
#include "esc.hpp"
#include "utility.hpp"

#define ENABLE_LOGGING
// #define ESC_CALIBRATION


typedef float f32;
typedef double f64;
typedef mylib::Vector3<f64> vec3_t;
typedef mylib::Quaternion<f64> quat_t;

enum class ctrl_mode_t {
    ACRO = 0,
    ANGLE = 1,
    HORIZON = 2,
    MAX = 3,
};

const vec3_t VEC3_UP(0.0, 1.0, 0.0);


// System Configuration
const int64_t wakeup_delay_ms = 500;
const int64_t looptime_freq_hz = 1000;
const int64_t looptime_t_us = 1000000 / looptime_freq_hz;

#ifdef ENABLE_LOGGING
const uart_port_t log_uart_port = UART_NUM_0;
const int log_uart_baud_rate = 921600;
const uart_word_length_t log_uart_data_bits = UART_DATA_8_BITS;
const uart_parity_t log_uart_parity = UART_PARITY_DISABLE;
const uart_stop_bits_t log_uart_stop_bits = UART_STOP_BITS_1;
const size_t log_uart_buffer_size = 1024;
const int log_uart_gpio_tx = 1;
const int log_uart_gpio_rx = 3;

const int log_freq_hz = 500;
const int system_log_buff_size = 4;

typedef struct __attribute__((packed)) {
    f32 usage, usage_max;
    f32 dt, dt_max;
    f32 raw_accel[3], raw_gyro[3], accel[3], gyro[3], temp;
    f32 rot[4], target_tilt_quat[4];
    f32 raw_rc_throttle, raw_rc_roll, raw_rc_pitch, raw_rc_yaw, rc_throttle, rc_roll, rc_pitch, rc_yaw;
    uint8_t rc_arm, rc_ctrl_mode, drone_arm;
    f32 pid_val[3];
    f32 esc_throttle[4];
} log_data_t;

static_assert(sizeof(log_data_t) == 163, "Unexpected log_data_t size (update serializer if struct changed).");

// (추가) CRC16-CCITT (FALSE) 구현
static uint16_t crc16_ccitt(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; b++) {
            if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
            else crc <<= 1;
        }
    }
    return crc;
}

static void send_log_data(uart_port_t port, const log_data_t& data) {
    constexpr uint8_t SOF[2] = {0xAA, 0x55};
    constexpr uint16_t payload_len = sizeof(log_data_t);
    uint8_t frame[2 + 2 + payload_len + 2]; // SOF(2) + LEN(2) + PAYLOAD + CRC(2)

    size_t idx = 0;
    frame[idx++] = SOF[0];
    frame[idx++] = SOF[1];

    frame[idx++] = (uint8_t)(payload_len & 0xFF);        // Length LSB
    frame[idx++] = (uint8_t)((payload_len >> 8) & 0xFF); // Length MSB

    memcpy(&frame[idx], &data, payload_len);
    idx += payload_len;

    uint16_t crc = crc16_ccitt((const uint8_t*)&data, payload_len);
    frame[idx++] = (uint8_t)(crc & 0xFF);
    frame[idx++] = (uint8_t)((crc >> 8) & 0xFF);

    uart_write_bytes(port, (const char*)frame, idx);
}
#endif

// Drone Configuration
// imu (spi)
const spi_host_device_t imu_spi_port = SPI2_HOST;
const int imu_mosi = 4;
const int imu_miso = 16;
const int imu_sclk = 0;
const int imu_cs = 17;
const mylib::MPU9250::gyro_dlpf_cfg_t imu_gyro_dlpf_cfg = mylib::MPU9250::GYRO_DLPF_CFG[6]; // 20Hz 9.9ms
const mylib::MPU9250::gyro_scale_cfg_t imu_gyro_scale_cfg = mylib::MPU9250::GYRO_SCALE_CFG[3]; // 2000dps
const mylib::MPU9250::accel_dlpf_cfg_t imu_accel_dlpf_cfg = mylib::MPU9250::ACCEL_DLPF_CFG[6]; // 10.2Hz 16.83ms
const mylib::MPU9250::accel_scale_cfg_t imu_accel_scale_cfg = mylib::MPU9250::ACCEL_SCALE_CFG[3]; // 16g
// receiver (uart)
const uart_port_t receiver_uart_port = UART_NUM_1;
const int receiver_rx = 13;
// esc (pwm)
const ledc_timer_t esc_timer = LEDC_TIMER_0;
const mylib::PWMESC::protocol_t esc_protocol = mylib::PWMESC::protocol_t::ONESHOT_125;
const ledc_channel_t esc_fl_channel = LEDC_CHANNEL_0;
const ledc_channel_t esc_fr_channel = LEDC_CHANNEL_1;
const ledc_channel_t esc_bl_channel = LEDC_CHANNEL_2;
const ledc_channel_t esc_br_channel = LEDC_CHANNEL_3;
const int esc_fl_pin = 21;
const int esc_fr_pin = 27;
const int esc_bl_pin = 25;
const int esc_br_pin = 22;
// imu axis
const int accel_axis_order[3] = {0, 2, 1};
const bool accel_axis_inv[3] = {true, true, true};
const int gyro_axis_order[3] = {0, 2, 1};
const bool gyro_axis_inv[3] = {true, true, true};
// imu calibration
const vec3_t accel_vec_offset(-0.15, -0.18, -0.22);
const vec3_t gyro_vec_offset(5.1, -1.0, 2.4);
// aux mapping
const int throttle_aux_idx = 2;
const int roll_aux_idx = 0;
const int pitch_aux_idx = 1;
const int yaw_aux_idx = 3;
const int arm_aux_idx = 4;
const int ctrl_mode_aux_idx = 6;
// imu fusion
const f64 imu_fusion_accel_ratio = 0.001;
// low-pass filter
const int accel_lpf_order = 2;
const f64 accel_lpf_cutoff_hz = 20.0;
const int gyro_lpf_order = 2;
const f64 gyro_lpf_cutoff_hz = 200.0;
const int dterm_lpf_order = 2;
const f64 dterm_lpf_cutoff_hz = 200.0;
const int rc_lpf_order = 3;
const f64 rc_lpf_cutoff_hz = 30.0;
// acro/angle/horizon mode
const f64 acro_roll_angvel = 800.0;
const f64 acro_pitch_angvel = 800.0;
const f64 acro_yaw_angvel = 800.0;
const f64 angle_tilt_ang = 50.0;
const f64 horizon_tilt_ang = 50.0;
const f64 horizon_rc_mix_ang1 = 40.0;
const f64 horizon_rc_mix_ang2 = 50.0;
const f64 horizon_tilt_mix_ang1 = 50.0;
const f64 horizon_tilt_mix_ang2 = 80.0;
// rc rate
const f64 roll_rc_rate = 0.7;
const f64 pitch_rc_rate = 0.7;
const f64 yaw_rc_rate = 0.7;
// angle p control
const vec3_t ang_pid_kp(6.5, 6.5, 6.5);
// angvel pid control
const vec3_t pid_setpoint_clip(1000.0, 1000.0, 1000.0);
const vec3_t pid_kp(0.0005, 0.0005, 0.0005);
const vec3_t pid_kd(0.0000045, 0.0, 0.0000045);
const vec3_t pid_ki(0.002, 0.002, 0.002);
const vec3_t pid_ki_clip(0.02, 0.02, 0.02);
const vec3_t pid_clip(1.0, 1.0, 1.0);
// throttle clipping
const f64 usr_throttle_max = 0.90;
const f64 usr_throttle_min = 0.02;
const f64 usr_safearm_throttle = 0.035;
// aux mapping
const int AUX_MAX = 2000 + 10;
const int AUX_MIN = 1000 - 10;
const mylib::ContinuousAuxMapper throttle_aux_mapper(AUX_MIN, AUX_MAX, false);
const mylib::ContinuousAuxMapper roll_aux_mapper(AUX_MIN, AUX_MAX, true);
const mylib::ContinuousAuxMapper pitch_aux_mapper(AUX_MIN, AUX_MAX, false);
const mylib::ContinuousAuxMapper yaw_aux_mapper(AUX_MIN, AUX_MAX, false);
const mylib::DiscreteAuxMapper arm_aux_mapper(AUX_MIN, AUX_MAX, 2, false);
const mylib::DiscreteAuxMapper ctrl_mode_aux_mapper(AUX_MIN, AUX_MAX, (int)ctrl_mode_t::MAX, false);


vec3_t get_rotation_vec(const quat_t& quat) {
    f64 half_t = std::acos(std::clamp(quat.w, -mylib::VEC_COS_EPS, mylib::VEC_COS_EPS));

    return quat.to_vector().mul(
        half_t > mylib::RAD_EPS ? 2.0 * half_t / std::sin(half_t) : 2.0 // approximation
    );
}

vec3_t compute_angle_mode_target_angvel(const quat_t& rot, const quat_t& drone_heading, quat_t& target_tilt_quat, f64 yaw_angvel, f64 rc_pitch, f64 rc_roll, f64 max_tilt_ang) {
    // compute yaw angular velocity vector
    vec3_t yaw_angvel_w = vec3_t(0, yaw_angvel * mylib::DEG_TO_RAD, 0);
    vec3_t yaw_angvel_b = rot.conj().rotate(yaw_angvel_w);
    
    // compute target tilt orientation
    vec3_t rc_tilt_vec(rc_pitch, 0.0, rc_roll);

    if (rc_tilt_vec.norm() > mylib::VEC_NORM_EPS) {
        target_tilt_quat = drone_heading.quat_mul(
            quat_t(rc_tilt_vec.unit(), std::min(rc_tilt_vec.norm(), 1.0) * max_tilt_ang * mylib::DEG_TO_RAD)
        );
    }
    else {
        target_tilt_quat = drone_heading; // TODO: check using drone_heading
    }

    // compute tilt angular velocity vector
    quat_t tilt_diff_quat = rot.conj().quat_mul(target_tilt_quat).shortest();
    vec3_t tilt_diff_angvel_b = get_rotation_vec(tilt_diff_quat);

    // compute target angular velocity (angle p-control)
    return yaw_angvel_b.add(tilt_diff_angvel_b.mul(ang_pid_kp)).mul(mylib::RAD_TO_DEG);
}


extern "C" void app_main(void)
{
    // wakeup
    vTaskDelay(pdMS_TO_TICKS(wakeup_delay_ms));

    // uart configuration for logging
    #ifdef ENABLE_LOGGING
    // delete uart driver if already installed
    if (uart_is_driver_installed(log_uart_port)) {
        uart_driver_delete(log_uart_port);
    }
    // install uart driver
    uart_config_t log_uart_cfg = {
        .baud_rate = log_uart_baud_rate,
        .data_bits = log_uart_data_bits,
        .parity = log_uart_parity,
        .stop_bits = log_uart_stop_bits,
    };
    uart_param_config(log_uart_port, &log_uart_cfg);
    uart_set_pin(log_uart_port, log_uart_gpio_tx, log_uart_gpio_rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(log_uart_port, log_uart_buffer_size, log_uart_buffer_size, 0, NULL, 0);
    #endif

    // start imu
    mylib::MPU9250 imu(imu_spi_port, imu_mosi, imu_miso, imu_sclk, imu_cs);
    imu.spi_begin();
    imu.set_config(imu_gyro_dlpf_cfg, imu_gyro_scale_cfg, imu_accel_dlpf_cfg, imu_accel_scale_cfg);
    // start receiver
    mylib::Receiver receiver(receiver_uart_port, receiver_rx);
    receiver.uart_begin();
    // start motor
    mylib::PWMESC::set_timer(esc_timer, esc_protocol);
    mylib::PWMESC esc_fl(esc_timer, esc_fl_channel, esc_protocol, esc_fl_pin);
    mylib::PWMESC esc_fr(esc_timer, esc_fr_channel, esc_protocol, esc_fr_pin);
    mylib::PWMESC esc_bl(esc_timer, esc_bl_channel, esc_protocol, esc_bl_pin);
    mylib::PWMESC esc_br(esc_timer, esc_br_channel, esc_protocol, esc_br_pin);
    esc_fl.pwm_begin();
    esc_fr.pwm_begin();
    esc_bl.pwm_begin();
    esc_br.pwm_begin();

    // initialize imu fuser
    mylib::SimpleIMUFusion imu_fusion(imu_fusion_accel_ratio);
    // initialize accel low-pass filters
    mylib::Vec3LowPassFilter<accel_lpf_order> accel_lpf(looptime_freq_hz, accel_lpf_cutoff_hz, vec3_t(0.0, 0.0, 0.0));
    // initialize gyro low-pass filters
    mylib::Vec3LowPassFilter<gyro_lpf_order> gyro_lpf(looptime_freq_hz, gyro_lpf_cutoff_hz, vec3_t(0.0, 0.0, 0.0));
    // initialize dterm low-pass filters
    mylib::Vec3LowPassFilter<dterm_lpf_order> dterm_lpf(looptime_freq_hz, dterm_lpf_cutoff_hz, vec3_t(0.0, 0.0, 0.0));
    // initialize rc low-pass filters
    mylib::LowPassFilter<rc_lpf_order> rc_throttle_lpf(looptime_freq_hz, rc_lpf_cutoff_hz, 0.0);
    mylib::LowPassFilter<rc_lpf_order> rc_roll_lpf(looptime_freq_hz, rc_lpf_cutoff_hz, 0.0);
    mylib::LowPassFilter<rc_lpf_order> rc_pitch_lpf(looptime_freq_hz, rc_lpf_cutoff_hz, 0.0);
    mylib::LowPassFilter<rc_lpf_order> rc_yaw_lpf(looptime_freq_hz, rc_lpf_cutoff_hz, 0.0);

    // initial rc variables
    f64 rc_throttle = 0.0, raw_rc_throttle = 0.0;
    f64 rc_roll = 0.0, raw_rc_roll = 0.0;
    f64 rc_pitch = 0.0, raw_rc_pitch = 0.0;
    f64 rc_yaw = 0.0, raw_rc_yaw = 0.0;
    bool rc_arm = false, raw_rc_arm = false, last_rc_arm = false;
    ctrl_mode_t rc_ctrl_mode = ctrl_mode_t::ACRO, raw_rc_ctrl_mode = ctrl_mode_t::ACRO;

    // drone state variables
    bool drone_arm = false, drone_arm_on = false;
    ctrl_mode_t drone_ctrl_mode = ctrl_mode_t::ACRO;

    // internal variables
    vec3_t last_pid_error; // initialized to zero when "drone_arm_on == True"
    vec3_t pid_i_val; // initialized to zero when "drone_arm_on == True"

    // just for convenient
    const f64 duty_min_f = mylib::PWMESC::DUTY_MIN;
    const f64 duty_max_f = mylib::PWMESC::DUTY_MAX;


    // main loop
    int64_t last_frame_end_time = esp_timer_get_time();
    f64 dt = looptime_t_us / 1000000.0;

    #ifdef ENABLE_LOGGING
    int log_step_ctr = 0;
    int log_interval = looptime_freq_hz / log_freq_hz;
    log_data_t log_data;

    double usage_log_buff[system_log_buff_size] = {0.0};
    double dt_log_buff[system_log_buff_size] = {0.0};
    #endif

    while (true) {
        // read raw data from the IMU and apply axis corrections
        mylib::MPU9250::sensor_data_t imu_data = imu.read_data();
        vec3_t raw_accel_vec(
            imu_data.accel[accel_axis_order[0]] * (accel_axis_inv[0] ? -1 : 1),
            imu_data.accel[accel_axis_order[1]] * (accel_axis_inv[1] ? -1 : 1),
            imu_data.accel[accel_axis_order[2]] * (accel_axis_inv[2] ? -1 : 1)
        );
        vec3_t raw_gyro_vec(
            imu_data.gyro[gyro_axis_order[0]] * (gyro_axis_inv[0] ? -1 : 1),
            imu_data.gyro[gyro_axis_order[1]] * (gyro_axis_inv[1] ? -1 : 1),
            imu_data.gyro[gyro_axis_order[2]] * (gyro_axis_inv[2] ? -1 : 1)
        );
        f64 raw_temperature = imu_data.temp;

        // filter the gyro and accel data and apply offsets
        vec3_t accel_vec = accel_lpf.update(raw_accel_vec).add(accel_vec_offset);
        vec3_t gyro_vec = gyro_lpf.update(raw_gyro_vec).add(gyro_vec_offset);
        f64 temperature = raw_temperature;

        // compute rotation (imu fusion)
        quat_t rot = imu_fusion.update(accel_vec, gyro_vec.mul(mylib::DEG_TO_RAD), dt);

        // compute drone heading
        vec3_t drone_up = rot.rotate(VEC3_UP);
        quat_t drone_heading = (
            drone_up.dot(VEC3_UP) < mylib::VEC_COS_EPS ? quat_t(drone_up, VEC3_UP, 1.0).quat_mul(rot) : rot
        );
        drone_heading.x = 0.0;
        drone_heading.z = 0.0;
        drone_heading = drone_heading.unit();

        // read data from receiver and apply rc filter
        if (receiver.update()) {
            mylib::Receiver::receiver_data_t receiver_data = receiver.read_data();

            raw_rc_throttle = throttle_aux_mapper.map(receiver_data.aux[throttle_aux_idx]);
            raw_rc_roll = roll_aux_mapper.map(receiver_data.aux[roll_aux_idx]) * 2.0 - 1.0;
            raw_rc_pitch = pitch_aux_mapper.map(receiver_data.aux[pitch_aux_idx]) * 2.0 - 1.0;
            raw_rc_yaw = yaw_aux_mapper.map(receiver_data.aux[yaw_aux_idx]) * 2.0 - 1.0;

            raw_rc_arm = (bool)arm_aux_mapper.map(receiver_data.aux[arm_aux_idx]);
            raw_rc_ctrl_mode = (ctrl_mode_t)ctrl_mode_aux_mapper.map(receiver_data.aux[ctrl_mode_aux_idx]);
        }
        rc_throttle = rc_throttle_lpf.update(raw_rc_throttle);
        rc_roll = rc_roll_lpf.update(raw_rc_roll);
        rc_pitch = rc_pitch_lpf.update(raw_rc_pitch);
        rc_yaw = rc_yaw_lpf.update(raw_rc_yaw);
        last_rc_arm = rc_arm;
        rc_arm = raw_rc_arm;
        rc_ctrl_mode = raw_rc_ctrl_mode;

        // compute usr_throttle
        f64 usr_throttle = rc_throttle * (usr_throttle_max - usr_throttle_min) + usr_throttle_min;

        // drone state management
        drone_arm_on = (
            (rc_arm && !last_rc_arm) &&
            (usr_throttle <= usr_safearm_throttle) // safe arming
        );

        if (drone_arm_on) {
            drone_arm = true;
        }
        else if (!rc_arm) {
            drone_arm = false;
        }

        drone_ctrl_mode = rc_ctrl_mode;

        // update internal variables
        if (drone_arm_on) {
            last_pid_error = vec3_t();
            pid_i_val = vec3_t();
            dterm_lpf.clear(vec3_t());
        }

        // compute target rotation and target angular velocity according to each flying mode
        quat_t target_tilt_quat;
        vec3_t pid_setpoint;

        vec3_t acro_angvel(
            mylib::apply_cubic_rc_rate(rc_pitch, pitch_rc_rate) * acro_pitch_angvel,
            mylib::apply_cubic_rc_rate(rc_yaw, yaw_rc_rate) * acro_yaw_angvel,
            mylib::apply_cubic_rc_rate(rc_roll, roll_rc_rate) * acro_roll_angvel
        );
        vec3_t angle_angvel = compute_angle_mode_target_angvel(
            rot, drone_heading, target_tilt_quat, acro_angvel.y, rc_pitch, rc_roll,
            (drone_ctrl_mode == ctrl_mode_t::HORIZON ? horizon_tilt_ang : angle_tilt_ang)
        );

        if (drone_ctrl_mode == ctrl_mode_t::ACRO) {
            pid_setpoint = acro_angvel;
        }
        else if (drone_ctrl_mode == ctrl_mode_t::ANGLE) {
            pid_setpoint = angle_angvel;
        }
        else if (drone_ctrl_mode == ctrl_mode_t::HORIZON) {
            f64 rc_ang = std::clamp(sqrt(rc_pitch * rc_pitch + rc_roll * rc_roll), 0.0, 1.0) * horizon_tilt_ang;
            f64 tilt_ang = drone_up.angle(VEC3_UP) * mylib::RAD_TO_DEG;
            
            f64 rc_mix_rate = std::clamp(
                (rc_ang - horizon_rc_mix_ang1) / (horizon_rc_mix_ang2 - horizon_rc_mix_ang1), 0.0, 1.0
            );
            f64 tilt_mix_rate = std::clamp(
                (tilt_ang - horizon_tilt_mix_ang1) / (horizon_tilt_mix_ang2 - horizon_tilt_mix_ang1), 0.0, 1.0
            );
            f64 acro_rate = std::max(rc_mix_rate, tilt_mix_rate);
            
            pid_setpoint = acro_angvel.mul(acro_rate).add(
                angle_angvel.mul(1.0 - acro_rate)
            );
        }

        // pid control
        pid_setpoint.x = std::clamp(pid_setpoint.x, -pid_setpoint_clip.x, pid_setpoint_clip.x);
        pid_setpoint.y = std::clamp(pid_setpoint.y, -pid_setpoint_clip.y, pid_setpoint_clip.y);
        pid_setpoint.z = std::clamp(pid_setpoint.z, -pid_setpoint_clip.z, pid_setpoint_clip.z);

        vec3_t pid_error = pid_setpoint.add(gyro_vec.add_inv());
        vec3_t pid_error_diff = pid_error.add(last_pid_error.add_inv()).mul(1.0 / dt);

        vec3_t pid_p_val = pid_error.mul(pid_kp);
        vec3_t pid_d_val = dterm_lpf.update(pid_error_diff.mul(pid_kd));

        pid_i_val = pid_i_val.add(pid_error.mul(pid_ki).mul(dt));
        pid_i_val.x = std::clamp(pid_i_val.x, -pid_ki_clip.x, pid_ki_clip.x);
        pid_i_val.y = std::clamp(pid_i_val.y, -pid_ki_clip.y, pid_ki_clip.y);
        pid_i_val.z = std::clamp(pid_i_val.z, -pid_ki_clip.z, pid_ki_clip.z);

        vec3_t pid_val = pid_p_val.add(pid_i_val).add(pid_d_val);
        pid_val.x = std::clamp(pid_val.x, -pid_clip.x, pid_clip.x);
        pid_val.y = std::clamp(pid_val.y, -pid_clip.y, pid_clip.y);
        pid_val.z = std::clamp(pid_val.z, -pid_clip.z, pid_clip.z);

        last_pid_error = pid_error;
        
        // compute throttle for each motor (assume props-in direction)
        f64 esc_fl_throttle = std::clamp(usr_throttle - pid_val.x - pid_val.y - pid_val.z, usr_throttle_min, 1.0);
        f64 esc_fr_throttle = std::clamp(usr_throttle - pid_val.x + pid_val.y + pid_val.z, usr_throttle_min, 1.0);
        f64 esc_bl_throttle = std::clamp(usr_throttle + pid_val.x + pid_val.y - pid_val.z, usr_throttle_min, 1.0);
        f64 esc_br_throttle = std::clamp(usr_throttle + pid_val.x - pid_val.y + pid_val.z, usr_throttle_min, 1.0);

        // update esc pwm signal
        uint32_t esc_fl_duty = drone_arm ? (uint32_t)((duty_max_f - duty_min_f) * esc_fl_throttle + duty_min_f) : 0U;
        uint32_t esc_fr_duty = drone_arm ? (uint32_t)((duty_max_f - duty_min_f) * esc_fr_throttle + duty_min_f) : 0U;
        uint32_t esc_bl_duty = drone_arm ? (uint32_t)((duty_max_f - duty_min_f) * esc_bl_throttle + duty_min_f) : 0U;
        uint32_t esc_br_duty = drone_arm ? (uint32_t)((duty_max_f - duty_min_f) * esc_br_throttle + duty_min_f) : 0U;

        #ifdef ESC_CALIBRATION
        esc_fl_duty = (uint32_t)((duty_max_f - duty_min_f) * std::clamp(raw_rc_throttle * 2.0 - 0.5, 0.0, 1.0) + duty_min_f);
        esc_fr_duty = (uint32_t)((duty_max_f - duty_min_f) * std::clamp(raw_rc_throttle * 2.0 - 0.5, 0.0, 1.0) + duty_min_f);
        esc_bl_duty = (uint32_t)((duty_max_f - duty_min_f) * std::clamp(raw_rc_throttle * 2.0 - 0.5, 0.0, 1.0) + duty_min_f);
        esc_br_duty = (uint32_t)((duty_max_f - duty_min_f) * std::clamp(raw_rc_throttle * 2.0 - 0.5, 0.0, 1.0) + duty_min_f);
        #endif

        esc_fl.update(esc_fl_duty);
        esc_fr.update(esc_fr_duty);
        esc_bl.update(esc_bl_duty);
        esc_br.update(esc_br_duty);
        
        // logging
        #ifdef ENABLE_LOGGING
        log_step_ctr = (log_step_ctr + 1) % log_interval;

        if (log_step_ctr == 0) {
            // compute system metrics
            f64 usage_sum = 0.0, dt_sum = 0.0;
            f64 usage_max = 0.0, dt_max = 0.0;
            
            for (int i = 0; i < system_log_buff_size; i++) {
                usage_sum += usage_log_buff[i];
                dt_sum += dt_log_buff[i];
                usage_max = std::max(usage_max, usage_log_buff[i]);
                dt_max = std::max(dt_max, dt_log_buff[i]);
            }

            f64 usage_mean = usage_sum / (f64)system_log_buff_size;
            f64 dt_mean = dt_sum / (f64)system_log_buff_size;

            // serialize logging data
            log_data = {
                .usage = (f32)usage_mean,
                .usage_max = (f32)usage_max,
                .dt = (f32)dt_mean,
                .dt_max = (f32)dt_max,
                .raw_accel = {(f32)raw_accel_vec.x, (f32)raw_accel_vec.y, (f32)raw_accel_vec.z},
                .raw_gyro = {(f32)raw_gyro_vec.x, (f32)raw_gyro_vec.y, (f32)raw_gyro_vec.z},
                .accel = {(f32)accel_vec.x, (f32)accel_vec.y, (f32)accel_vec.z},
                .gyro = {(f32)gyro_vec.x, (f32)gyro_vec.y, (f32)gyro_vec.z},
                .temp = (f32)temperature,
                .rot = {(f32)rot.w, (f32)rot.x, (f32)rot.y, (f32)rot.z},
                .target_tilt_quat = {(f32)target_tilt_quat.w, (f32)target_tilt_quat.x, (f32)target_tilt_quat.y, (f32)target_tilt_quat.z},
                .raw_rc_throttle = (f32)raw_rc_throttle,
                .raw_rc_roll = (f32)raw_rc_roll,
                .raw_rc_pitch = (f32)raw_rc_pitch,
                .raw_rc_yaw = (f32)raw_rc_yaw,
                .rc_throttle = (f32)rc_throttle,
                .rc_roll = (f32)rc_roll,
                .rc_pitch = (f32)rc_pitch,
                .rc_yaw = (f32)rc_yaw,
                .rc_arm = (uint8_t)rc_arm,
                .rc_ctrl_mode = (uint8_t)drone_ctrl_mode,
                .drone_arm = (uint8_t)drone_arm,
                .pid_val = {(f32)pid_val.x, (f32)pid_val.y, (f32)pid_val.z},
                .esc_throttle = {(f32)esc_fl_throttle, (f32)esc_fr_throttle, (f32)esc_bl_throttle, (f32)esc_br_throttle}
            };
            send_log_data(log_uart_port, log_data);
        }
        #endif

        // guarantee fixed-frequency loop
        int64_t task_end_time = esp_timer_get_time();
        vPortYield();
        while (esp_timer_get_time() - last_frame_end_time < looptime_t_us) {
        }
        int64_t polling_end_time = esp_timer_get_time();
        
        #ifdef ENABLE_LOGGING
        // update system metrics buffer
        for (int i = system_log_buff_size - 1; i >= 0; i--) {
            if (i > 0) {
                usage_log_buff[i] = usage_log_buff[i - 1];
                dt_log_buff[i] = dt_log_buff[i - 1];                
            }
            else {
                usage_log_buff[i] = (f64)(task_end_time - last_frame_end_time) / (f64)looptime_t_us;
                dt_log_buff[i] = dt;
            }
        }
        #endif
        
        dt = (f64)(polling_end_time - last_frame_end_time) / 1000000.0;
        last_frame_end_time = polling_end_time;
    }

    // end connection (but actually not called)
    imu.spi_end();
    receiver.uart_end();
}
