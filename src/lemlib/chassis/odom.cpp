// The implementation below is mostly based off of
// the document written by 5225A (Pilons)
// Here is a link to the original document
// http://thepilons.ca/wp-content/uploads/2018/10/Tracking.pdf

#include <math.h>
#include <string.h>
#include "pros/rtos.hpp"
#include "pros/serial.hpp"
#include "lemlib/util.hpp"
#include "lemlib/chassis/odom.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "musty/cobs.h"

#define MUSTY_SERIALPORT 2
#define MUSTY_BAUDRATE 460800

#define TRANSMIT_PACKET_SIZE 2
#define MAX_BUFFER_SIZE 256
#define RECEIVE_OTOS_PACKET_SIZE 21

#define COMMAND_READ_LIDAR 200
#define COMMAND_READ_ENCODERS 100
#define COMMAND_READ_OTOS 201

// tracking thread
pros::Task* trackingTask = nullptr;

// global variables
pros::Serial* s;
lemlib::Drivetrain drive(nullptr, nullptr, 0, 0, 0, 0); // the drivetrain to be used for odometry
lemlib::Pose odomPose(0, 0, 0); // the pose of the robot
lemlib::Pose odomSpeed(0, 0, 0); // the speed of the robot
lemlib::Pose odomLocalSpeed(0, 0, 0); // the local speed of the robot

void lemlib::setDrivetrain(lemlib::Drivetrain drivetrain) { drive = drivetrain; }

lemlib::Pose lemlib::getPose(bool radians) {
    if (radians) return odomPose;
    else return lemlib::Pose(odomPose.x, odomPose.y, radToDeg(odomPose.theta));
}

void lemlib::setPose(lemlib::Pose pose, bool radians) {
    if (radians) odomPose = pose;
    else odomPose = lemlib::Pose(pose.x, pose.y, degToRad(pose.theta));
}

lemlib::Pose lemlib::getSpeed(bool radians) {
    if (radians) return odomSpeed;
    else return lemlib::Pose(odomSpeed.x, odomSpeed.y, radToDeg(odomSpeed.theta));
}

lemlib::Pose lemlib::getLocalSpeed(bool radians) {
    if (radians) return odomLocalSpeed;
    else return lemlib::Pose(odomLocalSpeed.x, odomLocalSpeed.y, radToDeg(odomLocalSpeed.theta));
}

lemlib::Pose lemlib::estimatePose(float time, bool radians) {
    // get current position and speed
    Pose curPose = getPose(true);
    Pose localSpeed = getLocalSpeed(true);
    // calculate the change in local position
    Pose deltaLocalPose = localSpeed * time;

    // calculate the future pose
    float avgHeading = curPose.theta + deltaLocalPose.theta / 2;
    Pose futurePose = curPose;
    futurePose.x += deltaLocalPose.y * sin(avgHeading);
    futurePose.y += deltaLocalPose.y * cos(avgHeading);
    futurePose.x += deltaLocalPose.x * -cos(avgHeading);
    futurePose.y += deltaLocalPose.x * sin(avgHeading);
    if (!radians) futurePose.theta = radToDeg(futurePose.theta);

    return futurePose;
}

void lemlib::update() {
    uint8_t transmit_buffer[TRANSMIT_PACKET_SIZE] = {COMMAND_READ_OTOS, 255};
    uint8_t receive_buffer[MAX_BUFFER_SIZE] = {0};
    uint8_t decode_buffer[MAX_BUFFER_SIZE] = {0};

    uint8_t* temp_buf_ptr = receive_buffer;
    int num_read_bytes = 0;
    int num_waiting_bytes = 0;
    int timeout_counter = 0;
    bool read_success = false;

    cobs_decode_result res;

    float x, y, h;

    // send the command to the Musty board
    s->write(transmit_buffer, TRANSMIT_PACKET_SIZE);

    // read the data from the Musty board
    while (true) {
        num_waiting_bytes = s->get_read_avail();

        if (num_waiting_bytes > 0) {
            num_read_bytes = s->read(temp_buf_ptr, num_waiting_bytes);
            temp_buf_ptr += num_read_bytes;
        }

        if (num_read_bytes == RECEIVE_OTOS_PACKET_SIZE) {
            read_success = true;
            break;

        } else if (num_read_bytes > RECEIVE_OTOS_PACKET_SIZE) {
            s->flush();
            break;

        } else {
            timeout_counter++;
            pros::delay(1);
            if (timeout_counter > 10) { break; }
        }

    }

    if (read_success) {
        res = musty_cobs_decode(decode_buffer, MAX_BUFFER_SIZE, receive_buffer, RECEIVE_OTOS_PACKET_SIZE);

        if (res.status == COBS_DECODE_OK) {
            memcpy(&x, &decode_buffer[0], sizeof(float));
            memcpy(&y, &decode_buffer[4], sizeof(float));
            memcpy(&h, &decode_buffer[8], sizeof(float));

            odomPose = lemlib::Pose(x, y, h);
        }
    }
}

void lemlib::init() {
    if (trackingTask == nullptr) {
        pros::Serial s1(MUSTY_SERIALPORT,
                        MUSTY_BAUDRATE); // need to initialize the serial port twice, so here is the first time
        pros::delay(100); // Let VEX OS configure port

        s = new pros::Serial(MUSTY_SERIALPORT, MUSTY_BAUDRATE); // create a serial port on #2 at 460800 baud
        pros::delay(100); // Let VEX OS configure port

        trackingTask = new pros::Task {[=] {
            while (true) {
                update();
                pros::delay(10);
            }
        }};
    }
}
