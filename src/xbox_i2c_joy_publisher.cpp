#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <memory>

#define I2C_BUS "/dev/i2c-8"          // Use the correct I2C bus
#define I2C_SLAVE_ADDR 0x32           // Slave address
#define READ_SIZE sizeof(xbox_info_t) // Size of xbox_info_t

// Struct to match the Xbox controller data layout
struct xbox_info_t {
    uint8_t battery;
    uint8_t btn_FUNC_ABXY;
    uint8_t btn_DIRS_LRBS;
    uint16_t joyLHori;
    uint16_t joyLVert;
    uint16_t joyRHori;
    uint16_t joyRVert;
    uint16_t trigLT;
    uint16_t trigRT;
};

// Function to convert xbox data to Joy message
sensor_msgs::msg::Joy convertToJoyMsg(const xbox_info_t& xboxData) {
    sensor_msgs::msg::Joy joy_msg;
    joy_msg.header.stamp = rclcpp::Clock().now();
    joy_msg.axes.resize(6); // Resize as needed

    // Normalize joystick values to [-1.0, 1.0]
    joy_msg.axes[0] = -(static_cast<float>(xboxData.joyLHori) / 32767.5 - 1.0);
    joy_msg.axes[1] = -(static_cast<float>(xboxData.joyLVert) / 32767.5 - 1.0);
    joy_msg.axes[2] = -(static_cast<float>(xboxData.joyRHori) / 32767.5 - 1.0);
    joy_msg.axes[3] = -(static_cast<float>(xboxData.joyRVert) / 32767.5 - 1.0);
    joy_msg.axes[4] = static_cast<float>(xboxData.trigLT) / 1024.0;
    joy_msg.axes[5] = static_cast<float>(xboxData.trigRT) / 1024.0;

    // Mapping buttons including LB and RB
    joy_msg.buttons.resize(2); // Resize based on the number of buttons you're using
    // joy_msg.buttons[0] = (xboxData.btn_FUNC_ABXY & 0x08) ? 1 : 0; // A button
    // joy_msg.buttons[1] = (xboxData.btn_FUNC_ABXY & 0x04) ? 1 : 0; // B button
    joy_msg.buttons[0] = (xboxData.btn_DIRS_LRBS & 0x08) ? 1 : 0; // LB is bit 3
    joy_msg.buttons[1] = (xboxData.btn_DIRS_LRBS & 0x04) ? 1 : 0; // RB is bit 2

    return joy_msg;
}

class XboxI2CJoyPublisher : public rclcpp::Node {
public:
    XboxI2CJoyPublisher() : Node("xbox_i2c_joy_publisher"), needToSendStartSignal(true) {
        joy_pub_ = this->create_publisher<sensor_msgs::msg::Joy>("joy", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(17), std::bind(&XboxI2CJoyPublisher::timer_callback, this));
        
        file_ = open(I2C_BUS, O_RDWR);
        if (file_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open the I2C bus");
            rclcpp::shutdown();
        }

        if (ioctl(file_, I2C_SLAVE, I2C_SLAVE_ADDR) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to acquire bus access and/or talk to slave.");
            close(file_);
            rclcpp::shutdown();
        }
    }

    ~XboxI2CJoyPublisher() {
        if (file_ >= 0) {
            close(file_);
        }
    }

private:

    void timer_callback() {
        if(needToSendStartSignal) {
            char data_to_send = 0x10; // Initial byte to signal the start

            // Signal to start data transmission
            if (write(file_, &data_to_send, 1) != 1) {
                RCLCPP_ERROR(this->get_logger(), "Failed to write to the I2C bus.");
                return;
            }
            needToSendStartSignal = false;
        }
        
        // Read data
        xbox_info_t xboxData;
        uint8_t xboxarr[READ_SIZE];
        if (read(file_, xboxarr, READ_SIZE) != READ_SIZE) {
            RCLCPP_ERROR(this->get_logger(), "Failed to read from the I2C bus.");
        } else {
            memcpy(&xboxData, xboxarr, READ_SIZE);
            auto joy_msg = convertToJoyMsg(xboxData);
            joy_pub_->publish(joy_msg);
        }
    }

    int file_;
    bool needToSendStartSignal;
    rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<XboxI2CJoyPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}