#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <iostream>
#include <exception>
#include <chrono>
#include "OledFont8x8.h"
#include "OledI2C.h"

class OledDisplayNode : public rclcpp::Node
{
public:
    OledDisplayNode()
        : Node("oled_display_node")
    {
        // Initialize the OLED display
        try
        {
            oled_ = std::make_unique<SSD1306::OledI2C>("/dev/i2c-1", 0x3C);
            oled_->clear();
            oled_->displayUpdate();
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize OLED: %s", e.what());
            rclcpp::shutdown();
        }

        // Create a subscription to the "oled_text" topic
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "oled_text", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                this->displayMessage(msg->data);
            });
    }

private:
    void displayMessage(const std::string &message)
    {
        try
        {
            // Clear the display
            oled_->clear();

            // Display the message on the OLED
            drawString8x8(SSD1306::OledPoint{0, 0}, message.c_str(), SSD1306::PixelStyle::Set, *oled_);

            // Update the display
            oled_->displayUpdate();
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Error displaying message: %s", e.what());
        }
    }

    std::unique_ptr<SSD1306::OledI2C> oled_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OledDisplayNode>());
    rclcpp::shutdown();
    return 0;
}
