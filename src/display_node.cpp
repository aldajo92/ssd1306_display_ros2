#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <iostream>
#include <exception>
#include <chrono>
#include <ifaddrs.h>
#include <arpa/inet.h>
#include <cstring>
#include "OledFont8x8.h"
#include "OledI2C.h"

class OledDisplayNode : public rclcpp::Node
{
public:
    OledDisplayNode()
        : Node("oled_display_node"), 
        first_row_("IP..."), 
        second_row_("Network.."), 
        third_row_("..."),
        four_row_("...")
    {
        // Declare parameters for topic names
        this->declare_parameter<std::string>("topic_first", "oled_text/first");
        this->declare_parameter<std::string>("topic_second", "oled_text/second");

        // Get parameter values
        std::string topic_first = this->get_parameter("topic_first").as_string();
        std::string topic_second = this->get_parameter("topic_second").as_string();

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

        // Create subscriptions using the configured topics
        subscription_first_ = this->create_subscription<std_msgs::msg::String>(
            topic_first, 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                this->third_row_ = msg->data; // Update the message variable
            });

        subscription_second_ = this->create_subscription<std_msgs::msg::String>(
            topic_second, 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                this->four_row_ = msg->data; // Update the message variable
            });

        // Timer to refresh the screen every 200 ms
        screen_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(200),
            [this]() { this->refreshScreen(); });

        // Timer to check the IP address every 5 seconds
        ip_timer_ = this->create_wall_timer(
            std::chrono::seconds(5),
            [this]() { this->updateIPAddress(); });
    }

private:
    void refreshScreen()
    {
        try
        {
            oled_->clear();

            // Display the IP address on the first row
            drawString8x8(SSD1306::OledPoint{0, 0}, first_row_.c_str(), SSD1306::PixelStyle::Set, *oled_);

            // Display the Name address on the second row
            drawString8x8(SSD1306::OledPoint{0, 8}, second_row_.c_str(), SSD1306::PixelStyle::Set, *oled_);

            // Display the message on the third row
            drawString8x8(SSD1306::OledPoint{0, 16}, third_row_.c_str(), SSD1306::PixelStyle::Set, *oled_);

            // Display the message on the fourth row
            drawString8x8(SSD1306::OledPoint{0, 24}, four_row_.c_str(), SSD1306::PixelStyle::Set, *oled_);

            oled_->displayUpdate();
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Error refreshing screen: %s", e.what());
        }
    }

    void updateIPAddress()
    {
        try
        {
            struct ifaddrs *ifAddrStruct = nullptr;
            getifaddrs(&ifAddrStruct);

            std::string new_ip = "No IP";
            std::string network_name = "No Network";

            for (struct ifaddrs *ifa = ifAddrStruct; ifa != nullptr; ifa = ifa->ifa_next)
            {
                if (ifa->ifa_addr && ifa->ifa_addr->sa_family == AF_INET)
                {
                    void *tmpAddrPtr = &((struct sockaddr_in *)ifa->ifa_addr)->sin_addr;
                    char addressBuffer[INET_ADDRSTRLEN];
                    inet_ntop(AF_INET, tmpAddrPtr, addressBuffer, INET_ADDRSTRLEN);

                    if (strncmp(ifa->ifa_name, "eth", 3) == 0)
                    {
                        new_ip = addressBuffer;
                        network_name = "Ethernet";
                        break;
                    }
                    else if (strncmp(ifa->ifa_name, "wlan", 4) == 0)
                    {
                        // Get the ESSID using the iw command
                        std::string essid = "Unknown";
                        std::string command = "iw dev " + std::string(ifa->ifa_name) + " link | grep SSID | awk '{print $2}'";
                        FILE *fp = popen(command.c_str(), "r");
                        if (fp)
                        {
                            char buffer[128];
                            if (fgets(buffer, sizeof(buffer), fp) != nullptr)
                            {
                                essid = std::string(buffer);
                                essid.erase(essid.find_last_not_of(" \n\r\t") + 1); // Trim whitespace
                            }
                            pclose(fp);
                        }

                        new_ip = addressBuffer;
                        network_name = essid;
                        break;
                    }
                }
            }

            if (ifAddrStruct != nullptr)
            {
                freeifaddrs(ifAddrStruct);
            }

            first_row_ = new_ip;       // Update the IP address variable
            second_row_ = network_name; // Update the network name variable
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Error updating IP address: %s", e.what());
        }
    }

    std::unique_ptr<SSD1306::OledI2C> oled_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr screen_timer_;
    rclcpp::TimerBase::SharedPtr ip_timer_;
    std::string first_row_;
    std::string second_row_;
    std::string third_row_;
    std::string four_row_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_first_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_second_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OledDisplayNode>());
    rclcpp::shutdown();
    return 0;
}