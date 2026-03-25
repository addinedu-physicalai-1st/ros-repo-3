#include <memory>
#include <string>
#include <vector>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "pinky_interfaces/srv/set_led.hpp"
#include "pinky_interfaces/srv/set_brightness.hpp"

extern "C" {
#include "ws2811.h"
}

// Default Configuration
#define TARGET_FREQ             WS2811_TARGET_FREQ
#define GPIO_PIN                18
#define DMA                     10
#define STRIP_TYPE              WS2811_STRIP_GRB
#define LED_COUNT               8

class LedServiceServer : public rclcpp::Node
{
public:
    LedServiceServer() : Node("led_service_server_cpp")
    {
        // Parameters
        this->declare_parameter<int>("led_count", LED_COUNT);
        this->declare_parameter<int>("gpio_pin", GPIO_PIN);
        this->declare_parameter<int>("dma_channel", DMA);
        this->declare_parameter<int>("brightness", 255);

        led_count_ = this->get_parameter("led_count").as_int();
        int gpio_pin = this->get_parameter("gpio_pin").as_int();
        int dma_channel = this->get_parameter("dma_channel").as_int();
        int initial_brightness = this->get_parameter("brightness").as_int();

        // Initialize ws2811 structure
        led_string_ = {
            .freq = TARGET_FREQ,
            .dmanum = dma_channel,
            .channel = {
                {
                    .gpionum = gpio_pin,
                    .invert = 0,
                    .count = led_count_,
                    .strip_type = STRIP_TYPE,
                    .brightness = static_cast<uint8_t>(initial_brightness),
                },
                {
                    .gpionum = 0,
                    .invert = 0,
                    .count = 0,
                    .strip_type = 0,
                    .brightness = 0,
                }
            }
        };

        if (ws2811_init(&led_string_) != WS2811_SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "ws2811_init failed");
            throw std::runtime_error("ws2811_init failed");
        }

        // Services
        led_service_ = this->create_service<pinky_interfaces::srv::SetLed>(
            "set_led", std::bind(&LedServiceServer::handle_set_led, this, std::placeholders::_1, std::placeholders::_2));
        
        brightness_service_ = this->create_service<pinky_interfaces::srv::SetBrightness>(
            "set_brightness", std::bind(&LedServiceServer::handle_set_brightness, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "C++ LED control service server is ready. (LEDs: %d, GPIO: %d)", led_count_, gpio_pin);
    }

    ~LedServiceServer()
    {
        clear_all();
        ws2811_fini(&led_string_);
    }

private:
    void handle_set_led(const std::shared_ptr<pinky_interfaces::srv::SetLed::Request> request,
                        std::shared_ptr<pinky_interfaces::srv::SetLed::Response> response)
    {
        std::string command = request->command;
        std::transform(command.begin(), command.end(), command.begin(), ::tolower);

        // Optimization: Use bitmasking for color packing (ARGB 32-bit format expected by ws2811)
        ws2811_led_t color = (static_cast<uint32_t>(request->r) << 16) |
                             (static_cast<uint32_t>(request->g) << 8) |
                             static_cast<uint32_t>(request->b);

        bool needs_render = false;

        if (command == "set_pixel") {
            for (int32_t idx : request->pixels) {
                if (idx >= 0 && idx < led_count_) {
                    led_string_.channel[0].leds[idx] = color;
                    needs_render = true;
                }
            }
            if (needs_render) {
                response->message = "Set pixel(s) successfully.";
            } else {
                response->success = false;
                response->message = "Invalid pixel indices.";
                return;
            }
        } else if (command == "fill") {
            for (int i = 0; i < led_count_; i++) {
                led_string_.channel[0].leds[i] = color;
            }
            needs_render = true;
            response->message = "Filled all LEDs.";
        } else if (command == "clear") {
            clear_all();
            response->message = "Cleared all LEDs.";
            response->success = true;
            return;
        } else {
            response->success = false;
            response->message = "Unknown command: " + request->command;
            return;
        }

        if (needs_render) {
            if (ws2811_render(&led_string_) == WS2811_SUCCESS) {
                response->success = true;
            } else {
                response->success = false;
                response->message = "ws2811_render failed.";
            }
        }
    }

    void handle_set_brightness(const std::shared_ptr<pinky_interfaces::srv::SetBrightness::Request> request,
                               std::shared_ptr<pinky_interfaces::srv::SetBrightness::Response> response)
    {
        led_string_.channel[0].brightness = static_cast<uint8_t>(std::clamp(request->brightness, 0, 255));
        
        if (ws2811_render(&led_string_) == WS2811_SUCCESS) {
            response->success = true;
            response->message = "Brightness set to " + std::to_string(led_string_.channel[0].brightness);
        } else {
            response->success = false;
            response->message = "ws2811_render failed.";
        }
    }

    void clear_all()
    {
        for (int i = 0; i < led_count_; i++) {
            led_string_.channel[0].leds[i] = 0;
        }
        ws2811_render(&led_string_);
    }

    ws2811_t led_string_;
    int led_count_;
    rclcpp::Service<pinky_interfaces::srv::SetLed>::SharedPtr led_service_;
    rclcpp::Service<pinky_interfaces::srv::SetBrightness>::SharedPtr brightness_service_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<LedServiceServer>());
    } catch (const std::exception &e) {
        RCLCPP_FATAL(rclcpp::get_logger("pinky_led_cpp"), "Exception: %s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}
