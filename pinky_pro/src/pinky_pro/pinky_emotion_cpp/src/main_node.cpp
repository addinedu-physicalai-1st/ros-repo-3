#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <map>
#include <filesystem>
#include <algorithm>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "pinky_interfaces/srv/emotion.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include <opencv2/opencv.hpp>
#include <wiringPi.h>
#include <wiringPiSPI.h>

using namespace std::chrono_literals;
namespace fs = std::filesystem;

// Pin Definitions
#define RST_PIN  27
#define DC_PIN   25
#define BL_PIN   18

// LCD Resolution
#define LCD_W 240
#define LCD_H 320

class PinkyLCD {
public:
    PinkyLCD() {
        wiringPiSetupGpio();
        pinMode(RST_PIN, OUTPUT);
        pinMode(DC_PIN, OUTPUT);
        pinMode(BL_PIN, OUTPUT);

        // Hardware SPI setup (bus 0, ce 0, 80MHz)
        if (wiringPiSPISetup(0, 80000000) < 0) {
            throw std::runtime_error("wiringPiSPISetup failed");
        }

        lcd_init();
        digitalWrite(BL_PIN, HIGH); // Backlight 100%
    }

    void write_cmd(uint8_t cmd) {
        digitalWrite(DC_PIN, LOW);
        wiringPiSPIDataRW(0, &cmd, 1);
    }

    void write_data(uint8_t data) {
        digitalWrite(DC_PIN, HIGH);
        wiringPiSPIDataRW(0, &data, 1);
    }

    void write_data_buffer(uint8_t *buf, int len) {
        digitalWrite(DC_PIN, HIGH);
        // SPI chunking if necessary, but wiringPi handle it
        wiringPiSPIDataRW(0, buf, len);
    }

    void reset() {
        digitalWrite(RST_PIN, HIGH);
        delay(10);
        digitalWrite(RST_PIN, LOW);
        delay(10);
        digitalWrite(RST_PIN, HIGH);
        delay(10);
    }

    void lcd_init() {
        reset();
        write_cmd(0x11); // Sleep out
        delay(120);

        // Standard ILI9341 Init sequence
        write_cmd(0xCF); write_data(0x00); write_data(0xC1); write_data(0X30);
        write_cmd(0xED); write_data(0x64); write_data(0x03); write_data(0X12); write_data(0X81);
        write_cmd(0xE8); write_data(0x85); write_data(0x00); write_data(0x79);
        write_cmd(0xCB); write_data(0x39); write_data(0x2C); write_data(0x00); write_data(0x34); write_data(0x02);
        write_cmd(0xF7); write_data(0x20);
        write_cmd(0xEA); write_data(0x00); write_data(0x00);
        write_cmd(0xC0); write_data(0x1D);
        write_cmd(0xC1); write_data(0x12);
        write_cmd(0xC5); write_data(0x33); write_data(0x3F);
        write_cmd(0xC7); write_data(0x92);
        write_cmd(0x3A); write_data(0x55); // RGB565
        write_cmd(0x36); write_data(0x08); // Memory Access
        write_cmd(0xB1); write_data(0x00); write_data(0x12);
        write_cmd(0xB6); write_data(0x0A); write_data(0xA2);
        write_cmd(0x44); write_data(0x02);
        write_cmd(0xF2); write_data(0x00);
        write_cmd(0x26); write_data(0x01);
        write_cmd(0x29); // Display on
    }

    void set_windows(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
        write_cmd(0x2A);
        write_data(x0 >> 8); write_data(x0 & 0xFF);
        write_data((x1-1) >> 8); write_data((x1-1) & 0xFF);
        write_cmd(0x2B);
        write_data(y0 >> 8); write_data(y0 & 0xFF);
        write_data((y1-1) >> 8); write_data((y1-1) & 0xFF);
        write_cmd(0x2C);
    }

    // Optimization: Efficient RGB888 to RGB565 conversion with bitmasking
    void display_mat(cv::Mat &frame) {
        cv::Mat resized;
        cv::resize(frame, resized, cv::Size(LCD_W, LCD_H));

        std::vector<uint8_t> buffer(LCD_W * LCD_H * 2);
        for (int i = 0; i < LCD_W * LCD_H; ++i) {
            cv::Vec3b pixel = resized.at<cv::Vec3b>(i);
            // RGB888 to RGB565 (Big Endian for LCD)
            // R (5 bits), G (6 bits), B (5 bits)
            uint16_t rgb565 = ((pixel[2] & 0xF8) << 8) | ((pixel[1] & 0xFC) << 3) | (pixel[0] >> 3);
            buffer[i * 2] = (rgb565 >> 8) & 0xFF;
            buffer[i * 2 + 1] = rgb565 & 0xFF;
        }

        set_windows(0, 0, LCD_W, LCD_H);
        write_data_buffer(buffer.data(), buffer.size());
    }

    void clear() {
        std::vector<uint8_t> black(LCD_W * LCD_H * 2, 0);
        set_windows(0, 0, LCD_W, LCD_H);
        write_data_buffer(black.data(), black.size());
    }
};

class PinkyEmotionNode : public rclcpp::Node {
public:
    PinkyEmotionNode() : Node("pinky_emotion_cpp") {
        this->declare_parameter<int>("load_frame_skip", 2);
        load_frame_skip_ = this->get_parameter("load_frame_skip").as_int();

        lcd_ = std::make_unique<PinkyLCD>();
        
        std::string pkg_share = ament_index_cpp::get_package_share_directory("pinky_emotion");
        emotion_dir_ = fs::path(pkg_share) / "emotion";

        preload_emotions();

        emotion_service_ = this->create_service<pinky_interfaces::srv::Emotion>(
            "set_emotion", std::bind(&PinkyEmotionNode::handle_set_emotion, this, std::placeholders::_1, std::placeholders::_2));

        timer_ = this->create_wall_timer(100ms, std::bind(&PinkyEmotionNode::timer_callback, this));
        
        // Default emotion
        current_frames_ = emotion_cache_["happy"];
        
        RCLCPP_INFO(this->get_logger(), "C++ Emotion Server Ready.");
    }

private:
    void preload_emotions() {
        RCLCPP_INFO(this->get_logger(), "Preloading GIFs...");
        for (const auto &entry : fs::directory_iterator(emotion_dir_)) {
            if (entry.path().extension() == ".gif") {
                std::string name = entry.path().stem().string();
                cv::VideoCapture cap(entry.path().string());
                std::vector<cv::Mat> frames;
                
                cv::Mat frame;
                int count = 0;
                while (cap.read(frame)) {
                    if (count++ % load_frame_skip_ == 0) {
                        frames.push_back(frame.clone());
                    }
                }
                emotion_cache_[name] = frames;
                RCLCPP_INFO(this->get_logger(), "  - Loaded '%s' (%zu frames)", name.c_str(), frames.size());
            }
        }
    }

    void handle_set_emotion(const std::shared_ptr<pinky_interfaces::srv::Emotion::Request> request,
                           std::shared_ptr<pinky_interfaces::srv::Emotion::Response> response) {
        std::string emo = request->emotion;
        std::lock_guard<std::mutex> lock(mutex_);
        if (emotion_cache_.count(emo)) {
            current_frames_ = emotion_cache_[emo];
            current_frame_idx_ = 0;
            response->response = "Emotion set to " + emo;
        } else {
            response->response = "Emotion not found";
        }
    }

    void timer_callback() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (current_frames_.empty()) return;

        lcd_->display_mat(current_frames_[current_frame_idx_]);
        current_frame_idx_ = (current_frame_idx_ + 1) % current_frames_.size();
    }

    std::unique_ptr<PinkyLCD> lcd_;
    int load_frame_skip_;
    fs::path emotion_dir_;
    std::map<std::string, std::vector<cv::Mat>> emotion_cache_;
    std::vector<cv::Mat> current_frames_;
    size_t current_frame_idx_ = 0;
    std::mutex mutex_;

    rclcpp::Service<pinky_interfaces::srv::Emotion>::SharedPtr emotion_service_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<PinkyEmotionNode>());
    } catch (const std::exception &e) {
        RCLCPP_FATAL(rclcpp::get_logger("pinky_emotion_cpp"), "Fatal: %s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}
