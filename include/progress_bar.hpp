#ifndef PROGRESS_BAR_HPP
#define PROGRESS_BAR_HPP

#include <iostream>
#include <chrono>
#include <string>
#include <iomanip>
#include <sstream>

class ProgressBar {
public:
    ProgressBar(int total, const std::string& description = "Progress")
        : total_(total), current_(0), description_(description) {
        start_time_ = std::chrono::steady_clock::now();
        // 打印初始进度条
        print();
    }

    void update(int increment = 1) {
        current_ += increment;
        print();
    }

    void finish() {
        current_ = total_;
        print();
        std::cout << std::endl;
    }

private:
    void print() {
        float progress = static_cast<float>(current_) / total_;
        int bar_width = 50;
        int filled_width = static_cast<int>(bar_width * progress);

        // 计算速度
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start_time_).count();
        float speed = elapsed > 0 ? static_cast<float>(current_) / elapsed : 0;

        // 计算剩余时间
        int remaining = speed > 0 ? static_cast<int>((total_ - current_) / speed) : 0;

        // 构建进度条字符串
        std::stringstream ss;
        ss << "\r" << description_ << " [";
        for (int i = 0; i < bar_width; ++i) {
            if (i < filled_width) ss << "=";
            else ss << " ";
        }
        ss << "] " << std::fixed << std::setprecision(1) 
           << (progress * 100.0) << "% "
           << current_ << "/" << total_ << " messages"
           << " (" << std::fixed << std::setprecision(1) << speed << " msg/s)";

        // 添加剩余时间
        if (remaining > 0) {
            int hours = remaining / 3600;
            int minutes = (remaining % 3600) / 60;
            int seconds = remaining % 60;
            ss << " ETA: ";
            if (hours > 0) ss << hours << "h ";
            if (minutes > 0) ss << minutes << "m ";
            ss << seconds << "s";
        }

        // 添加足够的空格来清除旧内容
        ss << std::string(20, ' ');

        // 输出并刷新
        std::cout << ss.str() << std::flush;
    }

    int total_;
    int current_;
    std::string description_;
    std::chrono::steady_clock::time_point start_time_;
};

#endif // PROGRESS_BAR_HPP 