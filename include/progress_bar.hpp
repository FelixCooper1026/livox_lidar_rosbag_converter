#ifndef PROGRESS_BAR_HPP
#define PROGRESS_BAR_HPP

#include <iostream>
#include <chrono>
#include <string>
#include <iomanip>
#include <sstream>
#include <sys/ioctl.h>
#include <unistd.h>

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
    int get_terminal_width() {
        struct winsize w;
        ioctl(STDOUT_FILENO, TIOCGWINSZ, &w);
        // 返回终端宽度，如果获取失败则返回默认值70
        return w.ws_col > 0 ? w.ws_col - 10 : 70;  // 减去10是为了留出边距
    }

    void print() {
        float progress = static_cast<float>(current_) / total_;
        int terminal_width = get_terminal_width();
        // 动态计算进度条宽度，确保不会超出终端宽度
        int bar_width = std::min(50, terminal_width - 60);  // 60是其他信息的预估宽度
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
            if (i < filled_width - 1) ss << "=";
            else if (i == filled_width - 1) ss << ">";
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