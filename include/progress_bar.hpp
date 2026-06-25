#ifndef PROGRESS_BAR_HPP
#define PROGRESS_BAR_HPP

#include <chrono>
#include <iomanip>
#include <iostream>
#include <string>

class ProgressBar {
public:
  ProgressBar(int total, const std::string & description = "Progress")
    : total_(total), current_(0), description_(description), finished_(false),
      last_print_time_(std::chrono::steady_clock::now())
  {
    print(true);
  }

  void update(int increment = 1)
  {
    current_ += increment;
    if (current_ > total_) {
      current_ = total_;
    }

    const auto now = std::chrono::steady_clock::now();
    const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_print_time_).count();
    if (elapsed_ms >= 100 || current_ >= total_) {
      print(false);
    }
  }

  void finish()
  {
    current_ = total_;
    if (!finished_) {
      print(false);
    }
    std::cout << std::endl;
    finished_ = true;
  }

private:
  int progress_percent() const
  {
    if (total_ == 0) {
      return 100;
    }
    return static_cast<int>((static_cast<double>(current_) * 100.0) / static_cast<double>(total_));
  }

  std::string progress_bar() const
  {
    const int bar_width = 36;
    const int filled_width = (bar_width * progress_percent()) / 100;

    std::string bar;
    bar.reserve(bar_width);
    for (int i = 0; i < bar_width; ++i) {
      if (i < filled_width) {
        bar += '=';
      } else if (i == filled_width && filled_width < bar_width) {
        bar += '>';
      } else {
        bar += ' ';
      }
    }
    return bar;
  }

  void print(bool force)
  {
    const auto now = std::chrono::steady_clock::now();
    if (!force && current_ < total_) {
      const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_print_time_).count();
      if (elapsed_ms < 100) {
        return;
      }
    }

    std::cout << "\r" << description_ << " "
              << "[" << progress_bar() << "] "
              << std::setw(3) << progress_percent() << "% "
              << current_ << "/" << total_ << std::flush;

    last_print_time_ = now;
    if (current_ >= total_) {
      finished_ = true;
    }
  }

  int total_;
  int current_;
  std::string description_;
  bool finished_;
  std::chrono::steady_clock::time_point last_print_time_;
};

#endif  // PROGRESS_BAR_HPP
