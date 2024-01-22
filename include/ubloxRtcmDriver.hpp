#pragma once
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <ros/ros.h>

class UbloxRtcmDriver
{
public:
    int _serial_port = -1;

    explicit UbloxRtcmDriver() = default;
    bool close_serial_port();
    ~UbloxRtcmDriver();

    int initialize_serial_port(const std::string port);

    void set_1005(const std::vector<uint8_t>& bufor)
    {
        std::lock_guard<std::mutex> lock(m);
        data_1005 = bufor;
        c.notify_one();
    }

    std::vector<uint8_t> get_1005() const
    {
        std::unique_lock<std::mutex> lock(m);
        c.wait(lock);
        return data_1005;
    }
private:
    std::vector<uint8_t> data_1005;
    mutable std::mutex m;
    mutable std::condition_variable c;
};