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

struct ubxNavPvtData {
    uint32_t iTOW = 0;
    uint16_t year = 0;
    uint8_t month = 0;
    uint8_t day = 0;
    uint8_t hour = 0;
    uint8_t min = 0;
    uint8_t sec = 0;
    uint8_t valid = 0;
    uint32_t tAcc = 0;
    uint8_t fixType = 0;
    uint8_t numSV = 0;
    int32_t lon = 0;
    int32_t lat = 0;
    int32_t height = 0;
    int32_t hMSL = 0;
    uint32_t hAcc = 0;
    uint32_t vAcc = 0;
    uint16_t pDOP = 0;
    int16_t magDec = 0;
    uint16_t magDecAcc = 0;
};

class UbloxRtcmDriver
{
public:
    int _serial_port = -1;
    ubxNavPvtData gpsData;

    explicit UbloxRtcmDriver() = default;
    bool close_serial_port();
    ~UbloxRtcmDriver();

    int initialize_serial_port(const std::string port);

    void setRtcmData(const std::vector<uint8_t>& bufor)
    {
        std::lock_guard<std::mutex> lock(m);
        rtcmData = bufor;
        c.notify_one();
    }

    std::vector<uint8_t> getRtcmData() const
    {
        std::unique_lock<std::mutex> lock(m);
        c.wait(lock);
        return rtcmData;
    }

    bool is_nav_pvt_message(const std::vector<uint8_t>& data) {
        // Check if the data contains UBX message header and is of type NAV-PVT
        if (data.size() < 8) {
            return false;
        }
        return (data[2] == 0x01 && data[3] == 0x07); // UBX message header for NAV-PVT
    }

    bool is_rtcm_message(const std::vector<uint8_t>& data) {
        // Check if the data contains UBX message header and is of type NAV-PVT
        if (data.size() < 6) {
            return false;
        }
        return (data[0] == 0xD3) && ((data[1] & 0xFC) == 0x00); // RTCM message header
    }
private:
    std::vector<uint8_t> rtcmData;
    mutable std::mutex m;
    mutable std::condition_variable c;
};