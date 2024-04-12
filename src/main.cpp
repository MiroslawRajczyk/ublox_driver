#include "../include/ubloxRtcmDriver.hpp"
#include <rtcm_msgs/Message.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/UInt8.h>

UbloxRtcmDriver::~UbloxRtcmDriver() {
    close_serial_port();
}

bool UbloxRtcmDriver::close_serial_port() {
    close(_serial_port);
}

int UbloxRtcmDriver::initialize_serial_port(const std::string port) {
    _serial_port = open(port.c_str(), O_RDWR);
    if (_serial_port < 0)
    {
        ROS_ERROR("Error opening COM ublox port.");
        return -1;
    }

    struct termios tty;

    if (tcgetattr(_serial_port, &tty) != 0)
    {
        ROS_ERROR("Error on reading settings.");
        return -1;
    }

    tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
    tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
    tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size
    tty.c_cflag |= CS8; // 8 bits per byte (most common)
    tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO; // Disable echo
    tty.c_lflag &= ~ECHOE; // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed

    tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    tty.c_cc[VMIN] = 0;

    cfsetispeed(&tty, B9600);
    cfsetospeed(&tty, B9600);

    if (tcsetattr(_serial_port, TCSANOW, &tty) != 0)
    {
        printf("Cwel\r\n");
        return -1;
    }
}

auto main(int argc, char** argv) -> int {
    ros::init(argc, argv, "ublox_rtk_base_driver");
    ROS_INFO("Started ublox rtcm driver node.");

    UbloxRtcmDriver rtcm_reader;
    rtcm_reader.initialize_serial_port("/dev/ttyACM1");

    auto usb_thread = std::thread([&] {
        ros::NodeHandle nh = ros::NodeHandle("ublox_rtk_base_driver");
        ros::Publisher fixPublisher = nh.advertise<sensor_msgs::NavSatFix>("fix", 1, true);
        ros::Publisher satellitesNumberPublisher = nh.advertise<std_msgs::UInt8>("satellites_number", 1, true);
        std::vector<uint8_t> bufor(1250, 0xFF);
        while (ros::ok()) {
            int num_bytes = read(rtcm_reader._serial_port, bufor.data(), 250);
            if (num_bytes < 5) {
                ROS_WARN("Error reading frame");
                continue;
            }

            if (rtcm_reader.is_nav_pvt_message(bufor)) {
                if (num_bytes < 36) {
                    ROS_ERROR("Invalid NAV-PVT message: Data size is less than expected.");
                    continue;
                } else {
                    // Extract fields from the message
                    rtcm_reader.gpsData.iTOW = (bufor[9] << 24) | (bufor[8] << 16) | (bufor[7] << 8) | bufor[6];
                    rtcm_reader.gpsData.year = (bufor[11] << 8) | bufor[10];
                    rtcm_reader.gpsData.month = bufor[12];
                    rtcm_reader.gpsData.day = bufor[13];
                    rtcm_reader.gpsData.hour = bufor[14];
                    rtcm_reader.gpsData.min = bufor[15];
                    rtcm_reader.gpsData.sec = bufor[16];
                    rtcm_reader.gpsData.valid = bufor[17];
                    rtcm_reader.gpsData.tAcc = (bufor[21] << 24) | (bufor[20] << 16) | (bufor[19] << 8) | bufor[18]; // in ns
                    rtcm_reader.gpsData.fixType = bufor[26]; // 0 - no fix, 1 - dead reckoning only, 2 - 2D-fix, 3 - 3D-fix, 4 - GNSS + dead reckoning, 5 - time only fix
                    rtcm_reader.gpsData.numSV = bufor[29]; // number of satellites used in Nav Solution
                    rtcm_reader.gpsData.lon = (bufor[33] << 24) | (bufor[32] << 16) | (bufor[31] << 8) | bufor[30]; // scale by 1e-7, in deg
                    rtcm_reader.gpsData.lat = (bufor[37] << 24) | (bufor[36] << 16) | (bufor[35] << 8) | bufor[34]; // scale by 1e-7, in deg
                    rtcm_reader.gpsData.height = (bufor[41] << 24) | (bufor[40] << 16) | (bufor[39] << 8) | bufor[38]; // in mm
                    rtcm_reader.gpsData.hMSL = (bufor[45] << 24) | (bufor[44] << 16) | (bufor[43] << 8) | bufor[42]; // in mm
                    rtcm_reader.gpsData.hAcc = (bufor[49] << 24) | (bufor[48] << 16) | (bufor[47] << 8) | bufor[46]; // in mm
                    rtcm_reader.gpsData.vAcc = (bufor[53] << 24) | (bufor[52] << 16) | (bufor[51] << 8) | bufor[50]; // in mm
                    rtcm_reader.gpsData.pDOP = (bufor[83] << 8) | bufor[82];
                    rtcm_reader.gpsData.magDec = (bufor[95] << 8) | bufor[94]; // in deg
                    rtcm_reader.gpsData.magDecAcc = (bufor[97] << 8) | bufor[96]; // in mm

                    sensor_msgs::NavSatFix fixMsg;
                    fixMsg.header.stamp = ros::Time::now();  // Timestamp
                    fixMsg.header.frame_id = "map";
                    if (rtcm_reader.gpsData.fixType == 0) {
                        fixMsg.status.status = -1;
                    } else {
                        fixMsg.status.status = 0;
                    }
                    fixMsg.status.service = 1;
                    fixMsg.latitude = rtcm_reader.gpsData.lat / 10000000.0;
                    fixMsg.longitude = rtcm_reader.gpsData.lon / 10000000.0;
                    fixMsg.altitude = rtcm_reader.gpsData.hMSL / 1000;
                    fixMsg.position_covariance_type = 2; // DIAGONAL_KNOWN
                    const double varH = pow(rtcm_reader.gpsData.hAcc / 10000.0, 2);
                    const double varV = pow(rtcm_reader.gpsData.vAcc / 10000.0, 2);
                    fixMsg.position_covariance[0] = varH;
                    fixMsg.position_covariance[4] = varH;
                    fixMsg.position_covariance[8] = varV;
                    fixPublisher.publish(fixMsg);
                    std_msgs::UInt8 satNumMsg;
                    satNumMsg.data = rtcm_reader.gpsData.numSV;
                    satellitesNumberPublisher.publish(satNumMsg);

                    // Output parsed data
                    //ROS_INFO("iTOW: %u", rtcm_reader.gpsData.iTOW);
                    //ROS_INFO("Year: %u", rtcm_reader.gpsData.year);
                    //ROS_INFO("Month: %u", rtcm_reader.gpsData.month);
                    //ROS_INFO("Day: %u", rtcm_reader.gpsData.day);
                    //ROS_INFO("Hour: %u", rtcm_reader.gpsData.hour);
                    //ROS_INFO("Minute: %u", rtcm_reader.gpsData.min);
                    //ROS_INFO("Second: %u", rtcm_reader.gpsData.sec);
                    //ROS_INFO("Validity Flags: %u", rtcm_reader.gpsData.valid);
                    //ROS_INFO("Time Accuracy: %uns", rtcm_reader.gpsData.tAcc);
                    //ROS_INFO("Fix Type: %u", rtcm_reader.gpsData.fixType);
                    //ROS_INFO("Number of satellites in Nav Solution: %u", rtcm_reader.gpsData.numSV);
                    //ROS_INFO("Latitude: %f", rtcm_reader.gpsData.lat / 10000000.0);
                    //ROS_INFO("Longitude: %f", rtcm_reader.gpsData.lon / 10000000.0);
                    //ROS_INFO("Height above ellipsoid: %f m", rtcm_reader.gpsData.height / 1000);
                    //ROS_INFO("Height above mean sea level: %f m", rtcm_reader.gpsData.hMSL / 1000);
                    //ROS_INFO("Horizontal accuracy estimate: %f m", rtcm_reader.gpsData.hAcc / 1000);
                    //ROS_INFO("Vertical accuracy estimate: %f m", rtcm_reader.gpsData.vAcc / 1000);
                    //ROS_INFO("Position DOP: %f m", rtcm_reader.gpsData.pDOP / 100);
                    //ROS_INFO("Magnetic declination: %f m", rtcm_reader.gpsData.magDec / 100);
                    //ROS_INFO("Magnetic declination accuracy: %f m", rtcm_reader.gpsData.magDecAcc / 100);
                    continue;
                }
            }
            if (rtcm_reader.is_rtcm_message(bufor)) {
                uint32_t messagePayloadSize = (((uint32_t)bufor[1] & 3) << 8) + ((uint32_t)bufor[2] << 0) + 6; // Header + CRC bytes
                if (messagePayloadSize != num_bytes) { continue; }
                uint16_t rtcmMessageType = ((uint16_t)bufor[3] << 4) + ((uint16_t)bufor[4] >> 4); // 12-bit (0..4095)
                ROS_INFO("RTCM3 frame %4d received, length: %4d", rtcmMessageType, messagePayloadSize);
                if (rtcmMessageType == 1005) {
                    rtcm_reader.setRtcmData(std::vector<uint8_t>(bufor.begin(), bufor.begin() + num_bytes));
                }
            }
            std::fill(bufor.begin(), bufor.end(), 0xFF);
        }
        rtcm_reader.setRtcmData(std::vector<uint8_t>(bufor.begin(), bufor.begin() + 1));
    });

    // Print content of Ublox message, for debug
    auto rtcm3_printer = std::thread([&rtcm_reader] {
        ros::NodeHandle nh = ros::NodeHandle("ublox_rtk_base_driver");
        ros::Publisher rtcmPublisher = nh.advertise<rtcm_msgs::Message>("/rtcm", 1, true);
        std::vector<uint8_t> rtcmMessage;
        uint32_t messagePayloadSize = 0;
        uint16_t rtcmMessageType = 0;
        while (ros::ok()) {
            rtcm_msgs::Message rtcmRosMsg;
            rtcmRosMsg.header.stamp = ros::Time::now();  // Timestamp
            rtcmRosMsg.header.frame_id = "map";  // Frame ID
            rtcmMessage = rtcm_reader.getRtcmData();
            messagePayloadSize = (((uint32_t)rtcmMessage[1] & 3) << 8) + ((uint32_t)rtcmMessage[2] << 0) + 6;
            rtcmMessageType = ((uint16_t)rtcmMessage[3] << 4) + ((uint16_t)rtcmMessage[4] >> 4); // 12-bit (0..4095)
            // copy message bytes into ROS message
            rtcmRosMsg.message.insert(rtcmRosMsg.message.end(), rtcmMessage.begin(), rtcmMessage.end());
            rtcmPublisher.publish(rtcmRosMsg);
        }
    });

    while(ros::ok()) {
        ros::spinOnce();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    rtcm_reader.close_serial_port();
    usb_thread.join();
    rtcm3_printer.join();
    return 0;
}
