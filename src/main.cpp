#include "../include/ubloxRtcmDriver.hpp"

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

// Vector of messages that we filter out from ublox connected via USB, to be filled in later
std::vector<uint16_t> rtcmMsgIds = {
    1005  // 0x3E 0xD0
};

auto main(int argc, char** argv) -> int
{
    ros::init(argc, argv, "ublox_driver");
    ROS_INFO("Started ublox rtcm driver node.");

    UbloxRtcmDriver rtcm_reader;
    rtcm_reader.initialize_serial_port("/dev/ttyACM0");

    auto usb_thread = std::thread([&] {
        std::vector<uint8_t> bufor(250, 0xFF);
        while (true)
        {
            int num_bytes = read(rtcm_reader._serial_port, bufor.data(), 250);
            ROS_INFO("Received frame with %i bytes.", num_bytes);
            if (num_bytes < 0)
            {
                ROS_ERROR("Error reading frame: %s", strerror(errno));
                return -1;
            }

            //for (const auto& byte : bufor)
            //{
            //    printf("%x ", byte);
            //}
            //printf("\r\n");

            if (num_bytes < 5)
            {
                continue;
            }

            if (bufor[0] == 0xd3 && bufor[3] == 0x3e && bufor[4] == 0xd0) // fuszerka ostra potem poprawic
            {
                ROS_INFO("Found RTCM3 frame in buffer");
                rtcm_reader.set_1005(
                    std::vector<uint8_t>(bufor.begin(), bufor.begin() + num_bytes)
                );
            }

            std::fill(bufor.begin(), bufor.end(), 0xFF);
        }
    });

    // Print content of Ublox message, for debug
    auto rtcm3_printer = std::thread([&rtcm_reader] {
        std::string byteString;
        std::stringstream ss;
        while (true)
        {
            byteString = "";
            auto rtcm3_1005_msg = rtcm_reader.get_1005();
            std::string byte_string = "";
            for (const auto& byte : rtcm3_1005_msg)
            {
                ss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte);
                byteString += ss.str() + " ";
                //printf("%x ", byte);
            }
            ROS_INFO("Received RTCM3 frame:\n%s", byte_string.c_str());
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    });
    while(ros::ok()) {
        ros::spinOnce();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    rtcm_reader.close_serial_port();
    usb_thread.join();
    rtcm3_printer.join();
    return 0;
}
