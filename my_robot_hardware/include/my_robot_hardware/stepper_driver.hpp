#ifndef STEPPER_DRIVER_HPP
#define STEPPER_DRIVER_HPP

#include <cstdio>
#include <cstring>
#include <iostream>
#include <string>

#include <fcntl.h>    // open, O_RDWR, O_NOCTTY, O_NDELAY
#include <termios.h>  // cấu hình cổng serial (baudrate, parity, ...)
#include <unistd.h>   // close, write, read

// Driver serial thuần POSIX (termios) để nói chuyện với vi điều khiển
// trung gian (STM32/Arduino) điều khiển động cơ bước.
// Không phụ thuộc thư viện ngoài (khác XL330Driver dùng dynamixel_sdk).
class StepperDriver
{
public:
    explicit StepperDriver(std::string device_name)
        : device_name_(std::move(device_name)), fd_(-1)
    {
    }

    ~StepperDriver()
    {
        closePort();
    }

    // Mở cổng serial và cấu hình baudrate. Trả về 0 nếu thành công, -1 nếu lỗi.
    int init(int baudrate = 115200)
    {
        fd_ = open(device_name_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        if (fd_ < 0) {
            std::cerr << "[StepperDriver] Khong mo duoc port: " << device_name_ << std::endl;
            return -1;
        }

        // Đưa fd về chế độ blocking (bỏ cờ O_NDELAY) để write()/read() ổn định
        int flags = fcntl(fd_, F_GETFL, 0);
        fcntl(fd_, F_SETFL, flags & ~O_NDELAY);

        struct termios tty{};
        if (tcgetattr(fd_, &tty) != 0) {
            std::cerr << "[StepperDriver] tcgetattr loi" << std::endl;
            close(fd_);
            fd_ = -1;
            return -1;
        }

        speed_t speed = baudrateToSpeed(baudrate);
        cfsetispeed(&tty, speed);
        cfsetospeed(&tty, speed);

        // 8N1, chế độ raw, tắt điều khiển luồng
        tty.c_cflag &= ~PARENB;        // Không parity
        tty.c_cflag &= ~CSTOPB;        // 1 stop bit
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;            // 8 data bit
        tty.c_cflag &= ~CRTSCTS;       // Tắt hardware flow control
        tty.c_cflag |= CREAD | CLOCAL; // Bật đọc, bỏ qua chân modem

        tty.c_lflag &= ~ICANON; // Raw mode, không xử lý theo dòng
        tty.c_lflag &= ~ECHO;
        tty.c_lflag &= ~ECHOE;
        tty.c_lflag &= ~ISIG;

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Tắt software flow control
        tty.c_iflag &= ~(ICRNL | INLCR);        // Không dịch ký tự CR/NL

        tty.c_oflag &= ~OPOST; // Không xử lý output đặc biệt

        tty.c_cc[VMIN]  = 0; // Không chờ tối thiểu byte nào
        tty.c_cc[VTIME] = 1; // Timeout đọc ~100ms

        if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
            std::cerr << "[StepperDriver] tcsetattr loi" << std::endl;
            close(fd_);
            fd_ = -1;
            return -1;
        }

        std::cout << "[StepperDriver] Da mo port " << device_name_
                   << " @ " << baudrate << " baud" << std::endl;
        return 0;
    }

    // Gửi vị trí đích 6 khớp dạng chuỗi text qua serial.
    // Định dạng: "J1:<rad1>,J2:<rad2>,J3:<rad3>,J4:<rad4>,J5:<rad5>,J6:<rad6>\n"
    void sendJointPositions(double j1, double j2, double j3,
                             double j4, double j5, double j6, double gripper)
    {
        if (fd_ < 0) return;

        char buf[192];
        int len = std::snprintf(buf, sizeof(buf),
            "J1:%.6f,J2:%.6f,J3:%.6f,J4:%.6f,J5:%.6f,J6:%.6f, G1:%.6f\n",
            j1, j2, j3, j4, j5, j6, gripper);
        if (len > 0) {
            ssize_t written = write(fd_, buf, static_cast<size_t>(len));
            (void)written; // open-loop: không bắt buộc kiểm tra ACK từ MCU
        }
    }

    void closePort()
    {
        if (fd_ >= 0) {
            close(fd_);
            fd_ = -1;
        }
    }

private:
    static speed_t baudrateToSpeed(int baudrate)
    {
        switch (baudrate) {
            case 9600:   return B9600;
            case 19200:  return B19200;
            case 38400:  return B38400;
            case 57600:  return B57600;
            case 115200: return B115200;
            case 230400: return B230400;
            default:
                std::cerr << "[StepperDriver] Baudrate khong duoc ho tro, dung mac dinh 115200"
                           << std::endl;
                return B115200;
        }
    }

    std::string device_name_;
    int fd_;
};

#endif // STEPPER_DRIVER_HPP