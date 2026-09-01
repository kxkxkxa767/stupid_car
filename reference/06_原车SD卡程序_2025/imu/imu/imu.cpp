#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>
#include <cmath>

using namespace std;

int open_serial(const char* port, int baudrate) {
    int fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) return -1;
    struct termios options;
    tcgetattr(fd, &options);
    cfsetispeed(&options, baudrate);
    cfsetospeed(&options, baudrate);
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CRTSCTS;
    options.c_iflag = IGNPAR;
    options.c_oflag = 0;
    options.c_lflag = 0;
    tcflush(fd, TCIFLUSH);
    tcsetattr(fd, TCSANOW, &options);
    return fd;
}

short to_short(unsigned char low, unsigned char high) {
    return (short)((high << 8) | low);
}

bool check_sum(const unsigned char* data) {
    unsigned char sum = 0;
    for (int i = 0; i < 10; ++i) sum += data[i];
    return (sum & 0xFF) == data[10];
}

int main() {
    const char* port = "/dev/ttyUSB0";
    int baudrate = B9600; // 可改为 B115200 等
    int fd = open_serial(port, baudrate);
    if (fd == -1) {
        cerr << "串口打开失败" << endl;
        return 1;
    }
    unsigned char buf[256];
    int key = 0;
    unsigned char frame[11];
    while (true) {
        int n = read(fd, buf, sizeof(buf));
        for (int i = 0; i < n; ++i) {
            if (key == 0 && buf[i] != 0x55) continue;
            frame[key++] = buf[i];
            if (key == 11) {
                if (!check_sum(frame)) {
                    key = 0;
                    continue;
                }
                if (frame[1] == 0x51) {
                    // 加速度
                    short ax = to_short(frame[2], frame[3]);
                    short ay = to_short(frame[4], frame[5]);
                    short az = to_short(frame[6], frame[7]);
                    float fax = ax / 32768.0f * 16 * 9.8f;
                    float fay = ay / 32768.0f * 16 * 9.8f;
                    float faz = az / 32768.0f * 16 * 9.8f;
                    cout << "加速度(m/s²): x=" << fax << " y=" << fay << " z=" << faz << endl;
                } else if (frame[1] == 0x52) {
                    // 角速度
                    short wx = to_short(frame[2], frame[3]);
                    short wy = to_short(frame[4], frame[5]);
                    short wz = to_short(frame[6], frame[7]);
                    float fwx = wx / 32768.0f * 2000 * M_PI / 180;
                    float fwy = wy / 32768.0f * 2000 * M_PI / 180;
                    float fwz = wz / 32768.0f * 2000 * M_PI / 180;
                    cout << "角速度(rad/s): x=" << fwx << " y=" << fwy << " z=" << fwz << endl;
                } else if (frame[1] == 0x53) {
                    // 欧拉角
                    short roll = to_short(frame[2], frame[3]);
                    short pitch = to_short(frame[4], frame[5]);
                    short yaw = to_short(frame[6], frame[7]);
                    float froll = roll / 32768.0f * 180;
                    float fpitch = pitch / 32768.0f * 180;
                    float fyaw = yaw / 32768.0f * 180;
                    cout << "欧拉角(°): roll=" << froll << " pitch=" << fpitch << " yaw=" << fyaw << endl;
                }
                key = 0;
            }
        }
        usleep(1000); // 防止CPU占用过高
    }
    close(fd);
    return 0;
}