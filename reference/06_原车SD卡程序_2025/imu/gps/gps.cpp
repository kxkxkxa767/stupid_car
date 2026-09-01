#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>
#include <cctype>

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

bool is_printable_line(const std::string& line) {
    int cnt = 0;
    for (char c : line) {
        if (isprint(c) || c == '\r' || c == '\n') cnt++;
    }
    return cnt > 5; // 至少6个可见字符
}

int main() {
    const char* port = "/dev/ttyUSB0"; // 根据实际情况修改
    int baudrate = B9600;
    int fd = open_serial(port, baudrate);
    if (fd == -1) {
        cerr << "串口打开失败" << endl;
        return 1;
    }
    char buf[256];
    string line;
    while (true) {
        int n = read(fd, buf, sizeof(buf) - 1);
        if (n > 0) {
            buf[n] = '\0';
            for (int i = 0; i < n; ++i) {
                line += buf[i];
                if (buf[i] == '\n' || buf[i] == '\r') {
                    if (is_printable_line(line)) {
                        cout << "[ASCII] " << line;
                    }
                    line.clear();
                }
            }
        }
        usleep(10000); // 10ms
    }
    close(fd);
    return 0;
}