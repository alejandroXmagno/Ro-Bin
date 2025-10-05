/* 
 * This file test the Waveshare JSON commands on Jetson Orin Nano runnin Linux 
 * and JetPack 6.2. No exta libraries are needed 
 * compile: g++ <this_file.cpp> -o <exec_file_name>
 * Author: Ziad Youssfi, April, 2025
 */
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <string.h>
#include <errno.h>

int main() {
    const char* port = "/dev/ttyACM0";  // Change as needed
    int fd = open(port, O_RDWR | O_NOCTTY);

    if (fd < 0) {
        std::cerr << "Failed to open port: " << strerror(errno) << std::endl;
        return 1;
    }

    // Configure serial: 115200 baud
    termios tty;
    if (tcgetattr(fd, &tty) != 0) {
        std::cerr << "tcgetattr error: " << strerror(errno) << std::endl;
        close(fd);
        return 1;
    }

    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);
    
    // Configure 8N1
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
    tty.c_cflag |= CREAD | CLOCAL;                  // Turn on READ and ignore modem ctrl lines
    tty.c_cflag &= ~(PARENB | PARODD);              // No parity
    tty.c_cflag &= ~CSTOPB;                         // 1 stop bit
    tty.c_cflag &= ~CRTSCTS;                        // No hardware flow control

    // Raw input/output mode
    tty.c_lflag = 0;                                // No signaling chars, no echo
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);         // No software flow control
    tty.c_oflag = 0;                                // No remapping, raw output

    tty.c_cc[VMIN]  = 1;                            // Read blocks until 1 char
    tty.c_cc[VTIME] = 1;                            // 0.1s read timeout

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        std::cerr << "Error from tcsetattr: " << strerror(errno) << std::endl;
        close(fd);
        return 1;
    }
    
    // Init JSON parsing on the waveshare base by sending a new line.
    std::string json = "\n";
    ssize_t written = write(fd, json.c_str(), json.size());

    // Forming a JSON string with numberic value that can be passed in as agrument.
    json = R"({"T":13,"X":)" + std::to_string(0.3) + R"(,"Z":)" + std::to_string(2.9) + R"(})";
    json = json + "\n";

    // Send JSON string
    written = write(fd, json.c_str(), json.size());

    if (written < 0) {
        std::cerr << "Write failed: " << strerror(errno) << std::endl;
    } else {
        std::cout << "JSON sent to serial port.\n";
    }
    
    sleep(3);

    // Stop the motors
    json = R"({"T":13,"X":0.0,"Z":0.0})";
    json = json + "\n";

    // Send JSON string
    written = write(fd, json.c_str(), json.size());

    if (written < 0) {
        std::cerr << "Write failed: " << strerror(errno) << std::endl;
    } else {
        std::cout << "JSON sent to serial port.\n";
    }

    close(fd);
    return 0;
}

