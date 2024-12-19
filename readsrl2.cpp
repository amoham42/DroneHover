#include <iostream>
#include <iomanip>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <vector>
#include <cstdint>
#include <cstring>

// MSP v2 Header Constants
const uint8_t MSP_HEADER1 = '$';
const uint8_t MSP_HEADER2 = 'X';
const uint8_t MSP_HEADER3 = '<';

struct __attribute__((packed)) MSP_Message {
    uint8_t flag;
    uint16_t function;
    uint16_t size;
    char *payload;
};

struct __attribute__((packed)) MSP_OPFLOW_Message {
    uint8_t quality; // [0;255]
    int32_t motionX;
    int32_t motionY;
};

struct __attribute__((packed)) MSP_RF_Message {
    uint8_t quality;    // [0;255]
    int32_t distanceMm; // Negative value for out of range
};


uint8_t crc8_dvb_s2(uint8_t crc, unsigned char a) {
    crc ^= a;
    for (int ii = 0; ii < 8; ++ii) {
        
        if (crc & 0x80) {
            crc = (crc << 1) ^ 0xD5;
        } else {
            crc = crc << 1;
        }
    }
    return crc;
}

uint8_t calculateCRC(const uint8_t *buf, uint16_t len) {
    uint8_t ck2 = 0;
    for (int i = 3; i < 8 + len; i++) {
        ck2 = crc8_dvb_s2(ck2, buf[i]);
    }
    return ck2;
}

int configureUART(const char *device, int baudrate) {
    int fd = open(device, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        std::cerr << "Error opening " << device << std::endl;
        return -1;
    }

    struct termios tty;
    memset(&tty, 0, sizeof tty);

    if (tcgetattr(fd, &tty) != 0) {
        std::cerr << "Error from tcgetattr" << std::endl;
        close(fd);
        return -1;
    }

    speed_t speed;
    switch (baudrate) {
    case 9600:   speed = B9600;   break;
    case 19200:  speed = B19200;  break;
    case 38400:  speed = B38400;  break;
    case 57600:  speed = B57600;  break;
    case 115200: speed = B115200; break;

    default:
        std::cerr << "Unsupported baud rate" << std::endl;
        close(fd);
        return -1;
    }

    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit chars
    tty.c_iflag &= ~IGNBRK;                     // disable break processing
    tty.c_lflag = 0;                            // no signaling chars, no echo, no canonical processing
    tty.c_oflag = 0;                            // no remapping, no delays
    tty.c_cc[VMIN] = 0;                         // non-blocking read
    tty.c_cc[VTIME] = 10;                       // 1 second read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl
    tty.c_cflag |= (CLOCAL | CREAD);        // ignore modem controls, enable reading
    tty.c_cflag &= ~(PARENB | PARODD);      // no parity
    tty.c_cflag &= ~CSTOPB;                 // one stop bit
    tty.c_cflag &= ~CRTSCTS;                // no hardware flow control

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        std::cerr << "Error from tcsetattr" << std::endl;
        close(fd);
        return -1;
    }

    return fd;
}

bool parseMSP(const uint8_t *buf, MSP_Message *msg, int buf_size) {
    
    if (buf_size < 9) {
        return false;
    }

    if (buf[0] != MSP_HEADER1 || buf[1] != MSP_HEADER2 || buf[2] != MSP_HEADER3) {
        return false;
    }

    msg->flag = buf[3];
    msg->function = buf[4] | (buf[5] << 8);
    msg->size = buf[6] | (buf[7] << 8);

    if (buf_size < 8 + msg->size + 1) {
        return false;
    }

    msg->payload = new char[msg->size];
    memcpy(msg->payload, &buf[8], msg->size);

    uint8_t receivedCRC = buf[8 + msg->size];
    uint8_t calculatedCRC = calculateCRC(buf, msg->size);

    return receivedCRC == calculatedCRC;
}

void printRawData(const uint8_t *buf, int buf_size) {
    
    std::cout << "Raw Data (" << buf_size << " bytes): ";
    for (int i = 0; i < buf_size; i++) {
        std::cout << "0x"
                  << std::uppercase
                  << std::hex << std::setw(2) << std::setfill('0')
                  << static_cast<int>(buf[i]) << " ";
    }
    std::cout << std::dec << std::endl;
}

void parseMSG(MSP_Message *msg, MSP_OPFLOW_Message *opflow, MSP_RF_Message *rf) {
    
    switch(msg->function) {
        case 7938: {
            opflow->quality = static_cast<uint8_t>(msg->payload[0]);
            opflow->motionX = (static_cast<int32_t>(msg->payload[4]) << 24) |
                              (static_cast<int32_t>(msg->payload[3]) << 16) |
                              (static_cast<int32_t>(msg->payload[2]) << 8)  |
                               static_cast<int32_t>(msg->payload[1]);
            opflow->motionY = (static_cast<int32_t>(msg->payload[8]) << 24) |
                              (static_cast<int32_t>(msg->payload[7]) << 16) |
                              (static_cast<int32_t>(msg->payload[6]) << 8)  |
                               static_cast<int32_t>(msg->payload[5]);
        } break;
        case 7937: {
            rf->quality = static_cast<uint8_t>(msg->payload[0]);
            rf->distanceMm =  (static_cast<int32_t>(msg->payload[4]) << 24)  |
                              (static_cast<int32_t>(msg->payload[3]) << 16) |
                              (static_cast<int32_t>(msg->payload[2]) << 8)  |
                               static_cast<int32_t>(msg->payload[1]);
        } break;
        default: std::cerr << "Invalid function code." << std::endl;
    }
}

int main() {
    
    const char *uartDevice = "/dev/ttyS3";
    int baudrate = 115200;

    int uart_fd = configureUART(uartDevice, baudrate);
    if (uart_fd < 0) {
        return 1;
    }

    std::cout << "UART configured successfully. Listening for MSP v2 data..." << std::endl;

    uint8_t buf[64];
    std::memset(buf, 0, sizeof(buf));
    MSP_OPFLOW_Message opflow;
    MSP_RF_Message rf;

    while (true) {
        
        int n = read(uart_fd, buf, sizeof(buf));
        if (n > 0) {
            
            MSP_Message msg;
            if (parseMSP(buf, &msg, n)) {
                parseMSG(&msg, &opflow, &rf);
                std::cout << "Received MSP Message:" << std::endl;
                std::cout << "Optical Flow: " << std::endl;
                std::cout << "          Quality: " << (int)opflow.quality << std::endl;
                std::cout << "          MotionX: " << opflow.motionX << std::endl;
                std::cout << "          MotionY: " << opflow.motionY << std::endl;
                std::cout << "Range Finder: " << std::endl;
                std::cout << "          Quality: " << (int)rf.quality << std::endl;
                std::cout << "    Distance (mm): " << rf.distanceMm << std::endl;
                std::cout << std::dec << std::endl;
                delete[] msg.payload;
            }
        }
    }

    close(uart_fd);
    return 0;
}
