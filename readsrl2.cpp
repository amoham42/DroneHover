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
const uint8_t MSP_HEADER3 = '<'; // or '>' depending on direction

// Function to calculate CRC (simple XOR)
uint8_t calculateCRC(const std::vector<uint8_t>& data) {
    uint8_t crc = 0;
    for(auto byte : data) {
        crc ^= byte;
    }
    return crc;
}

// Function to configure UART
int configureUART(const char* device, int baudrate) {
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

    // Set Baud Rate
    speed_t speed;
    switch(baudrate) {
        case 9600: speed = B9600; break;
        case 19200: speed = B19200; break;
        case 38400: speed = B38400; break;
        case 57600: speed = B57600; break;
        case 115200: speed = B115200; break;
        // Add more cases as needed
        default:
            std::cerr << "Unsupported baud rate" << std::endl;
            close(fd);
            return -1;
    }

    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    // Configure 8N1
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
    tty.c_iflag &= ~IGNBRK;                         // disable break processing
    tty.c_lflag = 0;                                // no signaling chars, no echo, no canonical processing
    tty.c_oflag = 0;                                // no remapping, no delays
    tty.c_cc[VMIN]  = 0;                            // non-blocking read
    tty.c_cc[VTIME] = 10;                           // 1 second read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);         // shut off xon/xoff ctrl
    tty.c_cflag |= (CLOCAL | CREAD);                // ignore modem controls, enable reading
    tty.c_cflag &= ~(PARENB | PARODD);              // no parity
    tty.c_cflag &= ~CSTOPB;                         // one stop bit
    tty.c_cflag &= ~CRTSCTS;                        // no hardware flow control

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        std::cerr << "Error from tcsetattr" << std::endl;
        close(fd);
        return -1;
    }

    return fd;
}

// Function to parse MSP v2 messages
struct MSPMessage {
    char direction; // '<' or '>'
    uint16_t size;
    uint8_t command;
    std::vector<uint8_t> data;
};

bool parseMSP(const std::vector<uint8_t>& buffer, MSPMessage& msg) {
    if(buffer.size() < 6) { // Minimum size
        return false;
    }

    // Check headers
    if(buffer[0] != MSP_HEADER1 || buffer[1] != MSP_HEADER2) {
        return false;
    }

    msg.direction = buffer[2];
    msg.size = buffer[3] | (buffer[4] << 8);
    msg.command = buffer[5];

    if(buffer.size() < 6 + msg.size + 1) { // Data + CRC
        return false;
    }

    msg.data.assign(buffer.begin() + 6, buffer.begin() + 6 + msg.size);
    uint8_t receivedCRC = buffer[6 + msg.size];
    
    // Calculate CRC
    std::vector<uint8_t> crcData(buffer.begin() + 2, buffer.begin() + 6 + msg.size);
    uint8_t calculatedCRC = calculateCRC(crcData);

    return receivedCRC == calculatedCRC;
}

// Function to print raw data in hexadecimal format
void printRawData(const std::vector<uint8_t>& data) {
    std::cout << "Raw Data (" << data.size() << " bytes): ";
    for(auto byte : data) {
        std::cout << "0x" 
                  << std::uppercase 
                  << std::hex << std::setw(2) << std::setfill('0') 
                  << static_cast<int>(byte) << " ";
    }
    std::cout << std::dec << std::endl; // Reset to decimal
}

int main() {
    const char* uartDevice = "/dev/ttyS3"; // Update this to your UART3 device
    int baudrate = 115200; // Set your baud rate

    int uart_fd = configureUART(uartDevice, baudrate);
    if(uart_fd < 0) {
        return 1;
    }

    std::cout << "UART configured successfully. Listening for MSP v2 data..." << std::endl;

    std::vector<uint8_t> readBuffer;
    uint8_t buf[256];
    while(true) {
        int n = read(uart_fd, buf, sizeof(buf));
        if(n > 0) {
            // Append received bytes to the buffer
            std::vector<uint8_t> incomingData(buf, buf + n);
            readBuffer.insert(readBuffer.end(), incomingData.begin(), incomingData.end());

            // Print raw data received in this read
            printRawData(incomingData);

            // Attempt to parse messages from the buffer
            MSPMessage msg;
            while(parseMSP(readBuffer, msg)) {
                // Process the MSP message
                std::cout << "Received MSP Message:" << std::endl;
                std::cout << " Direction: " << msg.direction << std::endl;
                std::cout << " Command: " << static_cast<int>(msg.command) << std::endl;
                std::cout << " Size: " << msg.size << std::endl;
                std::cout << " Data: ";
                for(auto byte : msg.data) {
                    std::cout << "0x" << std::uppercase << std::hex << std::setw(2) << std::setfill('0') 
                              << static_cast<int>(byte) << " ";
                }
                std::cout << std::dec << std::endl; // Reset to decimal

                // Optionally, print raw packet for the parsed message
                std::vector<uint8_t> rawPacket(readBuffer.begin(), readBuffer.begin() + 6 + msg.size + 1);
                printRawData(rawPacket);

                // Remove the parsed message from the buffer
                readBuffer.erase(readBuffer.begin(), readBuffer.begin() + 6 + msg.size + 1);
            }

            // Optional: Prevent buffer from growing indefinitely
            if(readBuffer.size() > 1024) {
                readBuffer.erase(readBuffer.begin(), readBuffer.end() - 1024);
            }
        }
    }

    close(uart_fd);
    return 0;
}

