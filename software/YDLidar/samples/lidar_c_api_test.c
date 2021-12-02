

//socket includes
#include <netinet/in.h>
#include <sys/socket.h>
#define PORT 8080

//-------------------
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#ifdef WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif
#include <string.h>
#include "ydlidar_sdk.h"
#if defined(_MSC_VER)
#pragma comment(lib, "ydlidar_sdk.lib")
#endif

//--------Kobuki--------------------
#include "../../libraries/kobuki/kobukiActuator.h"
#include "../../libraries/kobuki/kobukiSensorTypes.h"
#include "../../libraries/kobuki/kobukiSensorPoll.h"
#include "../../libraries/kobuki/kobukiUtilities.h"

#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>

extern int *serial_port;
int main(int argc, const char *argv[]) {
    printf("here");
    fflush(stdout);
    fprintf(stdout, "started");
    //--------------socket------------------------
//    int server_fd, new_socket, valread;
//     struct sockaddr_in address;
//     int opt = 1;
//     int addrlen = sizeof(address);
//    // perror("started");
//     // Creating socket file descriptor
//     if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0)
//     {   
//         perror("socket failed");
//         exit(EXIT_FAILURE);
//     }
       
//     // Forcefully attaching socket to the port 8080
//     if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT,
//                                                   &opt, sizeof(opt)))
//     {
//         perror("setsockopt");
//         exit(EXIT_FAILURE);
//     }
//     address.sin_family = AF_INET;
//     address.sin_addr.s_addr = INADDR_ANY;
//     address.sin_port = htons( PORT );
//        printf("here");
//     // Forcefully attaching socket to the port 8080
//     if (bind(server_fd, (struct sockaddr *)&address, 
//                                  sizeof(address))<0)
//     {   
//         perror("bind failed");
//         exit(EXIT_FAILURE);
//     }
//     if (listen(server_fd, 3) < 0)
//     {
//         perror("listen");
//         exit(EXIT_FAILURE);
//     }
//     if ((new_socket = accept(server_fd, (struct sockaddr *)&address, 
//                        (socklen_t*)&addrlen))<0)
//     {
//         perror("accept");
//         exit(EXIT_FAILURE);
//     }
    
//     //------------LIDAR init--------------------


//     os_init();
//     YDLidar *laser = lidarCreate();
//     //string prop
//     char port[50] = "/dev/ydlidar";
//     LidarPort ports;
//     int size = lidarPortList(&ports);
//     int i = 0;
//     for(i =0; i < size; i++) {
//         printf("port: %s\n", ports.port[i].data);
//         /// last port
//         strcpy(port, ports.port[i].data);
//     }
//     setlidaropt(laser, LidarPropSerialPort, port, sizeof(port));
//     strcpy(port, "");
//     setlidaropt(laser, LidarPropIgnoreArray,port, sizeof(port));

//     //int prop
//     int i_optvalue = 115200;
//     setlidaropt(laser, LidarPropSerialBaudrate, &i_optvalue, sizeof(int));
//     // i_optvalue = TYPE_TOF;
//     i_optvalue = TYPE_TRIANGLE;
//     setlidaropt(laser, LidarPropLidarType, &i_optvalue, sizeof(int));
//     i_optvalue = YDLIDAR_TYPE_SERIAL;
//     setlidaropt(laser, LidarPropDeviceType, &i_optvalue, sizeof(int));
//     i_optvalue = 3;
//     setlidaropt(laser, LidarPropSampleRate, &i_optvalue, sizeof(int));

//     //bool prop
//     bool b_optval = true;
//     setlidaropt(laser, LidarPropAutoReconnect, &b_optval, sizeof(bool));
//     b_optval = true;
//     setlidaropt(laser, LidarPropSingleChannel, &b_optval, sizeof(bool));
//     b_optval = false;
//     setlidaropt(laser, LidarPropIntenstiy, &b_optval, sizeof(bool));
//     setlidaropt(laser, LidarPropInverted, &b_optval, sizeof(bool));
//     setlidaropt(laser, LidarPropReversion, &b_optval, sizeof(bool));
//     b_optval = true;
//     setlidaropt(laser, LidarPropSupportMotorDtrCtrl, &b_optval, sizeof(bool));
//     b_optval = false;
//     setlidaropt(laser, LidarPropFixedResolution, &b_optval, sizeof(bool));

//     //float prop
//     float f_optval = 4.f;
//     setlidaropt(laser, LidarPropScanFrequency, &f_optval, sizeof(float));
//     f_optval = 360.0f;
//     setlidaropt(laser, LidarPropMaxAngle, &f_optval, sizeof(float));
//     f_optval = 0.0f;
//     setlidaropt(laser, LidarPropMinAngle, &f_optval, sizeof(float));
//     // f_optval = 8.f;
//     f_optval = 10.f;
//     setlidaropt(laser, LidarPropMaxRange, &f_optval, sizeof(float));
//     // f_optval = 0.12f;
//     f_optval = 0.12f;
//     setlidaropt(laser, LidarPropMinRange, &f_optval, sizeof(float));

//     getlidaropt(laser, LidarPropSerialBaudrate,&i_optvalue, sizeof(int));
//     printf("baudrate: %d\n", i_optvalue);

//     bool ret = initialize(laser);
//    if(ret) {
//        ret = turnOn(laser);
//    }

//     LaserFan scan;
//     LaserFanInit(&scan);

//     //------------------UART---------------------------
        struct termios UART;
        if (tcgetattr(*serial_port, &UART) != 0) {
            printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
        }

        UART.c_cflag &= ~PARENB; //disable parity
        UART.c_cflag &= ~CSTOPB; //one stop bit
        UART.c_cflag |= CS8; //8 bits
        UART.c_cflag &= ~CRTSCTS; //disable flow controll
        UART.c_cflag |= CREAD | CLOCAL;
        UART.c_lflag &= ~ICANON; //disable cannonical

        UART.c_lflag &= ~ECHO; // Disable echo
        UART.c_lflag &= ~ECHOE; // Disable erasure
        UART.c_lflag &= ~ECHONL; // Disable new-line echo

        UART.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
        UART.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
        UART.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes
        UART.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
        UART.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
        UART.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
        UART.c_cc[VMIN] = 0; 
        cfsetispeed(&UART,B115200);
        cfsetospeed(&UART,B115200);

        // Save tty settings, also checking for error
        if (tcsetattr(*serial_port, TCSANOW, &UART) != 0) {
            printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
        }
        printf("UART initialized?");
        kobukiDriveDirect(50,50);

//     while (ret && os_isOk()) {
//         if(doProcessSimple(laser, &scan)) {
          
// ;
//            for (int i = 0; i < scan.npoints; i++){
// 		        fprintf(stdout, "distance %f angle %.4f\n", scan.points[i].range*100, scan.points[i].angle * 57.29);
//                 point[0] = scan.points[i].range*100;
//                 point[1] = scan.points[i].angle * 57.29;
//                 // sprintf (msg, "%.0f, %.0f",scan.points[i].range*100, scan.points[i].angle * 57.29 );
//                 send(new_socket , point , 2 * sizeof(float) , 0 );
//                 fflush(stdout);
//            }

	       	
//         } else {
//             fprintf(stderr, "Failed to get Lidar Data\n");
//             fflush(stderr);
//         }
//     }
//     LaserFanDestroy(&scan);
//     turnOff(laser);
//     disconnecting(laser);
//     lidarDestroy(&laser);
    return 0;

}
