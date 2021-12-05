

//socket includes
#include <netinet/in.h>
#include <sys/socket.h>
#define PORT 8080

//-------------------
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include <time.h>

#ifdef WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif
#include <string.h>

// #include "./YDLidar/src/ydlidar_sdk.h"
#include "ydlidar_sdk.h"
#if defined(_MSC_VER)
#pragma comment(lib, "ydlidar_sdk.lib")
#endif

#include "kobuki.h"

// #include <fcntl.h>
// #include <errno.h>
// #include <termios.h>
// #include <unistd.h>


int main(int argc, const char *argv[])
{
    
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

        //------------LIDAR init--------------------

        os_init();
        YDLidar *laser = lidarCreate();
        //string prop
        char port[50] = "/dev/ydlidar";
        LidarPort ports;
        int size = lidarPortList(&ports);
        int i = 0;
        for(i =0; i < size; i++) {
            printf("port: %s\n", ports.port[i].data);
            /// last port
            strcpy(port, ports.port[i].data);
        }
        setlidaropt(laser, LidarPropSerialPort, port, sizeof(port));
        strcpy(port, "");
        setlidaropt(laser, LidarPropIgnoreArray,port, sizeof(port));

        //int prop
        int i_optvalue = 115200;
        setlidaropt(laser, LidarPropSerialBaudrate, &i_optvalue, sizeof(int));
        // i_optvalue = TYPE_TOF;
        i_optvalue = TYPE_TRIANGLE;
        setlidaropt(laser, LidarPropLidarType, &i_optvalue, sizeof(int));
        i_optvalue = YDLIDAR_TYPE_SERIAL;
        setlidaropt(laser, LidarPropDeviceType, &i_optvalue, sizeof(int));
        i_optvalue = 3;
        setlidaropt(laser, LidarPropSampleRate, &i_optvalue, sizeof(int));

        //bool prop
        bool b_optval = true;
        setlidaropt(laser, LidarPropAutoReconnect, &b_optval, sizeof(bool));
        b_optval = true;
        setlidaropt(laser, LidarPropSingleChannel, &b_optval, sizeof(bool));
        b_optval = false;
        setlidaropt(laser, LidarPropIntenstiy, &b_optval, sizeof(bool));
        setlidaropt(laser, LidarPropInverted, &b_optval, sizeof(bool));
        setlidaropt(laser, LidarPropReversion, &b_optval, sizeof(bool));
        b_optval = true;
        setlidaropt(laser, LidarPropSupportMotorDtrCtrl, &b_optval, sizeof(bool));
        b_optval = false;
        setlidaropt(laser, LidarPropFixedResolution, &b_optval, sizeof(bool));

        //float prop
        float f_optval = 4.f;
        setlidaropt(laser, LidarPropScanFrequency, &f_optval, sizeof(float));
        f_optval = 360.0f;
        setlidaropt(laser, LidarPropMaxAngle, &f_optval, sizeof(float));
        f_optval = 0.0f;
        setlidaropt(laser, LidarPropMinAngle, &f_optval, sizeof(float));
        // f_optval = 8.f;
        f_optval = 10.f;
        setlidaropt(laser, LidarPropMaxRange, &f_optval, sizeof(float));
        // f_optval = 0.12f;
        f_optval = 0.12f;
        setlidaropt(laser, LidarPropMinRange, &f_optval, sizeof(float));

        getlidaropt(laser, LidarPropSerialBaudrate,&i_optvalue, sizeof(int));
        printf("baudrate: %d\n", i_optvalue);

        bool ret = initialize(laser);
       if(ret) {
           ret = turnOn(laser);
       }

        LaserFan scan;
        LaserFanInit(&scan);




    printf("started\n");
    kobukiUARTInit();
    kobukiDriveDirect(40, 40);
    kobukiUARTInit();
    KobukiSensors_t sensors;
    sensors.leftWheelEncoder = 0;
    sensors.rightWheelEncoder = 0;

    // while(1){
    
    // kobukiSensorPoll(&sensors);
    // printf("wheel ticks: %d\n", sensors.leftWheelEncoder);
    // }
    kobukiUARTUnInit();
        while (ret && os_isOk()) {
            if(doProcessSimple(laser, &scan)) {
                kobukiSensorPoll(&sensors);
                float point[5] = {0}; // timestamp, distance, angle, ticks left, ticks right
                point[0] = scan.stamp;
                point[3] = sensors.leftWheelEncoder;
                point [4] = sensors.rightWheelEncoder;
               for (int i = 0; i < scan.npoints; i++){
    		        // fprintf(stdout, "distance %f angle %.4f\n", scan.points[i].range*100, scan.points[i].angle * 57.29);
                    point[1] = scan.points[i].range;
                    point[2] = scan.points[i].angle * 57.29;
                    printf("stamp: %d distance: %.2f angle: %.2f ticks (%d, %d)\n", point[0], point[1], point[2],
                                                                                             point[3], point[4]);
                    // sprintf (msg, "%.0f, %.0f",scan.points[i].range*100, scan.points[i].angle * 57.29 );
                    // send(new_socket , point , 2 * sizeof(float) , 0 );
                    fflush(stdout);
               }

            } else {
                fprintf(stderr, "Failed to get Lidar Data\n");
                fflush(stderr);
            }
        }
        kobukiDriveDirect(0,0);
        LaserFanDestroy(&scan);
        turnOff(laser);
        disconnecting(laser);
        lidarDestroy(&laser);
    return 0;
}
