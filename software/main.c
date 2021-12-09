

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
#define SEND true

// get the avg distance in 4 directions, out => [-1, 0, 90, 180, 270] 
//angles are in radian and range from -180 to 180
//didn't convert angles on the fly to save time
void getAvg(LaserFan *scan, float *out){
    memset(out, 0, sizeof(out));
    out[0] = -1;
    int temp_cnt[5] = {0}
    for (int i = 0; i < *scan.npoints; i++){
        if (-0.523599< scan->points[i].angle && scan->points[i]< 0.523599){ //-30 to 30
            out[1] += scan->points[i].range;
            temp_cnt[1]+=1;
        } else if (1.0472 < scan->points[i].angle && scan->points[i]< 2.0944){ // 60 to 120
            out[2] += scan->points[i].range;
            temp_cnt[2]+=1;
        } else if (2.61799 < scan->points[i].angle && scan->points[i]< 3.14159
                ||-3.14159 < scan->points[i].angle && scan->points[i]< -2.61799 ){ // 150 to 180 or -180 to -150
            out[3] += scan->points[i].range;
            temp_cnt[3]+=1;
        } else if (-2.0944 < scan->points[i].angle && scan->points[i]< -1.0472){ // -120 to -60
            out[4] += scan->points[i].range;
            temp_cnt[4]+=1;
        }
    }
    for (int i = 1; i <5; i++){
        out[i] = out[i] / temp_cnt[i];
    }
}


int main(int argc, const char *argv[])
{   int server_fd, new_socket, valread;
        struct sockaddr_in address;
        int opt = 1;
        int addrlen = sizeof(address);
    if (SEND){
    //--------------socket------------------------
       
       // perror("started");
        // Creating socket file descriptor
        if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0)
        {
            perror("socket failed");
            exit(EXIT_FAILURE);
        }

        // Forcefully attaching socket to the port 8080
        if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT,
                                                      &opt, sizeof(opt)))
        {
            perror("setsockopt");
            exit(EXIT_FAILURE);
        }
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = INADDR_ANY;
        address.sin_port = htons( PORT );
        // Forcefully attaching socket to the port 8080
        if (bind(server_fd, (struct sockaddr *)&address,
                                     sizeof(address))<0)
        {
            perror("bind failed");
            exit(EXIT_FAILURE);
        }
        if (listen(server_fd, 3) < 0)
        {
            perror("listen");
            exit(EXIT_FAILURE);
        }
        if ((new_socket = accept(server_fd, (struct sockaddr *)&address,
                           (socklen_t*)&addrlen))<0)
        {
            perror("accept");
            exit(EXIT_FAILURE);
        }
        }
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
        f_optval = 180.0f;
        setlidaropt(laser, LidarPropMaxAngle, &f_optval, sizeof(float));
        f_optval = -180.0f;
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

    typedef enum{
        Scanning,
        Send,
        Receive,
        Drive,
        Close
    }state_t;

    state_t state = Scanning;

    printf("started\n");

    kobukiDriveDirect(60, 60);
    
    KobukiSensors_t sensors;
    sensors.leftWheelEncoder = 0;
    sensors.rightWheelEncoder = 0;

    float avgs[5] = {0};
    
    // struct packet {
    //   float stamp;
    //   float range;
    //   float angle;
    //   int leftWheelEncoder;
    //   int rightWheelEncoder;      
    // };
    // struct packet pkt;
    // pkt.leftWheelEncoder = 0;
    // pkt.rightWheelEncoder = 0;
    
        while (1) {
            switch (state) {
                case Scanning:
                    if(ret && os_isOk()) {
                        if(doProcessSimple(laser, &scan)) {
                            kobukiSensorPoll(&sensors);
                            getAvg(&scan, avgs); 
                            state = Send;
                        }
                    } else {
                        fprintf(stderr, "Failed to get Lidar Data\n");
                        fflush(stderr);
                        state = Close;
                    }
                    break;
                case Send:
                    ;
                    float point[5] = {0}; // timestamp, distance, angle, ticks left, ticks right
                    // pkt.stamp = scan.stamp;
                    // pkt.leftWheelEncoder = sensors.leftWheelEncoder;
                    // pkt.rightWheelEncoder = sensors.rightWheelEncoder;
                    
                    point[0] = scan.stamp;
                    point[3] = sensors.leftWheelEncoder;
                    point[4] = sensors.rightWheelEncoder;
                    
                    for (int i = 0; i < scan.npoints; i++){
                            // fprintf(stdout, "distance %f angle %.4f\n", scan.points[i].range*100, scan.points[i].angle * 57.29);
                            point[1] = scan.points[i].range;
                            point[2] = scan.points[i].angle;
                            //TODO send 4 avgs
                            // pkt.range = scan.points[i].range;
                            // pkt.angle = scan.points[i].angle;
                            // printf("stamp: %d distance: %.2f angle: %.2f ticks (%u, %u)\n", pkt.stamp, pkt.range, pkt.angle,
                                                                                                        // pkt.leftWheelEncoder, pkt.rightWheelEncoder);
                            
                            printf("stamp: %d distance: %.2f angle: %.2f ticks (%u, %u)\n", point[0], point[1], point[2],
                                                                                                        point[3], point[4]);
                            // sprintf (msg, "%.0f, %.0f",scan.points[i].range/10, scan.points[i].angle * 57.29 );
                            if (SEND){
                                send(new_socket , point , 5 * sizeof(float) , 0 );
                            }
                            // send(new_socket, &pkt, sizeof(struct packet), 0 );
                            fflush(stdout);
                        }
                        printf("0: %f, 90: %f 180: %f 270: %f\n", avgs[1], avgs[2], avgs[3], avgs[4]);
                    state = Drive;
                    break;

                case Receive:
                    break;

                case Drive:
                    kobukiDriveDirect(60, 60);
                    state = Scanning;
                    break;
                
                case Close:
                    kobukiDriveDirect(0,0);
                    LaserFanDestroy(&scan);
                    turnOff(laser);
                    disconnecting(laser);
                    lidarDestroy(&laser);
                    return 0;
            }
        }        
}
