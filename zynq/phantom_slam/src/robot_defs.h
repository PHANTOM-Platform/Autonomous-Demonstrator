/*
 * Filename:        robot_defs.h
 * Author:          A. Moulds. University of York.
 * Date:            20 March 2019
 * Project:         PHANTOM - Robot Demo
 * Copyright:       University of York
 * Description:     Header file for use with robot_server and phantom_slam programs.
 * Current Version: 0.11 (development only)
 * Revisions:
 * 1.   21/03/19    Changed integer types for sensor_data_t and robotcmd_t structs (now using
 *                  (fixed width types necessary for cross-compiled ends of network sockets).
 * Notes:
 *                  Set HOST_IP_ADDRESS as required for robot controller server (Edison board).
 * 
*/



#ifndef __ROBOT_DEFS_H__
#define __ROBOT_DEFS_H__

#include <stdio.h>
#include <stdint.h>


/* compiler directive defines */
// #define PIONEER_3DX_ROBOT_VREP   // build for VREP simulator

#ifndef PIONEER_3DX_ROBOT_VREP
#define PHANTOM_ROBOT   // build for physical robot used in demo
#endif

#define NO_ROBOT // no physical robot presently so emulate based on simulator-generated data 

//#define LOCALHOST // use local host to act as robot server (instead of Edison board) - dev only

#ifdef PHANTOM_ROBOT

#ifndef NO_ROBOT // physical robot present
    #define ROBOT_WHEEL_RADIUS 0.0625 // radius of wheels in metres
    #define ROBOT_AXLE_LEN 0.25   
    #define ROBOT_HALF_AXLE_LEN ROBOT_AXLE_LEN*0.5 // half axle length between wheel sets in metres
    #define ROBOT_WHEEL_ENCODER_MAX 2000  // odometry wheel increments per full rotation
    #define ROBOT_WHEELS_RATIO 1.0 // ratio of wheel radius per left/right set
    #define ROBOT_LIDAR_POS_OFFSET 0 // offset in mm of LIDAR laser motor location from robot centre
    #define ROBOT_LIDAR_SCAN_SIZE 682
    #define ROBOT_LIDAR_SCAN_LEFT_ANGLE -120 // scanning start angle (from right) in degrees
    #define ROBOT_LIDAR_SCAN_RIGHT_ANGLE +120 // scanning end angle (left most) in degrees 
    #define ROBOT_LIDAR_SCANNING_ANGLE 85 // -90 to +90 (180) degree scan window     
    #define ROBOT_LIDAR_RANGE 3000 // set laser range in mm for accurate scans   
#else 
    #define ROBOT_WHEEL_RADIUS 0.0975 // radius of wheels in metres   
    #define ROBOT_AXLE_LEN 0.332 // for vrep scene5 (lua script value = 0.0335)
    #define ROBOT_HALF_AXLE_LEN ROBOT_AXLE_LEN*0.5 // half axle length between wheel sets in metres
    #define ROBOT_WHEEL_ENCODER_MAX 2000  // odometry wheel increments per full rotation
    #define ROBOT_WHEELS_RATIO 1.0 // ratio of wheel radius per left/right set
    #define ROBOT_LIDAR_POS_OFFSET 125 // distance from drive axle centre to lidar centre in mm
    #define ROBOT_LIDAR_SCAN_SIZE 682 // range of LIDAR scan in slit divisions (i.e. number of valid scan radials)
    #define ROBOT_LIDAR_SCAN_LEFT_ANGLE -120 // scanning start angle (from right) in degrees
    #define ROBOT_LIDAR_SCAN_RIGHT_ANGLE +120 // scanning end angle (left mo    ts_position_t startpos;st) in degrees   
    #define ROBOT_LIDAR_SCANNING_ANGLE 85; // -90 to +90 (682-170)*(360/1024) = 180) degree scan window       
    #define ROBOT_LIDAR_RANGE 3000 // set laser range in mm for accurate scans (4000 max)
#endif

#elif defined PIONEER_3DX_ROBOT_VREP // simulator
 
   // From robot's datasheet (Pioneer3DX-P3DX-RevA.pdf):
    // 0.380 m between outer face of wheels
    // Wheel dia = 0.195 mm
    // From VREP model:
    // whelels are approx. 0.050 m thick
    // Hence, axis length = 0.380-0.050 = 0.33 m
    // Therefore, half-axis = 0.165 m
    #define ROBOT_WHEEL_RADIUS 0.0975 // radius of wheels in metres   
    // (note: if incorrectly set too low or high the slam map is made to rotate during robot turns)
    // #define ROBOT_AXLE_LEN 0.33 // default value
    #define ROBOT_AXLE_LEN 0.332 // for vrep scene5 (lua script value = 0.0335)
    #define ROBOT_HALF_AXLE_LEN ROBOT_AXLE_LEN*0.5 // half axle length between wheel sets in metres
    #define ROBOT_WHEEL_ENCODER_MAX 2000  // odometry wheel increments per full rotation
    #define ROBOT_WHEELS_RATIO 1.0 // ratio of wheel radius per left/right set
 
    // set LIDAR parameters for URG-04LX
    // note: laser scans anti-clockwise from right to left (scan point value of 44 to 725 for URG-04LX)
    //#define ROBOT_LIDAR_POS_OFFSET 0 // put LIDAR above centre of drive axle (fixes bug for SLAM code ARM)
    #define ROBOT_LIDAR_POS_OFFSET 125 // distance from drive axle centre to lidar centre in mm
    #define ROBOT_LIDAR_SCAN_SIZE 682 // range of LIDAR scan in slit divisions (i.e. number of valid scan radials)
    #define ROBOT_LIDAR_SCAN_LEFT_ANGLE -120 // scanning start angle (from right) in degrees
    #define ROBOT_LIDAR_SCAN_RIGHT_ANGLE +120 // scanning end angle (left mo    ts_position_t startpos;st) in degrees 
    // set margin used to offset start and end scan (when zero scan starts at 44) //
    //#define ROBOT_LIDAR_SCANNING_ANGLE 255; // -30 to +30 (682-510)*(360/1024) = 60.5) degree scan window      
    //#define ROBOT_LIDAR_SCANNING_ANGLE 213; // -45 to +45 (682-426)*(360/1024) = 90) degree scan window
    //#define ROBOT_LIDAR_SCANNING_ANGLE 170; // -60 to +60 (682-340)*(360/1024) = 120) degree scan window    
    #define ROBOT_LIDAR_SCANNING_ANGLE 85; // -90 to +90 (682-170)*(360/1024) = 180) degree scan window       
    //#define ROBOT_LIDAR_SCANNING_ANGLE 6; // -117.5 to +117.5 (682-12)*(360/1024) = 235.5) degree scan window  
    //#define ROBOT_LIDAR_SCANNING_ANGLE 0; // -120 to +120 (682-0)*(360/1024) = 240) degree scan window    
    #define ROBOT_LIDAR_RANGE 3000 // set laser range in mm for accurate scans (4000 max)
 
#endif
 
 
// robot status and commands (commnication between Edison board and Zynq host)
#define ROBOT_IDLE 0
#define ROBOT_BUSY 1
#define ROBOT_ERROR 2
#define ROBOT_FWD           0x0100 // auto-stops on completion
#define ROBOT_UTURN_RIGHT   0x0110 // auto-stops on completion
#define ROBOT_UTURN_LEFT    0x0111 // auto-stops on completion
#define ROBOT_STOP          0x0200 // emergency stop!
#define ROBOT_HALT          0x0201 // halt (wait)

#ifdef LOCALHOST
#define HOST_IP_ADDRESS "127.0.0.1"  // localhost (for dev only)
#else
#define HOST_IP_ADDRESS "172.10.0.80"  // Static IP address of Intel Edison board (server)
#endif

#define ROBOT_PORT_NUMBER 60000 // use unique port number for robot demo
   
// struct containing complete sensor scan sent from robot server (Edison board)
typedef struct {
    uint16_t status;    // status of robot
    uint32_t timestamp;
    int32_t q1, q2;     // odometry distance for left and right wheels respectively
    int16_t d[ROBOT_LIDAR_SCAN_SIZE];
} sensor_data_t;


// struct used for nav commands issued from host
typedef struct {
    uint16_t cmd; 
    int16_t data;
} robotcmd_t;
    
   
#endif
