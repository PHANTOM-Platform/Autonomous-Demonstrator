/*
 * Filename:    phantom_slam_host.c
 * Author:      A. Moulds. University of York.
 * Date:        28 March 2019
 * Project:     PHANTOM - Robot Demo
 * Copyright:   University of York
 * 
 * Description: Program issues navigation commands to demo robot and builds SLAM map from read-back robot sensor data.
 *  `           Once the robot has completed its initial loop the map is finalized and used to correct its position in
 *              subsequent loops. 
 * 
 * 
 * Current Version: 0.11 (development only)
 * 
 * Revisions:
 *  0.11    28/3/19 Improved slam map build performance.
 *                  Modified structs used to communicate with Edison board; elements now have fixed bit lengths.
 * 
*/ 



#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <math.h>
#include <time.h>
#include <unistd.h>
#include <stdint.h>
#include "CoreSLAM.h"
#include "robot_defs.h"


/* defines */

#define DEBUG
#define DUMP_MAPS
#define MAX_NAV_CMDS 4
#define MAP_IMAGE_FILENAME "maps/scene_map"
#define MAX_ODOMETRY_DATA 500000


//#define DUMP_ODOMETRY 1
//#define DUMP_SENSORDATA 1   // creates files containing all sensor data per nav command

/* global types */
typedef struct {
    unsigned int timestamp;
    int q1, q2;
} odometry_t;


/* global variables */

#ifdef PIONEER_3DX_ROBOT_VREP
#include "extApi.h"
static simxUChar* vrep_signal;
static simxInt sLength;
static simxInt clientID;
#endif
static int kill_program = 0;
ts_map_t map;
ts_map_t trajectory; 
ts_map_t loop_map;
ts_sensor_data_t sensor_data;
char navcmd[20];
#ifdef PHANTOM_ROBOT
int sockfd; // communication socket with Edison board
#include <sys/socket.h>
#endif

#ifdef DUMP_SENSORDATA
sensor_data_t* sensordata;
#endif

/* global constants */
const int VREP_ROBOT_DATA_PKT_SIZE_MIN = 682;


/* fn prototypes */

int ts_read_scan(ts_sensor_data_t*, ts_laser_parameters_t*, unsigned short*);
int init_robot(void);
void setcmd(robotcmd_t*, unsigned short, short);
int send_navcmd(robotcmd_t*);



////////////////////////////////////////////////////////////////////////////////// 
void setcmd(robotcmd_t* nc, unsigned short cmd, short data)
{
    nc->cmd = cmd;
    nc->data = data;
}
 
   
    
////////////////////////////////////////////////////////////////////////////////// 
#ifdef PHANTOM_ROBOT
int send_navcmd(robotcmd_t* nc)
{
    int status = write(sockfd, nc, sizeof(robotcmd_t));
    if (status == 0 || status < 0)
    {
        printf("error: lost connection!\n");
        perror("ERROR");
        return -1;
    }
   return 0;
}
#endif



/*//////////////////////////////////////////////////////////////////////////////// 
 * 
 * Fn to read-in scan from robot's sensors (LiDAR plus odometry).
 * Return Value:
 *    '0' on success
 *   '-1' on failure
 * 
*//////////////////////////////////////////////////////////////////////////////// 
int ts_read_scan(ts_sensor_data_t* sd, ts_laser_parameters_t* laser_params, unsigned short* robot_status)
{

#ifdef PIONEER_3DX_ROBOT_VREP   
     
    char *str;

    // get robot data from VREP
    if (simxReadStringStream(clientID,"toClient_Data",&vrep_signal,&sLength,simx_opmode_buffer)==simx_return_ok)
    {     
        if(vrep_signal==NULL || sLength < 20)//|| sLength < VREP_ROBOT_DATA_PKT_SIZE_MIN)
            return -1;
        // read-in timestamp
        str = strtok((char*)vrep_signal," ");
        if(sscanf(str, "%u", &sd->timestamp) != 1)
        {
            printf("error: failed to get timestamp from robot\n");
            return -1;
        }  
        // read-in odometry data
        str = strtok(NULL, " ");
        if(sscanf(str, "%d", &sd->q1) != 1)
        {
            printf("error:failed to get q1 odometry from robot\n");
            return -1;
        }        
        str = strtok(NULL, " ");
        if(sscanf(str, "%d", &sd->q2) != 1)
        {
            printf("error: failed to get q2 odometry from robot\n");
            return -1;
        } 
        // read-in lidar data
        for (int i = 0; i < laser_params->scan_size; i++) 
        {
            str = strtok(NULL, " ");
            if (str != NULL) 
            {
                if(sscanf(str, "%d", &sd->d[i]) != 1)
                {
                    printf("error: failed to get lidar data from robot\n");
                    return -1;
                } 
                // check out-of-bound scan value. Added by ARM. CH#1.//                
                if(sd->d[i] > laser_params->distance_no_detection)
                    sd->d[i] = 0; 
            } 
            else 
                sd->d[i] = 0;
        }
        return 0; // success
    }
    else
        return -1; // failure
    
#elif defined  PHANTOM_ROBOT
    
    sensor_data_t sensordata;
    ssize_t status = read(sockfd, &sensordata, sizeof(sensor_data_t));   
    if(status != sizeof(sensor_data_t))
    {
        #ifdef DEBUG
        printf("error: wrong data size read from socket - got %d bytes\n",status);
        #endif
        return -1;
    }
    sd->timestamp = sensordata.timestamp;
    sd->q1 = sensordata.q1;
    sd->q2 = sensordata.q2;    
    for(int i=0; i< ROBOT_LIDAR_SCAN_SIZE; i++)
        sd->d[i] = sensordata.d[i];
    *robot_status = sensordata.status;
    return 0;
    
#endif

    return -1;      
}



/*//////////////////////////////////////////////////////////////////////////////// 
// Fn to initialise robot communication.
*//////////////////////////////////////////////////////////////////////////////// 
int init_robot()
{
    
#ifdef PIONEER_3DX_ROBOT_VREP
    
    printf("trying to establish remote connection to VREP...\n");
        
    // ensure opened connections closed
    simxFinish(-1);
    
    // waitUntilConnected=true, doNotReconnectOnceDisconnected=true,timeout=5s,packets=1(per ms).
    clientID = simxStart("127.0.0.1",19997,true,true,5000,1);
    if(clientID == -1)
    {
        printf("error: connection to vrep server not possible\n");
        return -1;
    }
    
    // init link
    simxReadStringStream(clientID,"toClient_Data",&vrep_signal,&sLength,simx_opmode_streaming);
    if (clientID < 0)
    {
        printf("error: failed to connect to vrep simulator\n");
        return -1;
    }
    
    printf("done.\n");
 
    // set synchronous comms with VREP, i.e. control simulator from this 'client' code
    simxSynchronous(clientID,true);
    simxStartSimulation(clientID, simx_opmode_oneshot);
   
    // switch on robot's LiDAR (need this for SLAM)
    simxSetIntegerSignal(clientID, "toServer_lidarSwitch", 1, simx_opmode_oneshot);
    
    // start robot navigation
    //simxAddStatusbarMessage(clientID,"*** client: started robot nav ***",simx_opmode_streaming);
    simxSetIntegerSignal(clientID, "toServer_startNav", 1, simx_opmode_oneshot); 
    
    
#elif defined PHANTOM_ROBOT
    
    /* TBD */
    
#endif

    return 0;   
}



/*//////////////////////////////////////////////////////////////////////////////// 
// fn to catch CTRL-C signal.
*//////////////////////////////////////////////////////////////////////////////// 
void signal_callback_handler(int signum)
{
    kill_program = 1;
    printf("\n");
}
    
    
    
/*//////////////////////////////////////////////////////////////////////////////// 
// Fn dumps odometry data to file.
*//////////////////////////////////////////////////////////////////////////////// 
int createOdometryFromScans(char *filename, ts_sensor_data_t *sensor_data, int noscans)
{
    FILE *fout;
    int i;
	printf("odometry_filename = %s\n",filename);
    fout = fopen(filename, "wt");
    
    if(fout == NULL)
    {
        printf("error: unable to create %s file\n", filename);
        return -1;
    }
    fprintf(fout, "# Extracted Odometry data\n");    
    fprintf(fout, "# Timestamp\tQ1\tQ2\n");
    for(i = 0; i < noscans; i++, sensor_data++)
        fprintf(fout, "%u\t%d\t%d\n", sensor_data->timestamp, sensor_data->q1, sensor_data->q2);
    fclose(fout);
    
    return 0;
}



//////////////////////////////////////////////////////////////////////////////// 
void store_odometry(odometry_t* odometry, ts_sensor_data_t* sensor_data)
{
    odometry->timestamp = sensor_data->timestamp;
    odometry->q1 = sensor_data->q1;
    odometry->q2 = sensor_data->q2;
}


           
////////////////////////////////////////////////////////////////////////////////         
void dump_odometry_to_file(char* filename, odometry_t* odometry)
{
    FILE *fout;
    int i;
    int scancount;
    time_t curtime;
    
    fout = fopen(filename, "wt");
    if(fout == NULL)
    {
        printf("error: unable to create %s file\n", filename);
        return;
    }
    
    // get scan count
    for(i=MAX_ODOMETRY_DATA-1; i > 10;i--)
    {
        if((odometry+i)->timestamp != 0)
            break;
    }
    
    scancount = i;
    time(&curtime);
    fprintf(fout, "# Extracted Odometry data\n");  
    fprintf(fout, "# %s", ctime(&curtime));
    fprintf(fout, "# Timestamp\tQ1\tQ2\t\tdeltaT\tdeltaQ1\tdeltaQ2\n");
    
    unsigned int prev_timestamp=0;
    int prev_q1=0;
    int prev_q2=0;
    for(i = 0; i < scancount; i++, odometry++)
    {
        fprintf(fout, "%010u\t%04d\t%04d", (odometry)->timestamp, (odometry)->q1, (odometry)->q2);
        fprintf(fout,"\t\t%u\t%d\t%d\n",(odometry)->timestamp-prev_timestamp, (odometry)->q1-prev_q1, (odometry)->q2-prev_q2);
        prev_timestamp=(odometry)->timestamp;
        prev_q1 = (odometry)->q1;
        prev_q2 = (odometry)->q2;
    }
    fclose(fout);    
}
     


/*//////////////////////////////////////////////////////////////////////////////// 
 * Function to plot robot trajectory only (does not splot map).
 * The robot's position is exaggerated by displaying larger marker.
 * 
 *//////////////////////////////////////////////////////////////////////////////// 
void ts_save_trajectory_pgm(ts_map_t *map, char *filename, int width, int height)
{
    #define MARKER_SIZE 6 // pixal size of robot on image (map)
    int x, y;
    FILE *output;
    
    unsigned char img[TS_MAP_SIZE+1][TS_MAP_SIZE+1];
    
    for(y=0; y < TS_MAP_SIZE+1; y++)
        for(x=0; x < TS_MAP_SIZE+1; x++)
            img[x][y] = 255;

    for (y = 0; y < height; y++) 
    {
        for (x = 0; x < width; x++)
        {
            if ((int)(map->map[ (TS_MAP_SIZE - 1 - y) * TS_MAP_SIZE + x]) >> 8 == 0)
            {
                for(int j=0;j<MARKER_SIZE;j++)
                    for(int k=0; k < MARKER_SIZE; k++)
                        img[x+j][y+k] = 0;
            }
        }
    }       
                
    output = fopen(filename, "wt");
    if(output==NULL)
    {
        printf("error: can't oepn %s for writing...\n",filename);
        return;
    }
    fprintf(output, "P2\n%d %d 255\n", width, height);
    
    for (y = 0; y < height; y++) 
    {
        for (x = 0; x < width; x++)
            fprintf(output, "%d ", img[x][y]);

        fprintf(output, "\n");  
    }
  
    fclose(output);
}



//////////////////////////////////////////////////////////////////////////////// 
void store_sensordata(sensor_data_t* sd, ts_sensor_data_t* sensor_data)
{
    sd->timestamp = sensor_data->timestamp;
    sd->q1 = sensor_data->q1;
    sd->q2 = sensor_data->q2;  
    for(int i=0; i< ROBOT_LIDAR_SCAN_SIZE; i++)
       sd->d[i] = sensor_data->d[i];
}
        


//////////////////////////////////////////////////////////////////////////////// 
void init_params(ts_laser_parameters_t* laser_params, ts_robot_parameters_t* params)
{
    params->r = ROBOT_WHEEL_RADIUS;
    params->R = ROBOT_HALF_AXLE_LEN;
    params->inc = ROBOT_WHEEL_ENCODER_MAX; 
    params->ratio = ROBOT_WHEELS_RATIO;
    laser_params->offset = ROBOT_LIDAR_POS_OFFSET;
    laser_params->scan_size = ROBOT_LIDAR_SCAN_SIZE;
    laser_params->angle_min = ROBOT_LIDAR_SCAN_LEFT_ANGLE;
    laser_params->angle_max = ROBOT_LIDAR_SCAN_RIGHT_ANGLE;
    laser_params->detection_margin = ROBOT_LIDAR_SCANNING_ANGLE;  
    laser_params->distance_no_detection = ROBOT_LIDAR_RANGE; 
}


    
////////////////////////////////////////////////////////////////////////////////     
char* navcmd_to_string(robotcmd_t* cmd)
{
    switch(cmd->cmd)
    {
        case ROBOT_UTURN_RIGHT:
            sprintf(navcmd,"utr %d\n", cmd->data);
        break;
        case ROBOT_FWD:
            sprintf(navcmd,"fwd %d\n", cmd->data);       
        break;
        case ROBOT_UTURN_LEFT:
            sprintf(navcmd,"utl %d\n", cmd->data);       
        break;
        case ROBOT_STOP:
            sprintf(navcmd,"stop %d\n", cmd->data);
        break;   
        case ROBOT_HALT:
            sprintf(navcmd,"halt %d\n", cmd->data);
        break;   
        default:
        break;
    }
   return navcmd;
}



////////////////////////////////////////////////////////////////////////////////  
////////////////////////       MAIN  PROGRAM       /////////////////////////////
////////////////////////////////////////////////////////////////////////////////            
int main(int argc, char **argv)
{
        
    ts_laser_parameters_t laser_params;
    ts_robot_parameters_t params;
    int loopCount;
    ts_position_t startpos;
    ts_position_t loop_startpos;
    ts_position_t loop_pos;
    ts_state_t state;
    int x, y;
    int err = -1;
    int quality;
    bool dump_maps = false;
    robotcmd_t navcmds[MAX_NAV_CMDS];
    int navcmd_idx;
    int iteration_idx;
    bool pskilled = false;

    #ifdef DUMP_ODOMETRY
    odometry_t odometry[MAX_ODOMETRY_DATA];
    for(i=0;i<MAX_ODOMETRY_DATA;i++)
        odometry[i].timestamp=0;
    #endif  

    #ifdef DUMP_MAPS 
    dump_maps = true;
    #endif
   
    // set handler for CTRL-C
    signal(SIGINT, signal_callback_handler);

    
    // put start position at centre of map //
    startpos.x = 0.5 * TS_MAP_SIZE / TS_MAP_SCALE;
    startpos.y = 0.5 * TS_MAP_SIZE / TS_MAP_SCALE; 
    startpos.theta = 0;


    // initialize maps //
    ts_map_init(&map);
    ts_map_init(&trajectory);
    ts_map_init(&loop_map);
    

    //loop_start = 0;
    loop_startpos = startpos;

    // fill robot and lidar params
    init_params(&laser_params, &params);
    
    
    // initialise state struct based on set parameters etc (added note ARM). //
    // hole width = 600; sigma_theta = 20; sigma_xy = 100
    ts_state_init(&state, &map, &params, &laser_params, &loop_startpos, 100, 20, 600, TS_DIRECTION_FORWARD);


    // initialize robot //
    if(init_robot())
    {
        printf("error: failed to initialize robot. Quitting...\n");
        return 0;
    }
    
    
    // set series of navigation commands for demo loop //
    setcmd(&navcmds[0],ROBOT_UTURN_RIGHT,1);
    setcmd(&navcmds[1],ROBOT_FWD,3);
    setcmd(&navcmds[2],ROBOT_UTURN_RIGHT,1);
    setcmd(&navcmds[3],ROBOT_FWD,3);  
    navcmd_idx = 0;
    
    // set navigation loopcount
    loopCount = 0;
 


    printf("\n\t\t\tPHANTOM DEMO\n\trobot controller host unit (Xilinx Zynq platform)\n\n");
    
    

    ////////////////////////////////////////////////////////////////////////////

#ifdef PIONEER_3DX_ROBOT_VREP

    ////////////////////////////////////////////////////////////////////////////

    int iteration_count; 
    int map_count;  
    char* str;  
    int pingtime;
    int prev_nav_status = 1;    
    int last_loopcmd;
    int dump_sensordata;

    // empty map directory
    #ifdef DEBUG
    system("rm -rf maps");
    system("mkdir maps");
    #endif

    #ifdef DUMP_SENSORDATA
    int navcmd_count;
    dump_sensordata = 1;
    int sensordumps_idx[MAX_NAV_CMDS+1]; // array holding sensor count for each nav command
    #define DUMP_SENSORDATA_SIZE 3000
    sensordata = (sensor_data_t*) malloc(MAX_NAV_CMDS*DUMP_SENSORDATA_SIZE*sizeof(sensor_data_t));
    #endif
   
    // ensure signals cleared
    while(simxClearIntegerSignal(clientID, "toClient_navStatus",simx_opmode_oneshot));
    while(simxClearStringSignal(clientID, "toServer_navCmd",simx_opmode_oneshot));

   
    //
    // loop forever until VREP simulation terminated //
    //
    
    while (simxGetConnectionId(clientID)!=-1)
    {
        // trigger simulator next timestep (simulated 10 ms)
        simxSynchronousTrigger(clientID);
        simxGetPingTime(clientID, &pingtime); // wait for simulation step to complete

        // send nav commands       
        int nav_status;
        simxGetIntegerSignal(clientID, "toClient_navStatus", &nav_status, simx_opmode_blocking);
        if(nav_status == 1 && prev_nav_status == 0) // check if robot idle
        {
            str = navcmd_to_string(&navcmds[navcmd_idx++]);
            printf("issuing nav command: %s",str);
            simxSetStringSignal(clientID, "toServer_navCmd", str ,strlen(str), simx_opmode_blocking);
            if(last_loopcmd) // check for completion of last command in initial loop
                loopCount++;
            if(navcmd_idx == MAX_NAV_CMDS)
            {
                navcmd_idx = 0;
                last_loopcmd = 1;
            }

            #ifdef DUMP_SENSORDATA
            if (navcmd_count <= MAX_NAV_CMDS+1)
            {
            sensordumps_idx[navcmd_count] = iteration_idx;
            iteration_idx = 0;
            navcmd_count++;
            }
            #endif
        }

        prev_nav_status = nav_status;
        
        // get robot sensor data from simulator //
        err = ts_read_scan(&sensor_data, &laser_params, NULL); // fumction gets odometry and scan data from vrep sim
        
        #ifdef DUMP_ODOMETRY
        if(!err) 
            store_odometry(&odometry[interation_count], &sensor_data);
        #endif
        
        #ifdef DUMP_SENSORDATA
        if(!err && navcmd_count && !loopCount)
            store_sensordata(sensordata+((navcmd_count-1)*DUMP_SENSORDATA_SIZE)+iteration_idx++, &sensor_data);
        #endif
        
        
        //
        // We need to create the slam map during inital loop //
        //
       
        if(loopCount==0)
        {
            if(kill_program)
            {
                #ifdef DUMP_ODOMETRY
                dump_odometry_to_file("slam_output_odometry.txt", odometry);
                #endif
                break;
            } 
            
            // if scan data read-in ok then update map //
            if(!err)
            {
                ts_iterative_map_building(&sensor_data, &state);

                // store robot's trajectory in 1d trajectory map //
                x = (int)floor(state.position.x * TS_MAP_SCALE + 0.5);
                y = ((int)floor(state.position.y * TS_MAP_SCALE + 0.5));
                if (x >= 0 && y >= 0 && x < TS_MAP_SIZE && y < TS_MAP_SIZE)
                    trajectory.map[y * TS_MAP_SIZE + x] = 0; // the zero value is used to indicate robot position
                
                #ifdef DEBUG   
                iteration_count++;
                char map_name[80];
                if(!(iteration_count%50) || iteration_count==1) // create progressive images of slam map
                {
                    sprintf(map_name,"%s%d.pgm",MAP_IMAGE_FILENAME,map_count);
                    ts_save_trajectory_pgm(&trajectory, "maps/trajectory.pgm", TS_MAP_SIZE, TS_MAP_SIZE);
                    ts_save_map_pgm(&map, &trajectory, map_name, TS_MAP_SIZE, TS_MAP_SIZE);
                    map_count += 1;
                }
                #endif  
            }
        }


        //
        // After first loop create and store image of map then use it to navigate robot in subsequent loops
        //
        
        else
        {
 
            if(kill_program)
            {
                #ifdef DEBUG
                printf("q1=%d q2=%d theta=%.3f\n",state.q1, state.q2,state.position.theta);
                #ifdef DUMP_ODOMETRY
                dump_odometry_to_file("slam_output_odometry.txt", odometry);
                #endif
                #endif
                break;
            } 

            if (dump_maps) // do this only once
            {
                #ifdef DEBUG
                ts_save_trajectory_pgm(&trajectory, "scene_trajectory.pgm", TS_MAP_SIZE, TS_MAP_SIZE);
                ts_save_map_pgm(&map, &trajectory, "scene_map.pgm", TS_MAP_SIZE, TS_MAP_SIZE);   
                system("ffmpeg -loglevel quiet -f image2 -i \"maps/scene_map%d.pgm\" -y scanmapping_video.avi");   
                #endif    
                dump_maps = false;
            }
            
            if(dump_sensordata) // do this only once
            {
                #ifdef DUMP_SENSORDATA
                char filename[80];
                FILE *fdump;
                for(int k = 0; k < MAX_NAV_CMDS; k++)
                {
                    sprintf(filename,"sensor_data_cmd%d.txt", k);
                    fdump = fopen(filename, "wt");
                    for(int i=0; i < sensordumps_idx[k+1]; i++)
                    {
                        sensor_data_t* sd = &sensordata[(k*DUMP_SENSORDATA_SIZE)+i];
                        fprintf(fdump,"%u\n%d\n%d\n", sd->timestamp, sd->q1, sd->q2);
                        for(int j=0; j < ROBOT_LIDAR_SCAN_SIZE; j++)
                           fprintf(fdump,"%d ", sd->d[j]);
                        fprintf(fdump,"\n");
                    }
                    fclose(fdump);
                }
                free(sensordata);
                dump_sensordata = false; 
                #endif
            }
            
          
            if(!err)
            {

                update_state(&sensor_data, &state);

                loop_pos = ts_close_loop_position(&state, &sensor_data, &map, &state.position, &quality);
                printf("dx=%.1f dy=%.1f dtheta=%.1f\n",loop_pos.x-state.position.x, loop_pos.y-state.position.y, loop_pos.theta-state.position.theta);     
            }
        }
    }

    simxPauseSimulation(clientID, simx_opmode_blocking);
    sleep(2);
    simxStopSimulation(clientID, simx_opmode_blocking);
    simxFinish(clientID);


    #ifdef DEBUG
    system("rm -rf maps");
    #endif



    //////////////////////////////////////////////////////////////////////////// 

#elif defined PHANTOM_ROBOT 

    //////////////////////////////////////////////////////////////////////////// 

    #include <netdb.h>
    #include <sys/socket.h>
    #include <sys/types.h>
    #include <errno.h>
    #include <sys/time.h>

  
    int portno;
    struct hostent *server;
    extern int h_errno,errno;
    struct sockaddr_in serv_addr;
    unsigned short prev_status;
    unsigned short sensor_status;
    struct timeval start,stop,res;

  

    // create client endpoint for comms 
    sockfd = socket(AF_INET, SOCK_STREAM, 0); // IPv4 format, sequenced 2-way connection type.
    if (sockfd < 0) 
    {
        printf("ERROR opening socket\n");
        return -1;
    }
 
    server = gethostbyname(HOST_IP_ADDRESS);
    if (server == NULL) 
    {
        printf("ERROR - unable to get server struct. %d\n", h_errno);
        return -1;
    }   
       
    portno = ROBOT_PORT_NUMBER;
    
    bzero((char *) &serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    bcopy((char *)server->h_addr, (char *)&serv_addr.sin_addr.s_addr, server->h_length);
    
    serv_addr.sin_port = htons(portno);
    
    if (connect(sockfd,(struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0) 
    {
        perror("ERROR");
        printf("- connecting to server on %s port %d failed!\n\n",server->h_name, portno);
        close(sockfd);
        return -1;
    }
        
        
    // now have connection to the robot's Edison board so able to recieve sensor data
    
    printf("building slam map from sensor data ...\n");
    
    navcmd_idx = 0;
    prev_status = ROBOT_IDLE;
    
    printf("issuing nav command: %s",navcmd_to_string(&navcmds[0]));
    

    //
    // Main loop to collect sensor data and issue navigation commands.
    // Currently, the code does not check robot position and does not attempt to correct for errors. TBD.
    // It simply builds the SLAM map on initial loop and quits.
    //
    
    while(1)
    {   
 
        #ifdef DEBUG
        gettimeofday(&start, NULL);
        #endif
        
       //
        // catch CTRL-C
        //
        if(kill_program)
        {
            pskilled = true;
            break;
        }
          
        //    
        // get sensor data from Edison board (blocking)
        //
        err = ts_read_scan(&sensor_data, &laser_params, &sensor_status);


  
        //
        // issue robot navigation commands
        //
        if ((prev_status == ROBOT_BUSY) && (sensor_status == ROBOT_IDLE))
        {
            navcmd_idx ++;
            printf("issuing nav command: %s",navcmd_to_string(&navcmds[navcmd_idx]));
        }
        if(navcmd_idx < MAX_NAV_CMDS)
        {
            prev_status = sensor_status;
            if(send_navcmd(&navcmds[navcmd_idx]))
                break;
        }
        else // all done so quit
            break;
            
      
        // 
        // build map 
        // 
        
        if(!err)
        {
 
            ts_iterative_map_building(&sensor_data, &state);
        
            // store robot's trajectory in 1-D map //
            x = (int)floor(state.position.x * TS_MAP_SCALE + 0.5);
            y = ((int)floor(state.position.y * TS_MAP_SCALE + 0.5));
            if (x >= 0 && y >= 0 && x < TS_MAP_SIZE && y < TS_MAP_SIZE)
                trajectory.map[y * TS_MAP_SIZE + x] = 0; // the zero value is used to indicate robot position
        }
        else
        {
            printf("error in reading scan data from robot!\n");
        }
 
        #ifdef DEBUG
        gettimeofday(&stop, NULL);
        timersub(&stop,&start, &res);
        printf("loop time = %.1f ms\n",(double) res.tv_usec/1000.0);
        #endif
        
    } // main loop
    
    
    // close comms socket with Edison board
    shutdown(sockfd,SHUT_RDWR);
    close(sockfd);
        

    //
    // create image of generated SLAM map
    //
    
    if (dump_maps && !pskilled)
    {
        #ifdef DEBUG
        printf("storing map as scene_map.pgm...\n");
        #endif
        ts_save_map_pgm(&map, &trajectory, "scene_map.pgm", TS_MAP_SIZE, TS_MAP_SIZE);  
    }
 

#endif // PHANTOM_ROBOT 


    printf("\n");
    
	return 0;
}
