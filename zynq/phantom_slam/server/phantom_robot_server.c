/*
 * Filename:    robot_server.c
 * Author:      A. Moulds. University of York.
 * Date:        20 March 2019
 * Project:     PHANTOM - Robot Demo
 * Copyright:   University of York
 * 
 * Description: Program receives navigation commands from host and manouvers demo robot accordingly. It
 *              is responsible for controlling all robot functions (motor drive, odometry, etc.) and also
 *              collecting all sensor data. The raw LiDAR sensor data is decoded and packed with odometry
 *              data; this data is sent to networked host sprior to reading next host nav command.
 *
 * Current Version: 0.10 (development only)
 * 
 * Revisions:
 * 
 * Notes:       Cross-compile using arm-linux-gnueabihf tool chain.
 * 
 * Target:      ARM (Xilinx Zynq7)
 * 
*/


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <sys/types.h> 
#include <sys/socket.h>
#include <netinet/in.h>
#include "robot_defs.h"


#define SIMULATED_ROBOT
#define REALTIME_SIMULATION
//#define DEBUG
//#define NOCONSOLE


/* Globals */
static sensor_data_t sensordata;   
static int sockfd;
static int newsockfd;

#ifdef SIMULATED_ROBOT
static sensor_data_t* sensptr; 
static int len_of_first_uturnr_sensordata;
static int len_of_second_uturnr_sensordata;
static int len_of_first_fwd_sensordata;
static int len_of_second_fwd_sensordata;
static sensor_data_t* first_uturnr_sensordata;
static sensor_data_t* second_uturnr_sensordata;  
static sensor_data_t* first_fwd_sensordata;  
static sensor_data_t* second_fwd_sensordata; 
static int do_robot_uturnr_idx;
static int do_robot_fwd_idx;
static unsigned long prev_timestamp;
#endif
 
 

/*//////////////////////////////////////////////////////////////////////////////// 
 * 
 */
void usdelay (unsigned int usecs) {
clock_t goal = usecs*CLOCKS_PER_SEC/1000000 + clock();  //convert usecs to clock count  
while ( goal > clock() );               // Loop until it arrives.

}



/*//////////////////////////////////////////////////////////////////////////////// 
 * 
 */
void msdelay (unsigned int msecs) {
clock_t goal = msecs*CLOCKS_PER_SEC/1000 + clock();  //convert msecs to clock count  
while ( goal > clock() );               // Loop until it arrives.

}


 
/*//////////////////////////////////////////////////////////////////////////////// 
 * 
 */
void get_sensordata_fromfile(FILE* fp, sensor_data_t** sptr, int* len)
{
    int lines=0;

    while(!feof(fp))
    {
        char ch = fgetc(fp);
        if(ch == '\n')
            lines++;
    }
    *len = lines/4;  
    *sptr = (sensor_data_t*) malloc(*len * sizeof(sensor_data_t));      
    sensor_data_t* ptr;
    ptr = *sptr;
    rewind(fp);
    for(int i=0; i < *len; i++,ptr++)
    { 
        fscanf(fp,"%u\n", &ptr->timestamp); 
        fscanf(fp,"%d\n", &ptr->q1);    
        fscanf(fp,"%d\n", &ptr->q2);
        char* linebuf=NULL;
        size_t buflen=0;
        getline(&linebuf,&buflen,fp);
        if(buflen < ROBOT_LIDAR_SCAN_SIZE*2 -1) // check we have enough LiDAR data
            break;
        char* str = strtok(linebuf, " ");
        for(int k=0; k<ROBOT_LIDAR_SCAN_SIZE; k++)
        {
            sscanf(str, "%hd", &ptr->d[k]);
            str = strtok(NULL, " ");           
        }
    }
}



/*//////////////////////////////////////////////////////////////////////////////// 
 * 
 */
unsigned short do_robot_fwd(unsigned short distance)
{
#ifdef SIMULATED_ROBOT
    static int idx = 0;
    static int len;
    if(!idx)
    {
        if(do_robot_fwd_idx == 0)  
        {    
            sensptr = first_fwd_sensordata;
            len = len_of_first_fwd_sensordata;
        }
        else
        {
            sensptr = second_fwd_sensordata;   
            len = len_of_second_fwd_sensordata;
        } 
        #ifndef NOCONSOLE
        printf("robot moving forward ...\n");
        #endif
    }
    sensordata.timestamp = sensptr->timestamp;
    sensordata.q1 = sensptr->q1;
    sensordata.q2 = sensptr->q2;
    memcpy(sensordata.d,sensptr->d,ROBOT_LIDAR_SCAN_SIZE*sizeof(short));
    sensptr++;
    #ifdef REALTIME_SIMULATION
    usdelay(sensordata.timestamp-prev_timestamp); // insert time delay to emulate realtime behaviour
    #endif
    prev_timestamp = sensordata.timestamp;
    if (idx == len-1)
    {
        #ifndef NOCONSOLE
        printf("completed forward movement\n");   
        #endif
        idx = 0;
        do_robot_fwd_idx++;
        return ROBOT_IDLE;
    }
    idx++;

#endif
    return ROBOT_BUSY;

}



/*//////////////////////////////////////////////////////////////////////////////// 
 * 
 */
unsigned short do_robot_uturnr(unsigned short radius)
{
#ifdef SIMULATED_ROBOT
    static int idx = 0;
    static int len;
    if(!idx)
    {
        if(do_robot_uturnr_idx == 0)  
        {    
            sensptr = first_uturnr_sensordata;
            len = len_of_first_uturnr_sensordata;
        }
        else
        {
            sensptr = second_uturnr_sensordata;   
            len = len_of_second_uturnr_sensordata;
        } 
        #ifndef NOCONSOLE
        printf("robot turning right (u-turn)...\n");
        #endif
    }
    sensordata.timestamp = sensptr->timestamp;
    sensordata.q1 = sensptr->q1;
    sensordata.q2 = sensptr->q2;
    memcpy(sensordata.d,sensptr->d,ROBOT_LIDAR_SCAN_SIZE*sizeof(short));
    sensptr++;
    #ifdef REALTIME_SIMULATION
    usdelay(sensordata.timestamp-prev_timestamp); // insert time delay to emulate realtime behaviour
    #endif
    prev_timestamp = sensordata.timestamp;
    if (idx == len-1)
    {
        #ifndef NOCONSOLE
        printf("completed u-turn\n");   
        #endif
        idx = 0;
        do_robot_uturnr_idx++;
        return ROBOT_IDLE;
    }
    idx++;


#endif
    return ROBOT_BUSY;
}


/*//////////////////////////////////////////////////////////////////////////////// 
 * 
 * Function to emulate robot left u-turn.
 * 
 */ 
unsigned short do_robot_uturnl(unsigned short radius)
{
    // TBD //
    
    return ROBOT_IDLE;
}



/*//////////////////////////////////////////////////////////////////////////////// 
 * 
 */ 
unsigned short do_robot_stop(void)
{
    // TBD //
    
    return ROBOT_IDLE;
}  
           
   

/*//////////////////////////////////////////////////////////////////////////////// 
 * 
 */        
unsigned short robot_action(robotcmd_t hostcommand)
{
    unsigned short status;   
    switch(hostcommand.cmd)
    {
        case ROBOT_FWD:
           status = do_robot_fwd(hostcommand.data);
        break;
        case ROBOT_UTURN_RIGHT:
           status = do_robot_uturnr(hostcommand.data);
        break;
        case ROBOT_UTURN_LEFT:
           status = do_robot_uturnl(hostcommand.data);
        break;
        case ROBOT_STOP:
           status = do_robot_stop();     
        break;
        default:
            status = ROBOT_IDLE;
        break;
    }
    return status;     
}
        
    
/*//////////////////////////////////////////////////////////////////////////////// 
 * 
 */ 
int initialize()
{
    
#ifdef SIMULATED_ROBOT
    
    FILE* fp;
   
    // load sensor data for first right u-turn movement
    #ifdef DEBUG
    printf("loading sensor_data_cmd0.txt data\n");
    #endif
    fp = fopen("../sensor_data/sensor_data_cmd0.txt", "rt");
    if(fp==NULL)
    {
        perror("ERROR");
        return -1;
    } 
    get_sensordata_fromfile(fp, &first_uturnr_sensordata, &len_of_first_uturnr_sensordata);
    fclose(fp);

    // load sensor data for first fwd movement
    #ifdef DEBUG
    printf("loading sensor_data_cmd1.txt data\n");
    #endif
    fp = fopen("../sensor_data/sensor_data_cmd1.txt", "rt");
    if(fp==NULL)
    {
        perror("ERROR");
        return -1;
    }  
    get_sensordata_fromfile(fp, &first_fwd_sensordata, &len_of_first_fwd_sensordata);
    fclose(fp);
  
    // load sensor data for second right u-turn movement
    #ifdef DEBUG
    printf("loading sensor_data_cmd2.txt data\n");
    #endif
    fp = fopen("../sensor_data/sensor_data_cmd2.txt", "rt");
    if(fp==NULL)
    {
        perror("ERROR");
        return -1;
    }  
    get_sensordata_fromfile(fp, &second_uturnr_sensordata, &len_of_second_uturnr_sensordata);
    fclose(fp);
    
    // load sensor data for second fwd movement
    #ifdef DEBUG
    printf("loading sensor_data_cmd1.txt data\n");
    #endif
    fp = fopen("../sensor_data/sensor_data_cmd3.txt", "rt");
    if(fp==NULL)
    {
        perror("ERROR");
        return -1;
    }  
    get_sensordata_fromfile(fp, &second_fwd_sensordata, &len_of_second_fwd_sensordata);
    fclose(fp);
    
#endif
    
    return 0;
}



/*//////////////////////////////////////////////////////////////////////////////// 
 * 
 */
int init_comms(void)   
{
    socklen_t clilen;
    struct sockaddr_in serv_addr, cli_addr;
    int portno;

    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) 
    {
        perror("ERROR opening socket");
        return -1;
    }
    bzero((char *) &serv_addr, sizeof(serv_addr));
    portno = ROBOT_PORT_NUMBER;
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = INADDR_ANY;
    serv_addr.sin_port = htons(portno);
    if (bind(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0) 
    {
        perror("ERROR on binding port");    
        close(sockfd);
        return -1;
    }
    #ifdef DEBUG
    printf("listening on port %d\n",portno);
    #endif
    #ifndef NOCONSOLE
    printf("listening...\n");    
    #endif
    listen(sockfd,5);

    clilen = sizeof(cli_addr);
    newsockfd = accept(sockfd, (struct sockaddr *) &cli_addr, &clilen);
    if (newsockfd < 0) 
    {
        perror("ERROR on accept");
        close(sockfd);
        return -1;          
    }
    #ifndef NOCONSOLE
    printf("host link established\n");
    #endif
    return 0;
}



/*//////////////////////////////////////////////////////////////////////////////// 
 * 
 */
void cleanup(void)
{  
     close(newsockfd);
     close(sockfd);        
 
#ifdef SIMULATED_ROBOT
     free(first_uturnr_sensordata);
     free(second_uturnr_sensordata);  
     free(first_fwd_sensordata);  
     free(second_fwd_sensordata);    
#endif
}


/*//////////////////////////////////////////////////////////////////////////////// 
 * 
 */
int main(int argc, char **argv)
{
    int i;
    robotcmd_t hostcommand;
    int status;

    #ifndef NOCONSOLE
    printf("\n\t\t\tPHANTOM DEMO\n\trobot controller slave unit (Intel Edison board)\n\n");
    #endif

    #ifndef NOCONSOLE
    printf("initializing robot...\n");
    #endif
    if(initialize())
    {
        printf("failed to initialize!\n");
        return -1;         
    }

    #ifndef NOCONSOLE
    printf("initializing comms...\n");
    #endif
    if(init_comms())
    {
        printf("failed to establish link with host! Quitting...\n");
        return -1;
    }

    // create cleared first scan
    sensordata.timestamp = 0L;
    sensordata.status = ROBOT_IDLE;
    sensordata.q1 = 0;
    sensordata.q2 = 0;
    for(i=0; i < ROBOT_LIDAR_SCAN_SIZE; i++)
        sensordata.d[i] = 0;

    //
    // The program sends robot sensor data and status to host (receiver) followed by command
    // read from host. These two operations are looped indefinately until link down (broken). Each host
    // command is issued repeatedly until 'status' indicates the robot command has completed.
    // This technique of continuous write and read between socket ends enables link synchromization and
    // health monitoring. 
    //

    while(1)
    {

        // always send sensor data to host
        #ifdef DEBUG 
        printf("sending sensor data...\n"); 
        #endif
        status = write(newsockfd, (void *) &sensordata, sizeof(sensor_data_t));
        if (status <= 0)
        {
            #ifndef NOCONSOLE
            printf("lost connection!\n");
            #endif
            break;
        }
 
        // always receive command from host
        #ifdef DEBUG 
        printf("receiving host command...\n");
        #endif
        status = read(newsockfd, (void *) &hostcommand, sizeof(robotcmd_t));
        if (status <= 0)
        {
            printf("lost connection!\n");
            break;
        }

        // execute host command
        sensordata.status = robot_action(hostcommand);     

    }     

    cleanup();

    return 0;
     
}