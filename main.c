/* 
 * Edited by: Sarayu Managoli
 * Author: Dr. Sam Siewert
 * Overview: This file contains the source code for the final project in the course ECEN5623 Real Time Operating Systems
 * Board Used: Raspberry Pi 3+ 
 * Code Leverage: Sharpen transformation - http://ecee.colorado.edu/~ecen5623/ecen/ex/Linux/computer-vision/sharpen-psf/sharpen.c
 * 				  Capture - http://ecee.colorado.edu/~ecen5623/ecen/ex/Linux/computer-vision/simple-capture/capture.c
 *                Scheduler - http://ecee.colorado.edu/~ecen5623/ecen/ex/Linux/code/sequencer_generic/seqgen.c
 *                Sending images over sockets - https://stackoverflow.com/questions/13097375/sending-images-over-sockets-in-c
 * 				  						https://stackoverflow.com/questions/15445207/sending-image-jpeg-through-socket-in-c-linux
 */

/*
 *
 *  Adapted by Sam Siewert for use with UVC web cameras and Bt878 frame
 *  grabber NTSC cameras to acquire digital video from a source,
 *  time-stamp each frame acquired, save to a PGM or PPM file.
 *
 *  The original code adapted was open source from V4L2 API and had the
 *  following use and incorporation policy:
 * 
 *  This program can be used and distributed without restrictions.
 *
 *      This program is provided with the V4L2 API
 * see http://linuxtv.org/docs.php for more information
 */
 
#define _GNU_SOURCE 
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <getopt.h>             /* getopt_long() */
#include <fcntl.h>              /* low-level i/o */
#include <unistd.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>
#include <pthread.h>
#include <syslog.h>
#include <semaphore.h>
#include <sched.h>
#include <sys/socket.h> 
#include <arpa/inet.h> 
#include <sys/sysinfo.h>
#include <linux/videodev2.h>
#include <sched.h>
#include <sys/utsname.h>

#define CLEAR(x) memset(&(x), 0, sizeof(x))
#define COLOR_CONVERT
#define ERROR (-1)
#define OK (0)
#define NSEC_PER_SEC (1000000000)
#define MY_CLOCK_TYPE CLOCK_MONOTONIC_RAW
#define RATE (1.2)
#define USEC_PER_MSEC (1000)
#define NANOSEC_PER_MSEC (1000000)
#define NANOSEC_PER_SEC (1000000000)
#define TRUE (1)
#define FALSE (0)
#define SHARPEN
#define K 4.0
#define PORT 8080
#define RIGHT_FRAME 30
//#define HERTZ 1
#define str_hres "640"
#define str_vres "480"

//definition of threads
typedef struct
{
	int threadIdx;
	unsigned long long sequencePeriods;
} threadParams_t;

typedef double FLOAT;

enum io_method 
{
        IO_METHOD_READ,
        IO_METHOD_MMAP,
        IO_METHOD_USERPTR,
};

struct buffer 
{
        void   *start;
        size_t  length;
};


//declarations
unsigned int framecnt=0;
struct timespec frame_time;
int flag = 0;
double deadline;
unsigned char bigbuffer[921600];
struct timespec start_time_val;
double start_realtime;
sem_t semS1,semS2,semS3;
int g_size;
struct utsname platform;
struct sockaddr_in server_address;
int sockfd;
int image_size;
static FILE *f;
unsigned char client_buf[921600];
int send_count = 0;
int socket_enable;
char jitter_S1[] = "Image_Capture.csv";
char jitter_S2[] = "Image_Dump.csv";
char jitter_Seq[] = "Sequencer.csv";
static FILE *S1;
static FILE *S2;
static FILE *Seq;

static char            *dev_name;
static enum io_method   io = IO_METHOD_MMAP;
static int              fd = -1;
struct buffer          *buffers;
static unsigned int     n_buffers;
static int              out_buf;
static int              force_format=1;
int frame_count;
int size;

static int dump_flag = 0;
int abortTest=FALSE;
int abortS1=FALSE, abortS2=FALSE, abortS3=FALSE;

int missed;
pthread_t main_thread;
pthread_attr_t main_sched_attr;
int rt_max_prio, rt_min_prio, min;
struct sched_param main_param;

static int HRES_STR;
static int VRES_STR;

static char ppm_header[]="P6\n#Time: 9999999999 sec 9999999999 msec \n#Platform: 9999999999999 9999999\n"str_hres" "str_vres"\n255\n";
char ppm_dumpname[]="test00000000.ppm";

char hres_string[3];
char vres_string[3];

double exec_time, exec_time_max;
double service1_averagetime, service2_averagetime, service3_averagetime,service1_averagewcet,service2_averagewcet;
double sequence_averagetime,sequence_averagewcet;
double sequence_averagejitter, service1_averagejitter, service2_averagejitter, service3_averagejitter;
unsigned char arr_img[60][921600];

FLOAT PSF[9] = {-K/8.0, -K/8.0, -K/8.0, -K/8.0, K+1.0, -K/8.0, -K/8.0, -K/8.0, -K/8.0};
static struct v4l2_format fmt;

static void mainloop(void);

double realtime(struct timespec *tsptr)
{
    return ((double)(tsptr->tv_sec) + (((double)tsptr->tv_nsec)/1000000000.0));
}

/* @Function name : time_ms
 * @Description : Timestamp generated in milliseconds
 * @Return value : double
 */
 
double time_ms()
{
    struct timespec timing = {0,0};
    clock_gettime(CLOCK_REALTIME, &timing);
    return(((double)timing.tv_sec*(double)1000) + ((double)timing.tv_nsec/(double)1000000));
}

/* @Function name : dump_ppm
 * @Description : Generates PPM files from the buffer input
 * @Return value : void
 */

static void dump_ppm(const void *p, int size, unsigned int tag, struct timespec *time)
{
    int written, i, total, dumpfd;
    syslog(LOG_INFO,"In PPM Dump");
    snprintf(&ppm_dumpname[4], 9, "%08d", tag);
    strncat(&ppm_dumpname[12], ".ppm", 5);
    dumpfd = open(ppm_dumpname, O_WRONLY | O_NONBLOCK | O_CREAT, 00666);

    snprintf(&ppm_header[0], 100, "P6\n#Time: %010d sec %010d msec \n#Platform: %s %s\n"str_hres" "str_vres"\n255\n", 
		(int)time->tv_sec, (int)((time->tv_nsec)/1000000), platform.nodename, platform.machine);
      
    written=write(dumpfd, ppm_header, sizeof(ppm_header));
    total=0;
    do
    {
        written=write(dumpfd, p, size);
        total+=written;
    } while(total < size);
    
    close(dumpfd);
}

/* @Function name : Service_1
 * @Description : This service captures and processes raw images at 1 or 10 Hz
 * @Return value : void
 */

void *Service_1(void *threadp)
{
    struct timespec current_time_val;
    double current_realtime;
    int S1Cnt=0;
    int j = 0;
    double service1_starttime = 0;
    double service1_endtime;
    double service1_time;
    double service1_wcet = 0;
    double service1_totaltime = 0;
    double service1_jitter, service1_refjitter, service1_starttime_prev = 0;
    double service1_totaljitter = 0;
    
    S1 = fopen(jitter_S1,"w+");
    fprintf(S1,"Frame Count,Start Time(in ms),End Time(in ms),Execution Time(in ms),Jitter(in ms)");
    
    threadParams_t *threadParams = (threadParams_t *)threadp;

    clock_gettime(MY_CLOCK_TYPE, &current_time_val); current_realtime=realtime(&current_time_val);
    syslog(LOG_CRIT, "S1 thread @ sec=%6.9lf\n", current_realtime-start_realtime);
    while(!abortS1)
    {
        sem_wait(&semS1);
        j++;
        service1_starttime_prev = service1_starttime;
        service1_starttime = time_ms();
        syslog(LOG_INFO,"Service 1 started at %lf ms",service1_starttime);
    
        mainloop();
        
        service1_endtime = time_ms();
        syslog(LOG_INFO,"Service 1 ended at %lf ms",service1_endtime);
        
        service1_time = service1_endtime - service1_starttime;
        printf("\nImage captured!\tTime taken: %lf ms\n",service1_time);
        service1_totaltime += service1_time;
        
        if(S1Cnt!=0)
        if(service1_wcet < service1_time) service1_wcet = service1_time;
        
        if(S1Cnt != 0)
        {
            #ifdef HERTZ
            service1_jitter = (service1_starttime_prev + 1000) - service1_starttime;
            #else
            service1_jitter = (service1_starttime_prev + 100) - service1_starttime;
            #endif
            service1_totaljitter += service1_jitter;
        }
        
        if(S1Cnt >= RIGHT_FRAME)
            fprintf(S1,"\n%d,%lf,%lf,%lf,%lf",(S1Cnt-RIGHT_FRAME+1),service1_starttime,service1_endtime,service1_time,service1_jitter);
        
        S1Cnt++;
        syslog(LOG_INFO,"S1Cnt = %d",S1Cnt);
    
        
        for(int i=0;i<921600;i++)
        {
            arr_img[j % 60][i] = bigbuffer[i];
        }
    }
    
    printf("\n*********************SUMMARY*********************\n");
    service1_averagetime = (service1_totaltime/frame_count);
    printf("\nService 1 Average Execution Time %lf ms\n",service1_averagetime);
    printf("\nService 1 WCET = %lf ms",service1_wcet);
    service1_averagejitter = service1_totaljitter/frame_count;
    printf("\nService 1 Average Jitter %lf ms\n",service1_averagejitter);
    printf("\n*************************************************\n");  

    pthread_exit((void *)0);
}

/* @Function name : Service_2
 * @Description : This service calls dump_ppm() to generate images at 1 or 10 Hz
 * @Return value : void
 */


void *Service_2(void *threadp)
{
    printf("\nIn Service 1 outside\n");
    struct timespec current_time_val;
    double current_realtime;
    int S2Cnt=0;
    double service2_starttime = 0;
    double service2_endtime;
    double service2_time = 0;
    double service2_jitter, service2_refjitter, service2_starttime_prev = 0;
    double service2_totaljitter = 0;
    double service2_wcet = 0;
    double service2_totaltime = 0;
    threadParams_t *threadParams = (threadParams_t *)threadp;
    
    S2 = fopen(jitter_S2,"w+");
    fprintf(S2,"Frame Count,Start Time(in ms),End Time(in ms),Execution Time(in ms),Jitter(in ms)");

    clock_gettime(MY_CLOCK_TYPE, &current_time_val); current_realtime=realtime(&current_time_val);
    syslog(LOG_CRIT, "S2 thread @ sec=%6.9lf\n", current_realtime-start_realtime);

    while(!abortS2)
    {
        sem_wait(&semS2);
        service2_starttime_prev = service2_starttime;

        service2_starttime = time_ms();
        syslog(LOG_INFO,"Service 2 started at %lf ms",service2_starttime);
        if(S2Cnt >= RIGHT_FRAME)
        {
            dump_ppm((arr_img + ((S2Cnt) % 60)), g_size, framecnt, &frame_time);
            framecnt++;
            if(socket_enable == 1)
            dump_flag = 1;
        }
        service2_endtime = time_ms();
        syslog(LOG_INFO,"Service 2 ended at %lf ms",service2_endtime);
        
        service2_time = service2_endtime - service2_starttime;
        printf("\nImage dumped!\tTime taken:  %lf ms\n",service2_time);
        service2_totaltime += service2_time;        
        
        if(service2_wcet < service2_time) service2_wcet = service2_time;
        
        if(S2Cnt >= RIGHT_FRAME)
            fprintf(S2,"\n%d,%lf,%lf,%lf,%lf",(S2Cnt-RIGHT_FRAME+1),service2_starttime,service2_endtime,service2_time,service2_jitter);
        
        if(S2Cnt != 0) 
        {
            #ifdef HERTZ
            service2_jitter = (service2_starttime_prev + 1000) - service2_starttime;
            #else
            service2_jitter = (service2_starttime_prev + 100) - service2_starttime;
            #endif
            service2_totaljitter += service2_jitter;
        }
        
        S2Cnt++;
        syslog(LOG_INFO,"S2Cnt = %d",S2Cnt);
        
        if((socket_enable == 1) && (dump_flag == 1))
        {   
            sem_post(&semS3);
        }
        
    }
    
    printf("\n*********************SUMMARY*********************\n");
    service2_averagetime = (service2_totaltime/frame_count);
    printf("\nService 2 Average Execution Time %lf ms\n",service2_averagetime);
    printf("\nService 2 WCET = %lf ms",service2_wcet);
    service2_averagejitter = service2_totaljitter/frame_count;
    printf("\nService 2 Average Jitter %lf ms\n",service2_averagejitter);
    printf("\n*************************************************\n");
      
    pthread_exit((void *)0);
}

/* @Function name : Service_3
 * @Description : This service sends the generated PPM files to the server system over TCP connection.
 *                This is called if socket_enable is true.
 * @Return value : void
 */

void *Service_3(void *threadp)
{
    struct timespec current_time_val;
    double current_realtime;
    int S3Cnt=0;
    double service3_starttime = 0;
    double service3_endtime;
    double service3_time;
    double service3_jitter, service3_refjitter, service3_starttime_prev = 0;
    double service3_totaljitter = 0;
    double service3_wcet = 0;
    double service3_totaltime = 0;
    int image_number = 0;
    int image_size;
    int send_success;
    
    threadParams_t *threadParams = (threadParams_t *)threadp;

    clock_gettime(MY_CLOCK_TYPE, &current_time_val); current_realtime=realtime(&current_time_val);
    syslog(LOG_CRIT, "S3 thread @ sec=%6.9lf\n", current_realtime-start_realtime);

    while(!abortS3)
    {
        sem_wait(&semS3);
        syslog(LOG_INFO,"S3Cnt = %d\n",S3Cnt);
        syslog(LOG_INFO,"Num = %d",image_number);
        service3_starttime_prev = service3_starttime;
        service3_starttime = time_ms();
        syslog(LOG_INFO,"Service 3 started at %lf ms",service3_starttime);
                
        if(dump_flag == 1)
        {
            dump_flag = 0;
            snprintf(&ppm_dumpname[4], 9, "%08d", image_number);
            strncat(&ppm_dumpname[12], ".ppm", 5);
            f = fopen(ppm_dumpname,"rb");
            fseek(f,0,SEEK_END);
            image_size = ftell(f);
            fseek(f,0,SEEK_SET);
            
            image_size = fread(client_buf,1,sizeof(client_buf),f);
            send_success = send(sockfd,(char *)&client_buf,image_size,0);

            if(send_success == -1)
            {
                syslog(LOG_INFO,"Unable to send socket");
            }
            else
            {
                syslog(LOG_INFO,"Socket Send %d SUCCESSFUL = %d bytes",send_count,send_success);
                printf("\nSent sockets successfully\n");
                send_count++;
            }
            fclose(f);
            image_number++;
        }
       
        service3_endtime = time_ms();
        syslog(LOG_INFO,"Service 3 ended at %lf ms",service3_endtime);
        
        service3_time = service3_endtime - service3_starttime;
        printf("\nImage sent!\tTime taken:  %lf ms\n",service3_time);
        service3_totaltime += service3_time;
        
        
        if(service3_wcet < service3_time) service3_wcet = service3_time;
        
        if(S3Cnt != 0) 
        {
            #ifdef HERTZ
            service3_jitter = (service3_starttime_prev + 1000) - service3_starttime;
            #else
            service3_jitter = (service3_starttime_prev + 100) - service3_starttime;
            #endif
            service3_totaljitter += service3_jitter;
        }
         S3Cnt++;
        
        syslog(LOG_INFO,"S3Cnt = %d",S3Cnt);
        
        clock_gettime(MY_CLOCK_TYPE, &current_time_val); current_realtime=realtime(&current_time_val);
        syslog(LOG_CRIT, "S3 20 Hz on core %d for release %llu @ sec=%6.9lf\n", sched_getcpu(), S3Cnt, current_realtime-start_realtime);
    }
    service3_averagetime = (service3_totaltime/frame_count);
    printf("\n*********************SUMMARY*********************\n");
    printf("\nService 3 Average Execution Time %lf ms\n",service3_averagetime);
    printf("\nService 3 WCET = %lf ms",service3_wcet);
    service3_averagejitter = service3_totaljitter/frame_count;
    printf("\nService 3 Average Jitter %lf ms\n",service3_averagejitter);
    printf("\n*************************************************\n");
    
    pthread_exit((void *)0);
}


/* @Function name : Sequencer
 * @Description : This function is reponsible for calling each of the services
 *                at the agreed upon frequency.
 * @Return value : void
 */


void *Sequencer(void *threadp)
{
    int delay_cnt, rc;
    struct timeval current_time_val;
#ifdef HERTZ
    struct timespec delay_time = {1,0}; // delay for 33.33 msec, 30 Hz
#else
    struct timespec delay_time = {0,100000000};
#endif
    struct timespec remaining_time;
    double current_time;
    double residual;
    double sequence_jitter, sequence_refjitter, sequence_starttime_prev = 0;
    double sequence_totaljitter = 0;
    double sequence_starttime = 0;
    double sequence_endtime = 0;
    double sequence_time;
    double sequence_wcet = 0;
    double sequence_totaltime = 0;
    int seqCnt=0;
    
    Seq = fopen(jitter_Seq,"w+");
    fprintf(Seq,"Frame Count,Start Time(in ms),End Time(in ms),Execution Time(in ms),Jitter(in ms)");
    
    threadParams_t *threadParams = (threadParams_t *)threadp;

 

    gettimeofday(&current_time_val, (struct timezone *)0);
    syslog(LOG_CRIT, "Sequencer thread @ sec=%d, msec=%d\n", (int)(current_time_val.tv_sec-start_time_val.tv_sec), (int)current_time_val.tv_usec/USEC_PER_MSEC);
    //printf("Sequencer thread @ sec=%d, msec=%d\n", (int)(current_time_val.tv_sec-start_time_val.tv_sec), (int)current_time_val.tv_usec/USEC_PER_MSEC);

    do
    {
        delay_cnt=0; residual=0.0;

        //gettimeofday(&current_time_val, (struct timezone *)0);
        //syslog(LOG_CRIT, "Sequencer thread prior to delay @ sec=%d, msec=%d\n", (int)(current_time_val.tv_sec-start_time_val.tv_sec), (int)current_time_val.tv_usec/USEC_PER_MSEC);
        do
        {
            rc=nanosleep(&delay_time, &remaining_time);

            if(rc == EINTR)
            { 
                residual = remaining_time.tv_sec + ((double)remaining_time.tv_nsec / (double)NANOSEC_PER_SEC);

                if(residual > 0.0) printf("residual=%lf, sec=%d, nsec=%d\n", residual, (int)remaining_time.tv_sec, (int)remaining_time.tv_nsec);
 
                delay_cnt++;
            }
            else if(rc < 0)
            {
                perror("Sequencer nanosleep");
                exit(-1);
            }
           
        } while((residual > 0.0) && (delay_cnt < 100));

    
        if(seqCnt == 0) sequence_refjitter = time_ms();
       
        //gettimeofday(&current_time_val, (struct timezone *)0);
       // syslog(LOG_CRIT, "Sequencer cycle %llu @ sec=%d, msec=%d\n", seqCnt, (int)(current_time_val.tv_sec-start_time_val.tv_sec), (int)current_time_val.tv_usec/USEC_PER_MSEC);

        sequence_starttime_prev = sequence_starttime;        
        sequence_starttime = time_ms();
        syslog(LOG_INFO,"Sequence started at %lf ms",sequence_starttime);
        
        if(delay_cnt > 1) printf("Sequencer looping delay %d\n", delay_cnt);
        // Servcie_1 = RT_MAX-1	@ 1 Hz
        if((seqCnt % 1) == 0) 
        {
            sem_post(&semS1);
        }
        // Release each service at a sub-rate of the generic sequencer rate
        // Service_2 = RT_MAX-2	@ 1 Hz
        if((seqCnt % 1) == 0)
        {
            sem_post(&semS2);
        }
        
        sequence_endtime = time_ms();
        syslog(LOG_INFO,"Sequence ended at %lf ms",sequence_endtime);
        
        sequence_time = sequence_endtime - sequence_starttime;
        printf("\nSequencer executed!\tTime taken:  %lf ms\n",sequence_time);
        sequence_totaltime += sequence_time;
        
        if(sequence_wcet < sequence_time) sequence_wcet = sequence_time;
        
        if(seqCnt >= RIGHT_FRAME)
            fprintf(Seq,"\n%d,%lf,%lf,%lf,%lf",(seqCnt-RIGHT_FRAME+1),sequence_starttime,sequence_endtime,sequence_time,sequence_jitter);
        
        if(seqCnt != 0)
        {
            #ifdef HERTZ
            sequence_jitter = (sequence_starttime_prev + 1000) - sequence_starttime;
            #else
            sequence_jitter = (sequence_starttime_prev + 100) - sequence_starttime;
            #endif
            sequence_totaljitter += sequence_jitter;
        }
        

        seqCnt++;
        syslog(LOG_INFO,"SeqCnt = %d", seqCnt);
        
    } while(!abortTest && (seqCnt <= (frame_count + RIGHT_FRAME)));

    sem_post(&semS1); 
    sem_post(&semS2); 
    
    abortS1=TRUE; 
    abortS2=TRUE;
    abortS3=TRUE;
    printf("\n*********************SUMMARY*********************\n");
    sequence_averagetime = (sequence_totaltime/frame_count);
    printf("\nSequence Average Execution Time %lf ms\n",sequence_averagetime);
    printf("\nSequence WCET = %lf ms",sequence_wcet);
    sequence_averagejitter = sequence_totaljitter/frame_count;
    printf("\nSequence Average Jitter %lf ms\n",sequence_averagejitter);
    printf("\n*************************************************\n");
    

    pthread_exit((void *)0);
}

/* @Function name : print_scheduler
 * @Description : This function prints the sceduling policy adhered to by the program.
 * @Return value : void
 */
 
void print_scheduler(void)
{
   int schedType;
   schedType = sched_getscheduler(getpid());

   switch(schedType)
   {
     case SCHED_FIFO:
           printf("Pthread Policy is SCHED_FIFO\n");
           break;
     case SCHED_OTHER:
           printf("Pthread Policy is SCHED_OTHER\n");
       break;
     case SCHED_RR:
           printf("Pthread Policy is SCHED_OTHER\n");
           break;
     default:
       printf("Pthread Policy is UNKNOWN\n");
   }
}

static void errno_exit(const char *s)
{
        fprintf(stderr, "%s error %d, %s\n", s, errno, strerror(errno));
        exit(EXIT_FAILURE);
}

static int xioctl(int fh, int request, void *arg)
{
        int r;

        do 
        {
            r = ioctl(fh, request, arg);

        } while (-1 == r && EINTR == errno);

        return r;
}



// This is probably the most acceptable conversion from camera YUYV to RGB
//
// Wikipedia has a good discussion on the details of various conversions and cites good references:
// http://en.wikipedia.org/wiki/YUV
//
// Also http://www.fourcc.org/yuv.php
//
// What's not clear without knowing more about the camera in question is how often U & V are sampled compared
// to Y.
//
// E.g. YUV444, which is equivalent to RGB, where both require 3 bytes for each pixel
//      YUV422, which we assume here, where there are 2 bytes for each pixel, with two Y samples for one U & V,
//              or as the name implies, 4Y and 2 UV pairs
//      YUV420, where for every 4 Ys, there is a single UV pair, 1.5 bytes for each pixel or 36 bytes for 24 pixels

void yuv2rgb(int y, int u, int v, unsigned char *r, unsigned char *g, unsigned char *b)
{
   int r1, g1, b1;

   // replaces floating point coefficients
   int c = y-16, d = u - 128, e = v - 128;       

   // Conversion that avoids floating point
   r1 = (298 * c           + 409 * e + 128) >> 8;
   g1 = (298 * c - 100 * d - 208 * e + 128) >> 8;
   b1 = (298 * c + 516 * d           + 128) >> 8;

   // Computed values may need clipping.
   if (r1 > 255) r1 = 255;
   if (g1 > 255) g1 = 255;
   if (b1 > 255) b1 = 255;

   if (r1 < 0) r1 = 0;
   if (g1 < 0) g1 = 0;
   if (b1 < 0) b1 = 0;

   *r = r1 ;
   *g = g1 ;
   *b = b1 ;
}


/* @Function name : process_image
 * @Description : Processes the image from YUV to RGB format.
 * @Return value : void
 */
 
static void process_image(const void *p, int size)
{
    int i, k, newsize=0;
    unsigned char *pptr = (unsigned char *)p;

    // record when process was called
    clock_gettime(CLOCK_REALTIME, &frame_time);

    // This just dumps the frame to a file now, but you could replace with whatever image
    // processing you wish.
    //
    int y_temp, y2_temp, u_temp, v_temp;
    for(i=0, k=0; i<size; i=i+4, k=k+6)
    {
        y_temp=(int)pptr[i]; u_temp=(int)pptr[i+1]; y2_temp=(int)pptr[i+2]; v_temp=(int)pptr[i+3];
        yuv2rgb(y_temp, u_temp, v_temp, &bigbuffer[k], &bigbuffer[k+1], &bigbuffer[k+2]);
        yuv2rgb(y2_temp, u_temp, v_temp, &bigbuffer[k+3], &bigbuffer[k+4], &bigbuffer[k+5]);
    }
    
    g_size = (size*6)/4;
    
    fflush(stderr);
    fflush(stdout);
}

/* @Function name : read_frame
 * @Description : Allocates memory and captures the image in its raw state.
 * @Return value : int
 */
 
static int read_frame(void)
{
    struct v4l2_buffer buf;
    unsigned int i;
   

    CLEAR(buf);

    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;

    if (-1 == xioctl(fd, VIDIOC_DQBUF, &buf))
    {
        switch (errno)
        {
            case EAGAIN:
                return 0;

            case EIO:
                /* Could ignore EIO, but drivers should only set for serious errors, although some set for
                   non-fatal errors too.
                 */
                return 0;


            default:
                printf("mmap failure\n");
                errno_exit("VIDIOC_DQBUF");
        }
    }

    assert(buf.index < n_buffers);

    process_image(buffers[buf.index].start, buf.bytesused);

    if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
            errno_exit("VIDIOC_QBUF");

}

/* @Function name : mainloop
 * @Description : Sets the read delay and each frame is read
 * @Return value : void
 */
 
static void mainloop(void)
{
    unsigned int count;
    float rate = 0;
    struct timespec read_delay;
    struct timespec time_error;
	double worst_exec=0;
    int frame_number = 0;

    read_delay.tv_sec=0;
    read_delay.tv_nsec=30000;
	
	rate = RATE;

    count = frame_count;
    fd_set fds;
    struct timeval tv;
    int r;

    FD_ZERO(&fds);
    FD_SET(fd, &fds);

    /* Timeout. */
    tv.tv_sec = 2;
    tv.tv_usec = 0;

    r = select(fd + 1, &fds, NULL, NULL, &tv);

    if (-1 == r)
    {
        errno_exit("select");
    }

    if (0 == r)
    {
        fprintf(stderr, "select timeout\n");
        exit(EXIT_FAILURE);
    }
    if (read_frame())
    {
        if(nanosleep(&read_delay, &time_error) != 0)
            perror("nanosleep");
    }
}


/* @Function name : stop_capturing
 * @Description : Stop frames from being read.
 * @Return value : void
 */

static void stop_capturing(void)
{
        enum v4l2_buf_type type;

        switch (io) {
        case IO_METHOD_READ:
                /* Nothing to do. */
                break;

        case IO_METHOD_MMAP:
        case IO_METHOD_USERPTR:
                type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                if (-1 == xioctl(fd, VIDIOC_STREAMOFF, &type))
                        errno_exit("VIDIOC_STREAMOFF");
                break;
        }
}

/* @Function name : start_capturing
 * @Description : Start reading from video0 device.
 * @Return value : void
 */
 
static void start_capturing(void)
{
        unsigned int i;
        enum v4l2_buf_type type;

        switch (io) 
        {

        case IO_METHOD_READ:
                /* Nothing to do. */
                break;

        case IO_METHOD_MMAP:
                for (i = 0; i < n_buffers; ++i) 
                {
                        struct v4l2_buffer buf;

                        CLEAR(buf);
                        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                        buf.memory = V4L2_MEMORY_MMAP;
                        buf.index = i;

                        if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
                                errno_exit("VIDIOC_QBUF");
                }
                type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                if (-1 == xioctl(fd, VIDIOC_STREAMON, &type))
                        errno_exit("VIDIOC_STREAMON");
                break;

        case IO_METHOD_USERPTR:
                for (i = 0; i < n_buffers; ++i) {
                        struct v4l2_buffer buf;

                        CLEAR(buf);
                        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                        buf.memory = V4L2_MEMORY_USERPTR;
                        buf.index = i;
                        buf.m.userptr = (unsigned long)buffers[i].start;
                        buf.length = buffers[i].length;

                        if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
                                errno_exit("VIDIOC_QBUF");
                }
                type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                if (-1 == xioctl(fd, VIDIOC_STREAMON, &type))
                        errno_exit("VIDIOC_STREAMON");
                break;
        }
}


/* @Function name : init_read
 * @Description : Initialises reading from a video0 dev.
 * @Return value : void
 */
 
static void init_read(unsigned int buffer_size)
{
        buffers = calloc(1, sizeof(*buffers));

        if (!buffers) 
        {
                fprintf(stderr, "Out of memory\n");
                exit(EXIT_FAILURE);
        }

        buffers[0].length = buffer_size;
        buffers[0].start = malloc(buffer_size);

        if (!buffers[0].start) 
        {
                fprintf(stderr, "Out of memory\n");
                exit(EXIT_FAILURE);
        }
}

static void init_mmap(void)
{
        struct v4l2_requestbuffers req;

        CLEAR(req);

        req.count = 6;
        req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory = V4L2_MEMORY_MMAP;

        if (-1 == xioctl(fd, VIDIOC_REQBUFS, &req)) 
        {
                if (EINVAL == errno) 
                {
                        fprintf(stderr, "%s does not support "
                                 "memory mapping\n", dev_name);
                        exit(EXIT_FAILURE);
                } else 
                {
                        errno_exit("VIDIOC_REQBUFS");
                }
        }

        if (req.count < 2) 
        {
                fprintf(stderr, "Insufficient buffer memory on %s\n", dev_name);
                exit(EXIT_FAILURE);
        }

        buffers = calloc(req.count, sizeof(*buffers));

        if (!buffers) 
        {
                fprintf(stderr, "Out of memory\n");
                exit(EXIT_FAILURE);
        }

        for (n_buffers = 0; n_buffers < req.count; ++n_buffers) {
                struct v4l2_buffer buf;

                CLEAR(buf);

                buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                buf.memory      = V4L2_MEMORY_MMAP;
                buf.index       = n_buffers;

                if (-1 == xioctl(fd, VIDIOC_QUERYBUF, &buf))
                        errno_exit("VIDIOC_QUERYBUF");

                buffers[n_buffers].length = buf.length;
                buffers[n_buffers].start =
                        mmap(NULL /* start anywhere */,
                              buf.length,
                              PROT_READ | PROT_WRITE /* required */,
                              MAP_SHARED /* recommended */,
                              fd, buf.m.offset);

                if (MAP_FAILED == buffers[n_buffers].start)
                        errno_exit("mmap");
        }
}

static void init_userp(unsigned int buffer_size)
{
        struct v4l2_requestbuffers req;

        CLEAR(req);

        req.count  = 4;
        req.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory = V4L2_MEMORY_USERPTR;

        if (-1 == xioctl(fd, VIDIOC_REQBUFS, &req)) {
                if (EINVAL == errno) {
                        fprintf(stderr, "%s does not support "
                                 "user pointer i/o\n", dev_name);
                        exit(EXIT_FAILURE);
                } else {
                        errno_exit("VIDIOC_REQBUFS");
                }
        }

        buffers = calloc(4, sizeof(*buffers));

        if (!buffers) {
                fprintf(stderr, "Out of memory\n");
                exit(EXIT_FAILURE);
        }

        for (n_buffers = 0; n_buffers < 4; ++n_buffers) {
                buffers[n_buffers].length = buffer_size;
                buffers[n_buffers].start = malloc(buffer_size);

                if (!buffers[n_buffers].start) {
                        fprintf(stderr, "Out of memory\n");
                        exit(EXIT_FAILURE);
                }
        }
}

/* @Function name : init_device
 * @Description : Initialises video0 dev.
 * @Return value : void
 */
 
static void init_device(void)
{
    struct v4l2_capability cap;
    struct v4l2_cropcap cropcap;
    struct v4l2_crop crop;
    unsigned int min;

    if (-1 == xioctl(fd, VIDIOC_QUERYCAP, &cap))
    {
        if (EINVAL == errno) {
            fprintf(stderr, "%s is no V4L2 device\n",
                     dev_name);
            exit(EXIT_FAILURE);
        }
        else
        {
                errno_exit("VIDIOC_QUERYCAP");
        }
    }

    if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE))
    {
        fprintf(stderr, "%s is no video capture device\n",
                 dev_name);
        exit(EXIT_FAILURE);
    }

    switch (io)
    {
        case IO_METHOD_READ:
            if (!(cap.capabilities & V4L2_CAP_READWRITE))
            {
                fprintf(stderr, "%s does not support read i/o\n",
                         dev_name);
                exit(EXIT_FAILURE);
            }
            break;

        case IO_METHOD_MMAP:
        case IO_METHOD_USERPTR:
            if (!(cap.capabilities & V4L2_CAP_STREAMING))
            {
                fprintf(stderr, "%s does not support streaming i/o\n",
                         dev_name);
                exit(EXIT_FAILURE);
            }
            break;
    }


    /* Select video input, video standard and tune here. */


    CLEAR(cropcap);

    cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    if (0 == xioctl(fd, VIDIOC_CROPCAP, &cropcap))
    {
        crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        crop.c = cropcap.defrect; /* reset to default */

        if (-1 == xioctl(fd, VIDIOC_S_CROP, &crop))
        {
            switch (errno)
            {
                case EINVAL:
                    /* Cropping not supported. */
                    break;
                default:
                    /* Errors ignored. */
                        break;
            }
        }

    }
    else
    {
        /* Errors ignored. */
    }


    CLEAR(fmt);

    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    if (force_format)
    {
        printf("FORCING FORMAT\n");
        fmt.fmt.pix.width       = HRES_STR;
        fmt.fmt.pix.height      = VRES_STR;

        // Specify the Pixel Coding Formate here

        // This one work for Logitech C200
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
        fmt.fmt.pix.field       = V4L2_FIELD_NONE;

        if (-1 == xioctl(fd, VIDIOC_S_FMT, &fmt))
                errno_exit("VIDIOC_S_FMT");

        /* Note VIDIOC_S_FMT may change width and height. */
    }
    else
    {
        printf("ASSUMING FORMAT\n");
        /* Preserve original settings as set by v4l2-ctl for example */
        if (-1 == xioctl(fd, VIDIOC_G_FMT, &fmt))
                    errno_exit("VIDIOC_G_FMT");
    }

    /* Buggy driver paranoia. */
    min = fmt.fmt.pix.width * 2;
    if (fmt.fmt.pix.bytesperline < min)
            fmt.fmt.pix.bytesperline = min;
    min = fmt.fmt.pix.bytesperline * fmt.fmt.pix.height;
    if (fmt.fmt.pix.sizeimage < min)
            fmt.fmt.pix.sizeimage = min;

    switch (io)
    {
        case IO_METHOD_READ:
            init_read(fmt.fmt.pix.sizeimage);
            break;

        case IO_METHOD_MMAP:
            init_mmap();
            break;

        case IO_METHOD_USERPTR:
            init_userp(fmt.fmt.pix.sizeimage);
            break;
    }
}

/* @Function name : uninit_device
 * @Description : Unitialises video0 dev.
 * @Return value : void
 */

static void uninit_device(void)
{
int i;
        switch (io) {
        case IO_METHOD_READ:
                free(buffers[0].start);
                break;

        case IO_METHOD_MMAP:
                for (i = 0; i < n_buffers; ++i)
                        if (-1 == munmap(buffers[i].start, buffers[i].length))
                                errno_exit("munmap");
                break;

        case IO_METHOD_USERPTR:
                for (i = 0; i < n_buffers; ++i)
                        free(buffers[i].start);
                break;
        }

        free(buffers);
}


static void open_device(void)
{
    
        struct stat st;

        if (-1 == stat(dev_name, &st)) {
                fprintf(stderr, "Cannot identify '%s': %d, %s\n",
                         dev_name, errno, strerror(errno));
                exit(EXIT_FAILURE);
        }

        if (!S_ISCHR(st.st_mode)) {
                fprintf(stderr, "%s is no device\n", dev_name);
                exit(EXIT_FAILURE);
        }

        fd = open(dev_name, O_RDWR /* required */ | O_NONBLOCK, 0);

        if (-1 == fd) {
                fprintf(stderr, "Cannot open '%s': %d, %s\n",
                         dev_name, errno, strerror(errno));
                exit(EXIT_FAILURE);
        }
}

static void usage(FILE *fp, int argc, char **argv)
{
        fprintf(fp,
                 "Usage: %s [options]\n\n"
                 "Version 1.3\n"
                 "Options:\n"
                 "-d | --device name   Video device name [%s]\n"
                 "-h | --help          Print this message\n"
                 "-m | --mmap          Use memory mapped buffers [default]\n"
                 "-r | --read          Use read() calls\n"
                 "-u | --userp         Use application allocated buffers\n"
                 "-o | --output        Outputs stream to stdout\n"
                 "-f | --format        Force format to 640x480 GREY\n"
                 "-c | --count         Number of frames to grab [%i]\n"
                 "",
                 argv[0], dev_name, frame_count);
}

static const char short_options[] = "d:hmruofc:";

static const struct option
long_options[] = {
        { "device", required_argument, NULL, 'd' },
        { "help",   no_argument,       NULL, 'h' },
        { "mmap",   no_argument,       NULL, 'm' },
        { "read",   no_argument,       NULL, 'r' },
        { "userp",  no_argument,       NULL, 'u' },
        { "output", no_argument,       NULL, 'o' },
        { "format", no_argument,       NULL, 'f' },
        { "count",  required_argument, NULL, 'c' },
        { 0, 0, 0, 0 }
};

static void close_device(void)
{
        if (-1 == close(fd))
                errno_exit("close");

        fd = -1;
}

/* @Function name : main
 * @Description : This function accepts user inputs, allocates scheduling policies, creates threads, 
 *                initialises semaphores, assigns CPU cores for each of the threads.
 * @Return value : int
 */

int main(int argc, char **argv)
{
	int i,rc,scope;
    cpu_set_t threadcpu;
	threadParams_t threadParams[1];
	pthread_attr_t rt_sched_attr[1];
	int rt_max_prio, rt_min_prio;
	struct sched_param rt_param[1];
	struct sched_param main_param;
    pid_t mainpid;
    cpu_set_t allcpuset;
    int num_threads;
    
    system("uname -a");
    uname(&platform);
    
	pthread_attr_t main_attr;
    dev_name = "/dev/video0";
   
    HRES_STR = atoi(argv[1]);
    VRES_STR = atoi(argv[2]);
    frame_count = atoi(argv[3]);
    socket_enable = atoi(argv[4]);
    
    printf("\nResolution requested is:\nHorizontal = %d\nVertical = %d\n",HRES_STR,VRES_STR);
    strncpy(hres_string,argv[1],sizeof(hres_string));
    strncpy(vres_string,argv[2],sizeof(vres_string));
    
    if(socket_enable == 1)
    {
        printf("\nYou have enabled the socket!\n");
        num_threads = 4;
    }
    else
    {
        printf("\nYou have disabled the socket!\n");
        num_threads = 3;
    }
        
    pthread_t threads[num_threads];
    
    syslog(LOG_INFO,"*****************PROGRAM BEGINS*****************");

    int idx,c;
    c = getopt_long(argc, argv, short_options, long_options, &idx);

    
    if (sem_init (&semS1, 0, 0)) { printf ("Failed to initialize S1 semaphore\n"); exit (-1); }
    if (sem_init (&semS2, 0, 0)) { printf ("Failed to initialize S2 semaphore\n"); exit (-1); }
    if (sem_init (&semS3, 0, 0)) { printf ("Failed to initialize S2 semaphore\n"); exit (-1); }
    
    mainpid=getpid();

    rt_max_prio = sched_get_priority_max(SCHED_FIFO);
    rt_min_prio = sched_get_priority_min(SCHED_FIFO);

    rc=sched_getparam(mainpid, &main_param);
    main_param.sched_priority=rt_max_prio;
    rc=sched_setscheduler(getpid(), SCHED_FIFO, &main_param);
    if(rc < 0) perror("main_param");
    print_scheduler();


    pthread_attr_getscope(&main_attr, &scope);

    if(scope == PTHREAD_SCOPE_SYSTEM)
      printf("PTHREAD SCOPE SYSTEM\n");
    else if (scope == PTHREAD_SCOPE_PROCESS)
      printf("PTHREAD SCOPE PROCESS\n");
    else
      printf("PTHREAD SCOPE UNKNOWN\n");

    printf("rt_max_prio=%d\n", rt_max_prio);
    printf("rt_min_prio=%d\n", rt_min_prio);

    for(i=0; i < num_threads; i++)
    {
      rc=pthread_attr_init(&rt_sched_attr[i]);
      rc=pthread_attr_setinheritsched(&rt_sched_attr[i], PTHREAD_EXPLICIT_SCHED);
      rc=pthread_attr_setschedpolicy(&rt_sched_attr[i], SCHED_FIFO);
      pthread_attr_setschedparam(&rt_sched_attr[i], &rt_param[i]);
      threadParams[i].threadIdx=i;
    }
   
    printf("Service threads will run on %d CPU cores\n", CPU_COUNT(&threadcpu));

    open_device();
    init_device();
    start_capturing();
    
    if(socket_enable == 1)
    {
        printf("\nSocket enabled\n");
        sockfd = socket(AF_INET, SOCK_STREAM, 0);
        server_address.sin_family = AF_INET;
        server_address.sin_port = htons(PORT);

       
        if(inet_pton(AF_INET, "10.0.0.196", &server_address.sin_addr)<=0)
        {
            syslog(LOG_ERR,"\nInvalid address/ Address not supported \n");
            return -1;
        }

        if (connect(sockfd, (struct sockaddr *)&server_address, sizeof(server_address)) < 0)
        {
            syslog(LOG_ERR,"\nConnection Failed \n");
            return -1;
        }
    }

    CPU_ZERO(&threadcpu);
    CPU_SET(3, &threadcpu);
    
    rc=pthread_attr_setaffinity_np(&rt_sched_attr[1], sizeof(cpu_set_t), &threadcpu);
    
    rt_param[1].sched_priority=rt_max_prio-1;
    rc=pthread_create(&threads[1], &rt_sched_attr[1], Service_1, (void *)&(threadParams[1]));
    if(rc < 0)
        perror("pthread_create for service 1");
    else
        printf("pthread_create successful for service 1\n");
        
    printf("Start sequencer\n");
    threadParams[0].sequencePeriods=900;  
        
    CPU_ZERO(&threadcpu);
    CPU_SET(3, &threadcpu);
    
    rc=pthread_attr_setaffinity_np(&rt_sched_attr[0], sizeof(cpu_set_t), &threadcpu);
    rt_param[0].sched_priority=rt_max_prio;
    
    rc=pthread_create(&threads[0], &rt_sched_attr[0], Sequencer, (void *)&(threadParams[0]));
    if(rc < 0)
        perror("pthread_create for sequencer");
    else
        printf("pthread_create successful for sequencer\n"); 
    
        
    CPU_ZERO(&threadcpu);
    CPU_SET(3, &threadcpu);
    
    rc=pthread_attr_setaffinity_np(&rt_sched_attr[2], sizeof(cpu_set_t), &threadcpu);
    rt_param[2].sched_priority=rt_max_prio - 2;
    rc=pthread_create(&threads[2], &rt_sched_attr[2], Service_2, (void *)&(threadParams[2]));
    if(rc < 0)
        perror("pthread_create for service 2");
    else
        printf("pthread_create successful for service 2\n");
    
    if(socket_enable == 1)
    {
        CPU_ZERO(&threadcpu);
        CPU_SET(2, &threadcpu);
        
        rc=pthread_attr_setaffinity_np(&rt_sched_attr[3], sizeof(cpu_set_t), &threadcpu);
        rt_param[3].sched_priority=rt_max_prio - 3;
        rc=pthread_create(&threads[3], &rt_sched_attr[3], Service_3, (void *)&(threadParams[3]));
        if(rc < 0)
            perror("pthread_create for service 3");
        else
            printf("pthread_create successful for service 3\n");
    }
    
    //Join to wait for completion of thread execution
    for(i=0;i<num_threads;i++)
       pthread_join(threads[i], NULL);
    
    stop_capturing();
    uninit_device();
    close_device();
    fprintf(stderr, "\n");

    fclose(S1);
    fclose(S2);
    fclose(Seq);

    return 0;
}
