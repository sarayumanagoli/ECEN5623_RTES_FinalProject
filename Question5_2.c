/* 
 * Edited by: Sarayu Managoli
 * Author: Dr. Sam Siewert
 * Overview: Solution for Question 5 Part B
 * Board Used: Raspberry Pi 3+ 
 * Code Leverage: Sharpen transformation - http://ecee.colorado.edu/~ecen5623/ecen/ex/Linux/computer-vision/sharpen-psf/sharpen.c
 * 				  Capture - http://ecee.colorado.edu/~ecen5623/ecen/ex/Linux/computer-vision/simple-capture/capture.c
 *                Delta T - http://ecee.colorado.edu/~ecen5623/ecen/ex/Linux/code/RT-Clock/posix_clock.c
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
#include <semaphore.h>
#include <sched.h>

#include <sys/sysinfo.h>
#include <linux/videodev2.h>
#include <sched.h>
#include <sys/utsname.h>

#define CLEAR(x) memset(&(x), 0, sizeof(x))
#define COLOR_CONVERT
#define ERROR (-1)
#define OK (0)
#define NSEC_PER_SEC (1000000000)


//selection between various transforms

#define SHARPEN
//#define BW
//#define GRAY

//definition of threads
typedef struct
{
    int threadIdx;
    int timeperiod;
} threadParams_t;


//declarations
unsigned int framecnt=0;
struct timespec frame_time;
int flag = 0;
double deadline;
unsigned char bigbuffer[(640*480*3)];


//to calculate the time difference for jitters
static struct timespec img = {0, 0};
static struct timespec img_start_time = {0, 0};
static struct timespec img_stop_time = {0, 0};

enum io_method 
{
        IO_METHOD_READ,
        IO_METHOD_MMAP,
        IO_METHOD_USERPTR,
};

#define RATE (1.2)
static char            *dev_name;
//static enum io_method   io = IO_METHOD_USERPTR;
//static enum io_method   io = IO_METHOD_READ;
static enum io_method   io = IO_METHOD_MMAP;
static int              fd = -1;
struct buffer          *buffers;
static unsigned int     n_buffers;
static int              out_buf;
static int              force_format=1;
static int              frame_count = 90;

static void mainloop(void);

int missed;
pthread_t main_thread;
pthread_attr_t main_sched_attr;
int rt_max_prio, rt_min_prio, min;
struct sched_param main_param;

char ppm_header[50];
char pgm_header[50];
char pgm_dumpname[]="test00000000.pgm";
char ppm_dumpname[]="test00000000.ppm";

//resolutions
static int HRES;
static int VRES;

char hres_string[3];
char vres_string[3];

double exec_time, exec_time_max;


typedef double FLOAT;
#define K 4.0

double total_jitter=0;		
double jitter=0;		

FLOAT PSF[9] = {-K/8.0, -K/8.0, -K/8.0, -K/8.0, K+1.0, -K/8.0, -K/8.0, -K/8.0, -K/8.0};
static struct v4l2_format fmt;


struct buffer 
{
        void   *start;
        size_t  length;
};

//PPM image format
static void dump_ppm(const void *p, int size, unsigned int tag, struct timespec *time)
{
    struct utsname hostname;
    char timestampbuffer[100] = "\0";
    int written, i, total, dumpfd;
    uname(&hostname);
   
    snprintf(&ppm_dumpname[4], 9, "%08d", tag);
    strncat(&ppm_dumpname[12], ".ppm", 5);
    dumpfd = open(ppm_dumpname, O_WRONLY | O_NONBLOCK | O_CREAT, 00666);
 
    snprintf(&ppm_header[4], 11, "%010d", (int)time->tv_sec);
    strncat(&ppm_header[14], " sec ", 5);
    snprintf(&ppm_header[19], 11, "%010d", (int)((time->tv_nsec)/1000000));
    written=write(dumpfd, ppm_header, sizeof(ppm_header));

    total=0;

    do
    {
        written=write(dumpfd, p, size);
        total+=written;
    } while(total < size);
    snprintf(timestampbuffer, sizeof(timestampbuffer), "./datetime.sh %s %s %s %s", hostname.machine, hostname.sysname, hostname.nodename, ppm_dumpname);
    system(timestampbuffer);
    close(dumpfd);
    
}

//PGM image format
static void dump_pgm(const void *p, int size, unsigned int tag, struct timespec *time)
{
    int written, i, total, dumpfd;
   
    snprintf(&pgm_dumpname[4], 9, "%08d", tag);
    strncat(&pgm_dumpname[12], ".pgm", 5);
    dumpfd = open(pgm_dumpname, O_WRONLY | O_NONBLOCK | O_CREAT, 00666);

    snprintf(&pgm_header[4], 11, "%010d", (int)time->tv_sec);
    strncat(&pgm_header[14], " sec ", 5);
    snprintf(&pgm_header[19], 11, "%010d", (int)((time->tv_nsec)/1000000));
    written=write(dumpfd, pgm_header, sizeof(pgm_header));

    total=0;

    do
    {
        written=write(dumpfd, p, size);
        total+=written;
    } while(total < size);
    close(dumpfd);
    
}


int delta_t(struct timespec *stop, struct timespec *start, struct timespec *delta_t)
{
  int dt_sec=stop->tv_sec - start->tv_sec;
  int dt_nsec=stop->tv_nsec - start->tv_nsec;

  // case 1 - less than a second of change
  if(dt_sec == 0)
  {
	  if(dt_nsec >= 0)
	  {
		  delta_t->tv_sec = 0;
		  delta_t->tv_nsec = dt_nsec;
	  }

	  else // dt_nsec < 0 means stop is earlier than start
	  {
		 return(ERROR);  
	  }
  }

  // case 2 - more than a second of change, check for roll-over
  else if(dt_sec > 0)
  {
	  if(dt_nsec >= 0)
	  {
		  delta_t->tv_sec = dt_sec;
		  delta_t->tv_nsec = dt_nsec;
	  }

	  else // dt_nsec < 0 means roll over
	  {
		  delta_t->tv_sec = dt_sec-1;
		  delta_t->tv_nsec = NSEC_PER_SEC - dt_nsec;
	  }
  }

  return(OK);
}


//Thread calling funtion for all the transforms
void *ThreadTransform(void *threadp)
{
	threadParams_t *threadParams = (threadParams_t *)threadp;
	printf("Prototype Analysis\n");
    //Deadline is calculated for each of the frames
    mainloop();
    //jitter calculation flag
    flag = 1;
	mainloop();
}

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

//Function for BW transformation
void blackwhite_image(const void *image_pointer,int dimension)
{
    int k;
	unsigned char *pptr = (unsigned char *)image_pointer;
	for(k=0;k<dimension;k++)
	{
		if(pptr[k] < 100) //100 is the pixel value
		{
			pptr[k] = 0; //black
		}
		else
		{
			pptr[k] = 255; //white
		}
	}
}

//all transformations are done here
/*void transform(const void *p, int size)
{
    int i,j,k;
    int y_temp, y2_temp, u_temp, v_temp;
    unsigned char *pptr = (unsigned char *)p;
#ifdef SHARPEN
    for(i=0, k=0; i<size; i=i+4, k=k+6)
    {
        y_temp=(int)pptr[i]; u_temp=(int)pptr[i+1]; y2_temp=(int)pptr[i+2]; v_temp=(int)pptr[i+3];
        yuv2rgb(y_temp, u_temp, v_temp, &bigbuffer[k], &bigbuffer[k+1], &bigbuffer[k+2]);
        yuv2rgb(y2_temp, u_temp, v_temp, &bigbuffer[k+3], &bigbuffer[k+4], &bigbuffer[k+5]);
    }
   // sharpening(bigbuffer,((size*6)/4));
    dump_ppm(bigbuffer, ((size*6)/4), framecnt, &frame_time);
#endif
#ifdef BW
// Pixels are YU and YV alternating, so YUYV which is 4 bytes
// We want Y, so YY which is 2 bytes
//
    for(i=0, k=0; i<size; i=i+4, k=k+2)
    {
    // Y1=first byte and Y2=third byte
    bigbuffer[k]=pptr[i];
    bigbuffer[k+1]=pptr[i+2];
    }
    blackwhite_image(bigbuffer,(size/2));
    dump_pgm(bigbuffer, (size/2), framecnt, &frame_time);
#endif

#ifdef GRAY
    for(i=0, k=0; i<size; i=i+4, k=k+2)
    {
            // Y1=first byte and Y2=third byte
            bigbuffer[k]=pptr[i];
            bigbuffer[k+1]=pptr[i+2];
    }
    dump_pgm(bigbuffer, (size/2), framecnt, &frame_time);
#endif

}*/

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



void yuv2rgb_float(float y, float u, float v, 
                   unsigned char *r, unsigned char *g, unsigned char *b)
{
    float r_temp, g_temp, b_temp;

    // R = 1.164(Y-16) + 1.1596(V-128)
    r_temp = 1.164*(y-16.0) + 1.1596*(v-128.0);  
    *r = r_temp > 255.0 ? 255 : (r_temp < 0.0 ? 0 : (unsigned char)r_temp);

    // G = 1.164(Y-16) - 0.813*(V-128) - 0.391*(U-128)
    g_temp = 1.164*(y-16.0) - 0.813*(v-128.0) - 0.391*(u-128.0);
    *g = g_temp > 255.0 ? 255 : (g_temp < 0.0 ? 0 : (unsigned char)g_temp);

    // B = 1.164*(Y-16) + 2.018*(U-128)
    b_temp = 1.164*(y-16.0) + 2.018*(u-128.0);
    *b = b_temp > 255.0 ? 255 : (b_temp < 0.0 ? 0 : (unsigned char)b_temp);
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


/*Function: Sharpening
 *Description: Sharpens the input image provided through pointer p and for the size specified*/
void sharpening(const void *p, int size)
{
    int i,j,k;
    FLOAT temp;
    unsigned char R[VRES*HRES];
    unsigned char G[VRES*HRES];
    unsigned char B[VRES*HRES];
    unsigned char convR[VRES*HRES];
    unsigned char convG[VRES*HRES];
    unsigned char convB[VRES*HRES];

    for(i=0,k=0; i<size; (i=i+3),k++)
    {
        R[k]=*((unsigned char *)(p+i));
        G[k]=*((unsigned char *)(p+i+1));
        B[k]=*((unsigned char *)(p+i+2));
    }

    // Skip first and last row, no neighbors to convolve with
    for(i=1; i<((VRES)-1); i++)
    {
        // Skip first and last column, no neighbors to convolve with
        for(j=1; j<((HRES)-1); j++)
        {
            temp=0;
            temp += (PSF[0] * (FLOAT)R[((i-1)*HRES)+j-1]);
            temp += (PSF[1] * (FLOAT)R[((i-1)*HRES)+j]);
            temp += (PSF[2] * (FLOAT)R[((i-1)*HRES)+j+1]);
            temp += (PSF[3] * (FLOAT)R[((i)*HRES)+j-1]);
            temp += (PSF[4] * (FLOAT)R[((i)*HRES)+j]);
            temp += (PSF[5] * (FLOAT)R[((i)*HRES)+j+1]);
            temp += (PSF[6] * (FLOAT)R[((i+1)*HRES)+j-1]);
            temp += (PSF[7] * (FLOAT)R[((i+1)*HRES)+j]);
            temp += (PSF[8] * (FLOAT)R[((i+1)*HRES)+j+1]);
	    if(temp<0.0) temp=0.0;
	    if(temp>255.0) temp=255.0;
	    convR[(i*HRES)+j]=(unsigned char)temp;

            temp=0;
            temp += (PSF[0] * (FLOAT)G[((i-1)*HRES)+j-1]);
            temp += (PSF[1] * (FLOAT)G[((i-1)*HRES)+j]);
            temp += (PSF[2] * (FLOAT)G[((i-1)*HRES)+j+1]);
            temp += (PSF[3] * (FLOAT)G[((i)*HRES)+j-1]);
            temp += (PSF[4] * (FLOAT)G[((i)*HRES)+j]);
            temp += (PSF[5] * (FLOAT)G[((i)*HRES)+j+1]);
            temp += (PSF[6] * (FLOAT)G[((i+1)*HRES)+j-1]);
            temp += (PSF[7] * (FLOAT)G[((i+1)*HRES)+j]);
            temp += (PSF[8] * (FLOAT)G[((i+1)*HRES)+j+1]);
	    if(temp<0.0) temp=0.0;
	    if(temp>255.0) temp=255.0;
	    convG[(i*HRES)+j]=(unsigned char)temp;

            temp=0;
            temp += (PSF[0] * (FLOAT)B[((i-1)*HRES)+j-1]);
            temp += (PSF[1] * (FLOAT)B[((i-1)*HRES)+j]);
            temp += (PSF[2] * (FLOAT)B[((i-1)*HRES)+j+1]);
            temp += (PSF[3] * (FLOAT)B[((i)*HRES)+j-1]);
            temp += (PSF[4] * (FLOAT)B[((i)*HRES)+j]);
            temp += (PSF[5] * (FLOAT)B[((i)*HRES)+j+1]);
            temp += (PSF[6] * (FLOAT)B[((i+1)*HRES)+j-1]);
            temp += (PSF[7] * (FLOAT)B[((i+1)*HRES)+j]);
            temp += (PSF[8] * (FLOAT)B[((i+1)*HRES)+j+1]);
	  if(temp<0.0) temp=0.0;
	    if(temp>255.0) temp=255.0;
	    convB[(i*HRES)+j]=(unsigned char)temp;
        }
    }

    for(i=0,k=0; k<size; i++,(k=k+3))
    {
        *((unsigned char *)(p+k))=convR[i];
        *((unsigned char *)(p+k+1))=convG[i];
        *((unsigned char *)(p+k+2))=convB[i];
    }

}

static void process_image(const void *p, int size)
{
    int i, k, newsize=0;
    unsigned char *pptr = (unsigned char *)p;

    // record when process was called
    clock_gettime(CLOCK_REALTIME, &frame_time);    

    framecnt++;

    // This just dumps the frame to a file now, but you could replace with whatever image
    // processing you wish.
    //

    if(fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_GREY)
    {
        printf("Dump graymap as-is size %d\n", size);
        dump_pgm(p, size, framecnt, &frame_time);
    }

    else if(fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_YUYV)
    {

       int i,j,k;
    int y_temp, y2_temp, u_temp, v_temp;
    unsigned char *pptr = (unsigned char *)p;
    for(i=0, k=0; i<size; i=i+4, k=k+6)
    {
        y_temp=(int)pptr[i]; u_temp=(int)pptr[i+1]; y2_temp=(int)pptr[i+2]; v_temp=(int)pptr[i+3];
        yuv2rgb(y_temp, u_temp, v_temp, &bigbuffer[k], &bigbuffer[k+1], &bigbuffer[k+2]);
        yuv2rgb(y2_temp, u_temp, v_temp, &bigbuffer[k+3], &bigbuffer[k+4], &bigbuffer[k+5]);
    }
   // sharpening(bigbuffer,((size*6)/4));
    dump_ppm(bigbuffer, ((size*6)/4), framecnt, &frame_time);
    }

    else if(fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_RGB24)
    {
        printf("Dump RGB as-is size %d\n", size);
        dump_ppm(p, size, framecnt, &frame_time);
    }
    else
    {
        printf("ERROR - unknown dump format\n");
    }

    fflush(stderr);
    fflush(stdout);
}

//Each frame is read
static int read_frame(void)
{
    struct v4l2_buffer buf;
    unsigned int i;

    switch (io)
    {

        case IO_METHOD_READ:
            if (-1 == read(fd, buffers[0].start, buffers[0].length))
            {
                switch (errno)
                {

                    case EAGAIN:
                        return 0;

                    case EIO:
                        /* Could ignore EIO, see spec. */

                        /* fall through */

                    default:
                        errno_exit("read");
                }
            }

            process_image(buffers[0].start, buffers[0].length);
            break;

        case IO_METHOD_MMAP:
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
            break;

        case IO_METHOD_USERPTR:
            CLEAR(buf);

            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_USERPTR;

            if (-1 == xioctl(fd, VIDIOC_DQBUF, &buf))
            {
                switch (errno)
                {
                    case EAGAIN:
                        return 0;

                    case EIO:
                        /* Could ignore EIO, see spec. */

                        /* fall through */

                    default:
                        errno_exit("VIDIOC_DQBUF");
                }
            }

            for (i = 0; i < n_buffers; ++i)
                    if (buf.m.userptr == (unsigned long)buffers[i].start
                        && buf.length == buffers[i].length)
                            break;

            assert(i < n_buffers);

            process_image((void *)buf.m.userptr, buf.bytesused);

            if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
                    errno_exit("VIDIOC_QBUF");
            break;
    }
    return 1;
}


static void mainloop(void)
{
    unsigned int count;
    float rate = 0;
    struct timespec read_delay;
    struct timespec time_error;
	double worst_exec=0;
    int frame_number = 0;

    read_delay.tv_sec=1;
    read_delay.tv_nsec=0;
	
	rate = RATE;

    count = frame_count;
    while (count > 0)
    {
        for (;;)
        {
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
                if (EINTR == errno)
                    continue;
                errno_exit("select");
            }

            if (0 == r)
            {
                fprintf(stderr, "select timeout\n");
                exit(EXIT_FAILURE);
            }
			clock_gettime(CLOCK_REALTIME,&img_start_time);
            if (read_frame())
            {
                if(nanosleep(&read_delay, &time_error) != 0)
                    perror("nanosleep");
                    frame_number++;
                count--;
                break;
            }

            /* EAGAIN - continue select loop unless count done. */
            if(count <= 0) break;
        }
		clock_gettime(CLOCK_REALTIME, &img_stop_time);
        //time difference between each frame read is calculated
		delta_t(&img_stop_time, &img_start_time, &img);
		exec_time = ((double)img.tv_sec + (double)(((double)(img.tv_nsec))/((double)1000000000)));
        if(flag == 0)
		{
            worst_exec += exec_time;
            if(exec_time_max < exec_time)
            {
                exec_time_max = exec_time;            
            }
            deadline = (((double)worst_exec)/((double)frame_count))*rate;
        }
        else
        {
            //jitter is defined as the difference between the execution time and the dead like
            jitter = exec_time - deadline;
            if(jitter > 0)
            {
                printf("Jitter is positive for frame %d\n",frame_number);
            }
            total_jitter += jitter;
        }
        if(count <= 0) break;
    }
    if(flag == 1)
    {
        printf("Time elapsed = %ld, nanoseconds = %ld\n", img.tv_sec, img.tv_nsec);
        printf("Total Jitter = %lf\n",total_jitter);
        printf("Deadline = %lf\n",deadline);
        printf("Worst execution time = %lf\n",worst_exec);
    }
}


//Stop frames from being read
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

//Start reading from video0 device
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
                        printf("allocated buffer %d\n", i);
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


//initialise reading from a video0 dev
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


//Initialise video0 device
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
        fmt.fmt.pix.width       = HRES;
        fmt.fmt.pix.height      = VRES;

        // Specify the Pixel Coding Formate here

        // This one work for Logitech C200
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;

        //fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_UYVY;
        //fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_VYUY;

        // Would be nice if camera supported
        //fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_GREY;
        //fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB24;

        //fmt.fmt.pix.field       = V4L2_FIELD_INTERLACED;
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
int main(int argc, char **argv)
{
	int rc;
	pthread_t imagethread[1];
	threadParams_t threadParams[1];
	pthread_attr_t rt_sched_attr[1];
	int rt_max_prio, rt_min_prio;
	struct sched_param rt_param[1];
	struct sched_param main_param;
	pthread_attr_t main_attr;
    dev_name = "/dev/video0";
    HRES = atoi(argv[1]);
    VRES = atoi(argv[2]);
    printf("Resolution requested is:\nHorizontal = %d\nVertical = %d\n",HRES,VRES);
    strncpy(hres_string,argv[1],sizeof(hres_string));
    strncpy(vres_string,argv[2],sizeof(vres_string));
    sprintf(ppm_header,"P6\n#9999999999 sec 9999999999 msec \n%s %s\n255\n",hres_string,vres_string);
    sprintf(pgm_header,"P5\n#9999999999 sec 9999999999 msec \n%s %s\n255\n",hres_string,vres_string);
    #ifdef SHARPEN
		printf("Image transformation selected is:\nSHARPENING\n");
    #endif
    #ifdef BW
		printf("Image transformation selected is:\nBLACK AND WHITE\n");
    #endif
    #ifdef GRAY
		printf("Image transformation selected is:\nGRAYSCALE\n");
    #endif
	
	printf("Before adjustments to scheduling policy:\n");
	print_scheduler();

	pthread_attr_init(&main_sched_attr);
	pthread_attr_setinheritsched(&main_sched_attr, PTHREAD_EXPLICIT_SCHED);
	pthread_attr_setschedpolicy(&main_sched_attr, SCHED_FIFO);

	rt_max_prio = sched_get_priority_max(SCHED_FIFO);
	rt_min_prio = sched_get_priority_min(SCHED_FIFO);

	main_param.sched_priority = rt_max_prio;
	rc=sched_setscheduler(getpid(), SCHED_FIFO, &main_param);


	if (rc)
	{
	   printf("ERROR; sched_setscheduler rc is %d\n", rc);
	   perror("sched_setschduler"); exit(-1);
	}

	printf("After adjustments to scheduling policy:\n");
	print_scheduler();

	pthread_attr_setschedparam(&main_sched_attr, &main_param);
    rc=pthread_attr_init(&rt_sched_attr[0]);
    rc=pthread_attr_setinheritsched(&rt_sched_attr[0], PTHREAD_EXPLICIT_SCHED);
    rc=pthread_attr_setschedpolicy(&rt_sched_attr[0], SCHED_FIFO);

    rt_param[0].sched_priority=rt_max_prio;
    pthread_attr_setschedparam(&rt_sched_attr[0], &rt_param[0]);

    threadParams[0].threadIdx=0;
	
	for (;;)
    {
        int idx;
        int c;

        c = getopt_long(argc, argv,
                    short_options, long_options, &idx);

        if (-1 == c)
            break;

        switch (c)
        {
            case 0: /* getopt_long() flag */
                break;

            case 'd':
                dev_name = optarg;
                break;

            case 'h':
                usage(stdout, argc, argv);
                exit(EXIT_SUCCESS);

            case 'm':
                io = IO_METHOD_MMAP;
                break;

            case 'r':
                io = IO_METHOD_READ;
                break;

            case 'u':
                io = IO_METHOD_USERPTR;
                break;

            case 'o':
                out_buf++;
                break;

            case 'f':
                force_format++;
                break;

            case 'c':
                errno = 0;
                frame_count = strtol(optarg, NULL, 0);
                if (errno)
                        errno_exit(optarg);
                break;

            default:
                usage(stderr, argc, argv);
                exit(EXIT_FAILURE);
        }
    }

    open_device();
    init_device();
    start_capturing();


    rc=pthread_create(&imagethread[0], &rt_sched_attr[0],         
					//(void *)0,               // default attributes
					ThreadTransform, (void *)&(threadParams[0])
					);

    //Join to wait for completion of thread execution
    pthread_join(imagethread[0], NULL);

    stop_capturing();
    uninit_device();
    close_device();
    fprintf(stderr, "\n");
    return 0;
}
