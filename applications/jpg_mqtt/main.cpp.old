#include <libpynq.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <jpeglib.h>
#include <stdlib.h>
#include <string.h>

#define CAMERA_DEVICE "/dev/video0"
#define FRAME_WIDTH 640
#define FRAME_HEIGHT 480
#define BUFFER_COUNT 4

struct buffer {
    void *start;
    size_t length;
};

static struct buffer *buffers = NULL;
static int camera_fd = -1;

void init_camera() {
    struct v4l2_format fmt = {0};
    struct v4l2_requestbuffers req = {0};
    
    // Open camera device
    camera_fd = open(CAMERA_DEVICE, O_RDWR);
    if (camera_fd == -1) {
        perror("Cannot open camera device");
        exit(EXIT_FAILURE);
    }

    // Set format
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = FRAME_WIDTH;
    fmt.fmt.pix.height = FRAME_HEIGHT;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
    
    if (ioctl(camera_fd, VIDIOC_S_FMT, &fmt) == -1) {
        perror("Cannot set format");
        exit(EXIT_FAILURE);
    }

    // Request buffers
    req.count = BUFFER_COUNT;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    
    if (ioctl(camera_fd, VIDIOC_REQBUFS, &req) == -1) {
        perror("Cannot request buffers");
        exit(EXIT_FAILURE);
    }

    // Map buffers
    buffers = calloc(req.count, sizeof(*buffers));
    for (size_t i = 0; i < req.count; ++i) {
        struct v4l2_buffer buf = {0};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;

        if (ioctl(camera_fd, VIDIOC_QUERYBUF, &buf) == -1) {
            perror("Cannot query buffer");
            exit(EXIT_FAILURE);
        }

        buffers[i].length = buf.length;
        buffers[i].start = mmap(NULL, buf.length, PROT_READ | PROT_WRITE,
                               MAP_SHARED, camera_fd, buf.m.offset);
        
        if (buffers[i].start == MAP_FAILED) {
            perror("Cannot mmap buffer");
            exit(EXIT_FAILURE);
        }
    }

    // Start capturing
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(camera_fd, VIDIOC_STREAMON, &type) == -1) {
        perror("Cannot start streaming");
        exit(EXIT_FAILURE);
    }
}

void compress_and_send_frame(const unsigned char *frame_data, size_t frame_size) {
    struct jpeg_compress_struct cinfo;
    struct jpeg_error_mgr jerr;
    unsigned char *outbuffer = NULL;
    unsigned long outsize = 0;

    // Initialize JPEG compression
    cinfo.err = jpeg_std_error(&jerr);
    jpeg_create_compress(&cinfo);
    jpeg_mem_dest(&cinfo, &outbuffer, &outsize);

    // Set JPEG parameters
    cinfo.image_width = FRAME_WIDTH;
    cinfo.image_height = FRAME_HEIGHT;
    cinfo.input_components = 3;
    cinfo.in_color_space = JCS_RGB;

    jpeg_set_defaults(&cinfo);
    jpeg_set_quality(&cinfo, 80, TRUE);
    jpeg_start_compress(&cinfo, TRUE);

    // Convert YUYV to RGB and compress
    unsigned char *rgb_line = malloc(FRAME_WIDTH * 3);
    JSAMPROW row_pointer[1];
    
    while (cinfo.next_scanline < cinfo.image_height) {
        // Convert one line of YUYV to RGB
        for (int i = 0; i < FRAME_WIDTH; i++) {
            int j = (cinfo.next_scanline * FRAME_WIDTH + i) * 2;
            int y = frame_data[j];
            int u = frame_data[j & ~1 + 1];
            int v = frame_data[j | 1 + 1];

            // YUYV to RGB conversion
            int c = y - 16;
            int d = u - 128;
            int e = v - 128;
            
            rgb_line[i * 3 + 0] = (unsigned char)((298 * c + 409 * e + 128) >> 8);
            rgb_line[i * 3 + 1] = (unsigned char)((298 * c - 100 * d - 208 * e + 128) >> 8);
            rgb_line[i * 3 + 2] = (unsigned char)((298 * c + 516 * d + 128) >> 8);
        }
        
        row_pointer[0] = rgb_line;
        jpeg_write_scanlines(&cinfo, row_pointer, 1);
    }

    jpeg_finish_compress(&cinfo);
    free(rgb_line);

    // Send the compressed JPEG data over UART
    uint32_t length = outsize;
    uint8_t* len_bytes = (uint8_t*)&length;
    
    // Send length first
    for(uint32_t i = 0; i < 4; i++) {
        uart_send(UART0, len_bytes[i]);
    }
    
    // Send JPEG data
    for(uint32_t i = 0; i < length; i++) {
        uart_send(UART0, outbuffer[i]);
    }

    // Clean up
    free(outbuffer);
    jpeg_destroy_compress(&cinfo);
}

int main() {
    // Initialize PYNQ
    switchbox_init();
    switchbox_set_pin(IO_AR0, SWB_UART0_RX);
    switchbox_set_pin(IO_AR1, SWB_UART0_TX);
    uart_init(UART0);
    uart_reset_fifos(UART0);

    // Initialize camera
    init_camera();

    while (1) {
        struct v4l2_buffer buf = {0};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;

        // Queue buffer
        if (ioctl(camera_fd, VIDIOC_QBUF, &buf) == -1) {
            perror("Cannot queue buffer");
            break;
        }

        // Dequeue buffer
        if (ioctl(camera_fd, VIDIOC_DQBUF, &buf) == -1) {
            perror("Cannot dequeue buffer");
            break;
        }

        // Process and send frame
        compress_and_send_frame(buffers[buf.index].start, buf.bytesused);
        
        // Small delay to control frame rate
        usleep(33333); // ~30 fps
    }

    // Cleanup
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ioctl(camera_fd, VIDIOC_STREAMOFF, &type);
    
    for (int i = 0; i < BUFFER_COUNT; i++) {
        munmap(buffers[i].start, buffers[i].length);
    }
    
    free(buffers);
    close(camera_fd);
    uart_reset_fifos(UART0);
    uart_destroy(UART0);
    pynq_destroy();
    
    return EXIT_SUCCESS;
}