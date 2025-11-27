//test_data_capture.c
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/videodev2.h>

#define DEVICE "/dev/video0"
#define WIDTH 640
#define HEIGHT 480
#define FORMAT V4L2_PIX_FMT_YUYV  // CIF常用格式

// 缓冲区结构
struct buffer {
    void *start;
    size_t length;
};

int main() {
    int fd;
    struct v4l2_format fmt;
    struct v4l2_requestbuffers req;
    struct v4l2_buffer buf;
    struct buffer *buffers;
    unsigned int i, n_buffers;
    
    // 1. 打开设备
    fd = open(DEVICE, O_RDWR);
    if (fd == -1) {
        perror("打开设备失败");
        return -1;
    }
    
    // 2. 设置视频格式
    memset(&fmt, 0, sizeof(fmt));
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = WIDTH;
    fmt.fmt.pix.height = HEIGHT;
    fmt.fmt.pix.pixelformat = FORMAT;
    fmt.fmt.pix.field = V4L2_FIELD_NONE;
    
    if (ioctl(fd, VIDIOC_S_FMT, &fmt) == -1) {
        perror("设置格式失败");
        close(fd);
        return -1;
    }
    
    // 3. 申请缓冲区
    memset(&req, 0, sizeof(req));
    req.count = 4;  // 4个缓冲区
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;  // 内存映射方式
    
    if (ioctl(fd, VIDIOC_REQBUFS, &req) == -1) {
        perror("申请缓冲区失败");
        close(fd);
        return -1;
    }
    
    // 4. 内存映射缓冲区
    buffers = calloc(req.count, sizeof(*buffers));
    
    for (n_buffers = 0; n_buffers < req.count; ++n_buffers) {
        memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = n_buffers;
        
        if (ioctl(fd, VIDIOC_QUERYBUF, &buf) == -1) {
            perror("查询缓冲区失败");
            close(fd);
            return -1;
        }
        
        buffers[n_buffers].length = buf.length;
        buffers[n_buffers].start = mmap(NULL, buf.length, 
                                       PROT_READ | PROT_WRITE, 
                                       MAP_SHARED, fd, buf.m.offset);
        
        if (buffers[n_buffers].start == MAP_FAILED) {
            perror("内存映射失败");
            close(fd);
            return -1;
        }
    }
    
    // 5. 将缓冲区加入队列
    for (i = 0; i < n_buffers; ++i) {
        memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;
        
        if (ioctl(fd, VIDIOC_QBUF, &buf) == -1) {
            perror("缓冲区入队失败");
            close(fd);
            return -1;
        }
    }
    
    // 6. 开始采集
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd, VIDIOC_STREAMON, &type) == -1) {
        perror("开始采集失败");
        close(fd);
        return -1;
    }
    
    // 7. 读取图像数据
    for (i = 0; i < 10; ++i) {  // 读取10帧
        memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        
        // 等待一帧数据
        if (ioctl(fd, VIDIOC_DQBUF, &buf) == -1) {
            perror("获取缓冲区失败");
            break;
        }
        
        // 处理图像数据
        printf("获取到第%d帧: 大小=%d, 序号=%d\n", 
               i, buf.bytesused, buf.sequence);
        
        // 保存为文件（示例：保存为raw数据）
        char filename[32];
        snprintf(filename, sizeof(filename), "frame_%d.raw", i);
        FILE *fp = fopen(filename, "wb");
        fwrite(buffers[buf.index].start, buf.bytesused, 1, fp);
        fclose(fp);
        
        // 将缓冲区重新加入队列
        if (ioctl(fd, VIDIOC_QBUF, &buf) == -1) {
            perror("重新入队失败");
            break;
        }
    }
    
    // 8. 停止采集
    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd, VIDIOC_STREAMOFF, &type) == -1) {
        perror("停止采集失败");
    }
    
    // 9. 清理资源
    for (i = 0; i < n_buffers; ++i) {
        munmap(buffers[i].start, buffers[i].length);
    }
    free(buffers);
    close(fd);
    
    return 0;
}