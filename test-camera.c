/*
 * 打开摄像头设备：INIT → GET_INFO → GET_FRAME，将帧数据写入 .jpg 文件。
 * 假定模组经协议下发的是 JPEG 压缩流（常见 SOI 为 FF D8）；否则文件可能无法用
 * 看图软件打开，需按 camera_info.format 另行编码。
 *
 * ioctl 与 api/src/vfs/dev/cvi_camera.rs 一致：1=INIT，2=GetInfo，3=GetFrame；
 * GET_FRAME 将帧字节写入 arg 指向缓冲区，返回值（成功时）为帧长度。
 *
 * 用法: test-camera [设备路径] [输出.jpg]
 * 编译: cc -O2 -o test-camera test-camera.c
 */

#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#define CVI_CAM_IOCTL_INIT      1u
#define CVI_CAM_IOCTL_GET_INFO  2u
#define CVI_CAM_IOCTL_GET_FRAME 3u

/* 与内核 MAX_FRAME_SIZE（2MiB）一致，避免截断 */
#define FRAME_BUF_SIZE (2u * 1024u * 1024u)

struct camera_info {
    uint16_t width;
    uint16_t height;
    uint8_t format;
    uint8_t connected;
} __attribute__((packed));

static void die(const char *msg)
{
    perror(msg);
    exit(1);
}

static int is_jpeg_soi(const unsigned char *p, size_t n)
{
    return n >= 2 && p[0] == 0xff && p[1] == 0xd8;
}

int main(int argc, char **argv)
{
    const char *dev_path = (argc > 1) ? argv[1] : "/dev/cvi-camera";
    const char *jpg_path = (argc > 2) ? argv[2] : "capture.jpg";

    int fd = open(dev_path, O_RDWR);
    if (fd < 0)
        die("open");

    if (ioctl(fd, CVI_CAM_IOCTL_INIT, (unsigned long)0) < 0)
        die("ioctl INIT");

    struct camera_info info;
    memset(&info, 0, sizeof(info));
    if (ioctl(fd, CVI_CAM_IOCTL_GET_INFO, (unsigned long)&info) < 0)
        die("ioctl GET_INFO");

    printf("camera_info: width=%u height=%u format=%u connected=%u\n",
           (unsigned)info.width, (unsigned)info.height,
           (unsigned)info.format, (unsigned)info.connected);

    static unsigned char frame_buf[FRAME_BUF_SIZE];

    long nbytes = ioctl(fd, CVI_CAM_IOCTL_GET_FRAME, (unsigned long)frame_buf);
    if (nbytes < 0)
        die("ioctl GET_FRAME");
    if (nbytes == 0) {
        fprintf(stderr, "GET_FRAME: empty frame\n");
        close(fd);
        return 1;
    }
    if ((size_t)nbytes > FRAME_BUF_SIZE) {
        fprintf(stderr, "GET_FRAME: length %ld exceeds buffer\n", nbytes);
        close(fd);
        return 1;
    }

    FILE *out = fopen(jpg_path, "wb");
    if (!out) {
        perror("fopen output");
        close(fd);
        return 1;
    }
    if (fwrite(frame_buf, 1, (size_t)nbytes, out) != (size_t)nbytes) {
        perror("fwrite");
        fclose(out);
        close(fd);
        return 1;
    }
    fclose(out);
    close(fd);

    if (!is_jpeg_soi(frame_buf, (size_t)nbytes))
        fprintf(stderr,
                "提示: 数据头非 JPEG SOI(FF D8)，若模组输出为原始像素，需编码后再存为 "
                "JPG。\n");

    printf("已写入 %ld 字节 -> %s\n", nbytes, jpg_path);
    return 0;
}
