/*
 * 网球识别 + 机器人抓取联动程序
 *
 * 通过 cviruntime TPU 推理检测网球，根据检测框位置控制底盘与机械臂:
 *   - 网球偏左 → 左转对准
 *   - 网球偏右 → 右转对准
 *   - 网球居中且较远 → 前进靠近
 *   - 网球居中且足够近 → 停车 + 机械臂抓取
 *
 * 依赖: tennis.cpp (TPU YOLO 检测)
 * 编译: riscv64-unknown-linux-musl-g++ -O2 -o test-roboctl test-roboctl.c tennis.cpp \
 *       -lopencv_core -lopencv_imgcodecs -lopencv_imgproc -lcviruntime
 */

#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

/* ─── 摄像头 IOCTL ─────────────────────────────────────────────────────────── */
#define CVI_CAM_INIT      1u
#define CVI_CAM_GET_INFO  2u
#define CVI_CAM_GET_FRAME 3u
#define FRAME_BUF_SIZE    (2u * 1024u * 1024u)

/* ─── 机器人 IOCTL ─────────────────────────────────────────────────────────── */
#define ROBO_INIT        1
#define ROBO_TURN_LEFT   2
#define ROBO_TURN_RIGHT  3
#define ROBO_FORWARD     4
#define ROBO_BACKWARD    5
#define ROBO_GRAB        6
#define ROBO_RELEASE     7
#define ROBO_STOP        8

/* ─── tennis.h (TPU YOLO 检测接口) ─────────────────────────────────────────── */
#define MAX_DETECTIONS 10

typedef struct {
    float cx, cy;
    float w, h;
    float x1, y1, x2, y2;
    float score;
} TennisBox;

typedef struct {
    int image_width;
    int image_height;
    int count;
    TennisBox boxes[MAX_DETECTIONS];
} TennisResult;

#ifdef __cplusplus
extern "C" {
#endif
int  initTennisDetector(const char *model_path);
void deinitTennisDetector(void);
int  detectTennis(const char *jpeg_data, int len, TennisResult *result);
void printTennisResult(const TennisResult *result);
#ifdef __cplusplus
}
#endif

/* ─── 视觉伺服参数 ─────────────────────────────────────────────────────────── */

/*
 * CENTER_BAND: 画面中心带宽度占比（0.0~1.0），网球中心落在此范围内视为"居中"。
 * GRAB_RATIO:  网球 bbox 面积占画面面积比例阈值，大于此值视为"足够近"可以抓取。
 * TURN_MS:     单次转向持续时间 (ms)。
 * FORWARD_MS:  单次前进持续时间 (ms)。
 */
#define CENTER_BAND   0.30f
#define GRAB_RATIO    0.08f
#define TURN_MS       300
#define FORWARD_MS    500

struct camera_info {
    uint16_t width;
    uint16_t height;
    uint8_t  format;
    uint8_t  connected;
} __attribute__((packed));

static unsigned char frame_buf[FRAME_BUF_SIZE];

static void die(const char *msg)
{
    perror(msg);
    exit(1);
}

static void msleep(unsigned ms)
{
    usleep(ms * 1000u);
}

/*
 * 从检测结果中选出置信度最高的网球。
 * 返回指向最佳检测框的指针，无结果返回 NULL。
 */
static const TennisBox *best_detection(const TennisResult *r)
{
    if (r->count <= 0)
        return NULL;
    const TennisBox *best = &r->boxes[0];
    for (int i = 1; i < r->count; i++) {
        if (r->boxes[i].score > best->score)
            best = &r->boxes[i];
    }
    return best;
}

int main(int argc, char **argv)
{
    const char *model_path = (argc > 1) ? argv[1] : "/root/tennis.cvimodel";
    const char *cam_dev    = (argc > 2) ? argv[2] : "/dev/cvi-camera";
    const char *robo_dev   = (argc > 3) ? argv[3] : "/dev/robo-ctl";

    /* ── 初始化 TPU 检测器 ── */
    printf("[init] 加载模型: %s\n", model_path);
    if (initTennisDetector(model_path) != 0)
        die("initTennisDetector");

    /* ── 打开设备 ── */
    int fd_cam = open(cam_dev, O_RDWR);
    if (fd_cam < 0) die("open camera");
    int fd_robo = open(robo_dev, O_RDWR);
    if (fd_robo < 0) die("open robo-ctl");

    /* ── 初始化摄像头 ── */
    printf("[init] 初始化摄像头...\n");
    if (ioctl(fd_cam, CVI_CAM_INIT, 0UL) < 0)
        die("camera INIT");

    struct camera_info cam_info;
    memset(&cam_info, 0, sizeof(cam_info));
    if (ioctl(fd_cam, CVI_CAM_GET_INFO, (unsigned long)&cam_info) < 0)
        die("camera GET_INFO");
    printf("[init] 摄像头 %ux%u  format=%u\n",
           cam_info.width, cam_info.height, cam_info.format);

    /* ── 初始化机器人 ── */
    printf("[init] 初始化机械臂与电机...\n");
    if (ioctl(fd_robo, ROBO_INIT, 0UL) < 0)
        die("robo INIT");
    msleep(500);

    printf("\n=== 开始连续检测网球 ===\n\n");

    int frame_no = 0;

    while (1) {
        frame_no++;

        /* ── 1. 抓帧 ── */
        long nbytes = ioctl(fd_cam, CVI_CAM_GET_FRAME, (unsigned long)frame_buf);
        if (nbytes <= 0) {
            printf("[cam] 第 %d 帧: 获取失败, 重试...\n", frame_no);
            msleep(300);
            continue;
        }

        /* ── 2. TPU 推理检测 ── */
        TennisResult result;
        if (detectTennis((const char *)frame_buf, (int)nbytes, &result) != 0) {
            printf("[det] 第 %d 帧: 推理失败\n", frame_no);
            msleep(200);
            continue;
        }

        const TennisBox *ball = best_detection(&result);
        if (!ball) {
            printf("[det] 第 %d 帧: 未发现网球\n", frame_no);
            msleep(100);
            continue;
        }

        float img_w = (float)result.image_width;
        float img_h = (float)result.image_height;
        float img_area = img_w * img_h;
        float ball_area = ball->w * ball->h;
        float cx_norm = ball->cx / img_w;          /* 0.0 左 ~ 1.0 右 */
        float area_ratio = ball_area / img_area;

        printf("[det] 第 %d 帧: 网球 score=%.0f%%  中心=(%.0f,%.0f)  "
               "大小=%.0fx%.0f  面积占比=%.1f%%\n",
               frame_no, ball->score * 100,
               ball->cx, ball->cy, ball->w, ball->h,
               area_ratio * 100);

        /* ── 3. 视觉伺服决策 ── */
        float left_edge  = (1.0f - CENTER_BAND) / 2.0f;
        float right_edge = 1.0f - left_edge;

        if (cx_norm < left_edge) {
            /* 网球偏左 → 左转对准 */
            printf("[act] 网球偏左(%.0f%%), 左转\n", cx_norm * 100);
            ioctl(fd_robo, ROBO_TURN_LEFT, 0UL);
            msleep(TURN_MS);
            ioctl(fd_robo, ROBO_STOP, 0UL);

        } else if (cx_norm > right_edge) {
            /* 网球偏右 → 右转对准 */
            printf("[act] 网球偏右(%.0f%%), 右转\n", cx_norm * 100);
            ioctl(fd_robo, ROBO_TURN_RIGHT, 0UL);
            msleep(TURN_MS);
            ioctl(fd_robo, ROBO_STOP, 0UL);

        } else if (area_ratio < GRAB_RATIO) {
            /* 网球居中但还较远 → 前进靠近 */
            printf("[act] 网球居中但较远(面积%.1f%%), 前进\n", area_ratio * 100);
            ioctl(fd_robo, ROBO_FORWARD, 0UL);
            msleep(FORWARD_MS);
            ioctl(fd_robo, ROBO_STOP, 0UL);

        } else {
            /* 网球居中且足够近 → 抓取 */
            printf("[act] >>> 网球就位, 执行抓取! <<<\n");
            ioctl(fd_robo, ROBO_STOP, 0UL);
            msleep(200);

            ioctl(fd_robo, ROBO_GRAB, 0UL);
            msleep(3000);

            printf("[act] 抓取完成, 后退...\n");
            ioctl(fd_robo, ROBO_BACKWARD, 0UL);
            msleep(1500);
            ioctl(fd_robo, ROBO_STOP, 0UL);

            printf("[act] 释放\n");
            ioctl(fd_robo, ROBO_RELEASE, 0UL);
            msleep(2000);
        }

        msleep(50);
    }

    ioctl(fd_robo, ROBO_STOP, 0UL);
    deinitTennisDetector();
    close(fd_cam);
    close(fd_robo);
    return 0;
}
