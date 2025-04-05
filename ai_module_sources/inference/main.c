#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <chrono>
#include <functional>
#include <map>
#include <sstream>
#include <string>
#include <vector>
#include <signal.h>
#include "core/cvi_tdl_types_mem_internal.h"
#include "core/utils/vpss_helper.h"
#include "cvi_tdl.h"
#include "cvi_tdl_media.h"
#include <string>
#include <iostream>
#include <string>
#include <unistd.h>
#include <netinet/in.h>
#include <cstring>
#include <unistd.h>
#include <arpa/inet.h>
#include <netdb.h>

#include "MJPEGWriter.h"
#include "config.h"

#define BUFFER_SIZE 1024
int c_x = 0, c_y = 0;

volatile uint8_t interrupted = 0;

void interrupt_handler(int signum)
{
    printf("Signal: %d\n", signum);
    interrupted = 1;
}

CVI_S32 init_param(const cvitdl_handle_t tdl_handle)
{
    // setup preprocess
    YoloPreParam preprocess_cfg =
        CVI_TDL_Get_YOLO_Preparam(tdl_handle, CVI_TDL_SUPPORTED_MODEL_YOLOV8_DETECTION);

    for (int i = 0; i < 3; i++)
    {
        printf("asign val %d \n", i);
        preprocess_cfg.factor[i] = MODEL_SCALE;
        preprocess_cfg.mean[i] = MODEL_MEAN;
    }
    preprocess_cfg.format = PIXEL_FORMAT_RGB_888_PLANAR;

    printf("setup yolov8 param \n");
    CVI_S32 ret = CVI_TDL_Set_YOLO_Preparam(tdl_handle, CVI_TDL_SUPPORTED_MODEL_YOLOV8_DETECTION,
                                            preprocess_cfg);
    if (ret != CVI_SUCCESS)
    {
        printf("Can not set yolov8 preprocess parameters %#x\n", ret);
        return ret;
    }

    YoloAlgParam yolov8_param =
        CVI_TDL_Get_YOLO_Algparam(tdl_handle, CVI_TDL_SUPPORTED_MODEL_YOLOV8_DETECTION);
    yolov8_param.cls = MODEL_CLASS_CNT;

    printf("setup yolov8 algorithm param \n");
    ret =
        CVI_TDL_Set_YOLO_Algparam(tdl_handle, CVI_TDL_SUPPORTED_MODEL_YOLOV8_DETECTION, yolov8_param);
    if (ret != CVI_SUCCESS)
    {
        printf("Can not set yolov8 algorithm parameters %#x\n", ret);
        return ret;
    }

    // set theshold
    CVI_TDL_SetModelThreshold(tdl_handle, CVI_TDL_SUPPORTED_MODEL_YOLOV8_DETECTION, MODEL_THRESH);
    CVI_TDL_SetModelNmsThreshold(tdl_handle, CVI_TDL_SUPPORTED_MODEL_YOLOV8_DETECTION, MODEL_NMS_THRESH);

    printf("yolov8 algorithm parameters setup success!\n");
    return ret;
}

int main(int argc, char *argv[])
{

    signal(SIGINT, interrupt_handler);
    MJPEGWriter test(7777);

    cv::VideoCapture cap;
    cv::Mat bgr;

    cap.open(0);
    cap >> bgr;

    test.write(bgr);
    test.start();

    CVI_S32 ret;

    cvitdl_handle_t tdl_handle = NULL;
    ret = CVI_TDL_CreateHandle(&tdl_handle);
    if (ret != CVI_SUCCESS)
    {
        printf("Create tdl handle failed with %#x!\n", ret);
        return ret;
    }

    ret = init_param(tdl_handle);
    ret = CVI_TDL_OpenModel(tdl_handle, CVI_TDL_SUPPORTED_MODEL_YOLOV8_DETECTION, argv[1]);

    if (ret != CVI_SUCCESS)
    {
        printf("open model failed with %#x!\n", ret);
        return ret;
    }
    // long long counter = 0;

    while (!interrupted)
    {
        std::pair<void *, void *> imagePtrs = cap.capture(bgr);
        void *image_ptr = imagePtrs.first;

        VIDEO_FRAME_INFO_S *frameInfo = reinterpret_cast<VIDEO_FRAME_INFO_S *>(image_ptr);

        cvtdl_object_t obj_meta = {0};
        cv::Point2f coords[4];
        CVI_TDL_YOLOV8_Detection(tdl_handle, frameInfo, &obj_meta);
        cap.releaseImagePtr();
        image_ptr = nullptr;
        std::string json_data = "";

        for (uint32_t i = 0; i < obj_meta.size; i++)
        {
            cv::Rect r = cv::Rect(obj_meta.info[i].bbox.x1, obj_meta.info[i].bbox.y1, obj_meta.info[i].bbox.x2 - obj_meta.info[i].bbox.x1, obj_meta.info[i].bbox.y2 - obj_meta.info[i].bbox.y1);
            c_x = obj_meta.info[i].bbox.x1 + (obj_meta.info[i].bbox.x2 - obj_meta.info[i].bbox.x1) / 2;
            c_y = obj_meta.info[i].bbox.y1 + (obj_meta.info[i].bbox.y2 - obj_meta.info[i].bbox.y1) / 2;
            cv::circle(bgr, cv::Point(c_x, c_y), 5, cv::Scalar(0, 0, 255), -1);
            cv::rectangle(bgr, r, color_map[obj_meta.info[i].classes], 1, 8, 0);
            json_data += std::to_string(obj_meta.info[i].classes) + ',' + std::to_string(c_x) + ',' + std::to_string(c_y) + ';';
        }
        cv::flip(bgr, bgr, -1);

        std::string host = "10.160.209.100";
        int port = 5000;
        std::string path = "/data";

        std::string request =
            "POST " + path + " HTTP/1.1\r\n" +
            "Host: " + host + "\r\n" +
            "Content-Type: application/json\r\n" +
            "Content-Length: " + std::to_string(json_data.length()) + "\r\n" +
            "Connection: close\r\n\r\n" +
            json_data;

        hostent *server = gethostbyname(host.c_str());
        if (!server)
        {
            perror("Host error");
        }

        int sockfd = socket(AF_INET, SOCK_STREAM, 0);
        if (sockfd < 0)
        {
            perror("Socket error");
            return 1;
        }

        sockaddr_in serv_addr{};
        serv_addr.sin_family = AF_INET;
        serv_addr.sin_port = htons(port);
        std::memcpy(&serv_addr.sin_addr.s_addr, server->h_addr, server->h_length);

        if (connect(sockfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
        {
        }
        else
        {
            send(sockfd, request.c_str(), request.length(), 0);

            char buffer[2048];
            int bytes = recv(sockfd, buffer, sizeof(buffer) - 1, 0);
            if (bytes > 0)
            {
                buffer[bytes] = '\0';
            }

            close(sockfd);
        }

        test.write(bgr);
        bgr.release();
    }

    printf("Stopping stream...\n");
    cap.releaseImagePtr();
    test.stop();
    cap.release();

    CVI_TDL_DestroyHandle(tdl_handle);

    return ret;
}