# YOLOv8 实时目标检测部署

本项目演示了如何将经过训练的 YOLOv8 模型部署到 Jetson Nano 开发板上，并通过串口传输识别结果到下位机。以下是整个部署流程的步骤。

### 文件结构

```bash
.
├── infer        # 实现 YOLOv8 模型部署 
├── tools        # 可能有用的工具
└── ultralytics  # YOLOv8 官方库
```

## 运行指南

### 训练和导出模型

首先，需要使用标注好的数据集训练 YOLOv8 模型。这里使用 ultralytics 官方源代码进行训练。

训练完成后，借助 `./起重机资料/code/ultralytics/export.py` 将模型导出成 ONNX 格式

将导出的 ONNX 模型和 infer 代码上传到 Jetson nano 上

### 运行推理

在 Jetson nano 上运行推理引擎以执行实时目标检测。您可以使用以下命令：

```bash
make run
```

这将启动实时目标检测，并将检测到的结果显示在屏幕上。

## 串口传输

将Jetson Nano连接到下位机，并使用串口传输识别结果。您可以使用 `open_port` 和 `set_uart_config` 这两个函数来初始化和配置串口通信。

然后，在实时目标检测代码中添加逻辑，将识别结果通过串口传输给下位机。这需要修改 `infer` 目录中的代码，确保在识别到目标后将结果发送到串口。

### 串口测试程序

```cpp
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <errno.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <iconv.h>
#include <string>
#include <iostream>
 
// 函数声明部分
int open_port(int com_port);
int set_uart_config(int fd, int baud_rate, int data_bits, char parity, int stop_bits);
 
/*
* 打开串口
*/
int open_port(int com_port)
{
    int fd;
    /* 使用普通串口 */
    // TODO::在此处添加串口列表
    char* dev[] = { "/dev/ttyTHS1", "/dev/ttyUSB0" };
 
    //O_NDELAY 同 O_NONBLOCK。
    fd = open(dev[com_port], O_RDWR | O_NOCTTY);
    if (fd < 0)
    {
        perror("open serial port");
        return(-1);
    }
 
    //恢复串口为阻塞状态 
    //非阻塞：fcntl(fd,F_SETFL,FNDELAY)  
    //阻塞：fcntl(fd,F_SETFL,0) 
    if (fcntl(fd, F_SETFL, 0) < 0)
    {
        perror("fcntl F_SETFL\n");
    }
    /*测试是否为终端设备*/
    if (isatty(STDIN_FILENO) == 0)
    {
        perror("standard input is not a terminal device");
    }
 
    return fd;
}
 
/*
* 串口设置
*/
int set_uart_config(int fd, int baud_rate, int data_bits, char parity, int stop_bits)
{
    struct termios opt;
    int speed;
    if (tcgetattr(fd, &opt) != 0)
    {
        perror("tcgetattr");
        return -1;
    }
 
    /*设置波特率*/
    switch (baud_rate)
    {
    case 2400:  speed = B2400;  break;
    case 4800:  speed = B4800;  break;
    case 9600:  speed = B9600;  break;
    case 19200: speed = B19200; break;
    case 38400: speed = B38400; break;
    default:    speed = B115200; break;
    }
    cfsetispeed(&opt, speed);
    cfsetospeed(&opt, speed);
    tcsetattr(fd, TCSANOW, &opt);
 
    opt.c_cflag &= ~CSIZE;
 
    /*设置数据位*/
    switch (data_bits)
    {
    case 7: {opt.c_cflag |= CS7; }break;//7个数据位  
    default: {opt.c_cflag |= CS8; }break;//8个数据位 
    }
 
    /*设置奇偶校验位*/
    switch (parity) //N
    {
    case 'n':case 'N':
    {
        opt.c_cflag &= ~PARENB;//校验位使能     
        opt.c_iflag &= ~INPCK; //奇偶校验使能  
    }break;
    case 'o':case 'O':
    {
        opt.c_cflag |= (PARODD | PARENB);//PARODD使用奇校验而不使用偶校验 
        opt.c_iflag |= INPCK;
    }break;
    case 'e':case 'E':
    {
        opt.c_cflag |= PARENB;
        opt.c_cflag &= ~PARODD;
        opt.c_iflag |= INPCK;
    }break;
    case 's':case 'S': /*as no parity*/
    {
        opt.c_cflag &= ~PARENB;
        opt.c_cflag &= ~CSTOPB;
    }break;
    default:
    {
        opt.c_cflag &= ~PARENB;//校验位使能     
        opt.c_iflag &= ~INPCK; //奇偶校验使能          	
    }break;
    }
 
    /*设置停止位*/
    switch (stop_bits)
    {
    case 1: {opt.c_cflag &= ~CSTOPB; } break;
    case 2: {opt.c_cflag |= CSTOPB; }   break;
    default: {opt.c_cflag &= ~CSTOPB; } break;
    }
 
    /*处理未接收字符*/
    tcflush(fd, TCIFLUSH);
 
    /*设置等待时间和最小接收字符*/
    opt.c_cc[VTIME] = 1000;
    opt.c_cc[VMIN] = 0;
 
    /*关闭串口回显*/
    opt.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHOK | ECHONL | NOFLSH);
 
    /*禁止将输入中的回车翻译为新行 (除非设置了 IGNCR)*/
    opt.c_iflag &= ~ICRNL;
    /*禁止将所有接收的字符裁减为7比特*/
    opt.c_iflag &= ~ISTRIP;
 
    /*激活新配置*/
    if ((tcsetattr(fd, TCSANOW, &opt)) != 0)
    {
        perror("tcsetattr");
        return -1;
    }
 
    return 0;
}

// 使用实例
int main()
{
    // begin::第一步，串口初始化
    int UART_fd = open_port(0);
	if (set_uart_config(UART_fd, 115200, 8, 'N', 1) < 0)
	{
		perror("set_com_config");
		exit(1);
	}
    // end::串口初始化
    
    // begin::第二步，读下位机上发的一行数据
    char str[128];
	char buff[1];
	int len = 0;
	while (1)
	{
		if (read(UART_fd, buff, 1)) {
			if (buff[0] == '\n') {
				break;
			}else {
				str[len++] = buff[0];
			}
		}
	}
    printf("content:%s\n",str);
    // end::读下位机上发的一行数据
 
    // begin::第三步，向下位机发送数据
    write(UART_fd, str, len);
    // end::向下位机发送数据
 
    return 0;
}
```

## 推理代码

以下是 infer/src/main.cpp 的核心函数 yolo_video_demo 分析

```cpp
static void yolo_video_demo(const string &engine_file)
{
  // 加载YOLOv8推理引擎
  auto yolo = yolo::load(engine_file, yolo::Type::V8);
  if (yolo == nullptr)
    return;

  // 打开串口通信并配置
  int UART_fd = open_port(0);
  if (set_uart_config(UART_fd, 115200, 8, 'N', 1) < 0)
  {
    perror("set_com_config");
    exit(1);
  }

  // 初始化数据包
  char data[12];
  data[0] = 'A';
  data[11] = 'D';
  int count = 12;

  // 初始化图像处理所需的Mat对象
  cv::Mat frame;
  cv::Mat gray;
  cv::Mat dilated, eroded;
  cv::Mat gaussianblured;
  cv::Mat thresholded;
  cv::Mat edges;

  // 打开摄像头
  cv::VideoCapture cap(0);
  if (!cap.isOpened())
  {
    printf("Engine is nullptr");
    return;
  }

  // 设置摄像头参数
  cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
  cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
  cap.set(cv::CAP_PROP_FPS, 60);

  int sq_counter = 0;
  float preCenterX = -1.0f;
  float preCenterY = -1.0f;
  float circle_center_x = -1.0f;
  float circle_center_y = -1.0f;

  while (true)
  {
    // 从摄像头读取一帧图像
    cap.read(frame);
    float fps = cap.get(cv::CAP_PROP_FPS);
    cout << fps << endl;

    // 进行目标检测
    auto objs = yolo->forward(cvimg(frame));

    if (objs.empty())
    {
      printf("nothing\n");
      data[0] = 'A';
      data[11] = 'D';
      write(UART_fd, data, count);
    }
    else
    {
      // 在检测到的目标中找到最右边的目标
      auto &right_obj = *std::max_element(objs.begin(), objs.end(), [](const yolo::Box &obj1, const yolo::Box &obj2)
                                          { return obj1.right < obj2.right; });

      // 对检测到的目标进行处理
      for (const auto &obj : objs)
      {
        auto name = mylabels[obj.class_label];
        if (strcmp(name, "box") == 0)
        {
          right_obj = obj;
          break;
        }
      }

      auto right_name = mylabels[right_obj.class_label];

      // 计算目标中心点和摄像头中心点之间的距离
      float camera_center_x = frame.cols / 2;
      float camera_center_y = frame.rows / 2;

      float right_obj_center_x = (right_obj.left + right_obj.right) / 2;
      float right_obj_center_y = (right_obj.top + right_obj.bottom) / 2;

      // 如果检测到的目标是"cola"（可乐）
      if (!strcmp(right_name, "cola"))
      {
        // 扩展目标边界框
        float expansion_factor = 1.4f;
        int box_width = right_obj.right - right_obj.left;
        int box_height = right_obj.bottom - right_obj.top;

        int new_left = std::max(0.0f, right_obj.left - static_cast<int>(box_width * (expansion_factor - 1.0f) / 2.0f));
        int new_top = std::max(0.0f, right_obj.top - static_cast<int>(box_height * (expansion_factor - 1.0f) / 2.0f));
        int new_right = std::min((float)frame.cols, right_obj.right + static_cast<int>(box_width * (expansion_factor - 1.0f) / 2.0f));
        int new_bottom = std::min((float)frame.rows, right_obj.bottom + static_cast<int>(box_height * (expansion_factor - 1.0f) / 2.0f));

        // 提取目标区域的图像
        cv::Rect roi(new_left, new_top, new_right - new_left, new_bottom - new_top);
        cv::Mat object_img = frame(roi);
        cv::cvtColor(object_img, gray, cv::COLOR_BGR2GRAY);

        // 对目标区域进行图像处理
        cv::GaussianBlur(gray, gaussianblured, cv::Size(9, 9), 3, 3);
        cv::dilate(gaussianblured, dilated, cv::Mat(), cv::Point(-1, -1), 2);
        cv::Canny(dilated, edges, 50, 150);

        // 检测圆形物体
        std::vector<cv::Vec3f> circles;
        cv::HoughCircles(edges, circles, cv::HOUGH_GRADIENT, 1, edges.rows / 4, 200, 30, edges.rows / 8, 0);

        if (!circles.empty())
        {
          cout << "detect" << endl;

          // 绘制检测到的圆形物体
          cv::Vec3i c = circles[0];
          right_obj_center_x = c[0] + new_left;
          right_obj_center_y = c[1] + new_top;
          cv::circle(frame, cv::Point(right_obj_center_x, right_obj_center_y), c[2], cv::Scalar(0, 0, 255), 3, cv::LINE_AA);
          cv::circle(frame, cv::Point(right_obj_center_x, right_obj_center_y), 2, cv::Scalar(0, 0, 255), -1);
        }
        imshow("edges", edges);
      }
      else if (!strcmp(right_name, "box"))
      {
        cv::circle(frame, cv::Point(right_obj_center_x, right_obj_center_y), 5, cv::Scalar(0, 0, 255), -1);
      }

      // 计算目标在X和Y方向上的相对偏移
      float xx = (camera_center_x - right_obj_center_x - BASE_X) / 8.0f * 0.01f;
      float mean_squared = -1.0f;
      float yy = -1.0f;

      // 如果目标是"cola"
      if (!strcmp(right_name, "cola"))
      {
        // 计算平均平方误差
        cv::circle(frame, cv::Point(camera_center_x - BASE_X, camera_center_y - BASE_Y_COLA), RADIUS_COLA, cv::Scalar(0, 0, 255));
        mean_squared = pow(right_obj_center_x + BASE_X - camera_center_x, 2) + pow(right_obj_center_y + BASE_Y_COLA - camera_center_y, 2);
        yy = (camera_center_y - right_obj_center_y - BASE_Y_COLA) * 0.01f;
      }
      // 如果目标是"box"
      else if (!strcmp(right_name, "box"))
      {
        cv::circle(frame, cv::Point(camera_center_x - BASE_X, camera_center_y - BASE_Y_BOX), RADIUS_BOX, cv::Scalar(0, 0, 255));
        mean_squared = pow(right_obj_center_x + BASE_X - camera_center_x, 2) + pow(right_obj_center_y + BASE_Y_BOX - camera_center_y, 2);
        yy = (camera_center_y - right_obj_center_y - BASE_Y_BOX) * 0.01f;
      }

      // 计算平方根误差
      float sq = sqrt(mean_squared);

      // 根据条件判断是否执行动作
      if (!strcmp(right_name, "cola") && sq < RADIUS_COLA)
      {
        sq_counter++;
        if (sq_counter >= 2)
        {
          // 执行动作，向下位机发送数据
          data[1] = '1';
          sq_counter = 0;
        }
      }
      else if (!strcmp(right_name, "box") && sq < RADIUS_BOX)
      {
        sq_counter++;
        if (sq_counter >= 5)
        {
          // 执行动作，向下位机发送数据
          data[1] = '1';
          sq_counter = 0;
        }
      }
      else
      {
        data[1] = '0';
        sq_counter = 0;
      }
      if (!strcmp(right_name, "cola"))
      {
        printf("%s\n", right_name);
        data[2] = '1';
      }
      else if (!strcmp(right_name, "box"))
      {
        printf("%s\n", right_name);
        data[2] = '0';
      }

      // 计算并传输X和Y方向的值
      float value_x = -xx / 0.6;
      value_x = atan(value_x);

      float value_y = yy;
      if (value_y > 0.4f)
        value_y = 1.5f;
      else if (value_y < -0.4f)
        value_y = -1.5f;
      else if (0.2f < value_y < 0.4f)
        value_y *= 2;
      else if (-0.4f < value_y < -0.2f)
        value_y *= 2;

      cout << sq << " " << data[1] << " " << data[2] << " " << value_x << " " << value_y << endl;

      // 将X和Y方向的值存储到数据包中
      char *tmp1 = (char *)&value_x;
      char *tmp2 = (char *)&value_y;

      data[3] = tmp1[0];
      data[4] = tmp1[1];
      data[5] = tmp1[2];
      data[6] = tmp1[3];

      data[7] = tmp2[0];
      data[8] = tmp2[1];
      data[9] = tmp2[2];
      data[10] = tmp2[3];

      // 向下位机发送数据
      write(UART_fd, data, count);

      // 绘制检测框和标签
      for (auto &obj : objs)
      {
        auto name = mylabels[obj.class_label];

        uint8_t b, g, r;
        tie(b, g, r) = yolo::random_color(obj.class_label);
        cv::rectangle(frame, cv::Point(obj.left, obj.top), cv::Point(obj.right, obj.bottom),
                      cv::Scalar(b, g, r), 5);

        auto caption = cv::format("%s %.2f", name, obj.confidence);
        int width = cv::getTextSize(caption, 0, 1, 2, nullptr).width + 10;

        // 绘制目标中心点
        int obj_center_x = (obj.left + obj.right) / 2;
        int obj_center_y = (obj.top + obj.bottom) / 2;

        cv::rectangle(frame, cv::Point(obj.left - 3, obj.top - 33),
                      cv::Point(obj.left + width, obj.top), cv::Scalar(b, g, r), -1);
        cv::putText(frame, caption, cv::Point(obj.left, obj.top - 5), 0, 1, cv::Scalar::all(0), 2, 16);
      }
    }

    // 显示处理后的图像
    imshow("frame", frame);

    // 处理用户按键事件
    int key = cv::waitKey(1);
    if (key == 27)
      break;
  }

  // 释放摄像头资源和关闭窗口
  cap.release();
  cv::destroyAllWindows();
  return;
}
```

这段代码是一个函数`yolo_video_demo`，它用于进行实时视频目标检测和处理，并且根据检测到的目标执行一些动作，然后将数据通过串口发送给下位机。以下是这段代码的主要功能和流程的分析：

1. **加载YOLOv8推理引擎**：
    - 这段代码首先尝试加载一个YOLOv8的推理引擎，该引擎用于目标检测。如果加载失败（`yolo`为空指针），则函数直接返回，表示无法进行目标检测。
2. **打开串口通信并配置**：
    - 函数接着打开一个串口通信端口（串口号0），并配置其参数，包括波特率、数据位、校验位和停止位等。如果配置失败，它会打印错误信息并退出程序。
3. **初始化数据包**：
    - 创建一个名为`data`的字符数组，用于存储将要通过串口发送的数据。这个数据包的长度为12字节，以字母'A'开始和字母'D'结束。
4. **初始化图像处理所需的Mat对象**：
    - 创建了一系列`cv::Mat`对象，用于图像处理中的不同阶段。这些对象包括`frame`（原始摄像头帧）、`gray`（灰度图像）、`dilated`、`gaussianblured`、`thresholded`和`edges`，它们将在后续的图像处理过程中使用。
5. **打开摄像头**：
    - 使用OpenCV的`cv::VideoCapture`类打开摄像头。如果无法打开摄像头，将打印一条错误消息并返回。
6. **设置摄像头参数**：
    - 使用`cap.set`方法设置摄像头的参数，包括帧宽度、帧高度和帧速率。这些参数用于配置摄像头以获得所需的视频流。
7. **进入主循环**：
    - 代码进入一个无限循环，用于连续处理摄像头捕获的视频帧。
8. **读取视频帧**：
    - 通过`cap.read(frame)`从摄像头读取一帧图像，并获取帧的帧速率。
9. **目标检测**：
    - 使用YOLOv8推理引擎（`yolo->forward(cvimg(frame))`）对当前帧进行目标检测，结果存储在`objs`中。如果没有检测到任何目标，会打印"nothing"并将数据包中的一部分重置为初始值，然后通过串口发送。
10. **目标处理和动作执行**：
    - 如果检测到了目标，代码会找到最右边的目标（根据`right_obj`的右侧坐标）。
    - 根据检测到的目标名称，分别处理"cola"和"box"目标：
        - 对于"cola"目标，扩展目标边界框，提取目标区域的图像，然后进行图像处理，包括高斯模糊、膨胀、边缘检测等。
        - 对于"box"目标，只是在目标中心画了一个小圆点。
    - 计算目标中心点与摄像头中心点之间的距离，并计算平方误差。
    - 根据条件判断是否执行特定动作，例如向下位机发送数据。动作的条件基于距离和目标类型。
    - 将相关信息（如动作、目标类型、偏移值等）存储在`data`数据包中。
11. **数据发送和显示**：
    - 将数据包通过串口发送给下位机。
    - 绘制检测到的目标框和标签，并在帧上显示。
12. **用户按键事件处理**：
    - 检测是否有用户按键事件，如果按下"ESC"键（键码27），则退出主循环。
13. **释放资源**：
    - 退出主循环后，释放摄像头资源，关闭摄像头窗口，函数返回。

这段代码的主要目的是实时处理来自摄像头的视频流，在检测到特定类型的目标时，执行相应的动作，并将结果通过串口发送给下位机。它结合了图像处理、目标检测、条件判断和串口通信等多个功能。

## 鸣谢

<https://github.com/ultralytics/ultralytics>

<https://github.com/shouxieai/infer>