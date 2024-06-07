#include <chrono>
#include <filesystem>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <iostream>
#include <limits>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

using namespace cv;
using namespace std;
namespace fs = std::filesystem;

DEFINE_int32(roi_x, 2, " ");
DEFINE_int32(roi_y, 160, " ");
DEFINE_int32(roi_width, 396, " ");
DEFINE_int32(roi_height, 198, " ");
DEFINE_int32(dividing_line, 200, "对原图进行的一个划分");

DEFINE_int32(rectTopLeft_x, -2000, " ");
DEFINE_int32(rectTopLeft_y, -1800, " ");
DEFINE_int32(rectBottomRight_x, 2000, " ");
DEFINE_int32(rectBottomRight_y, 360, " ");

struct RectROI
{
    int x;
    int y;
    int width;
    int height;
};

void clipPointToImage(Point &p, const Size &imageSize)
{
    if (p.x < 0)
        p.x = 0;
    if (p.x >= imageSize.width)
        p.x = imageSize.width - 1;
    if (p.y < 0)
        p.y = 0;
    if (p.y >= imageSize.height)
        p.y = imageSize.height - 1;
}

/**
 * @brief Get the Line End Points object
 *
 * @param p 直线通过的某一点坐标
 * @param theta 直线的方位角
 * @param imageSize
 * @param p1 直线与边框的第一个交点
 * @param p2 直线与边框的第二个交点
 */
void getLineEndPoints(Point &p, double theta, const Size &imageSize, Point &p1, Point &p2, const RectROI &rect_roi) //
{
    // 判断是否为原始图的x方向最小点
    if (imageSize.height >= 399 && imageSize.width >= 359)
    {
        // 原图的x方向最小点，调整最小值为roi区域的坐标
        p.x -= rect_roi.x;
        p.y -= rect_roi.y;
    }

    double a = tan(theta);    // 直线的斜率
    double b = p.y - a * p.x; // 直线的截距
    int w = imageSize.width;
    int h = imageSize.height;

    if (theta == 0 || theta == M_PI) // 如果直线与 x 轴平行
    {
        p1 = Point(0, p.y);     // 第一个交点位于图像左边界
        p2 = Point(w - 1, p.y); // 第二个交点位于图像右边界
    }
    else if (theta == M_PI / 2) // 如果直线与 y 轴平行
    {
        p1 = Point(p.x, h - 1); // 第一个交点位于图像下边界
        p2 = Point(p.x, 0);     // 第二个交点位于图像上边界
    }
    else
    {
        p1 = Point(0, b);                   // 假设交点在图像左边界上
        p2 = Point((h - 1 - b) / a, h - 1); // 根据直线方程计算可能的第二个交点

        // 如果第二个交点超出了图像右边界，调整为图像右边界上的交点
        if (p2.x < 0)
        {
            p2 = Point(0, b); // 调整为图像左边界上的交点
        }
        else if (p2.x >= w)
        {
            p2 = Point(w - 1, a * (w - 1) + b); // 调整为图像右边界上的交点
        }

        // 如果第一个交点超出了图像下边界，调整为图像下边界上的交点
        if (p1.y < 0)
        {
            p1 = Point((0 - b) / a, 0); // 调整为图像上边界上的交点
        }
        else if (p1.y >= h)
        {
            p1 = Point(w - 1, a * (w - 1) + b); // 调整为图像下边界上的交点
        }
    }

    // 确保 p1 到 p2 的方向在 0 到 pi 之间
    if ((p1.x > p2.x && p1.y == p2.y) || (p1.x == p2.x && p1.y < p2.y) ||
        (p1.x > p2.x && p1.y < p2.y) || (p1.x < p2.x && p1.y < p2.y))
    {
        std::swap(p1, p2);
    }
}

/**
 * @brief 获取直线与矩形框的交点
 *
 * @param p 直线通过的某一点坐标
 * @param theta 直线的方位角
 * @param rectTopLeft 矩形框的左上角坐标
 * @param rectBottomRight 矩形框的右下角坐标
 * @param p1 直线与边框的第一个交点
 * @param p2 直线与边框的第二个交点
 */
void getLineEndPoints(Point &p, double theta, const Point &rectTopLeft, const Point &rectBottomRight,
                      Point &p1, Point &p2)
{
    double a = tan(theta);    // 直线的斜率
    double b = p.y - a * p.x; // 直线的截距
    int left = rectTopLeft.x;
    int right = rectBottomRight.x;
    int top = rectTopLeft.y;
    int bottom = rectBottomRight.y;

    if (theta == 0 || theta == M_PI)
    {                           // 如果直线与 x 轴平行
        p1 = Point(left, p.y);  // 第一个交点位于矩形左边界
        p2 = Point(right, p.y); // 第二个交点位于矩形右边界
    }
    else if (theta == M_PI / 2 || theta == 3 * M_PI / 2)
    {                            // 如果直线与 y 轴平行
        p1 = Point(p.x, top);    // 第一个交点位于矩形上边界
        p2 = Point(p.x, bottom); // 第二个交点位于矩形下边界
    }
    else
    {
        // 假设交点在矩形框的左边界上
        p1 = Point(left, a * left + b);
        // 根据直线方程计算可能的第二个交点
        p2 = Point(right, a * right + b);

        // 如果第一个交点超出了矩形框的上下边界，调整为上下边界上的交点
        if (p1.y < top || p1.y > bottom)
        {
            if (p1.y < top)
            {
                p1 = Point((top - b) / a, top);
            }
            else if (p1.y > bottom)
            {
                p1 = Point((bottom - b) / a, bottom);
            }
        }

        // 如果第二个交点超出了矩形框的上下边界，调整为上下边界上的交点
        if (p2.y < top || p2.y > bottom)
        {
            if (p2.y < top)
            {
                p2 = Point((top - b) / a, top);
            }
            else if (p2.y > bottom)
            {
                p2 = Point((bottom - b) / a, bottom);
            }
        }
    }

    // 确保 p1 到 p2 的方向在 0 到 pi 之间
    if ((p1.x > p2.x && p1.y == p2.y) || (p1.x == p2.x && p1.y < p2.y) ||
        (p1.x > p2.x && p1.y < p2.y) || (p1.x < p2.x && p1.y < p2.y))
    {
        std::swap(p1, p2);
    }
}

// 判断线段是否包含白色像素
bool containsWhitePixel(const Mat &image, const Point &p1, const Point &p2)
{
    LineIterator it(image, p1, p2, 4);
    for (int i = 0; i < it.count; i++, ++it)
    {
        if (image.at<uchar>(it.pos()) == 255)
        {
            return true;
        }
    }
    return false;
}

/**
 * @brief 检查两条线段是否相交，并返回交点
 *
 * @param p0
 * @param p1
 * @param p2
 * @param p3
 * @param i
 * @return true
 * @return false
 */
bool getLineIntersection(cv::Point2f p0, cv::Point2f p1, cv::Point2f p2, cv::Point2f p3, cv::Point2f &i)
{
    cv::Point2f s1, s2;
    s1.x = p1.x - p0.x;
    s1.y = p1.y - p0.y;
    s2.x = p3.x - p2.x;
    s2.y = p3.y - p2.y;

    float s, t;
    float det = (-s2.x * s1.y + s1.x * s2.y);

    if (det == 0)
    {
        return false; // 平行线，无交点
    }

    s = (-s1.y * (p0.x - p2.x) + s1.x * (p0.y - p2.y)) / det;
    t = (s2.x * (p0.y - p2.y) - s2.y * (p0.x - p2.x)) / det;

    if (s >= 0 && s <= 1 && t >= 0 && t <= 1)
    {
        // 线段相交
        i.x = p0.x + (t * s1.x);
        i.y = p0.y + (t * s1.y);
        return true;
    }

    return false; // 线段不相交
}

/**
 * @brief 遍历轮廓中的每一条线段，判断是否与直线(p1, p2) 相交。
 *
 * @param min_x_point 轮廓x方向最小的点
 * @param theta 直线的方向
 * @param image 原图或roi图
 * @param contours_approx
 * @param p1 直线的起始端点
 * @param p2 直线的结束端点
 * @return true
 * @return false
 */
bool isIntersectLineAndContour(const cv::Point &min_x_point, double theta,
                               const cv::Mat &image, const std::vector<std::vector<cv::Point>> &contours_approx,
                               Point &p1, Point &p2,
                               const RectROI &rect_roi)
{
    // 1. 根据自定义矩形框，生成theta角度下直线的两个端点
    Point rectTopLeft(FLAGS_rectTopLeft_x, FLAGS_rectTopLeft_y);
    Point rectBottomRight(FLAGS_rectBottomRight_x, FLAGS_rectBottomRight_y);
    cv::Point min_x_point_clone(min_x_point);
    // getLineEndPoints(min_x_point, theta, image.size(), p1, p2, rect_roi);
    getLineEndPoints(min_x_point_clone, theta, rectTopLeft, rectBottomRight, p1, p2);

    // 2. 遍历轮廓中的每一条线段，判断是否与 (p1, p2) 相交
    for (const auto &contour : contours_approx)
    {
        for (size_t i = 0; i < contour.size(); ++i)
        {
            cv::Point q1 = contour[i];
            cv::Point q2 = contour[(i + 1) % contour.size()]; // 循环获取轮廓的下一个点

            // 判断是否相交
            cv::Point2f intersection;
            if (getLineIntersection(q1, q2, p1, p2, intersection))
            {
                // 如果相交，画出 (p1, p2) 直线
                // cv::line(image, p1, p2, cv::Scalar(255), 2);
                // imshow("Result", image);
                // waitKey(0);
                // break; // 找到一个相交就可以退出内层循环

                return true;
            }
        }
    }

    return false;
}

/**
 * @brief 返回false，表示bev不存在非草地，则向右原地旋转。返回true，发路径给控制算法
 *
 * @param img
 * @return true
 * @return false
 */
bool isHaveNonGrass(const cv::Mat &img)
{
    cv::Mat img_mat = img.clone();

    // 转换为灰度图像
    if (img_mat.channels() == 3)
    {
        cv::cvtColor(img_mat, img_mat, cv::COLOR_BGR2GRAY);
    }

    // 二值化处理，阈值设为128，最大值为255
    cv::Mat binaryImg;
    cv::threshold(img_mat, binaryImg, 128, 255, cv::THRESH_BINARY);

    // // 定义感兴趣区域(ROI)，以矩形的左上角点（x, y）和宽度、高度
    // int x = 0;        // 左上角点的x坐标
    // int y = 160;      // 左上角点的y坐标
    // int width = 400;  // 矩形区域的宽度
    // int height = 200; // 矩形区域的高度

    // // 创建ROI
    // cv::Rect roi(x, y, width, height);
    // // 提取ROI对应的图像部分
    // cv::Mat imageROI = binaryImg(roi);

    // // 显示原图像和提取的ROI图像
    // cv::imshow("Original Image", binaryImg);
    // cv::imshow("ROI Image", imageROI);

    // // 等待按键按下
    // cv::waitKey(0);

    // 遍历图像左边 (x < 200) 区域，检查是否存在白色像素
    for (int row = 0; row < binaryImg.rows; ++row)
    {
        for (int col = 0; col < binaryImg.cols; ++col)
        {
            // 如果找到一个白色像素 (值为255)
            if (binaryImg.at<uchar>(row, col) == 255)
            {
                return true;
            }
        }
    }

    return false;
}

/**
 * @brief
 *
 * @param img
 * @return true
 * @return false
 * @note 作用1. 原图小车的左边存在障碍物，则使用roi区域生成路径
 * @note 作用2. 返回true，表示小车的左边存在非草地，则原地向左旋转。返回false，发路径给控制算法
 */
bool isNonGrassOnRight(const cv::Mat &img, const int &dividing_line = 200)
{
    cv::Mat img_mat = img.clone();

    // 转换为灰度图像
    if (img_mat.channels() == 3)
    {
        cv::cvtColor(img_mat, img_mat, cv::COLOR_BGR2GRAY);
    }

    // 二值化处理，阈值设为128，最大值为255
    cv::Mat binaryImg;
    cv::threshold(img_mat, binaryImg, 128, 255, cv::THRESH_BINARY);

    // 遍历图像左边 (x < 200) 区域，检查是否存在白色像素
    for (int row = 0; row < binaryImg.rows; ++row)
    {
        for (int col = 0; col < dividing_line; ++col)
        {
            // 如果找到一个白色像素 (值为255)
            if (binaryImg.at<uchar>(row, col) == 255)
            {
                return true;
            }
        }
    }

    return false;
}

/**
 * @brief Get the Image R O I object
 *
 * @param image_orin
 * @param rect_roi
 * @return cv::Mat
 */
cv::Mat getImageROI(const cv::Mat &image_orin, const RectROI &rect_roi)
{
    // 创建ROI
    cv::Rect roi(rect_roi.x, rect_roi.y, rect_roi.width, rect_roi.height);
    // 提取ROI对应的图像部分
    cv::Mat imageROI = image_orin(roi).clone();

    // // 显示原图像和提取的ROI图像
    // cv::imshow("Original Image", image_orin);
    // cv::imshow("ROI Image", imageROI);

    // // 等待按键按下
    // cv::waitKey(0);

    return imageROI;
}

// #include <algorithm>
// #include <vector>

// class Solution
// {
// public:
//     int maxSubArray(std::vector<int> &nums)
//     {
//         if (nums.empty())
//         {
//             return 0;
//         }

//         int current_sum = nums[0]; // 表示当前子数组的和从第一个元素开始
//         int max_sum = nums[0];     // 记录目前为止的最大子数组和

//         for (size_t i = 1; i < nums.size(); ++i)
//         {
//             // 更新 current_sum
//             current_sum = std::max(nums[i], current_sum + nums[i]);
//             // 更新 max_sum
//             max_sum = std::max(max_sum, current_sum);
//         }

//         return max_sum;
//     }
// };

/**
 * @brief 获取图像中轮廓的x方向最小的点，同时向左移动5个像素
 *
 * @param image
 * @param contours_approx 获取曲线逼近后的轮廓
 * @return cv::Point
 */
cv::Point CalXMinPoint(const cv::Mat &image, std::vector<std::vector<cv::Point>> &contours_approx)
{
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(image, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE); // 只检测外轮廓,存储所有的轮廓点

    // 遍历所有轮廓并进行多边形逼近
    for (size_t i = 0; i < contours.size(); i++)
    {
        std::vector<cv::Point> approx;
        cv::approxPolyDP(contours[i], approx, 5.0, true); // 进行多边形逼近
        contours_approx.emplace_back(approx);
    }

    cv::Point min_x_point;                       // 用于存储x坐标最小的点
    int min_x = std::numeric_limits<int>::max(); // 初始化为最大整数值

    // 遍历所有轮廓
    for (size_t i = 0; i < contours_approx.size(); i++)
    {
        // 遍历轮廓中的每个点
        for (size_t j = 0; j < contours_approx[i].size(); j++)
        {
            // 如果当前点的x坐标小于已记录的最小x坐标，则更新最小x坐标和点
            if (contours_approx[i][j].x < min_x)
            {
                min_x = contours_approx[i][j].x;
                min_x_point = contours_approx[i][j];
            }
        }
    }

    // 将点向左移动5个像素
    min_x_point.x -= 5;
    if (min_x_point.x <= 0)
    {
        min_x_point.x += 5;
    }

    std::cout << "min_x_point = " << min_x_point.x << " " << min_x_point.y << std::endl;
    return min_x_point;
}

/**
 * @brief Get the Follow Path object
 *
 * @param image_orin 原始图像
 * @param imageROI roi图像
 * @return std::vector<cv::Point>
 * @note 1. 获取图像中轮廓的x方向最小的点   2. 根据min_x_point，获取与轮廓恰好相交的直线，求出与自定义边框的交点p1和p2
 */
std::vector<cv::Point> GetFollowPath(const cv::Mat &image_orin, const RectROI &rect_roi)
{
    // 获取当前时间点作为开始时间
    auto start = std::chrono::high_resolution_clock::now();

    Point p1, p2;
    {
        cv::Mat image = image_orin.clone();

        // 转换为灰度图像
        if (image.channels() == 3)
        {
            cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);
            cv::threshold(image, image, 127, 255, cv::THRESH_BINARY);
        }

        // 1. 获取图像中轮廓的x方向最小的点
        std::vector<std::vector<cv::Point>> contours_approx; // 获取曲线逼近后的轮廓
        cv::Point min_x_point = CalXMinPoint(image, contours_approx);

        // 2. 根据min_x_point，获取与轮廓恰好相交的直线，求出与自定义边框的交点p1和p2
        if (isIntersectLineAndContour(min_x_point, M_PI / 2, image, contours_approx, p1, p2, rect_roi))
        {
            bool is_last_intersected = true; // 上一个是否相交
            Point p1_temp, p2_temp;
            // 遍历每个角度
            for (double theta = M_PI / 2; theta > 0; theta -= M_PI / 180)
            {
                if (!isIntersectLineAndContour(min_x_point, theta, image, contours_approx, p1_temp, p2_temp, rect_roi) && is_last_intersected)
                {
                    p1 = p1_temp;
                    p2 = p2_temp;
                    is_last_intersected = false;
                }
            }
        }
        else
        {
            bool is_last_intersected = false; // 上一个是否相交
            Point p1_temp, p2_temp;
            for (double theta = M_PI / 2; theta < M_PI; theta += M_PI / 180)
            {
                if (isIntersectLineAndContour(min_x_point, theta, image, contours_approx, p1_temp, p2_temp, rect_roi) && !is_last_intersected)
                {
                    p1 = p1_temp;
                    p2 = p2_temp;
                    is_last_intersected = true;
                }
            }
        }
    }

    // 获取当前时间点作为结束时间
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    std::cout << "函数执行耗时: " << elapsed.count() << " 秒" << std::endl;

    // 可视化
    //  std::cout << "p1= " << p1 << " p2= " << p2 << std::endl;
    // line(image, p1, p2, Scalar(127), 2);
    // cv::circle(image, p1, 15, cv::Scalar(127), -1);
    // cv::circle(image, p2, 15, cv::Scalar(127), -1);
    // imshow("Result", image);
    // waitKey(0);

    // p1和p2肯定存在
    return std::vector<cv::Point>{p1, p2};
}

void func()
{
    // 1. 如果障碍物出现在左边。且有且只有两个轮廓的情况

    // 1.1
}

int main(int argc, char *argv[])
{
    // 设置用法信息
    gflags::SetUsageMessage("Usage: path [options]");
    // 解析命令行参数
    gflags::ParseCommandLineFlags(&argc, &argv, true);

    // 初始化glog
    google::InitGoogleLogging(argv[0]);

    FLAGS_logtostderr = true; // 关闭输出到stderr

    // 设置日志目录
    // FLAGS_log_dir = ".";

    // 设置日志级别
    FLAGS_minloglevel = google::INFO;
    FLAGS_colorlogtostderr = true; // 启用彩色输出

    // 1. 定义图像文件夹路径
    string folder_path = "/home/binghe/orbbec_code/opencv_test/picture/testBEV";

    // 存储图像文件路径
    vector<string> image_paths;

    // 遍历文件夹中的所有图像文件
    for (const auto &entry : fs::directory_iterator(folder_path))
    {
        if (entry.is_regular_file())
        {
            string file_path = entry.path().string();
            // 仅添加特定扩展名的文件，如 .png 或 .jpg
            if (file_path.substr(file_path.find_last_of(".") + 1) == "png" ||
                file_path.substr(file_path.find_last_of(".") + 1) == "jpg")
            {
                image_paths.push_back(file_path);
            }
        }
    }

    std::cout << "image_paths = " << image_paths.size() << std::endl;

    // 遍历每个图像路径
    for (const string &image_path : image_paths)
    {
        std::cout << "image_path = " << image_path << std::endl;

        // 读取图像
        Mat image_orin = imread(image_path, IMREAD_GRAYSCALE);
        if (image_orin.empty())
        {
            cerr << "Image load failed!" << endl;
            return -1;
        }

        // 2. 生成路径
        RectROI rect_roi;
        rect_roi.x = FLAGS_roi_x;
        rect_roi.y = FLAGS_roi_y;
        rect_roi.width = FLAGS_roi_width;
        rect_roi.height = FLAGS_roi_height;

        std::vector<cv::Point> path;
        std::vector<cv::Point> before_path;
        cv::Mat imageROI = getImageROI(image_orin, rect_roi);

        if (isNonGrassOnRight(image_orin, FLAGS_dividing_line)) // 原图小车的左边存在障碍物，则使用roi区域生成路径
        {
            LOG(ERROR) << "使用roi区域生成路径";
            // 使用roi区域生成路径
            path = GetFollowPath(imageROI, rect_roi);
            LOG(WARNING) << "后处理前 path = " << path[0].x << " " << path[0].y;

            // 后处理。
            // 1. 判断是否相交，将交点放入路径的起始点
            cv::Point2f intersection;
            if (getLineIntersection(path[0], path[1],
                                    cv::Point(FLAGS_rectTopLeft_x, FLAGS_roi_height),
                                    cv::Point(FLAGS_rectBottomRight_x, FLAGS_roi_height), intersection))
            {
                path[0].x = static_cast<int>(intersection.x);
                path[0].y = static_cast<int>(intersection.y);
            }

            LOG(WARNING) << "后处理后 path = " << path[0].x << " " << path[0].y;

            // 可视化
            line(image_orin, path[0], path[1], Scalar(127), 2);
            cv::circle(image_orin, path[0], 15, cv::Scalar(127), -1);
            cv::circle(image_orin, path[1], 15, cv::Scalar(127), -1);
            namedWindow("Result", WINDOW_AUTOSIZE);
            imshow("Result", image_orin);
            waitKey(0);
        }
        else
        {
            LOG(ERROR) << "使用原图生成路径";
            // 使用原图生成路径
            before_path = GetFollowPath(image_orin, rect_roi);

            LOG(WARNING) << "before_path = " << before_path[0].x << " " << before_path[0].y;

            // 后处理。
            // 1. 将生成的路径转为roi坐标系下
            for (const auto &p : before_path)
            {
                path.emplace_back(cv::Point(p.x - FLAGS_roi_x, p.y - FLAGS_roi_y));
            }

            // 2. 判断是否相交，将交点放入路径的起始点
            cv::Point2f intersection;
            if (getLineIntersection(path[0], path[1],
                                    cv::Point(FLAGS_rectTopLeft_x, FLAGS_rectBottomRight_y),
                                    cv::Point(FLAGS_rectBottomRight_x, FLAGS_rectBottomRight_y), intersection))
            {
                path[0].x = static_cast<int>(intersection.x);
                path[0].y = static_cast<int>(intersection.y);
            }

            LOG(WARNING) << "后处理后 path = " << path[0].x << " " << path[0].y;

            // 可视化
            line(image_orin, before_path[0], before_path[1], Scalar(127), 2);
            cv::circle(image_orin, before_path[0], 15, cv::Scalar(127), -1);
            cv::circle(image_orin, before_path[1], 15, cv::Scalar(127), -1);
            namedWindow("Result", WINDOW_AUTOSIZE);
            imshow("Result", image_orin);
            waitKey(0);
        }

        // 3.利用生成路径的端点，绘图
        cv::Point p1 = path[0];
        cv::Point p2 = path[1];

        std::cout << "p1= " << p1 << " p2= " << p2 << std::endl;

        line(imageROI, p1, p2, Scalar(127), 2);

        cv::circle(imageROI, p1, 15, cv::Scalar(127), -1);
        cv::circle(imageROI, p2, 15, cv::Scalar(127), -1);
        cv::namedWindow("11", WINDOW_AUTOSIZE);
        imshow("11", imageROI);
        waitKey(0);
    }

    // 关闭glog
    google::ShutdownGoogleLogging();

    return 0;
}
