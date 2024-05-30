#include <chrono>
#include <filesystem>
#include <iostream>
#include <limits>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

using namespace cv;
using namespace std;
namespace fs = std::filesystem;

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

// 计算直线与图像边界的交点
void getLineEndPoints(const Point &p, double theta, const Size &imageSize, Point &p1, Point &p2)
{
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

bool HaveIntersection(const cv::Point &min_x_point, double theta,
                      const cv::Mat &image, const std::vector<std::vector<cv::Point>> &contours_approx,
                      Point &p1, Point &p2)
{
    // Point p1, p2;
    getLineEndPoints(min_x_point, theta, image.size(), p1, p2);

    // if (containsWhitePixel(image, p1, p2))
    // {
    //     bestP1 = p1;
    //     bestP2 = p2;
    //     foundLine = true;

    //     cout << "theta = " << theta << endl;

    //     line(image, bestP1, bestP2, Scalar(127), 2);
    //     imshow("Result", image);
    //     waitKey(0);
    //     // break;
    // }

    // 遍历轮廓中的每一条线段，判断是否与 (p1, p2) 相交
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

int main()
{
    // 定义图像文件夹路径
    string folder_path = "/home/binghe/orbbec_code/test/picture/testBEV/";

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

        // 获取当前时间点作为开始时间
        auto start = std::chrono::high_resolution_clock::now();

        cv::Mat image = image_orin.clone();

        // 转换为灰度图像
        if (image.channels() == 3)
        {
            cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);
            cv::threshold(image, image, 127, 255, cv::THRESH_BINARY);
        }

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(image, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE); // 只检测外轮廓,存储所有的轮廓点

        // 遍历所有轮廓并进行多边形逼近
        std::vector<std::vector<cv::Point>> contours_approx;
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

        min_x_point.x -= 5;

        if (min_x_point.x <= 0)
        {
            min_x_point.x += 5;
        }

        std::cout << "min_x_point = " << min_x_point.x << " " << min_x_point.y << std::endl;

        // // 找到所有白色像素中x坐标最小的那个
        // Point minPoint(image.cols, image.rows);
        // for (int y = 0; y < image.rows; y++)
        // {
        //     for (int x = 0; x < image.cols; x++)
        //     {
        //         if (image.at<uchar>(y, x) == 255 && x < minPoint.x)
        //         {
        //             minPoint = Point(x, y);
        //         }
        //     }
        // }

        // if (minPoint.x == image.cols)
        // {
        //     cerr << "No white pixels found!" << endl;
        //     return -1;
        // }

        Point bestP1, bestP2;
        bool foundLine = false;
        // 遍历每个角度
        Point p1, p2;
        if (HaveIntersection(min_x_point, M_PI / 2, image, contours_approx, p1, p2))
        {
            bool is_last_intersect = true; // 上一个是否相交
            Point p1_temp, p2_temp;
            for (double theta = M_PI / 2; theta > 0; theta -= M_PI / 180)
            {
                if (!HaveIntersection(min_x_point, theta, image, contours_approx, p1_temp, p2_temp) && is_last_intersect)
                {
                    p1 = p1_temp;
                    p2 = p2_temp;
                    is_last_intersect = false;
                }
            }
        }
        else
        {
            bool is_last_intersect = false; // 上一个是否相交
            Point p1_temp, p2_temp;
            for (double theta = M_PI / 2; theta < M_PI; theta += M_PI / 180)
            {
                if (HaveIntersection(min_x_point, theta, image, contours_approx, p1_temp, p2_temp) && !is_last_intersect)
                {
                    p1 = p1_temp;
                    p2 = p2_temp;
                    is_last_intersect = true;
                }
            }
        }

        // 获取当前时间点作为结束时间
        auto end = std::chrono::high_resolution_clock::now();

        // 计算时间差，单位为秒
        std::chrono::duration<double> elapsed = end - start;

        // 输出耗时，单位为秒
        std::cout << "函数执行耗时: " << elapsed.count() << " 秒" << std::endl;

        std::cout << "p1= " << p1 << " p2= " << p2 << std::endl;

        line(image, p1, p2, Scalar(127), 2);

        cv::circle(image, p1, 15, cv::Scalar(127), -1);
        cv::circle(image, p2, 15, cv::Scalar(127), -1);
        imshow("Result", image);
        waitKey(0);
    }
    return 0;
}
