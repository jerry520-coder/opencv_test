#include <chrono> // 引入chrono头文件
#include <iostream>
#include <opencv2/opencv.hpp>

#define vis 1

void drawContoursVec(const std::vector<std::vector<cv::Point>> &contours_approx, cv::Mat &img_rgb)
{
    for (size_t i = 0; i < contours_approx.size(); i++)
    {
        // 绘制轮廓上的每个点
        for (size_t j = 0; j < contours_approx[i].size(); j++)
        {
            cv::Point point = contours_approx[i][j];

            if (j == 0)
            {
                // 绘制第一点
                cv::circle(img_rgb, point, 10, cv::Scalar(0, 255, 0), -1); // 画一个半径为1的绿色点
            }
            else if (j == 1)
            {
                // 绘制第二点
                cv::circle(img_rgb, point, 7, cv::Scalar(255, 0, 0), -1); // 画一个半径为1的蓝色点
            }
            else
            {
                // 绘制其他点
                cv::circle(img_rgb, point, 5, cv::Scalar(0, 0, 255), -1); // 画一个半径为1的红色点
            }
        }
    }
}

/**
 * @brief 在图像坐标系下的角度转换
 *
 * @param pt1 起始点
 * @param pt2 目标点
 * @return float
 */
float calculateAngle(const cv::Point &pt1, const cv::Point &pt2)
{
    float deltaY = (-pt2.y) - (-pt1.y);
    float deltaX = pt2.x - pt1.x;

    std::cout << "atan2f(deltaY, deltaX) = " << atan2f(deltaY, deltaX) << std::endl;
    return atan2f(deltaY, deltaX); // 计算方向角度
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

// 检查路径点是否在轮廓范围内的函数
std::vector<cv::Point> filterPointsWithinContour(const std::vector<cv::Point> &path, const std::vector<cv::Point> &contour)
{
    std::vector<cv::Point> path_within;
    for (const cv::Point &point : path)
    {
        double result = cv::pointPolygonTest(contour, point, false);
        if (result >= 0)
        {
            // 点在轮廓内，保留
            path_within.push_back(point);
        }
    }
    return path_within;
}

std::vector<cv::Point> handlePathWithIntersections(const std::vector<cv::Point> &path, const cv::Point &p1, const cv::Point &p2)
{
    std::vector<cv::Point> path_temp2;

    if (path.size() == 0)
    {
        // 如果路径点为0，无法进行路径插值，直接返回空路径
        std::cerr << "路径点数为0, 无法处理交叉点。" << std::endl;
        return std::vector<cv::Point>();
    }
    else
    {
        for (size_t i = 0; i < path.size() - 1; ++i)
        {
            cv::Point2f intersection;
            if (getLineIntersection(path[i], path[i + 1], p1, p2, intersection))
            {
                std::cout << "机器人前方20cm相交点: (" << intersection.x << ", " << intersection.y << ")" << std::endl;
                path_temp2.emplace_back(cv::Point(static_cast<int>(intersection.x), static_cast<int>(intersection.y)));
            }
            else
            {
                path_temp2.emplace_back(path[i]);
            }
        }
        path_temp2.emplace_back(path[path.size() - 1]);
    }

    return path_temp2;
}

/**
 * @brief
 *
 * @param path_temp1
 * @param contour
 * @param p1 相交直线的两个端点
 * @param p2
 * @return std::vector<cv::Point>
 */
std::vector<cv::Point> processPath(const std::vector<cv::Point> &path_temp1, const std::vector<cv::Point> &contour, const cv::Point &p1, const cv::Point &p2)
{
    // 2.2.1 检查每个路径点是否在轮廓范围内,点在轮廓内，保留。用于后续剔除
    std::vector<cv::Point> path_within = filterPointsWithinContour(path_temp1, contour);

    // 2.2.2 检查路径中的每个线段与直线p1p2是否相交
    std::vector<cv::Point> path_temp2 = handlePathWithIntersections(path_temp1, p1, p2);

    // 2.2.3 path_temp2 中剔除在 path_within 中存在的点，同时保持 path_temp2 点的顺序不变，
    path_temp2.erase(
        std::remove_if(path_temp2.begin(), path_temp2.end(),
                       [&path_within](const cv::Point &p)
                       {
                           return std::find(path_within.begin(), path_within.end(), p) != path_within.end();
                       }),
        path_temp2.end());

    return path_temp2;
}

std::vector<cv::Point> GetFollowPath(const cv::Mat &input_img,
                                     const std::vector<cv::Point> &left_dead_line,
                                     const std::vector<cv::Point> &right_dead_line,
                                     const float &resolution)
{
    cv::Mat img_mat = input_img.clone();

    // 转换为灰度图像
    if (img_mat.channels() == 3)
    {
        cv::cvtColor(img_mat, img_mat, cv::COLOR_BGR2GRAY);
    }

    std::cout << img_mat.channels() << std::endl;

    // cv::imshow("img_mat", img_mat);
    // cv::waitKey(0);

    cv::threshold(img_mat, img_mat, 127, 255, cv::THRESH_BINARY);

    // 确保二值图像不是空的
    if (img_mat.empty())
    {
        std::cerr << "二值化处理后图像为空" << std::endl;
        return std::vector<cv::Point>{};
    }

#if 0
    // 对二值化后的画布进行形态学膨胀、腐蚀以及闭处理
    // float robot_radius = 0.3;
    // int size = static_cast<int>(ceil(robot_radius / resolution)); // kernel容易受小车半径和图像分辨率的影响
    int size = 20;
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(size, size), cv::Point(-1, -1));
    // cv::morphologyEx(img_mat, img_mat, cv::MORPH_OPEN, kernel, cv::Point(-1, -1), 1);   // 1
    cv::morphologyEx(img_mat, img_mat, cv::MORPH_DILATE, kernel, cv::Point(-1, -1), 1); // 1

#endif

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(img_mat, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE); // 只检测外轮廓,存储所有的轮廓点

#if vis
    // 遍历每个轮廓
    cv::Mat img_rgb2 = cv::Mat::zeros(img_mat.size(), CV_8UC3);

    cv::drawContours(img_rgb2, contours, -1, cv::Scalar(255, 255, 255), cv::FILLED);

    for (size_t i = 0; i < contours.size(); i++)
    {
        // 绘制轮廓上的每个点
        for (size_t j = 0; j < contours[i].size(); j++)
        {
            cv::Point point = contours[i][j];
            // 绘制点
            cv::circle(img_rgb2, point, 2, cv::Scalar(0, 0, 255), -1); // 画一个半径为1的绿色点
        }
    }
    cv::namedWindow("img_rgb2", cv::WINDOW_AUTOSIZE);
    cv::imshow("img_rgb2", img_rgb2);
    cv::waitKey(0);

#endif

    // 遍历所有轮廓并进行多边形逼近
    std::vector<std::vector<cv::Point>> contours_approx;
    std::cout << "contours_approx.size() = " << contours_approx.size() << std::endl;
    for (size_t i = 0; i < contours.size(); i++)
    {
        std::vector<cv::Point> approx;
        cv::approxPolyDP(contours[i], approx, 30.0, true); // 进行多边形逼近
        contours_approx.emplace_back(approx);
    }

#if vis
    // 遍历每个轮廓
    cv::Mat img_rgb = cv::Mat::zeros(img_mat.size(), CV_8UC3);

    for (size_t i = 0; i < contours.size(); i++)
    {
        // 绘制轮廓上的每个点
        for (size_t j = 0; j < contours[i].size(); j++)
        {
            cv::Point point = contours[i][j];
            // 绘制点
            cv::circle(img_rgb, point, 1, cv::Scalar(0, 255, 0), -1); // 画一个半径为1的绿色点
        }
    }

    drawContoursVec(contours_approx, img_rgb);

    cv::namedWindow("img_rgb", cv::WINDOW_AUTOSIZE);

    cv::imshow("img_rgb", img_rgb);
    cv::waitKey(0);

#endif

    std::cout << "contours_approx.size() = " << contours_approx.size() << std::endl;

    // 1. 对多边形逼近后的所有轮廓，按照逆时针进行排列
    std::vector<cv::Point> path;

    if (contours_approx.size() == 0)
    {
        return std::vector<cv::Point>{};
    }
    else if (contours_approx.size() == 1)
    {
        std::cout << "进入轮廓为1" << std::endl;

        std::vector<cv::Point> contour = contours_approx.at(0);
        std::reverse(contour.begin(), contour.end());
        path = contour;

#if vis
        cv::Mat img_rgb1 = cv::Mat::zeros(img_mat.size(), CV_8UC3);
        drawContoursVec(std::vector<std::vector<cv::Point>>{path}, img_rgb1);
        cv::namedWindow("img_rgb1", cv::WINDOW_AUTOSIZE);
        cv::imshow("img_rgb1", img_rgb1);
        cv::waitKey(0);

#endif
    }
    else
    {
        std::cout << "进入轮廓为多个" << std::endl;

        int x_min = std::numeric_limits<int>::max();
        int y_min = std::numeric_limits<int>::max();
        int x_max = std::numeric_limits<int>::min();
        int y_max = std::numeric_limits<int>::min();

        for (const auto &contour : contours_approx)
        {
            for (const auto &point : contour)
            {
                if (point.x < x_min)
                    x_min = point.x;
                if (point.y < y_min)
                    y_min = point.y;
                if (point.x > x_max)
                    x_max = point.x;
                if (point.y > y_max)
                    y_max = point.y;
            }
        }

        cv::Point p1(x_min, y_max);
        cv::Point p2(x_min, y_min);

        path.emplace_back(p1);
        path.emplace_back(p2);
    }

    std::cout << "path = " << path[0] << path[1] << std::endl;

    // 2. 对生成的路径，剔除不合适的点
    std::vector<cv::Point> final_path;

    // 2.1 保留指定方向向量的点
    std::vector<cv::Point> path_temp1;
    for (size_t i = 1; i < path.size(); ++i)
    {
        if (i == path.size() - 1)
        {
            float angle = calculateAngle(path[i - 1], path[i]);

            // 判断角度是否在0到pi的范围内 (0,pi]
            if (angle > 0 && angle <= M_PI)
            {
                path_temp1.push_back(path[i - 1]);
                path_temp1.push_back(path[i]);
            }
        }
        else
        {
            float angle = calculateAngle(path[i - 1], path[i]);

            // 判断角度是否在0到pi的范围内 (0,pi]
            if (angle > 0 && angle <= M_PI)
            {
                path_temp1.push_back(path[i - 1]);
            }
        }
    }

    std::cout << "path_temp1.size() = " << path_temp1.size() << std::endl;
    std::cout << "path_temp1 = " << path_temp1[0] << path_temp1[1] << std::endl;

    // 2.2 剔除机器人前方5cm以内的点
    std::vector<cv::Point> path_temp2;
    int distance_front = static_cast<int>(0.05 / resolution); // 0.2m
    int width = input_img.cols;
    int height = input_img.rows;

    cv::Point p1(0, height - distance_front);
    cv::Point p2(width, height - distance_front);
    cv::Point p3(0, height);
    cv::Point p4(width, height);
    std::vector<cv::Point> contour = {p1, p2, p4, p3};

    std::cout << p1 << "," << p2 << std::endl;

    path_temp2 = processPath(path_temp1, contour, p1, p2);

    // 2.3 剔除盲区的路径点
    std::vector<cv::Point> path_temp3;

    std::vector<cv::Point> contour_left = left_dead_line;
    contour_left.emplace_back(p3);
    std::vector<cv::Point> contour_right = right_dead_line;
    contour_right.emplace_back(p4);

    path_temp3 = processPath(path_temp2, contour_left, left_dead_line[0], left_dead_line[1]);

    final_path = processPath(path_temp3, contour_right, right_dead_line[0], right_dead_line[1]);

    // 2.4 将最后生成的路径，进行可行性判断。如果路径的两点穿过膨胀后的白色区域，则剔除上一个路径点，且如果最后剔除只剩下了最后一个点，则将最后一个点也剔除

    // // 判断连线是否穿过白色区域
    // bool isLineCrossingWhite(const cv::Mat &image, const cv::Point &p1, const cv::Point &p2)
    // {
    //     cv::LineIterator it(image, p1, p2, 8);
    //     for (int i = 0; i < it.count; ++i, ++it)
    //     {
    //         if (image.at<uchar>(it.pos()) == 255)
    //         { // 判断当前像素是否为白色
    //             return true;
    //         }
    //     }
    //     return false;
    // }

    // // 判断路径可行性并剔除不符合条件的点
    // std::vector<cv::Point> validatePath(const cv::Mat &image, std::vector<cv::Point> &path)
    // {
    //     std::vector<cv::Point> validPath;

    //     for (size_t i = 0; i < path.size() - 1; ++i)
    //     {
    //         if (!isLineCrossingWhite(image, path[i], path[i + 1]))
    //         {
    //             validPath.push_back(path[i]); // 如果没有穿过白色区域，则将
    //         }
    //         else
    //         {
    //             if (!validPath.empty())
    //             {
    //                 validPath.pop_back();
    //             }
    //         }
    //     }

    //     if (!path.empty() && !isLineCrossingWhite(image, path[path.size() - 2], path[path.size() - 1]))
    //     {
    //         validPath.push_back(path.back());
    //     }

    //     if (validPath.size() == 1)
    //     {
    //         validPath.clear();
    //     }

    //     return validPath;
    // }

#if vis
    // 可视化
    cv::Mat img_filter = cv::Mat::zeros(img_mat.size(), CV_8UC3);
    cv::circle(img_filter, p1, 10, cv::Scalar(0, 0, 255), -1);
    cv::circle(img_filter, p2, 10, cv::Scalar(0, 0, 255), -1);
    cv::line(img_filter, p1, p2, cv::Scalar(0, 0, 255), 2);

    cv::circle(img_filter, left_dead_line[0], 10, cv::Scalar(0, 255, 255), -1);
    cv::circle(img_filter, left_dead_line[1], 10, cv::Scalar(0, 255, 255), -1);
    cv::line(img_filter, left_dead_line[0], left_dead_line[1], cv::Scalar(0, 255, 255), 2);

    cv::circle(img_filter, right_dead_line[0], 10, cv::Scalar(0, 255, 255), -1);
    cv::circle(img_filter, right_dead_line[1], 10, cv::Scalar(0, 255, 255), -1);
    cv::line(img_filter, right_dead_line[0], right_dead_line[1], cv::Scalar(0, 255, 255), 2);

    drawContoursVec(std::vector<std::vector<cv::Point>>{final_path}, img_filter);
    cv::namedWindow("img_filter", cv::WINDOW_AUTOSIZE);
    cv::imshow("img_filter", img_filter);
    cv::waitKey(0);

#endif

    return final_path;
}

bool isNonGrassOnRight(const cv::Mat &img)
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
        for (int col = 0; col < 200; ++col)
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

std::vector<cv::Point> getFollowPath(const cv::Mat &img)
{
    // 二值化处理，阈值设为128，最大值为255
    cv::Mat binaryImg;
    cv::threshold(img, binaryImg, 128, 255, cv::THRESH_BINARY);

    std::vector<cv::Point> path;

    // 查找y=140行中的第一个白色像素
    for (int col = 0; col < binaryImg.cols; ++col)
    {
        if (binaryImg.at<uchar>(140, col) == 255)
        {
            path.push_back(cv::Point(col, 140));
            break;
        }
    }
    // 查找y=0行中的第一个白色像素
    for (int col = 0; col < binaryImg.cols; ++col)
    {
        if (binaryImg.at<uchar>(0, col) == 255)
        {
            path.push_back(cv::Point(col, 0));
            break;
        }
    }

    std::cout << "path =" << path.size() << std::endl;

    return path;
}

void func()
{
}

int main()
{
    // // 图像的宽和高
    // int width = 800;
    // int height = 400;

    // // 创建一个全黑的图像
    // cv::Mat image = cv::Mat::zeros(height, width, CV_8UC1);

    // // 在图像的右半部分绘制一个白色矩形
    // cv::rectangle(image, cv::Point(width / 2, 0), cv::Point(width, height), cv::Scalar(255), cv::FILLED);

    // 读取图像
    cv::Mat src = cv::imread("/home/binghe/orbbec_code/test/picture/testBEV/001766_0544_403480219_1716951962392967_Color_1280x720.png");
    if (src.empty())
    {
        std::cerr << "无法打开图像文件" << std::endl;
        return -1;
    }

    // 获取当前时间点作为开始时间
    auto start = std::chrono::high_resolution_clock::now();

    std::vector<cv::Point> left_dead_line{cv::Point(92, 359),
                                          cv::Point(0, 148)};
    std::vector<cv::Point> right_dead_line{cv::Point(309, 359),
                                           cv::Point(399, 148)};
    std::vector<cv::Point> path = GetFollowPath(src, left_dead_line, right_dead_line, 0.0025);
    // std::vector<cv::Point> path = getFollowPath(src);

    std::cout << "返回true，表示小车的左边存在非草地，则原地旋转: " << isNonGrassOnRight(src) << std::endl;
    ;

    // 获取当前时间点作为结束时间
    auto end = std::chrono::high_resolution_clock::now();

    // 计算时间差，单位为秒
    std::chrono::duration<double> elapsed = end - start;

    // 输出耗时，单位为秒
    std::cout << "函数执行耗时: " << elapsed.count() << " 秒" << std::endl;

    // 转换为灰度图像
    // cv::Mat gray;
    // cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);

    // 应用阈值处理，生成二值图像
    // cv::Mat binary;
    // cv::threshold(gray, binary, 128, 255, cv::THRESH_BINARY);

    // 查找轮廓
    // std::vector<std::vector<cv::Point>> contours;
    // std::vector<cv::Vec4i> hierarchy;
    // cv::findContours(binary, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

#if vis
    // 创建一张空白图像，用于绘制轮廓
    cv::Mat drawing = src.clone();

    std::cout << "path.size() =" << path.size() << std::endl;

    // 绘制所有轮廓
    for (size_t i = 0; i < path.size() - 1; i++)
    {
        cv::line(drawing, path[i], path[i + 1], cv::Scalar(0, 0, 255), 2);
    }

    // 显示结果
    cv::namedWindow("Contours", cv::WINDOW_AUTOSIZE);
    cv::imshow("Contours", drawing);
    cv::waitKey(0);

#endif

    return 0;
}
