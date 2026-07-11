/**
 * Code for thinning a binary image using Zhang-Suen algorithm.
 *
 * Author:  Nash (nash [at] opencv-code [dot] com) 
 * Website: http://opencv-code.com
 */
#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>  
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/MapMetaData.h"
#include "ros/ros.h"
#include <mutex>
#include <string>
#include "visualization_msgs/Marker.h"
/**
 * Perform one thinning iteration.
 * Normally you wouldn't call this function directly from your code.
 *
 * Parameters:
 * 		im    Binary image with range = [0,1]
 * 		iter  0=even, 1=odd
 */
typedef std::pair<int, int> Point;
using namespace cv;
using namespace std ;
std::mutex _mt;


void thinningIteration(cv::Mat& img, int iter)
{
    CV_Assert(img.channels() == 1);
    CV_Assert(img.depth() != sizeof(uchar));
    CV_Assert(img.rows > 3 && img.cols > 3);

    cv::Mat marker = cv::Mat::zeros(img.size(), CV_8UC1);  //CV_8UC1   8U：代表每个像素元素是一个无符号的 8 位整数（即 uchar 类型），范围在 0 到 255 之间。 C1：代表图像是单通道的，即灰度图像。

    int nRows = img.rows;
    int nCols = img.cols;

    if (img.isContinuous()) {
        nCols *= nRows;  //图像的总像素数
        nRows = 1;  //将整个图像展开为一行，并且后续的处理可以方便地在这一行上进行。
    }

    int x, y;
    uchar *pAbove;
    uchar *pCurr;
    uchar *pBelow;
    uchar *nw, *no, *ne;    // north (pAbove)
    uchar *we, *me, *ea;
    uchar *sw, *so, *se;    // south (pBelow)

    uchar *pDst;

    // initialize row pointers
    pAbove = NULL;
    pCurr  = img.ptr<uchar>(0);
    pBelow = img.ptr<uchar>(1);

    for (y = 1; y < img.rows-1; ++y) {
        // shift the rows up by one
        pAbove = pCurr;
        pCurr  = pBelow;
        pBelow = img.ptr<uchar>(y+1);

        pDst = marker.ptr<uchar>(y);

        // initialize col pointers
        no = &(pAbove[0]);
        ne = &(pAbove[1]);
        me = &(pCurr[0]);
        ea = &(pCurr[1]);
        so = &(pBelow[0]);
        se = &(pBelow[1]);

        for (x = 1; x < img.cols-1; ++x) {
            // shift col pointers left by one (scan left to right)
            nw = no;
            no = ne;
            ne = &(pAbove[x+1]);
            we = me;
            me = ea;
            ea = &(pCurr[x+1]);
            sw = so;
            so = se;
            se = &(pBelow[x+1]);

            int A  = (*no == 0 && *ne == 1) + (*ne == 0 && *ea == 1) + 
                     (*ea == 0 && *se == 1) + (*se == 0 && *so == 1) + 
                     (*so == 0 && *sw == 1) + (*sw == 0 && *we == 1) +
                     (*we == 0 && *nw == 1) + (*nw == 0 && *no == 1);
            int B  = *no + *ne + *ea + *se + *so + *sw + *we + *nw;
            int m1 = iter == 0 ? (*no * *ea * *so) : (*no * *ea * *we);
            int m2 = iter == 0 ? (*ea * *so * *we) : (*no * *so * *we);

            if (A == 1 && (B >= 2 && B <= 6) && m1 == 0 && m2 == 0)
                pDst[x] = 1;
        }
    }

    img &= ~marker;
}


/**
 * Function for thinning the given binary image
 *
 * Parameters:
 * 		src  The source image, binary with range = [0,255]
 * 		dst  The destination image
 */
void thinning(const cv::Mat& src, cv::Mat& dst)
{
    dst = src.clone();
    dst /= 255;         // convert to binary image

    cv::Mat prev = cv::Mat::zeros(dst.size(), CV_8UC1);
    cv::Mat diff;

    do {
        thinningIteration(dst, 0);
        thinningIteration(dst, 1);
        cv::absdiff(dst, prev, diff);
        dst.copyTo(prev);
    } 
    while (cv::countNonZero(diff) > 0);    //diff保存的是这一次和上一次迭代的差值，如果为0则意味着本次迭代没有任何改动，可以结束了

    dst *= 255;
}

void cut(cv::Mat& skeleton)
{
int nRows = skeleton.rows;
int nCols = skeleton.cols;

if (skeleton.isContinuous()) {
    nCols *= nRows;
    nRows = 1;
}

int x, y;
uchar *pCurr;
uchar *nw, *no, *ne;    // north (pAbove)
uchar *we, *me, *ea;
uchar *sw, *so, *se;    // south (pBelow)
skeleton /= 255;         // convert to binary image
for (y = 1; y < skeleton.rows-1; ++y) {
    pCurr  = skeleton.ptr<uchar>(y);

    no = &(pCurr[-skeleton.step]);  // -step是因为我们从上向下遍历像素
    ne = &(pCurr[-skeleton.step+1]);
    me = &(pCurr[0]);
    ea = &(pCurr[1]);
    so = &(pCurr[skeleton.step]);
    se = &(pCurr[skeleton.step+1]);

    for (x = 1; x < skeleton.cols-1; ++x) {
        nw = no;
        no = ne;
        ne = &(pCurr[-skeleton.step+x+1]);
        we = me;
        me = ea;
        ea = &(pCurr[x+1]);
        sw = so;
        so = se;
        se = &(pCurr[skeleton.step+x+1]);

        // 如果该顶点的周围没有连接边，则将其删除
        if (*me == 1 && (*no + *ne + *we + *ea + *so + *sw + *nw + *se == 0))
            *me = 0;

        else if(*me == 1 && (*no + *ne + *we + *ea + *so + *sw + *nw + *se == 8))
            *me = *no = *ne = *we = *ea = *so = *sw = *nw =*se=0;
 
        
    }
}
 skeleton *= 255;

}
std::vector<cv::Point> find_vertexs( cv::Mat& skeleton)
{
    std::vector<cv::Point> vertices;


int nRows = skeleton.rows;
int nCols = skeleton.cols;

if (skeleton.isContinuous()) {
    nCols *= nRows;
    nRows = 1;
}

int x, y;
uchar *pCurr;
uchar *nw, *no, *ne;    // north (pAbove)
uchar *we, *me, *ea;
uchar *sw, *so, *se;    // south (pBelow)


    for (y = 1; y < skeleton.rows-1; ++y) {
        pCurr = skeleton.ptr<uchar>(y);
        no = &(pCurr[-skeleton.step]);  // -step是因为我们从上向下遍历像素
        ne = &(pCurr[-skeleton.step+1]);
        me = &(pCurr[0]);
        ea = &(pCurr[1]);
        so = &(pCurr[skeleton.step]);
        se = &(pCurr[skeleton.step+1]);
        for (x = 1; x < skeleton.cols-1; ++x) {
            nw = no;
            no = ne;
            ne = &(pCurr[-skeleton.step+x+1]);
            we = me;
            me = ea;
            ea = &(pCurr[x+1]);
            sw = so;
            so = se;
            se = &(pCurr[skeleton.step+x+1]);
            if(*me != 0){

            if ( *we != 0 && *no != 0 && *ea !=0) {
                vertices.push_back(cv::Point(x, y));
            }
            if (*we != 0 && *so != 0 && *ea !=0) {
                vertices.push_back(cv::Point(x, y));
            }
            if (*ea != 0 && *no != 0 && *so !=0) {
                vertices.push_back(cv::Point(x, y));
            }
            if ( *we != 0 && *no != 0 && *so !=0) {
                vertices.push_back(cv::Point(x, y));
            }
            if (*se!= 0 && *sw != 0 && *no != 0) {
                 vertices.push_back(cv::Point(x, y));
             }
            if (*so!= 0 && *nw != 0 && *ne != 0) {
                 vertices.push_back(cv::Point(x, y));
             }
             if (*we!= 0 && *se != 0 && *ne != 0) {
                 vertices.push_back(cv::Point(x, y));
             }
             if (*ea!= 0 && *nw != 0 && *sw != 0) {
                 vertices.push_back(cv::Point(x, y));
             }
 
        }
    }
    }
    return vertices;
}

//只要是像素值为1的像素都纳入点集
std::vector<cv::Point> findPixels(const cv::Mat& binaryImage)
{
    std::vector<cv::Point> points;

    for (int y = 0; y < binaryImage.rows; ++y) {
        for (int x = 0; x < binaryImage.cols; ++x) {
            if (binaryImage.at<uchar>(y, x) == 255) {
                points.push_back(cv::Point(x, y));
            }
        }
    }

    return points;
}

std::vector<cv::Point> getPoints(const cv::Mat &thinSrc, unsigned int raudis , unsigned int thresholdMax)
{
	assert(thinSrc.type() == CV_8UC1);
	int width = thinSrc.cols;
	int height = thinSrc.rows;
	cv::Mat tmp;
	thinSrc.copyTo(tmp);
	std::vector<cv::Point> points;
	for (int i = 0; i < height; ++i)
	{
		for (int j = 0; j < width; ++j)
		{
			if (*(tmp.data + tmp.step * i + j) == 0)
			{
				continue;
			}
			int count=0;
			for (int k = i - raudis; k < i + raudis+1; k++)
			{
				for (int l = j - raudis; l < j + raudis+1; l++)
				{
					if (k < 0 || l < 0||k>height-1||l>width-1)
					{
						continue;
						
					}
					else if (*(tmp.data + tmp.step * k + l) == 1)
					{
						count++;
					}
				}
			}
 
			if (count > thresholdMax )
			{
				cv::Point point(j, i);
				points.push_back(point);
			}
		}
	}
	return points;
}
double distance(cv::Point p1, cv::Point p2)
{
    return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
}
void mergePoints(std::vector<cv::Point>& points, int threshold)
{
    std::vector<cv::Point> mergedPoints;
    while (!points.empty()) {
        cv::Point p = points.back();
        points.pop_back();

        std::vector<cv::Point> closePoints;
        closePoints.push_back(p);

        for (int i = 0; i < points.size(); i++) {
            if (distance(p, points[i]) < threshold) {
                closePoints.push_back(points[i]);
                points.erase(points.begin() + i);
                i--;
            }
        }

        int sumX = 0, sumY = 0;
        for (int i = 0; i < closePoints.size(); i++) {
            sumX += closePoints[i].x;
            sumY += closePoints[i].y;
        }

        cv::Point center(sumX / closePoints.size(), sumY / closePoints.size());

        mergedPoints.push_back(center);
    }

    points = mergedPoints;
}

cv::Mat eroded_dilated(cv::Mat& src)
{

    cv::Mat binary;
    cv::Mat filtered;
    // cv::Mat gray;
    // cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);
    cv::threshold(src, binary,225, 255, cv::THRESH_BINARY);
    cv::Mat eroded;
    cv::erode(binary, eroded, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));
    cv::Mat dilated;
    cv::dilate(eroded, dilated, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));
    filtered = dilated.clone();
    cv::Mat skeleton;
    thinning(filtered, skeleton);
    cut(skeleton);

    return skeleton;
}

ros::Publisher skeleton_publisher_;
ros::Publisher marker_publisher_;
ros::Publisher marker_publisher_2;
void occupancyGridCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    std::cout<<"i can in the occupancyGridCallback"<<endl;
    // Convert the OccupancyGrid message to an OpenCV image
    cv::Mat map(msg->info.height, msg->info.width, CV_8UC1);
    for (int row = 0; row < msg->info.height; ++row) {
        for (int col = 0; col < msg->info.width; ++col) {
            int8_t value = msg->data[row * msg->info.width + col];
            if (value == -1) {
                map.at<uchar>(row, col) = 200;
            }
            if (value == 0)
            {
                map.at<uchar>(row, col) = 255;
            }
            if (value == 100)
            {
                map.at<uchar>(row, col) = 0;
            }
        }
    }
    // def fromOccupancyGridToImg(occupancyGrid):
    // gridMap = MyGridMap(occupancyGrid)
    // map = np.zeros((gridMap.mapHeight, int(gridMap.mapWidth)))
    // height, width = map.shape
    // for i in range(height):
    //     for j in range(width):
    //         val = gridMap.getData(j, i)
    //         if val == 0:
    //             map[i,j] = 255
    //         if val == -1:
    //             map[i,j] = 200
    //         if val == 100:
    //             map[i,j] = 0
    // map = np.array(map)
    // map = (map).astype(np.uint8)
    // return map
    cv::Mat skeleton = eroded_dilated(map);
    ROS_INFO("Success3");
    std::vector<cv::Point>  vertexs = find_vertexs(skeleton);
    // for (int i = 0; i < vertexs.size(); i++) {
    //      circle(skeleton, vertexs[i], 3, 220, 1); // 红色圆，只使用BGR通道的第一个通道，其余通道值为0
    // }
    std::vector<cv::Point> Points = findPixels(skeleton);
    // for (int i = 0; i < vertexs.size(); i++) {
    //      circle(skeleton, vertexs[i], 2, 255, 1); 
    // }

    visualization_msgs::Marker marker;
    marker.header.frame_id = msg->header.frame_id;  // 设置参考坐标系
    marker.header.stamp = ros::Time::now();
    marker.ns = "skeleton_points";
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.scale.x = 0.1;  // 点的大小
    marker.scale.y = 0.1;
    marker.color.g = 1.0;  // 点的颜色
    marker.color.a = 1.0;

    // 填充点坐标
    for (const auto& point : Points ){
        geometry_msgs::Point p;
        p.x = point.x * msg->info.resolution + msg->info.origin.position.x;
        p.y = point.y * msg->info.resolution + msg->info.origin.position.y;
        p.z = 0;
        marker.points.push_back(p);
    }

    // 发布 Marker 消息到 skeleton_points 话题
    marker_publisher_.publish(marker);

    visualization_msgs::Marker marker2;
    marker2.header.frame_id = msg->header.frame_id;
    marker2.header.stamp = ros::Time::now();
    marker2.ns = "skeleton_vertexs";
    marker2.action = visualization_msgs::Marker::ADD;
    marker2.pose.orientation.w = 1.0;
    marker2.type = visualization_msgs::Marker::POINTS;
    marker2.scale.x = 0.3;
    marker2.scale.y = 0.3;
    marker2.color.r = 1.0;
    marker2.color.a = 1.0;

    for (const auto& point : vertexs) {
        geometry_msgs::Point p1;
        p1.x = point.x * msg->info.resolution + msg->info.origin.position.x;
        p1.y = point.y * msg->info.resolution + msg->info.origin.position.y;
        p1.z = 0;
        marker2.points.push_back(p1);
    }

    marker_publisher_2.publish(marker2);

    // Publish the skeleton as an OccupancyGrid message
    nav_msgs::OccupancyGrid skeletonMsg;
    skeletonMsg.header.stamp = ros::Time::now();
    skeletonMsg.header.frame_id = msg->header.frame_id;
    skeletonMsg.info = msg->info;
    skeletonMsg.data.resize(skeleton.total());
    for (int row = 0; row < msg->info.height; ++row) {
        for (int col = 0; col < msg->info.width; ++col) {
            u_int8_t value = skeleton.at<uchar>(row, col);
            if (value == 0) {
                skeletonMsg.data[row * msg->info.width + col] = 206;
            } 
            else if(value == 220){
              skeletonMsg.data[row * msg->info.width + col] = 150;
            }
            else{
              skeletonMsg.data[row * msg->info.width + col] = 200;
            }
        }
    }
    ROS_INFO("Success4");
  skeleton_publisher_.publish(skeletonMsg);

}

/**
 * This is an example on how to call the thinning funciton above
 */
int main(int argc, char **argv)
{   
    ros::init(argc, argv, "thinning_purge");
    std::string map_topic;
    ros::NodeHandle nh;
    ros::param::param<std::string>("/map", map_topic, "/map");
    ros::Subscriber map_subscriber_ = nh.subscribe("/map", 1, &occupancyGridCallback);
    ROS_INFO("Success");
    skeleton_publisher_ = nh.advertise<nav_msgs::OccupancyGrid>("skeleton_map", 1); 
    marker_publisher_ = nh.advertise<visualization_msgs::Marker>("skeleton_points", 1);
    marker_publisher_2 = nh.advertise<visualization_msgs::Marker>("skeleton_vertexs", 1);
    ROS_INFO("Success2");
    ros::spin();  
	// 
	return 0;
}

// cv::Mat src = cv::imread("/home/oseasy/topological_map/maps2.pgm");
	// if (!src.data)
	// 	return -1;

	// cv::imshow("src", src);
	// cv::Mat gray;
    // cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);
    // cv::Mat binary;
    // cv::Mat filtered;
    // cv::threshold(gray, binary,225, 255, cv::THRESH_BINARY);
    // cv::imshow("binary", binary);


    // // 腐蚀操作
    // cv::Mat eroded;
    // cv::erode(binary, eroded, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));
    //     // 膨胀操作
    // cv::Mat dilated;
    // cv::dilate(eroded, dilated, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));

    // // 获得滤波后的图像
    // filtered = dilated.clone();

    // cv::imshow("filtered ", filtered);
    // cv::Mat skeleton;
    // thinning(filtered, skeleton);
    // cv::imshow("skeleton", skeleton);
    //     // Create a blank image to draw vertices and edges
    // cut(skeleton);
    // cv::imshow("removed1", skeleton);
    // skeleton /= 255;
	// std::vector<cv::Point> points = getPoints(skeleton,6,20);
    // skeleton *= 255;
    // // mergePoints(points, 35); // 合并半径小于15的相邻交点
    // // std::cout << "Number of points: " << points.size() << std::endl;

    // // for (int i = 0; i < points.size(); i++) {
    // //     circle(skeleton, points[i], 4, 255, 1);
    // // }

    // // cv::imshow("removed2", skeleton);

    // cv::Mat result;
    // cv::cvtColor(skeleton, skeleton, cv::COLOR_GRAY2BGR); // 将removed转换为3通道的灰度图像
    // cv::addWeighted(src, 0.7, skeleton, 0.3, 0, result);
    // cv::imshow("result", result);

	// cv::waitKey();
