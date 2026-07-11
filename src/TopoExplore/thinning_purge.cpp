/**
 * Code for thinning a binary image using Zhang-Suen algorithm.
 *
 * Author:  Nash (nash [at] opencv-code [dot] com) 
 * Website: http://opencv-code.com
 */
#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>  

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


/**
 * This is an example on how to call the thinning funciton above
 */
int main()
{   
   
	cv::Mat src = cv::imread("/home/oseasy/topological_map/maps2.pgm");
	if (!src.data)
		return -1;

	cv::imshow("src", src);
	cv::Mat gray;
    cv::cvtColor(src, gray, CV_BGR2GRAY);
    cv::Mat binary;
    cv::Mat filtered;
    cv::threshold(gray, binary,225, 255, CV_THRESH_BINARY);
    cv::imshow("binary", binary);


    // 腐蚀操作
    cv::Mat eroded;
    cv::erode(binary, eroded, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));
        // 膨胀操作
    cv::Mat dilated;
    cv::dilate(eroded, dilated, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));

    // 获得滤波后的图像
    filtered = dilated.clone();

    cv::imshow("filtered ", filtered);
    cv::Mat skeleton;
    thinning(filtered, skeleton);
    cv::imshow("skeleton", skeleton);
        // Create a blank image to draw vertices and edges
    cut(skeleton);
    cv::imshow("removed1", skeleton);
    skeleton /= 255;
	std::vector<cv::Point> points = getPoints(skeleton,6,20);
    skeleton *= 255;
    // mergePoints(points, 35); // 合并半径小于15的相邻交点
    // std::cout << "Number of points: " << points.size() << std::endl;

    // for (int i = 0; i < points.size(); i++) {
    //     circle(skeleton, points[i], 4, 255, 1);
    // }

    // cv::imshow("removed2", skeleton);

    cv::Mat result;
    cv::cvtColor(skeleton, skeleton, CV_GRAY2BGR); // 将removed转换为3通道的灰度图像
    cv::addWeighted(src, 0.7, skeleton, 0.3, 0, result);
    cv::imshow("result", result);

	cv::waitKey();
	return 0;
}

