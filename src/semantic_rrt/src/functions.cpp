#include "functions.h"


// rdm class, for gentaring random flot numbers
rdm::rdm() {i=time(0);}
float rdm::randomize() { i=i+1;  srand (i);  return float(rand())/float(RAND_MAX);}



//Norm function 
float Norm(std::vector<float> x1,std::vector<float> x2)
{
return pow(	(pow((x2[0]-x1[0]),2)+pow((x2[1]-x1[1]),2))	,0.5);
}


//sign function
float sign(float n)
{
if (n<0.0){return -1.0;}
else{return 1.0;}
}


//Nearest function
std::vector<float> Nearest(  std::vector< std::vector<float>  > V, std::vector<float>  x){

  float min=Norm(V[0],x);
  int   min_index;
  float temp;

  for (int j=0;j<V.size();j++)
  {
    temp=Norm(V[j],x);
    if (temp<=min){
      min=temp;
      min_index=j;
    }
  }
  return V[min_index];
}



//Steer function
std::vector<float> Steer(  std::vector<float> x_nearest , std::vector<float> x_rand, float eta){
  std::vector<float> x_new;
//Norm 最小二乘法
  if (Norm(x_nearest,x_rand)<=eta){
    x_new=x_rand;
  }
  else{ 
    float m=(x_rand[1]-x_nearest[1])/(x_rand[0]-x_nearest[0]);//斜率

    x_new.push_back(  (sign(x_rand[0]-x_nearest[0]))* (   sqrt( (pow(eta,2)) / ((pow(m,2))+1) )   )+x_nearest[0] );
    x_new.push_back(  m*(x_new[0]-x_nearest[0])+x_nearest[1] );//根据斜率和阈值eta计算出从起点到终点的路径段长度，并将其添加到起点横坐标上得到路径段的终点横坐标
     
  }
  return x_new;
}





//gridValue function
int gridValue(nav_msgs::OccupancyGrid &mapData,std::vector<float> Xp){

  float resolution=mapData.info.resolution;
  float Xstartx=mapData.info.origin.position.x;
  float Xstarty=mapData.info.origin.position.y;

  float width=mapData.info.width;
  std::vector<signed char> Data=mapData.data;

  //returns grid value at "Xp" location
  //map data:  100 occupied      -1 unknown       0 free
  float indx=(  floor((Xp[1]-Xstarty)/resolution)*width)+( floor((Xp[0]-Xstartx)/resolution) );

  return Data[int(indx)];
}



// ObstacleFree function-------------------------------------
// rrt cannot grow through obstacle or unknown
char ObstacleFree(std::vector<float> xnear, std::vector<float> &xnew, nav_msgs::OccupancyGrid mapsub){
  float resolution = mapsub.info.resolution;
  float rez=resolution*.2;
  float stepz=int(ceil(Norm(xnew,xnear))/rez); 
  std::vector<float> xi=xnear;
  char  obs=0; char unk=0; char out=0;
  geometry_msgs::Point p;
  enum enumType {Free='0', Frontier='2', Obstacle='1'};
  for (int c=0;c<stepz;c++){
    xi = Steer(xi,xnew,rez);
    if (gridValue(mapsub, xi) == 100){return enumType::Obstacle; }
    if (gridValue(mapsub, xi) == -1) { xnew=xi; return enumType::Frontier;}
    
    // if (gridValue(mapsub, xi) == -1) {
    //   int number_passable = 0;
    //   xi[0] = xi[0] + resolution;
    //   if (gridValue(mapsub, xi) == 0){number_passable++;}
    //   xi[1] = xi[1] + resolution;
    //   if (gridValue(mapsub, xi) == 0){number_passable++;}
    //   xi[1] = xi[1] - 2*resolution;
    //   if (gridValue(mapsub, xi) == 0){number_passable++;}
    //   xi[0] = xi[0] - 2*resolution;
    //   if (gridValue(mapsub, xi) == 0){number_passable++;}
    //   if(number_passable>1){xnew=xi; return enumType::Frontier;}
    //   else{return enumType::Obstacle;}
    //   }
  }
  return enumType::Free;
}//用于检查从一个点到另一个点的路径是否无障碍     函数首先从 mapsub 对象中获取地图的分辨率，并将其存储在 resolution 变量中。然后，计算出步长 rez，该步长是分辨率的 0.2 倍。接下来，根据 xnew 和 xnear 的欧氏距离除以步长取整，得到变量 stepz，表示从 xnear 到 xnew 需要经过的步数。
// xnear 和 xnew 之间的路径是通过直线移动来实现


std::vector<float> choose_end_point(std::vector<float> x_rand,std::vector<float> x_detect)
{
      // 使用静态变量记录调用次数和选择的向量
    static int count = 0;
    static std::vector<float> selected_vector;

    // 判断输入向量是否为空
    if (x_rand.empty()) {
        return x_detect; // 返回空向量
    }
    if (x_detect.empty()){
        return x_rand;
    }
    if (x_rand.empty() && x_detect.empty()) {
        return std::vector<float>(); // 返回空向量
    } 
    // 根据调用次数选择向量并生成随机索引
    if (count < 180) {
        selected_vector = x_rand;
        count++;
    } else if(count ==180){
        selected_vector = x_detect;
        count++;
        if (count > 179) {
            count = 0;
        }
    }

    // 返回具有随机选择的元素的结果向量

    return selected_vector;
}

bool isPointInRange(const visualization_msgs::Marker& lines_data, const geometry_msgs::Point& point,const geometry_msgs::Point& position)
{
    // 定义范围边界点的坐标
    float xmin = static_cast<float>(lines_data.points[0].x);
    float xmax = static_cast<float>(lines_data.points[0].x);
    float ymin = static_cast<float>(lines_data.points[0].y);
    float ymin2 = std::numeric_limits<float>::max();
    float ymax2 = std::numeric_limits<float>::lowest();
    float ymax = static_cast<float>(lines_data.points[0].y);
    float xmax2 = std::numeric_limits<float>::lowest();
    float xmin2 = std::numeric_limits<float>::max();


    // 遍历所有线段，找到范围边界点的坐标
    for (int i = 1; i < lines_data.points.size() ; i++)
    {

        // 更新范围边界点的坐标
        // xmin = std::min(xmin, std::min(p1.x, p2.x));
        // xmax = std::max(xmax, std::max(p1.x, p2.x));
        // ymin = std::min(ymin, std::min(p1.y, p2.y));
        // ymin2 = std::min(ymin2, std::max(static_cast<int>((p1.y)+position.y), static_cast<int>((p2.y)+position.y)));
        // ymax2 = std::max(ymax2, std::min(static_cast<int>((p1.y)+position.y), static_cast<int>((p2.y)+position.y)));
        // // ymax = std::max(ymax, std::max(p1.y, p2.y));
        // xmax2 = std::max(xmax2, std::min(static_cast<int>((p1.x)+position.x), static_cast<int>((p2.x)+position.x)));
        // xmin2 = std::min(xmin2, std::max(static_cast<int>((p1.x)+position.x), static_cast<int>((p2.x)+position.x)));
        if (static_cast<float>(lines_data.points[i].x)> xmax) {
            xmax2 = xmax;
            xmax = static_cast<float>(lines_data.points[i].x);
        } else if (static_cast<float>(lines_data.points[i].x) > xmax2 && static_cast<float>(lines_data.points[i].x) != xmax) {
            xmax2 = static_cast<float>(lines_data.points[i].x);
        }
        if (static_cast<float>(lines_data.points[i].x) < xmin) {
            xmin2 = xmin;
            xmin = static_cast<float>(lines_data.points[i].x);
        } else if (static_cast<float>(lines_data.points[i].x) < xmin2 && static_cast<float>(lines_data.points[i].x) != xmin) {
            xmin2 = static_cast<float>(lines_data.points[i].x);
        }
        // 寻找第二大和第二小的y坐标
        if (static_cast<float>(lines_data.points[i].y) > ymax) {
            ymax2 = ymax;
            ymax = static_cast<float>(lines_data.points[i].y);
        } else if (static_cast<float>(lines_data.points[i].y) > ymax2 && static_cast<float>(lines_data.points[i].y) != ymax) {
            ymax2 = static_cast<float>(lines_data.points[i].y);
        }

        if (static_cast<float>(lines_data.points[i].y) < ymin) {
            ymin2 = ymin;
            ymin = static_cast<float>(lines_data.points[i].y);
        } else if (static_cast<float>(lines_data.points[i].y) < ymin2 && static_cast<float>(lines_data.points[i].y) != ymin) {
            ymin2 = static_cast<float>(lines_data.points[i].y);
        }
    }
    xmin2 = static_cast<float>(static_cast<int>((xmin2 + position.x) * 100) / 100.0);
    xmax2 = static_cast<float>(static_cast<int>((xmax2 + position.x) * 100) / 100.0);
    ymin2 = static_cast<float>(static_cast<int>((ymin2 + position.y) * 100) / 100.0);
    ymax2 = static_cast<float>(static_cast<int>((ymax2 + position.y) * 100) / 100.0);
    ymax = static_cast<float>(static_cast<int>((ymax + position.y) * 100) / 100.0);
    ymin = static_cast<float>(static_cast<int>((ymin + position.y) * 100) / 100.0);

    // std::cout<<"xmin2:"<<xmin2<<"ymin2:"<<ymin2<<"xmax2:"<<xmax2<<"ymax2:"<<ymax2<<"    froniter:"<<point.x<<" , "<<point.y<<std::endl;

    // 判断点是否在范围内
    if (static_cast<float>(point.x) >= xmin  && static_cast<float>(point.x) <= xmax  &&
        static_cast<float>(point.y) >= ymin && static_cast<float>(point.y) <= ymax )
    {
        // 点在范围内
        return true;
    }
    else
    {
        // 点不在范围内
        return false;
    }
}