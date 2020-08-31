#include "path.h"

using namespace HybridAStar;


//###################################################
//                                         CLEAR PATH
//###################################################

void Path::clear() {
  Node3D node;
  path.poses.clear();
  pathNodes.markers.clear();
  pathVehicles.markers.clear();
  addNode(node, 0);
  addVehicle(node, 1);

  pathParking.enable_mode = 0;  //enable_mode =1 给控制节点发路点 =0 不发
  pathParking.speedlimit = 0;
  // publishPath();
  // publishPathNodes();
  // publishPathVehicles();
  // publishPathParking();
}

////###################################################
////                                         TRACE PATH
////###################################################
//// __________
//// TRACE PATH
//void Path::tracePath(const Node3D* node, int i) {
//  if (i == 0) {
//    path.header.stamp = ros::Time::now();
//  }

//  if (node == nullptr) { return; }

//  addSegment(node);
//  addNode(node, i);
//  i++;
//  addVehicle(node, i);
//  i++;

//  tracePath(node->getPred(), i);
//}

//###################################################
//                                         TRACE PATH
//###################################################
// __________
// TRACE PATH
void Path::updatePath(std::vector<Node3D> nodePath, Node3D nStart) {
  path.header.stamp = ros::Time::now();
  int k = 0;

  for (size_t i = 0; i < nodePath.size(); ++i) {
    addSegment(nodePath[i]);
    addNode(nodePath[i], k);
    k++;
    addVehicle(nodePath[i], k);
    k++;
    if(i<301) addPathParking(nodePath[nodePath.size()-i-1], i, nStart); //将路点转换至车载坐标系
  }
  // std::cout <<"node num:" <<nodePath.size() <<std::endl;

  if(nodePath.size()<301)                //路点少于301个，补齐
  for(size_t i = nodePath.size(); i <301; i++)
  {
    addPathParking(nodePath[0], i, nStart);
  }

  pathParking.enable_mode = 1; //给控制节点发路点
  pathParking.header.frame_id = "path";
  for(int i=1;i<10;i++)
  {
    if(pathParking.y[i]==0) continue;
    else
    {
      if(pathParking.y[i]<0) pathParking.speedlimit = -6.0;  // 后退速度限制
      else pathParking.speedlimit = 6.0;                    // 前进速度限制
      break;
    }
  }
  if(nodePath.size()<10) pathParking.speedlimit = 0.0;


  return;
}
// ___________
// ADD SEGMENT
void Path::addSegment(const Node3D& node) {
  geometry_msgs::PoseStamped vertex;
  vertex.pose.position.x = node.getX() * Constants::cellSize;
  vertex.pose.position.y = node.getY() * Constants::cellSize;
  vertex.pose.position.z = 0;
  vertex.pose.orientation.x = 0;
  vertex.pose.orientation.y = 0;
  vertex.pose.orientation.z = 0;
  vertex.pose.orientation.w = 0;
  path.poses.push_back(vertex);
}

// ________
// ADD NODE
void Path::addNode(const Node3D& node, int i) {
  visualization_msgs::Marker pathNode;

  // delete all previous markers
  if (i == 0) {
    pathNode.action = 3;
  }

  pathNode.header.frame_id = "path";
  pathNode.header.stamp = ros::Time(0);
  pathNode.id = i;
  pathNode.type = visualization_msgs::Marker::SPHERE;
  pathNode.scale.x = 0.1;
  pathNode.scale.y = 0.1;
  pathNode.scale.z = 0.1;
  pathNode.color.a = 1.0;

  if (smoothed) {
    pathNode.color.r = Constants::pink.red;
    pathNode.color.g = Constants::pink.green;
    pathNode.color.b = Constants::pink.blue;
  } else {
    pathNode.color.r = Constants::purple.red;
    pathNode.color.g = Constants::purple.green;
    pathNode.color.b = Constants::purple.blue;
  }

  pathNode.pose.position.x = node.getX() * Constants::cellSize;
  pathNode.pose.position.y = node.getY() * Constants::cellSize;
  pathNodes.markers.push_back(pathNode);
}

// ________
//ADD VEHICLE
void Path::addVehicle(const Node3D& node, int i) {
  visualization_msgs::Marker pathVehicle;

  // delete all previous markersg
  if (i == 1) {
    pathVehicle.action = 3;
  }

  pathVehicle.header.frame_id = "path";
  pathVehicle.header.stamp = ros::Time(0);
  pathVehicle.id = i;
  pathVehicle.type = visualization_msgs::Marker::CUBE;
  pathVehicle.scale.x = Constants::length - Constants::bloating * 2;
  pathVehicle.scale.y = Constants::width - Constants::bloating * 2;
  pathVehicle.scale.z = 1;
  pathVehicle.color.a = 0.1;

  if (smoothed) {
    pathVehicle.color.r = Constants::orange.red;
    pathVehicle.color.g = Constants::orange.green;
    pathVehicle.color.b = Constants::orange.blue;
  } else {
    pathVehicle.color.r = Constants::teal.red;
    pathVehicle.color.g = Constants::teal.green;
    pathVehicle.color.b = Constants::teal.blue;
  }

  pathVehicle.pose.position.x = node.getX() * Constants::cellSize;
  pathVehicle.pose.position.y = node.getY() * Constants::cellSize;
  pathVehicle.pose.orientation = tf::createQuaternionMsgFromYaw(node.getT());
  pathVehicles.markers.push_back(pathVehicle);
}


void Path::addPathParking(const Node3D& node, int i, Node3D nStart) {
  float theta = nStart.getT() + 1.570796;
  // float theta = nStart.getT()
  std::cout <<theta <<std::endl;

  float x = (node.getX() - nStart.getX()) * Constants::cellSize;
  float y = (node.getY() - nStart.getY()) * Constants::cellSize;
  
  float x_new = -x * cos(theta) - y * sin(theta); //计算路点在车载坐标系下的位置坐标（旋转变换）
  float y_new = x * sin(theta) - y * cos(theta);
  
  std::cout <<"x:" <<x <<"y:" <<y <<std::endl;
  std::cout <<"x_new:" <<x_new <<"y_new:" <<y_new <<std::endl;

  pathParking.x[i] = round(x_new/0.02);
  pathParking.y[i] = round(y_new/0.02);
}

