

#include "planner.h"
#include <nav_msgs/OccupancyGrid.h>

using namespace cv;
using namespace std;
using namespace HybridAStar;
//###################################################
//                                        CONSTRUCTOR
//###################################################
Planner::Planner() {
  // _____
  // TODOS
  //    initializeLookups();
  // Lookup::collisionLookup(collisionLookup);
  // ___________________
  // COLLISION DETECTION
  //    CollisionDetection configurationSpace;

  // _________________
  // TOPICS TO PUBLISH
  pubStart = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/start", 1);
  //pubGoal = n.advertise<geometry_msgs::PoseStamped>("/defined_goal", 1);

  // ___________________
  // TOPICS TO SUBSCRIBE
  if (Constants::manual) {
    subMap = n.subscribe("/map", 1, &Planner::setMap, this);
  } else {
    subMap = n.subscribe("/occ_map", 1, &Planner::setMap, this);
  }

  subGoal = n.subscribe("/move_base_simple/goal", 1, &Planner::setGoal, this);
  subStart = n.subscribe("/initialpose", 1, &Planner::setStart, this);


};

//###################################################
//                                       LOOKUPTABLES
//###################################################
void Planner::initializeLookups() {
  if (Constants::dubinsLookup) {
    Lookup::dubinsLookup(dubinsLookup);
  }

  Lookup::collisionLookup(collisionLookup);
}

//###################################################
//                                                MAP
//###################################################
void Planner::setMap(const nav_msgs::OccupancyGrid::Ptr map) {
  if (Constants::coutDEBUG) {
    std::cout << "I am seeing the map..." << std::endl;
  }

  //std::cout << "I am seeing the map..." << std::endl;
  grid = map;
  //update the configuration space with the current map
  configurationSpace.updateGrid(map);
  //create array for Voronoi diagram
//  ros::Time t0 = ros::Time::now();
  int height = map->info.height;
  int width = map->info.width;
  bool** binMap;
  binMap = new bool*[width];

  for (int x = 0; x < width; x++) { binMap[x] = new bool[height]; }

  for (int x = 0; x < width; ++x) {
    for (int y = 0; y < height; ++y) {
      binMap[x][y] = map->data[y * width + x] ? true : false;
    }
  }

  voronoiDiagram.initializeMap(width, height, binMap);
  voronoiDiagram.update();
  voronoiDiagram.visualize();



//  ros::Time t1 = ros::Time::now();
//  ros::Duration d(t1 - t0);
//  std::cout << "created Voronoi Diagram in ms: " << d * 1000 << std::endl;

  // plan if the switch is not set to manual and a transform is available
  if (!Constants::manual && listener.canTransform("/map", ros::Time(0), "/base_link", ros::Time(0), "/map", nullptr)) {

    listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);

    // assign the values to start from base_link
    start.pose.pose.position.x = transform.getOrigin().x();
    start.pose.pose.position.y = transform.getOrigin().y();
    tf::quaternionTFToMsg(transform.getRotation(), start.pose.pose.orientation);

    if (grid->info.height >= start.pose.pose.position.y && start.pose.pose.position.y >= 0 &&
        grid->info.width >= start.pose.pose.position.x && start.pose.pose.position.x >= 0) {
      // set the start as valid and plan
      validStart = true;
    } else  {
      validStart = false;
    }

    plan();
  }
}



//###################################################
//                                   INITIALIZE START
//###################################################
void Planner::setStart(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& initial) {
  float x = initial->pose.pose.position.x;// Constants::cellSize;
  float y = initial->pose.pose.position.y;// Constants::cellSize;
  float t = tf::getYaw(initial->pose.pose.orientation);
  // publish the start without covariance for rviz
  geometry_msgs::PoseStamped startN;
  startN.pose.position = initial->pose.pose.position;
  startN.pose.orientation = initial->pose.pose.orientation;
  startN.header.frame_id = "map";
  startN.header.stamp = ros::Time::now();

  std::cout << "I am seeing a new start x:" << x << " y:" << y << " t:" << Helper::toDeg(t) << " t-Deg:" << Helper::normalizeHeadingRad(t) << std::endl;

  // printf("Before start if\n");

  if (grid != NULL) {

    if (grid->info.height >= y && y >= 0 && grid->info.width >= x && x >= 0) {
      //printf("After start if\n");
      validStart = true;
      start = *initial;

      // printf("After start = *initial;\n");

      if (Constants::manual) { plan();}

      // publish start for RViz
      pubStart.publish(startN);
    } else {
      // printf("Enter else\n");
      std::cout << "invalid start x:" << x << " y:" << y << " t:" << Helper::toDeg(t)<< std::endl;
    }

  } else {
    std::cout << "there is no map!" << std::endl;
  }
}

//###################################################
//                                    INITIALIZE GOAL
//###################################################
void Planner::setGoal(const geometry_msgs::PoseStamped::ConstPtr& end) {
  // retrieving goal position
  float x = end->pose.position.x / Constants::cellSize;
  float y = end->pose.position.y / Constants::cellSize;
  float t = tf::getYaw(end->pose.orientation);

  std::cout << "I am seeing a new goal x:" << x << " y:" << y << " t:" << Helper::toDeg(t) << std::endl;

  if (grid != NULL) {

    if (grid->info.height >= y && y >= 0 && grid->info.width >= x && x >= 0) {
      validGoal = true;
      goal = *end;

      if (Constants::manual) { plan();}

    } else {
      std::cout << "invalid goal x:" << x << " y:" << y << " t:" << Helper::toDeg(t) << std::endl;
    }

  } else {
    std::cout << "there is no map!" << std::endl;
  }
}

//###################################################
//                     PUBLISH DEFINED GOAL
//###################################################

void Planner::pubDefinedGoal(const Node3D nGoal) {
    //publish the defined start and goal
    geometry_msgs::PoseStamped goalN;
    goalN.pose.position.x = nGoal.getX()*0.3;
    goalN.pose.position.y = nGoal.getY()*0.3;
    goalN.pose.orientation = tf::createQuaternionMsgFromYaw(nGoal.getT());
    goalN.header.frame_id = "map";
    goalN.header.stamp = ros::Time::now();
    std::cout << "I am seeing a defined goal" << std::endl;

    // publish start for RViz
    pubGoal.publish(goalN);
//    validGoal = true;
//    if (Constants::manual) { plan();}
}


//###################################################
//                                      PLAN THE PATH
//###################################################
void Planner::plan() {
    printf("Enter plan\n");

    // ___________
    // DEBUG GOAL

    //nGoal nStart 输入最终algorithm的参数的是栅格坐标=真实坐标/cell_size,rviz显示需要的坐标是真实坐标
//    const Node3D nGoal(27.0/0.3, 20.0/0.3, 4.71, 0, 0, nullptr);
//    validGoal = true;
//    Node3D nStart(0, 0, 0, 0, 0, nullptr);
//    validStart = true;
//    pubDefinedGoal(nGoal);


  // if a start as well as goal are defined go ahead and plan
  if (validStart && validGoal) {

      // ___________________________
      // LISTS ALLOWCATED ROW MAJOR ORDER
      // printf("LISTS ALLOWCATED ROW MAJOR ORDER\n");
      int width = grid->info.width;
      int height = grid->info.height;
      int depth = Constants::headings;
      int length = width * height * depth;
      // define list pointers and initialize lists
      Node3D *nodes3D = new Node3D[length]();
      Node2D *nodes2D = new Node2D[width * height]();



      // ________________________
      // retrieving goal position
      // printf("retrieving goal position\n");
      
      float t = tf::getYaw(goal.pose.orientation);
      // set theta to a value (0,2PI]
      t = Helper::normalizeHeadingRad(t);
      float x = (goal.pose.position.x - Constants::threshold_length*cos(t)) / Constants::cellSize;
      float y = (goal.pose.position.y - Constants::threshold_length*sin(t)) / Constants::cellSize;
      const Node3D nGoal(x, y, t, 0, 0, nullptr);

      // __________
      // DEBUG GOAL
      //    const Node3D nGoal(155.349, 36.1969, 0.7615936, 0, 0, nullptr);

      // _________________________
      // retrieving start position
      // printf("retrieving start position\n");
      
      t = tf::getYaw(start.pose.pose.orientation);
      // set theta to a value (0,2PI]
      t = Helper::normalizeHeadingRad(t);
      x = (start.pose.pose.position.x - Constants::threshold_length*cos(t)) / Constants::cellSize;
      y = (start.pose.pose.position.y - Constants::threshold_length*sin(t)) / Constants::cellSize;
      Node3D nStart(x, y, t, 0, 0, nullptr);  //栅格坐标

      // ___________
      // DEBUG START
      //    Node3D nStart(108.291, 30.1081, 0, 0, 0, nullptr);

      if (isReplan) {
          // ___________________________
          // START AND TIME THE PLANNING
          ros::Time t0 = ros::Time::now();
          // CLEAR THE VISUALIZATION
          visualization.clear();
          // CLEAR THE PATH
          path.clear();
          smoothedPath.clear();


          // FIND THE PATH (return 指向goal的指针或者 nullptr)
          Node3D *nSolution = Algorithm::hybridAStar(nStart, nGoal, nodes3D, nodes2D, width, height, configurationSpace,
                                                     dubinsLookup, visualization);
          cout << "nSolution:" << nSolution << endl;
          if (nSolution != nullptr) {

              //save the last nGoal for the coordinate transform of the static global path due to the dynamic map
              //goal_last= goal;

              // TRACE THE PATH
              smoother.tracePath(nSolution);
              // CREATE THE UPDATED PATH
              path.updatePath(smoother.getPath(), nStart);
              // SMOOTH THE PATH

              smoother.smoothPath(voronoiDiagram);
              // CREATE THE UPDATED PATH
              smoothedPath.updatePath(smoother.getPath(), nStart);
              ros::Time t1 = ros::Time::now();
              ros::Duration d(t1 - t0);
              std::cout << "TIME in ms: " << d * 1000 << std::endl;
              // _________________________________
              // PUBLISH THE RESULTS OF THE SEARCH
              // path.publishPath();
              // path.publishPathNodes();
//              path.publishPathVehicles();
//              path.publishPathParking();
              //pubDefinedGoal(nGoal);

              smoothedPath.publishPath();
              smoothedPath.publishPathNodes();
              smoothedPath.publishPathVehicles();
              //smoothedPath.publishPathParking(); 
              //smoothedPath.publishPathParkingNodes();
              
//              visualization.publishNode3DCosts(nodes3D, width, height, depth);
//              visualization.publishNode2DCosts(nodes2D, width, height);



              //isReplan = false;  //初始规划一次即不再重新规划路径
              
          } else {
            isReplan = true;
          }

      } else {
          //pubDefinedGoal(nGoal);
//          //地图原点是否发生变化
//          if (goal_last.pose.position.x != goal.pose.position.x || goal_last.pose.position.y - goal.pose.position.y){
//              printf("update goal!\n");
//              //abs(goal_last.pose.position.x - goal.pose.position.x) < 0.3 && abs(goal_last.pose.position.y - goal.pose.position.y) <0.3
//              x = goal.pose.position.x - goal_last.pose.position.x;
//              y = goal.pose.position.y - goal_last.pose.position.y;
////             nStart.setX(nStart.getX()+ x / Constants::cellSize);
////              nStart.setY(nStart.getY()+ y / Constants::cellSize);
//              smoothedPath.convertStaticPath(x, y, nGoal,nStart, smoother.getPath());//更新路径及路点
//
//          }
//          goal_last = goal;

          
          smoothedPath.publishPath();
          smoothedPath.publishPathNodes();
          smoothedPath.updatePathParking(smoother.getPath(), nStart); //不重规划，只更新发送给控制节点的路点
          smoothedPath.publishPathParking(); 
          //smoothedPath.publishPathParkingNodes();
          printf("updatePathNodes!------isReplan=false--------\n");


      }

      delete[] nodes3D;
      delete[] nodes2D;
  }
  else {
      smoothedPath.clear();
      smoothedPath.publishPathParking();
      std::cout << "missing goal or start" << std::endl;
  }
}


