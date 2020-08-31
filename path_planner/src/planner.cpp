

#include "planner.h"
#include <nav_msgs/OccupancyGrid.h>
//#include <image_transport/image_transport.h>
//#include <opencv-3.3.1-dev/opencv2/highgui.hpp>
//#include <cv_bridge/cv_bridge.h>
//#include <opencv-3.3.1-dev/opencv2/core/mat.hpp>
//#include <opencv-3.3.1-dev/opencv2/imgcodecs.hpp>


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
  pubGoal = n.advertise<geometry_msgs::PoseStamped>("/defined_goal", 1);
  //pubMap = n.advertise<nav_msgs::OccupancyGrid>("/map", 1);

  // ___________________
  // TOPICS TO SUBSCRIBE
  if (Constants::manual) {
    subMap = n.subscribe("/map", 1, &Planner::setMap, this);
  } else {
    subMap = n.subscribe("/occ_map", 1, &Planner::setMap, this);
  }

//  subGoal = n.subscribe("/move_base_simple/goal", 1, &Planner::setGoal, this);
  subStart = n.subscribe("/initialpose", 1, &Planner::setStart, this);
//    subStart = n.subscribe("/vehicle_pose", 1, &Planner::setStart, this);
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
  float x = initial->pose.pose.position.x / Constants::cellSize;
  float y = initial->pose.pose.position.y / Constants::cellSize;
  float t = tf::getYaw(initial->pose.pose.orientation);
  // publish the start without covariance for rviz
  geometry_msgs::PoseStamped startN;
  startN.pose.position = initial->pose.pose.position;
  startN.pose.orientation = initial->pose.pose.orientation;
  startN.header.frame_id = "map";
  startN.header.stamp = ros::Time::now();

  std::cout << "I am seeing a new start x:" << x << " y:" << y << " t:" << Helper::toDeg(t) << std::endl;

  // printf("Before start if\n");

  if (grid != NULL) {

    if (grid->info.height >= y && y >= 0 && grid->info.width >= x && x >= 0) {
      printf("After start if\n");
      validStart = true;
      start = *initial;

      // printf("After start = *initial;\n");

      if (Constants::manual) { plan();}

      // publish start for RViz
      pubStart.publish(startN);
    } else {
      // printf("Enter else\n");
      std::cout << "invalid start x:" << x << " y:" << y << " t:" << Helper::toDeg(t) << std::endl;
    }

  } else {
    std::cout << "there is no map!" << std::endl;
  }
}

//###################################################
//                                    INITIALIZE GOAL
//###################################################
//void Planner::setGoal(const geometry_msgs::PoseStamped::ConstPtr& end) {
//  // retrieving goal position
//  float x = end->pose.position.x / Constants::cellSize;
//  float y = end->pose.position.y / Constants::cellSize;
//  float t = tf::getYaw(end->pose.orientation);
//
//  std::cout << "I am seeing a new goal x:" << x << " y:" << y << " t:" << Helper::toDeg(t) << std::endl;
//
//  if (grid != NULL) {
//
//    if (grid->info.height >= y && y >= 0 && grid->info.width >= x && x >= 0) {
//      validGoal = true;
//      goal = *end;
//
//      if (Constants::manual) { plan();}
//
//    } else {
//      std::cout << "invalid goal x:" << x << " y:" << y << " t:" << Helper::toDeg(t) << std::endl;
//    }
//
//  } else {
//    std::cout << "there is no map!" << std::endl;
//  }
//}

//###################################################
//                     PUBLISH DEFINED GOAL
//###################################################

void Planner::pubDefinedGoal(const Node3D nGoal) {
    //publish the defined start and goal
    geometry_msgs::PoseStamped goalN;
    goalN.pose.position.x = nGoal.getX();
    goalN.pose.position.y = nGoal.getY();
    goalN.header.frame_id = "map";
    goalN.header.stamp = ros::Time::now();
    std::cout << "I am seeing a defined goal" << goalN << std::endl;

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
//    const Node3D nGoal(155.349, 36.1969, 0.7615936, 0, 0, nullptr);
    const Node3D nGoal(14.0, 20.0, 0.0, 0, 0, nullptr);
    validGoal = true;
//    validStart = true;
    pubDefinedGoal(nGoal);


//    cv::Mat imag = cv::imread("/home/ww/catkin_ws/src/path_planner/map_demo.png");
//    if(imag.empty()){
//        printf("open error\n");
//    }
//    nav_msgs::OccupancyGrid definedMap;
//    definedMap.info.resolution = 0.3;
//    definedMap.info.width = imag.rows;
//    definedMap.info.width = imag.clos;




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
      float x = goal.pose.position.x / Constants::cellSize;
      float y = goal.pose.position.y / Constants::cellSize;
      float t = tf::getYaw(goal.pose.orientation);
      // set theta to a value (0,2PI]
//      t = Helper::normalizeHeadingRad(t);
//      const Node3D nGoal(x, y, t, 0, 0, nullptr);
      // __________
      // DEBUG GOAL
      //    const Node3D nGoal(155.349, 36.1969, 0.7615936, 0, 0, nullptr);

      // _________________________
      // retrieving start position
      // printf("retrieving start position\n");
      x = start.pose.pose.position.x / Constants::cellSize;
      y = start.pose.pose.position.y / Constants::cellSize;
      t = tf::getYaw(start.pose.pose.orientation);
      // set theta to a value (0,2PI]
      t = Helper::normalizeHeadingRad(t);
      Node3D nStart(x, y, t, 0, 0, nullptr);

      // ___________
      // DEBUG START
      //    Node3D nStart(108.291, 30.1081, 0, 0, 0, nullptr);

      //如果前面的路点经过不可通行区域  则重新规划  isReplan=true
      //TO DO---更新地图避障重规划
      //如果发送给控制节点的路点pathParking[0]为goal时,认为可判断当前位置抵达goal
      // TO DO---set the threshold between the real position and the goal


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
              path.publishPath();
              path.publishPathNodes();
              path.publishPathVehicles();
              //path.publishPathParking();
              pubDefinedGoal(nGoal);

              smoothedPath.publishPath();
              smoothedPath.publishPathNodes();
              smoothedPath.publishPathVehicles();
              smoothedPath.publishPathParking();
              //visualization.publishNode3DCosts(nodes3D, width, height, depth);
              //visualization.publishNode2DCosts(nodes2D, width, height);


              printf("planOnce,isReplan=", isReplan, "\n");
              isReplan = false;  //初始规划一次即不再重新规划路径
              delete[] nodes3D;
              delete[] nodes2D;
          }

      } else {
          printf("updatePathNode.\nisReplan=", isReplan, "\n");
          smoothedPath.updatePath(smoother.getPath(), nStart); //更新路点，发送给control_node
          smoothedPath.publishPathParking();
      }


  } else {
      smoothedPath.clear();
      smoothedPath.publishPathParking();
      std::cout << "missing goal or start" << std::endl;
  }
}


