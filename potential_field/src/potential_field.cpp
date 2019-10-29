#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include <vector>
#include <cstdlib> // Needed for rand()
#include <ctime> // Needed to seed random number generator with a time value
#include <bullet/LinearMath/btQuaternion.h> // Needed to convert rotation ...
#include <bullet/LinearMath/btMatrix3x3.h>  // ... quaternion into Euler angles
#include <cmath>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <tf2_ros/transform_listener.h> // tf2 for melodic and 18.04LTS
#include <geometry_msgs/TransformStamped.h>
struct Pose {
  double x; // in simulated Stage units
  double y; // in simulated Stage units
  double heading; // in radians
  ros::Time t; // last received time
  
  // Construct a default pose object with the time set to 1970-01-01
  Pose() : x(0), y(0), heading(0), t(0.0) {};
  
  // Process incoming pose message for current robot
  void poseCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    double roll, pitch;
    x = msg->pose.pose.position.x;
    y = msg->pose.pose.position.y;
    btQuaternion q = btQuaternion(msg->pose.pose.orientation.x, \
      msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, \
      msg->pose.pose.orientation.w);
    btScalar h = btScalar(heading);
    btScalar p = btScalar(pitch);
    btScalar r = btScalar(roll);
    btMatrix3x3(q).getEulerYPR(h, p, r);
    heading = double(h);
    t = msg->header.stamp;
    
  };
};


class PotFieldBot {
public:
  // Construst a new Potential Field controller object and hook up
  // this ROS node to the simulated robot's pose, velocity control,
  // and laser topics
  PotFieldBot(ros::NodeHandle& nh, int robotID, int n, \
      double gx, double gy) : ID(robotID), numRobots(n), \
      goalX(gx), goalY(gy), atGoal(false) {
    // Initialize random time generator
    srand(time(NULL));
    force_repulsive_x = 0.0;
    force_repulsive_y = 0.0;

    // Advertise a new publisher for the current simulated robot's
    // velocity command topic (the second argument iindicates that
    // if multiple command messages are in the queue to be sent,
    // only the last command will be sent)
    commandPub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    // Subscribe to the current simulated robot's laser scan topic and
    // tell ROS to call this->laserCallback() whenever a new message
    // is published on that topic
    laserSub = nh.subscribe("base_scan", 1, \
      &PotFieldBot::laserCallback, this);
    
    // Subscribe to each robot' ground truth pose topic
    // and tell ROS to call pose->poseCallback(...) whenever a new
    // message is published on that topic
    for (int i = 0; i < numRobots; i++) {
      pose.push_back(Pose());
    }
        for (int i = 0; i < numRobots; i++) {
 poseSubs.push_back(nh.subscribe(\
        "/base_pose_ground_truth", 1, \
        &Pose::poseCallback, &pose[i]));
    }
  };


  // Send a velocity command
  void move(double linearVelMPS, double angularVelRadPS) {
    geometry_msgs::Twist msg; // The default constructor will set all commands to 0
    msg.linear.x = linearVelMPS;
    msg.angular.z = angularVelRadPS;
    commandPub.publish(msg);
  };
  // Process incoming laser scan message
  void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    // TODO: parse laser data
    // (see http://www.ros.org/doc/api/sensor_msgs/html/msg/LaserScan.html)
    // angle of robot
    if (atGoal)
      return;
    for (int j = 0; j < numRobots; j++) {
      float angle_robot = pose[j].heading;
      force_repulsive_x = 0.0;
      force_repulsive_y = 0.0; 
      for (unsigned int i = 0; i < msg->ranges.size(); i++)
      {
        // current distance
        float dist_i = msg->ranges[i];
	// increment angle
        float angle_i = msg->angle_min + i*msg->angle_increment;
        // adjust angle so that it is range of atan2
        if (angle_i <= msg->angle_max/2 && angle_i >= msg->angle_min/2) {
          if (angle_robot < 0)
             angle_robot += 2*M_PI;
        // verify that laser data is within range
        if ((dist_i > msg->range_min/2) && (dist_i < msg->range_max/2))
        {
	  // initial Force Repulsive
	  double force_repulsive_i = 0.0;
	  // repulsive force vary depending on dist and gains
	  if (dist_i > (dsafe + epsilon) && dist_i < beta) {
	    force_repulsive_i = alpha / pow((dist_i - dsafe),2);
	  }
	   else if (dist_i < (dsafe + epsilon)) {
            force_repulsive_i = alpha / pow(epsilon,2);
	  }
	  else { 
            force_repulsive_i = 0.0;
	  }
	  double force_robot_x = force_repulsive_i * std::cos(-angle_i);
	  double force_robot_y = force_repulsive_i * std::sin(-angle_i);
          // add forces per sensor read to the repulsive force
	  force_repulsive_x += force_robot_x;
          force_repulsive_y += force_robot_y;
        }
	}
      } // inner for loop
      // force in stage coords
      double force_stage_x = (force_repulsive_x * std::cos(angle_robot) - force_repulsive_y * std::sin(angle_robot));
      double force_stage_y = (force_repulsive_x * std::sin(angle_robot) + force_repulsive_y * std::cos(angle_robot));
      if (force_stage_x != 0 && force_stage_y != 0)
      {
        force_repulsive_x = force_stage_x;
        force_repulsive_y = force_stage_y;
      }
    } // outer for loop
  };
  
  
  // Main FSM loop for ensuring that ROS messages are
  // processed in a timely manner, and also for sending
  // velocity controls to the simulated robot based on the FSM state
  void spin() {
    ros::Rate rate(30); // Specify the FSM loop rate in Hz
    int counter = 0; // counter for random force
    double sqrtDistDerivative = 0.0; // distance change over time
    double sqrtDistPrev = 0.0;  // running total of dist
    while (ros::ok()) { // Keep spinning loop until user presses Ctrl+C
      // TODO: remove demo code, compute potential function, actuate robot
    // calculate the attractive force which is dependant upon distance to goal
    ros::spinOnce();
    for (int i = 0 ; i < numRobots; i++) {
      double omega = 0.0;
      double velocity = 0.0;
      double force_attractive_angle = atan2(goalY - pose[i].y, goalX - pose[i].x);
      double sqrtDist = sqrt (pow(goalX - pose[i].x,2) + pow(goalY - pose[i].y,2));
      double force_attractive_x = gamma * sqrtDist * cos(force_attractive_angle);
      double force_attractive_y = gamma * sqrtDist * sin(force_attractive_angle);
      // add the forces
      double force_total_x = force_repulsive_x + force_attractive_x;
      double force_total_y = force_repulsive_y + force_attractive_y;

      // values
      std::cout  << "---------------------------------------\n";
      std::cout  << "--- Current Pose -- "<< pose[i].x << " " << pose[i].y << std::endl;
      std::cout  << "--- Heading -->  "<< pose[i].heading << std::endl;
      std::cout  << "--- Distance --> " << sqrtDist << std::endl;
      std::cout << "---- Force_attractive_x --> " << force_attractive_x << std::endl;
      std::cout << "---- Force_attractive_y --> " << force_attractive_y << std::endl;
      std::cout << "---- Force_repulsive_x --> " << force_repulsive_x << std::endl;
      std::cout << "---- Force_repulsive_y --> " << force_repulsive_y << std::endl;
      std::cout << "---- Force_total_x --> " << force_total_x << std::endl;
      std::cout << "---- Force_total_y --> " << force_total_y << std::endl;
      std::cout << "----- Omega --> " << omega << std::endl;
      if (sqrtDist > 0) {
        if (sqrtDist < dsafe) {
          atGoal = true;
	  std::cout << "----Mr. TurtleBot has arrived (^-^)~ " << std::endl;
	  velocity = 0.0;
          omega = 0.0;
          move(0,0);
        }
      else if (sqrtDist != 0) {
	// checking for local minima
	counter = counter + 1;
        sqrtDistPrev += sqrtDist;
	// current - average dist
        sqrtDistDerivative = (sqrtDistPrev/counter) - sqrtDist;
        std::cout << "---- counter  " << counter <<  std::endl;
        std::cout << "---- derivative  " << sqrtDistDerivative <<  std::endl;
	 if (sqrtDistDerivative <= localMinimaHigh && sqrtDistDerivative >= localMinimaLow ) {
         // add random repulsive force that increase with the counter
         // if odd then add random force to y
         std::cout << "ADDING RANDOM FORCE " << randomForce * counter << std::endl; 
         if (counter %2 != 0) {
           force_total_y += randomForce * counter;
         }
	 // add random force to x
         else {
           force_total_x += randomForce * counter;
         }
         }
        velocity = sqrt(pow(force_total_x,2) + pow(force_total_y,2));
	double angle_total = (atan2(force_total_y,force_total_x));
	std::cout << "---- Velocity --> " << velocity << "---------\n";
        std::cout  << "---- Rotation Angle -->  "<< fabs(angle_total - pose[i].heading) << std::endl;
	std::cout  << "---- Total angle -->  "<< angle_total << std::endl;
	omega = kappa*(angle_total - pose[i].heading);
        if (fabs(angle_total - pose[i].heading) > epsilon_angle) {
         // must rotate
         ros::Time rotateStartTime = ros::Time::now();
         ros::Duration rotateDuration = ros::Duration(1/gamma);
         // send move command based upon force calulated omega
	 std::cout << omega << "fsm = FSM_ROTATE\n";
	 
	 move(0,omega);
        }
        // angle is within cone so travel towards the goal
        else {
          std::cout << "fsm == FSM_MOVE_FORWARD \n";
          move(velocity, 0);
        }
	// reset values after sigmaMax reads
	if (counter >= sigmaMax){
          counter = 0;
	  sqrtDistPrev = 0.0;
	  sqrtDistDerivative = 0.0;
	}
      }
     }
    } // for loop
    ros::spinOnce(); // Need to call this function often to allow ROS to process incoming messages
    rate.sleep(); // Sleep for the rest of the cycle, to enforce the

    }  // while
  };

  // Tunable motion controller parameters
  const static double FORWARD_SPEED_MPS = 2.0;
  const static double ROTATE_SPEED_RADPS = M_PI/2;
  // potential field constants
  const static double dsafe = 1.0; // meters
  const static double alpha = 0.2;
  // attractive force scale factor
  const static double gamma = 2.5;
  // angular velocity omega scale
  const static double kappa = 1.8;
  // upper range of repulsion
  const static double beta = 2.0;
  // upper close range of increased repulsion 
  const static double epsilon = 0.05;
  const static double epsilon_angle = 0.25; // radians
  bool atGoal;
  // amount of readings for local minima 
  const static int sigma = 20;
  // upper limit of force magnitude 
  const static int sigmaMax = 200;
  // local minima range constants
  const static double localMinimaHigh = 0.1;
  const static double localMinimaLow = -0.1;
  const static double randomForce = 2.0;
protected:
  ros::Publisher commandPub; // Publisher to the current robot's velocity command topic
  ros::Subscriber laserSub; // Subscriber to the current robot's laser scan topic
  std::vector<ros::Subscriber> poseSubs; // List of subscribers to all robots' pose topics
  std::vector<Pose> pose; // List of pose objects for all robots
  int ID; // 0-indexed robot ID
  int numRobots; // Number of robots, positive value
  double goalX, goalY; // Coordinates of goal
  double force_repulsive_x;
  double force_repulsive_y;
};


int main(int argc, char **argv) {
  int robotID = -1, numRobots = 0;
  double goalX, goalY;
  bool printUsage = false;
  
  // Parse and validate input arguments
  if (argc <= 4) {
    printUsage = true;
  } else {
    try {
      robotID = boost::lexical_cast<int>(argv[1]);
      numRobots = boost::lexical_cast<int>(argv[2]);
      goalX = boost::lexical_cast<double>(argv[3]);
      goalY = boost::lexical_cast<double>(argv[4]);

      if (robotID < 0) { printUsage = true; }
      if (numRobots <= 0) { printUsage = true; }
    } catch (std::exception err) {
      printUsage = true;
    }
  }
  if (printUsage) {
    std::cout << "Usage: " << argv[0] << " [ROBOT_NUM_ID] [NUM_ROBOTS] [GOAL_X] [GOAL_Y]" << std::endl;
    return EXIT_FAILURE;
  }
  ros::init(argc, argv, "potfieldbot_" + std::string(argv[1])); // Initiate ROS node
  ros::NodeHandle n; // Create named handle "robot_#"
  PotFieldBot robbie(n, robotID, numRobots, goalX, goalY); // Create new random walk object
  robbie.spin(); // Execute FSM loop


  return EXIT_SUCCESS;
};
