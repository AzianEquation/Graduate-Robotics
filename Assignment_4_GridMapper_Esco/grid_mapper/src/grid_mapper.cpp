#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include <cstdlib> // Needed for rand()
#include <ctime> // Needed to seed random number generator with a time value
#include <boost/thread/mutex.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <tf/transform_listener.h>


using namespace boost::posix_time;

class GridMapper {
public:
  // Construst a new occupancy grid mapper  object and hook up
  // this ROS node to the simulated robot's pose, velocity control,
  // and laser topics
  GridMapper(ros::NodeHandle& nh, int width, int height) :
      canvas(height, width, CV_8UC1) {
    // Initialize random time generator
    srand(time(NULL));

    // Advertise a new publisher for the current simulated robot's
    // velocity command topic (the second argument indicates that
    // if multiple command messages are in the queue to be sent,
    // only the last command will be sent)
    // changed from cmd_vel
    commandPub = nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);

    // Subscribe to the current simulated robot's laser scan topic and
    // tell ROS to call this->laserCallback() whenever a new message
    // is published on that topic
           //     laserSub = nh.subscribe("scan", 1, \
                        &GridMapper::laserCallback, this);
    // changed from base_scan
    laserSub = nh.subscribe("scan", 1, \
      &GridMapper::laserCallback, this);
    
    // Subscribe to the current simulated robot' ground truth pose topic
    // and tell ROS to call this->poseCallback(...) whenever a new
    // message is published on that topic
//                poseSub = nh.subscribe("odom", 1, \
       &GridMapper::poseCallback, this);
       // changed from base_pose_ground_truth
    poseSub = nh.subscribe("odom", 1, \
      &GridMapper::poseCallback, this);
      
    // Create resizeable named window
    cv::namedWindow("Occupancy Grid Canvas", \
      CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO | CV_GUI_EXPANDED);
  };
  
  
  // Save a snapshot of the occupancy grid canvas
  // NOTE: image is saved to same folder where code was executed
  void saveSnapshot() {
    std::string filename = "grid_" + to_iso_string(second_clock::local_time()) + ".jpg";
    canvasMutex.lock();
    cv::imwrite(filename, canvas);
    canvasMutex.unlock();
  };
  
  
  // Update grayscale intensity on canvas pixel (x, y) (in robot coordinate frame)
  void plot(int x, int y, char value) {
    canvasMutex.lock();
    x+=canvas.rows/2;
    y+=canvas.cols/2;
    if (x >= 0 && x < canvas.rows && y >= 0 && y < canvas.cols) {
       // Does not overwrite robot's marked path
       if ((int)(canvas.at<char>(x,y)) != CELL_ROBOT) {
        canvas.at<char>(x, y) = value;
      }
    }
    canvasMutex.unlock();
  };

  // Update grayscale intensity on canvas pixel (x, y) (in image coordinate frame)
  void plotImg(int x, int y, char value) {
    canvasMutex.lock();
    if (x >= 0 && x < canvas.cols && y >= 0 && y < canvas.rows) {
      canvas.at<char>(y, x) = value;
    }
    canvasMutex.unlock();
  };

  void bres(double x0, double y0, double x1, double y1){
    int tempX,tempY,dx,dy,dx0,dy0,px,py,xe,ye;
    dx=x1-x0;
    dy=y1-y0;
    dx0=fabs(dx);
    dy0=fabs(dy);
    px=2*dy0-dx0;
    py=2*dx0-dy0;
      if(dy0<=dx0){
        if(dx>=0){
	  tempX = x0;
   	  tempY = y0;
	  xe=x1;
	  }
	else{
	  tempX = x1;
	  tempY = y1;
  	  xe=x0;
	}  
	for(int i=0; tempX<xe;i++){
	  tempX = tempX + 1;
	   if(px<0){
	   px=px+2*dy0;
         }
	 else{
	   if((dx<0 && dy<0) || (dx>0 && dy>0)){
	     tempY = tempY + 1;
	   }
	   else{
	     tempY =tempY - 1;
	   }
	   px=px+2*(dy0-dx0);
	 }
           plot(tempX, tempY, CELL_FREE);
	}
      }
      else{
        if(dy>=0){
	  tempX = x0;
	  tempY = y0;
	  ye=y1;
        }
	else{
	  tempX = x1;
	  tempY = y1;
	  ye=y0;
	 }
	 plot(tempX, tempY ,CELL_FREE);
	 for(int i=0;tempY < ye;i++){
	   tempY = tempY + 1;
	     if(py<=0){
	       py=py+2*dx0;
	     }
	     else{
	       if((dx<0 && dy<0) || (dx>0 && dy>0)){
	         tempX = tempX + 1;
	       }
	       else{
	         tempX = tempX - 1;
	       }
	       py=py+2*(dx0-dy0);
	     }
	   plot(tempX,tempY,CELL_FREE);	 
         }
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
    // TODO: parse laser data and update occupancy grid canvas
    //       (use CELL_OCCUPIED, CELL_UNKNOWN, CELL_FREE, and CELL_ROBOT values)
    // (see http://www.ros.org/doc/api/sensor_msgs/html/msg/LaserScan.html)
    for(unsigned int i = 0; i < msg->ranges.size(); i++) {
      // current distance
      float dist_i = msg->ranges[i];
      // current angle then increment angle
      float angle_i = msg->angle_min + i * msg->angle_increment;
      // stage heading
      float angle_stage = heading;
      // convert negative angles
      if(angle_stage < 0) {
	// shift by 2pi
        angle_stage +=2*M_PI;
      }
      // check that current range is within the min and max range for the sensor
      if(dist_i > msg->range_min && dist_i < msg->range_max) {
	// shift by 2pi
        angle_i = 2*M_PI + angle_i;
	// get x and y values relative to robot's coordinates
        double xdist_robot = dist_i*std::cos(angle_i);
        double ydist_robot = dist_i*std::sin(angle_i);
	// get values in stage coordinates
        double xdist_stage = xdist_robot * std::cos(angle_stage) - ydist_robot * std::sin(angle_stage) + x;
        double ydist_stage = xdist_robot * std::sin(angle_stage)  + ydist_robot * std::cos(angle_stage) + y;
        // draw a line from current position to obstacle position
       	bres(-scaleFactor*y, scaleFactor*x, -scaleFactor*ydist_stage, scaleFactor*xdist_stage);
        // plot the obstacle
	plot(-scaleFactor*ydist_stage, scaleFactor*xdist_stage,CELL_OCCUPIED);
       }
    }
      // plot the current position of robot
      plot(-scaleFactor*y, scaleFactor*x, CELL_ROBOT);
  };

  // Process incoming ground truth robot pose message
  void poseCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    double roll, pitch;
    x = msg->pose.pose.position.x;
    y = msg->pose.pose.position.y;
    heading=tf::getYaw(msg->pose.pose.orientation);
  };
  
  
  // Main FSM loop for ensuring that ROS messages are
  // processed in a timely manner, and also for sending
  // velocity controls to the simulated robot based on the FSM state
  void spin() {
    int key = 0;
    
    // Initialize all pixel values in canvas to CELL_UNKNOWN
    canvasMutex.lock();
    canvas = cv::Scalar(CELL_UNKNOWN);
    canvasMutex.unlock();
    
    while (ros::ok()) { // Keep spinning loop until user presses Ctrl+C
      // TODO: remove following demo code and make robot move around the environment
     // plot(-scale*y, scale*x, CELL_ROBOT); // Demo code: plot robot's current position on canvas      
      // NOTE: DO NOT REMOVE CODE BELOW THIS LINE
      cv::imshow("Occupancy Grid Canvas", canvas);
      ros::spinOnce(); // Need to call this function often to allow ROS to process incoming messages
      key = cv::waitKey(1000/SPIN_RATE_HZ); // Obtain keypress from user; wait at most N milliseconds
      if (key == 'x' || key == 'X') {
        break;
      } else if (key == ' ') {
        saveSnapshot();
      }
    }
    
    ros::shutdown(); // Ensure that this ROS node shuts down properly
  };

  // Tunable motion controller parameters
  // const static double FORWARD_SPEED_MPS = 2.0;
  // const static double ROTATE_SPEED_RADPS = M_PI/2;
  
  const static int SPIN_RATE_HZ = 30;
  
  const static char CELL_OCCUPIED = 0;
  const static char CELL_UNKNOWN = 86;
  const static char CELL_FREE = 172;
  const static char CELL_ROBOT = 255;


protected:
  ros::Publisher commandPub; // Publisher to the current robot's velocity command topic
  ros::Subscriber laserSub; // Subscriber to the current robot's laser scan topic
  ros::Subscriber poseSub; // Subscriber to the current robot's ground truth pose topic

  double x; // in simulated Stage units, + = East/right
  double y; // in simulated Stage units, + = North/up
  double heading; // in radians, 0 = East (+x dir.), pi/2 = North (+y dir.)
  
  cv::Mat canvas; // Occupancy grid canvas
  boost::mutex canvasMutex; // Mutex for occupancy grid canvas object
private:
  // scale factor (need a resolution where each pixel is ~ 5cm  
  const static int scaleFactor = 20;
};


int main(int argc, char **argv) {
  int width, height;
  bool printUsage = false;
  
  // Parse and validate input arguments
  if (argc <= 2) {
    printUsage = true;
  } else {
    try {
      width = boost::lexical_cast<int>(argv[1]);
      height = boost::lexical_cast<int>(argv[2]);

      if (width <= 0) { printUsage = true; }
      else if (height <= 0) { printUsage = true; }
    } catch (std::exception err) {
      printUsage = true;
    }
  }
  if (printUsage) {
    std::cout << "Usage: " << argv[0] << " [CANVAS_WIDTH] [CANVAS_HEIGHT]" << std::endl;
    return EXIT_FAILURE;
  }
  
  ros::init(argc, argv, "grid_mapper"); // Initiate ROS node
  ros::NodeHandle n; // Create default handle
  GridMapper robbie(n, width, height); // Create new grid mapper object
  robbie.spin(); // Execute FSM loop

  return EXIT_SUCCESS;
};
