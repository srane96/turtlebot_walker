#include "walker.h"
int main(int argc, char* argv[]) {
  // Initialize the ros node
  ros::init(argc, argv, "turtlebot_walker");
  // Create the walker object
  Walker robot;
  // Run the walker behaviour
  robot.startMotion();
  return 0;
}
