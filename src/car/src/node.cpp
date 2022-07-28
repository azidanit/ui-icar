#include "car.h"

/** Main node entry point. */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "car");
  ros::NodeHandle node;
  ros::Rate r(100);

  // create conversion class, which subscribes to raw data
  Car car(node, argv[1]);

  // handle callbacks until shut down
  while(ros::ok()){
    ros::spinOnce();
    car.update();
    r.sleep();
  }

  return 0;
}
