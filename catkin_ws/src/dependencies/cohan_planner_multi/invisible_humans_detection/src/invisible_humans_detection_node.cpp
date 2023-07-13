#include "invisible_humans_detection/map_scanner.h"

int main(int argc, char** argv){
  ros::init(argc, argv, "map_scanner_node");
  invisible_humans_detection::MapScanner mp_scanner;
  return 0;
}
