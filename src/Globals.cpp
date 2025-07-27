#include "Globals.h"

std::string Config::select_path = "";
bool Config::running = false;
bool Config::stopRun = true;
int Config::save_type =1;
std::string Config::lidar_64_path="lidar_64_line";
std::string Config::lidar_256_path="lidar_256_line";
std::string Config::usb_path="/mnt/usb";