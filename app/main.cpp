/**
 * @file main.cpp
 * @author Xin Li
 * @brief Main entrance of the project
 * @version 0.1
 * @date 2021-10-01
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <iostream>
#include "mySLAM/gui.h"
#include "mySLAM/imu.h"

int main()
{
  //mySLAM::GUI myGUI("mySLAM");
  mySLAM::IMU();

  while(1)
  {

  } 
  return 0;
}