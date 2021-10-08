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

#include "mySLAM/vslamMgr.h"

int main()
{
  mySLAM::VSlamManager::Ptr slamMgr(new mySLAM::VSlamManager);
  slamMgr->init();
  slamMgr->run();
  LOG(INFO) << "Program terminated" ;
  return 0;
}