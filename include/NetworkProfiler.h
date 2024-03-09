/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef NETWORKPROFILER_H
#define NETWORKPROFILER_H

#include <thread>
#include <vector>
#include <string>
#include <unistd.h>

namespace ORB_SLAM2
{

class NetworkProfiler
{
public:
  NetworkProfiler();
  void startServer(string args);
  void startClient(string ip, int t_secs);
  void stopClient();
  
protected:
  string outputFileName;
  bool killClient;
  thread* clientThread;
  vector<string> timestamps;
  vector<string> bandwidths;
};

} //namespace ORB_SLAM

#endif // NETWORKPROFILER_H
