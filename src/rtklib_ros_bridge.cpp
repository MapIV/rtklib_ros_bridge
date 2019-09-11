// Copyright (c) 2019, Map IV, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * Neither the name of the Map IV, Inc. nor the names of its contributors
//   may be used to endorse or promote products derived from this software
//   without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

/*
 * rtklib_ros_bridge.cpp
 * Program for connecting with RTKLIB
 * Author MapIV Sekino
 * Ver 1.00 2019/3/6
 * Ver 2.00 2019/9/11 Changed to output ecef xyz and latitude and longitude
 */

#include "ros/ros.h"
#include "rtklib_ros_bridge/rtklib_msgs.h"
#include "sensor_msgs/NavSatFix.h"
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <math.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rtklib_ros_bridge");
  ros::NodeHandle n;
  ros::Publisher pub1 = n.advertise<rtklib_ros_bridge::rtklib_msgs>("/rtklib_nav", 1);
  ros::Publisher pub2 = n.advertise<sensor_msgs::NavSatFix>("/fix", 1000);

  rtklib_ros_bridge::rtklib_msgs rtklib_nav;
  sensor_msgs::NavSatFix fix;

  struct sockaddr_in server;

  int sock;
  char recv_buf[256];
  int recv_packet_size;

  sock = socket(AF_INET, SOCK_STREAM, 0);

  server.sin_family = AF_INET;
  server.sin_port = htons(61111);
  server.sin_addr.s_addr = inet_addr("127.0.0.1");

  connect(sock, (struct sockaddr*)&server, sizeof(server));

  char data_buf[256];
  int i;
  memset(data_buf, 0, sizeof(data_buf));

  while (ros::ok())
  {
    ros::spinOnce();

    memset(recv_buf, 0, sizeof(recv_buf));
    recv_packet_size = recv(sock, recv_buf, sizeof(recv_buf), 0);

    if (recv_packet_size > 0)
    {
      std::vector<int> LF_index;

      for (i = 0; i < recv_packet_size; i++)
      {
        if (recv_buf[i] == 0x0a)
        {  // 0x0a = LF
          LF_index.push_back(i);
          // ROS_INFO("%d",i);
        }
      }

      memset(data_buf, 0, sizeof(data_buf));
      memcpy(data_buf, &recv_buf[5], 11);
      rtklib_nav.tow = atof(data_buf) * 1000;  // unit[ms]
      // ROS_INFO("tow=%d",tow);

      memset(data_buf, 0, sizeof(data_buf));
      memcpy(data_buf, &recv_buf[LF_index[1]], LF_index[1] - LF_index[0]);
      // ROS_INFO("data_buf=%s",data_buf);
      rtklib_nav.ecef_pos.x = atof(data_buf);
      // ROS_INFO("ecef_pos_x=%10.10lf",ecef_pos_x);

      memset(data_buf, 0, sizeof(data_buf));
      memcpy(data_buf, &recv_buf[LF_index[2]], LF_index[2] - LF_index[1]);
      // ROS_INFO("data_buf=%s",data_buf);
      rtklib_nav.ecef_pos.y = atof(data_buf);
      // ROS_INFO("ecef_pos_y=%10.10lf",ecef_pos_y);

      memset(data_buf, 0, sizeof(data_buf));
      memcpy(data_buf, &recv_buf[LF_index[3]], LF_index[3] - LF_index[2]);
      // ROS_INFO("data_buf=%s",data_buf);
      rtklib_nav.ecef_pos.z = atof(data_buf);
      // ROS_INFO("ecef_pos_z=%10.10lf",ecef_pos_z);

      memset(data_buf, 0, sizeof(data_buf));
      memcpy(data_buf, &recv_buf[LF_index[4]], LF_index[4] - LF_index[3]);
      // ROS_INFO("data_buf=%s",data_buf);
      rtklib_nav.ecef_vel.x = atof(data_buf);
      // ROS_INFO("ecef_vel_x=%10.10lf",ecef_vel_x);

      memset(data_buf, 0, sizeof(data_buf));
      memcpy(data_buf, &recv_buf[LF_index[5]], LF_index[5] - LF_index[4]);
      // ROS_INFO("data_buf=%s",data_buf);
      rtklib_nav.ecef_vel.y = atof(data_buf);
      // ROS_INFO("ecef_vel_y=%10.10lf",ecef_vel_y);

      memset(data_buf, 0, sizeof(data_buf));
      memcpy(data_buf, &recv_buf[LF_index[6]], LF_index[6] - LF_index[5]);
      // ROS_INFO("data_buf=%s",data_buf);
      rtklib_nav.ecef_vel.z = atof(data_buf);
      // ROS_INFO("ecef_vel_z=%10.10lf",ecef_vel_z);

      memset(data_buf, 0, sizeof(data_buf));
      memcpy(data_buf, &recv_buf[LF_index[7]], LF_index[7] - LF_index[6]);
      // ROS_INFO("data_buf=%s",data_buf);
      rtklib_nav.status.latitude = fix.latitude = atof(data_buf);
      // ROS_INFO("latitude=%10.10lf",latitude);

      memset(data_buf, 0, sizeof(data_buf));
      memcpy(data_buf, &recv_buf[LF_index[8]], LF_index[8] - LF_index[7]);
      // ROS_INFO("data_buf=%s",data_buf);
      rtklib_nav.status.longitude = fix.longitude = atof(data_buf);
      // ROS_INFO("longitude=%10.10lf",longitude);

      memset(data_buf, 0, sizeof(data_buf));
      memcpy(data_buf, &recv_buf[LF_index[9]], LF_index[9] - LF_index[8]);
      // ROS_INFO("data_buf=%s",data_buf);
      rtklib_nav.status.altitude = fix.altitude = atof(data_buf);
      // ROS_INFO("altitude=%10.10lf",altitude);

      rtklib_nav.header.stamp = rtklib_nav.status.header.stamp = fix.header.stamp = ros::Time::now();
      rtklib_nav.header.frame_id = rtklib_nav.status.header.frame_id = fix.header.frame_id = "gps";
      pub1.publish(rtklib_nav);
      pub2.publish(fix);

    }
    else if (recv_packet_size == 0)
    {
      ROS_WARN("RTKLIB has been disconnected");
      break;
    }
    else
    {
      ROS_WARN("RTKLIB is not started");
      break;
    }
  }

  close(sock);
  n.shutdown();

  return 0;
}
