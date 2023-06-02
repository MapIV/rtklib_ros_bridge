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
 * rtklib_bridge.cpp
 * Program for connecting with RTKLIB
 * Author MapIV Sekino
 * Ver 0.00 
 */

#include "rclcpp/rclcpp.hpp"
#include "rtklib_msgs/msg/rtklib_nav.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "hgeoid.hpp"
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <math.h>
#include <string>
#include <vector>

#include "rclcpp/time.hpp"
#include <chrono>
#include "rclcpp/clock.hpp"

#include "rclcpp/parameter.hpp"
#include "rclcpp/node.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("rtklib_bridge");
  auto pub1 = node->create_publisher<rtklib_msgs::msg::RtklibNav>("rtklib_nav", 1);
  auto pub2 = node->create_publisher<sensor_msgs::msg::NavSatFix>("rtklib/fix", 1000);

  rtklib_msgs::msg::RtklibNav rtklib_nav;
  sensor_msgs::msg::NavSatFix fix;
  std::string node_name = "rtklib_bridge";

  rclcpp::Clock ros_clock(RCL_ROS_TIME);

  std::string ip_adress = "127.0.0.1";
  int port = 61111;
  bool altitude_estimate = true;

  node->declare_parameter("ip_adress" , "127.0.0.1");
  node->declare_parameter("port" , 61111);
  node->declare_parameter("altitude_estimate" , true);

  ip_adress = node->get_parameter("ip_adress").as_string();
  port = node->get_parameter("port").as_int();
  altitude_estimate = node->get_parameter("altitude_estimate").as_bool();

  std::cout << "ip_adress " << ip_adress << std::endl;
  std::cout << "port " << port << std::endl;
  std::cout << "altitude_estimate " << altitude_estimate << std::endl;

  struct sockaddr_in server;

  int sock;
  char recv_buf[256];
  int recv_packet_size;

  sock = socket(AF_INET, SOCK_STREAM, 0);

  server.sin_family = AF_INET;
  server.sin_port = htons(port);
  server.sin_addr.s_addr = inet_addr(ip_adress.c_str());

  connect(sock, (struct sockaddr*)&server, sizeof(server));

  char data_buf[256];
  int i;
  memset(data_buf, 0, sizeof(data_buf));

  while (rclcpp::ok())
  {
    rclcpp::spin_some(node);

    memset(recv_buf, 0, sizeof(recv_buf));
    recv_packet_size = recv(sock, recv_buf, sizeof(recv_buf), 0);

    // RCLCPP_INFO(rclcpp::get_logger(node_name), "RAWdata:%s",recv_buf);

    if (recv_packet_size > 0)
    {

      rtklib_nav.header.stamp = rtklib_nav.status.header.stamp = fix.header.stamp = ros_clock.now();
      rtklib_nav.header.frame_id = rtklib_nav.status.header.frame_id = fix.header.frame_id = "gps";

      std::vector<int> LF_index;
      std::vector<int>::size_type index_size = 18;
      // recv_buf has 18 lines when correctly received from RTKLIB.

      for (i = 0; i < recv_packet_size; i++)
      {
        if (recv_buf[i] == 0x0a)
        {  // 0x0a = LF
          LF_index.push_back(i);
          //  RCLCPP_INFO(rclcpp::get_logger(node_name), "%d",i);
        }
      }

      if (LF_index.size() < index_size){
        RCLCPP_WARN(rclcpp::get_logger(node_name), "Received data is missing.");
        continue;
      }

      memset(data_buf, 0, sizeof(data_buf));
      memcpy(data_buf, &recv_buf[5], 11);
      rtklib_nav.tow = atof(data_buf) * 1000;  // unit[ms]
      // RCLCPP_INFO(rclcpp::get_logger(node_name), "tow=%d",rtklib_nav.tow);

      memset(data_buf, 0, sizeof(data_buf));
      memcpy(data_buf, &recv_buf[LF_index[1]], LF_index[1]);
      // RCLCPP_INFO(rclcpp::get_logger(node_name), "data_buf=%s",data_buf);
      rtklib_nav.ecef_pos.x = atof(data_buf);
      // RCLCPP_INFO(rclcpp::get_logger(node_name), "ecef_pos_x=%10.10lf",rtklib_nav.ecef_pos.x);

      memset(data_buf, 0, sizeof(data_buf));
      memcpy(data_buf, &recv_buf[LF_index[2]], LF_index[2]);
      // RCLCPP_INFO(rclcpp::get_logger(node_name), "data_buf=%s",data_buf);
      rtklib_nav.ecef_pos.y = atof(data_buf);
      // RCLCPP_INFO(rclcpp::get_logger(node_name), "ecef_pos_y=%10.10lf",rtklib_nav.ecef_pos.y);

      memset(data_buf, 0, sizeof(data_buf));
      memcpy(data_buf, &recv_buf[LF_index[3]], LF_index[3]);
      // RCLCPP_INFO(rclcpp::get_logger(node_name), "data_buf=%s",data_buf);
      rtklib_nav.ecef_pos.z = atof(data_buf);
      // RCLCPP_INFO(rclcpp::get_logger(node_name), "ecef_pos_z=%10.10lf",rtklib_nav.ecef_pos.z);

      memset(data_buf, 0, sizeof(data_buf));
      memcpy(data_buf, &recv_buf[LF_index[4]], LF_index[4]);
      // RCLCPP_INFO(rclcpp::get_logger(node_name), "data_buf=%s",data_buf);
      rtklib_nav.ecef_vel.x = atof(data_buf);
      // RCLCPP_INFO(rclcpp::get_logger(node_name), "ecef_vel_x=%10.10lf",rtklib_nav.ecef_vel.x);

      memset(data_buf, 0, sizeof(data_buf));
      memcpy(data_buf, &recv_buf[LF_index[5]], LF_index[5]);
      // RCLCPP_INFO(rclcpp::get_logger(node_name), "data_buf=%s",data_buf);
      rtklib_nav.ecef_vel.y = atof(data_buf);
      // RCLCPP_INFO(rclcpp::get_logger(node_name), "ecef_vel_y=%10.10lf",rtklib_nav.ecef_vel.y);

      memset(data_buf, 0, sizeof(data_buf));
      memcpy(data_buf, &recv_buf[LF_index[6]], LF_index[6]);
      // RCLCPP_INFO(rclcpp::get_logger(node_name), "data_buf=%s",data_buf);
      rtklib_nav.ecef_vel.z = atof(data_buf);
      // RCLCPP_INFO(rclcpp::get_logger(node_name), "ecef_vel_z=%10.10lf",rtklib_nav.ecef_vel.z);

      memset(data_buf, 0, sizeof(data_buf));
      memcpy(data_buf, &recv_buf[LF_index[7]], LF_index[7]);
      // RCLCPP_INFO(rclcpp::get_logger(node_name), "data_buf=%s",data_buf);
      rtklib_nav.status.latitude = fix.latitude = atof(data_buf);
      // RCLCPP_INFO(rclcpp::get_logger(node_name), "latitude=%10.10lf",rtklib_nav.status.latitude);

      memset(data_buf, 0, sizeof(data_buf));
      memcpy(data_buf, &recv_buf[LF_index[8]], LF_index[8]);
      // RCLCPP_INFO(rclcpp::get_logger(node_name), "data_buf=%s",data_buf);
      rtklib_nav.status.longitude = fix.longitude = atof(data_buf);
      // RCLCPP_INFO(rclcpp::get_logger(node_name), "longitude=%10.10lf",rtklib_nav.status.longitude);

      memset(data_buf, 0, sizeof(data_buf));
      memcpy(data_buf, &recv_buf[LF_index[9]], LF_index[9]);
      // RCLCPP_INFO(rclcpp::get_logger(node_name), "data_buf=%s",data_buf);
      rtklib_nav.status.altitude = fix.altitude = atof(data_buf);
      // RCLCPP_INFO(rclcpp::get_logger(node_name), "altitude=%10.10lf",rtklib_nav.status.altitude);

      memset(data_buf, 0, sizeof(data_buf));
      memcpy(data_buf, &recv_buf[LF_index[10]], LF_index[10]);
      // RCLCPP_INFO(rclcpp::get_logger(node_name), "data_buf=%s",data_buf);
      if(atoi(data_buf) == 1)
      {
        rtklib_nav.status.status.status = fix.status.status = 0;
      }
      else
      {
        rtklib_nav.status.status.status = fix.status.status = -1;
      }

      double llh[3] , height;

      if(altitude_estimate == true)
      {
        llh[0] = fix.latitude;
        llh[1] = fix.longitude;
        llh[2] = fix.altitude;
        hgeoid(llh ,  &height);
        rtklib_nav.status.altitude = fix.altitude = llh[2] - height;

      }

      rtklib_nav.status.status.service = fix.status.service = 1;  // Currently fixed value

      memset(data_buf, 0, sizeof(data_buf));
      memcpy(data_buf, &recv_buf[LF_index[11]], LF_index[11]);
      rtklib_nav.status.position_covariance[0] = fix.position_covariance[0] = atof(data_buf);
      memset(data_buf, 0, sizeof(data_buf));
      memcpy(data_buf, &recv_buf[LF_index[12]], LF_index[12]);
      rtklib_nav.status.position_covariance[4] = fix.position_covariance[4] = atof(data_buf);
      memset(data_buf, 0, sizeof(data_buf));
      memcpy(data_buf, &recv_buf[LF_index[13]], LF_index[13]);
      rtklib_nav.status.position_covariance[8] = fix.position_covariance[8] = atof(data_buf);
      memset(data_buf, 0, sizeof(data_buf));
      memcpy(data_buf, &recv_buf[LF_index[14]], LF_index[14]);
      rtklib_nav.status.position_covariance[1] = fix.position_covariance[1] = atof(data_buf);
      rtklib_nav.status.position_covariance[3] = fix.position_covariance[3] = atof(data_buf);
      memset(data_buf, 0, sizeof(data_buf));
      memcpy(data_buf, &recv_buf[LF_index[15]], LF_index[15]);
      rtklib_nav.status.position_covariance[5] = fix.position_covariance[5] = atof(data_buf);
      rtklib_nav.status.position_covariance[7] = fix.position_covariance[7] = atof(data_buf);
      memset(data_buf, 0, sizeof(data_buf));
      memcpy(data_buf, &recv_buf[LF_index[16]], LF_index[16]);
      rtklib_nav.status.position_covariance[2] = fix.position_covariance[2] = atof(data_buf);
      rtklib_nav.status.position_covariance[6] = fix.position_covariance[6] = atof(data_buf);
      rtklib_nav.status.position_covariance_type = fix.position_covariance_type = 3;

      memset(data_buf, 0, sizeof(data_buf));
      memcpy(data_buf, &recv_buf[LF_index[17]], sizeof("RTKLIB"));
      std::string check_packets_str(data_buf);
      if(check_packets_str.find("RTKLIB") == std::string::npos)
      {
        RCLCPP_WARN(rclcpp::get_logger(node_name), "Received packet is corrupted!");
        continue;
      }

      // RCLCPP_INFO(rclcpp::get_logger(node_name), "RAWdata:%s",recv_buf);

      pub1->publish(rtklib_nav);
      pub2->publish(fix);

      printf("GPST:%.3lf(s) latitude:%.9lf(deg)  longitude:%.9lf(deg)  altitude:%.4lf(m)\n" ,
          double(rtklib_nav.tow/1000.0) , rtklib_nav.status.latitude , rtklib_nav.status.longitude , rtklib_nav.status.altitude);

    }
    else if (recv_packet_size == 0)
    {
      RCLCPP_WARN(rclcpp::get_logger(node_name), "RTKLIB has been disconnected");
      break;
    }
    else
    {
      RCLCPP_WARN(rclcpp::get_logger(node_name), "RTKLIB is not started");
      break;
    }
  }

  close(sock);
  rclcpp::shutdown();

  return 0;
}
