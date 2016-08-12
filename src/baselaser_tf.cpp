/*
 Copyright (c) 2016, Juan Jimeno
 Source: http://wiki.ros.org/navigation/Tutorials/RobotSetup/TF
 All rights reserved. 
 
 Redistribution and use in source and binary forms, with or without 
 modification, are permitted provided that the following conditions are met: 
 
 * Redistributions of source code must retain the above copyright notice, 
 this list of conditions and the following disclaimer. 
 * Redistributions in binary form must reproduce the above copyright 
 notice, this list of conditions and the following disclaimer in the 
 documentation and/or other materials provided with the distribution. 
 * Neither the name of  nor the names of its contributors may be used to 
 endorse or promote products derived from this software without specific 
 prior written permission. 
 
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
 LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
 POSSIBILITY OF SUCH DAMAGE. 
 
 */
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
 
ros::NodeHandle *private_n; 

int main(int argc, char** argv){
  ros::init(argc, argv, "baselaser_tf_node");
  ros::NodeHandle nh; 
  private_n= new ros::NodeHandle("~"); 


  ros::Rate r(50);

  tf::TransformBroadcaster baselaser_broadcaster;
   int baselaser_tf_freq; 
   if(!private_n->getParam("tf_freq", baselaser_tf_freq)) { 
         ROS_WARN("Not provided: w_bias. Default=50"); 
         baselaser_tf_freq = 50;
   }

   float w_bias; 
   if(!private_n->getParam("w_bias", w_bias)) { 
         ROS_WARN("Not provided: w_bias. Default=-1.0==M_PI"); 
         w_bias = -1.0;
   }

   float x_bias; 
   if(!private_n->getParam("x_bias", x_bias)) { 
         ROS_WARN("Not provided: x_bias. Default=-0.30"); 
         x_bias  = -0.3;
   }
    // nh_private.param("wheel_diam", d_diam, 0.262); 
   float z_bias; 
   if(!private_n->getParam("z_bias", x_bias)) { 
         ROS_WARN("Not provided: z_bias. Default=0.38"); 
         x_bias  = 0.38;
   }

  while(nh.ok()){
    baselaser_broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, w_bias), tf::Vector3(x_bias, 0.0, z_bias)),
        ros::Time::now(),"base_link", "laser"));
    r.sleep();
  }
}
