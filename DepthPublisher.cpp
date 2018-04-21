#include <image_transport/image_transport.h>
#include <iostream>
#include <stdio.h>
#include <geometry_msgs/Vector3.h>
#include <math.h>
#include <nav_msgs/Odometry.h>

ros::Publisher depth_pub;

/* Loop over every column of pixels (640 pixel width means 640 columns), 
 * find the average of all of the depths in each column
 *
 * NOTE: For some reason some of the rows are filled with low numbers, which ends up dragging the averages down.
 * So we are only finding the average of the values that have depth data greater than 100 (10cm) to filter out the noise.
 */

int *ReadDepthData(sensor_msgs::ImageConstPtr depth_image) {
  int depth_arr[(int) depth_image->width]; // for storing depth averages
  int pixel_index;
  short pixel_depth;
  int counter; // keeps track of how many pixels have a depth greater than 100
  int avg; // average of a column of pixels

/////////////////

    const short* depth_row = reinterpret_cast<const short*>(&depth_image->data[0]);
    int row_step = depth_image->step / sizeof(short);
    for (int u = 0; u < (int)depth_image->width; ++u){
      // ROS_INFO("#####################################");
      avg = 0; // reset
      counter = 0;
      for (int v = 100; v < 380; ++v){
        pixel_index = v*row_step+u;
        pixel_depth = depth_row[pixel_index];
	 // ROS_INFO(" %d %d % d", u,v, pixel_depth);
        if(pixel_depth > 100){ // filter out low depth vals
           counter = counter + 1;
           avg += pixel_depth;
        }
      }
      
      if(counter != 0){ // sometimes columns are filled completely with 0s, so counter ends up being 0
        avg = (float)avg/counter;
      }
      //depth_arr[u] = avg;
      depth_arr[u] = (int)avg;

    }
/////////////////

/*
  for(int i = 0; i < width; i++){
    avg = 0; // reset
    counter = 0;
    for(int j = 0; j < height;j++){ 
      pixel_index = (j*depth_image->step) + (i*(depth_image->step/depth_image->width));
      pixel_depth = depth_image->data[pixel_index] + (depth_image->data[pixel_index + 1] << 8);
      if(pixel_depth > 100){ // filter out low depth vals
         counter = counter + 1;
         avg += pixel_depth;
      }
    }
    if(counter != 0){ // sometimes columns are filled completely with 0s, so counter ends up being 0
       avg = (int) avg/counter;
    }
    depth_arr[i] = avg;
  }
*/
  return depth_arr;
}


void ImageCallback(const sensor_msgs::ImageConstPtr& image) {
  int *depth_arr = ReadDepthData(image); // Width = 640, Height = 480
  
  int min_index = 100;
  for(int i = 100; i < 540; i++){
    if(depth_arr[i] < depth_arr[min_index] && depth_arr[i] > 500){
      min_index = i;
    }
  }
  int min = depth_arr[min_index];

  int diff = 320 - min_index;
  float x_radians_per_pixel = 60.0/57.0/640;
  float num_rads = diff*x_radians_per_pixel;
  
  nav_msgs::Odometry Depth;
  Depth.twist.twist.linear.x = min;
  Depth.twist.twist.linear.y = 0;
  Depth.twist.twist.angular.z = num_rads;

  depth_pub.publish(Depth);
  ROS_INFO("Depth: %d", min);
}

/******* MAIN *******/
int main(int argc, char **argv) {
  ros::init(argc, argv, "depth_publisher_node");
  ros::NodeHandle nh;

  depth_pub = nh.advertise<nav_msgs::Odometry>("Depth",100);
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber camera_sub = it.subscribe("/camera/depth/image_raw", 1, ImageCallback);

  ros::spin();
  return 0;
}
