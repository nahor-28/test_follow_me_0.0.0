#include <ros/ros.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <gnc_functions.hpp>
#include <iostream>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

// mode_g denotes the flight opperations
//		0 - search
//		1 - rescue 
int mode_g = 0;
int h = 480, w = 640;
int x_mid = w/2;
int err = 0;
void detection_cb(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg)
{
	for( int i=0; i < msg->bounding_boxes.size(); i++)
	{
		ROS_INFO("%s detected", msg->bounding_boxes[i].Class.c_str());	
		if(msg->bounding_boxes[i].Class == "person")
		{
			mode_g = 1; 
			//ROS_INFO("Person found. Starting Rescue Operation");
			int x1 = msg->bounding_boxes[i].xmin;
			int x2 = msg->bounding_boxes[i].xmax;
			int y1 = msg->bounding_boxes[i].ymin;
			int y2 = msg->bounding_boxes[i].ymax;
			std::cout<<"X min:"<<x1<<"\n";
			std::cout<<"Y min:"<<y1<<"\n";
			std::cout<<"X max:"<<x2<<"\n";
			std::cout<<"Y max:"<<y2<<"\n";
			std::cout<<"Width: "<<::w<<"\n";
			std::cout<<"Height: "<<::h<<"\n";
			std::cout<<"X axis mid value: "<<x_mid<<"\n";
			err = (x1+((x2-x1)/2)) - x_mid;
			std::cout<<"X axis error value: "<<err<<"\n";
		}

	}	

}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat src = cv_ptr->image;
        cv::imshow("source", src);
        cv::waitKey(30);

        ::w = src.size().width;
        ::h = src.size().height;
    }

    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "follow_node");
	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("/darknet_ros/bounding_boxes", 1, detection_cb);
	
	image_transport::ImageTransport it(n);
	image_transport::Subscriber img_sub = it.subscribe("/webcam/image_raw", 1, imageCallback);

	//initialize control publisher/subscribers
	init_publisher_subscriber(n);

	// wait for FCU connection
	wait4connect();

	//wait for used to switch to mode GUIDED
	wait4start();

	//create local reference frame 
	initialize_local_frame();

	//request takeoff
	takeoff(3);

	//specify some waypoints 
	std::vector<gnc_api_waypoint> waypointList;
	gnc_api_waypoint nextWayPoint;
	if(err > 0)
	{
		for(int i = 0; i < err; i++)
		{
			nextWayPoint.x = 1;
			nextWayPoint.y = 0;
			nextWayPoint.z = 3;
			nextWayPoint.psi = 0;
			waypointList.push_back(nextWayPoint);
		}
		ros::Rate rate(2.0);
		int counter = 0;
		while(ros::ok())
		{
			//ros::spinOnce();
			rate.sleep();
			if(check_waypoint_reached(.3) == 1)
			{
				if (counter < waypointList.size())
				{
					set_destination(waypointList[counter].x,waypointList[counter].y,waypointList[counter].z, waypointList[counter].psi);
					counter++;
				}
				else
				{
					ros::spin();
				}
			}
		}
	}
	else
	{
		err = err * (-1);
		for(int i = 0; i < err; i++)
		{
			nextWayPoint.x = -1;
			nextWayPoint.y = 0;
			nextWayPoint.z = 3;
			nextWayPoint.psi = 0;
			waypointList.push_back(nextWayPoint);
		}
		ros::Rate rate(2.0);
		int counter = 0;
		while(ros::ok())
		{
			//ros::spinOnce();
			rate.sleep();
			if(check_waypoint_reached(.3) == 1)
			{
				if (counter < waypointList.size())
				{
					set_destination(waypointList[counter].x,waypointList[counter].y,waypointList[counter].z, waypointList[counter].psi);
					counter++;
				}
				else
				{
					ros::spin();
				}
			}
		}
	}
	//ros::spin();
	return 0;
}