#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include "rrt_planner.h"


template <int n>
class PlannerInterface
{

public:
	PlannerInterface();


    void visualizeTree(const ros::Publisher& vis_pub,const planner::WaypointPtr<n> root);
    void visualizeObstacles(const ros::Publisher& vis_pub);
    void visualizePath(const ros::Publisher& vis_pub, const std::vector<planner::WaypointPtr<n>> path);

	void getParams(const ros::NodeHandle &nh);

	std::vector<double> ss_min;
	std::vector<double> ss_max;
	int ss_dim;
	double maxStepSize;
	double goalTol;
	std::vector<double> v_start;
	std::vector<double> v_goal;
	static int marker_id;

    planner::ObstaclesPtr<n> objects;

};

template<int n>
 int PlannerInterface<n>::marker_id =0; //static variable

template<int n>
PlannerInterface<n>::PlannerInterface()
{
	ss_dim =0;
	maxStepSize =0;
	goalTol =0;

	ss_min.push_back(0);
	ss_min.push_back(0);
	ss_max.push_back(100);
	ss_max.push_back(100);
	v_start.push_back(15);
	v_start.push_back(15);
	v_goal.push_back(85);
	v_goal.push_back(85);
    
    /*Obstacle object1 = new Object();
    object1.radius =15;
    State<*/


}



template<int n>
 void PlannerInterface<n>::visualizeTree(const ros::Publisher& vis_pub,const planner::WaypointPtr<n> root)
 {
  
   //visualize root

  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time();
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;
  

     

  if(!(root->nextWps.size()))
  {
  	return ;
  }
  else
  {
  	int cnt =root->nextWps.size();
  	int i=0;
  	while(cnt--)
  	{
  		 geometry_msgs:: Point point;

  		   point.x = root->coordinates[0];
  		   point.y = root->coordinates[1];
  		   point.z = 0;

  		   marker.points.push_back(point);
  		   point.x = (root->nextWps[i])->coordinates[0];
  		   point.y = (root->nextWps[i])->coordinates[1];
  		   point.z = 0;
  		   marker.points.push_back(point);
  		   marker.id = marker_id++;
     marker.scale.x = 0.1/*root->distance((root->nextWps[i])->coordinates)*/;
     marker.scale.y = 0.2;
     marker.scale.z = 0.7;
     marker.color.a = 1.0; // Don't forget to set the alpha!
     marker.color.r = 0.0;
     marker.color.g = 1.0;
     marker.color.b = 0.0;
     vis_pub.publish( marker );
     marker.points.clear();
  	    visualizeTree(vis_pub,root->nextWps[i++]);
  	}
  	
  }


 }
 
 template<int n>
 void PlannerInterface<n>::visualizeObstacles(const ros::Publisher& vis_pub)
 {
 	for(int i=0; i<(this->objects).size();i++)
 	{
 		visualization_msgs::Marker marker;
 		marker.header.frame_id = "map";
 		marker.header.stamp = ros::Time();
 		marker.type = visualization_msgs::Marker::SPHERE;
 		marker.action = visualization_msgs::Marker::ADD;
 		marker.id = marker_id++;
 		
 		marker.pose.position.x = ((this->objects[i]))->vertices[0][0];
 		marker.pose.position.y = ((this->objects[i]))->vertices[0][1];
 		marker.pose.position.z = 0;

 		marker.scale.x = ((this->objects)[i])->radius;
 		marker.scale.y = 0.2;
 		marker.scale.z = 0.3;
 		marker.color.a = 1.0; // Don't forget to set the alpha!
 		marker.color.r = 0.5;
 		marker.color.g = 0.5;
 		marker.color.b = 0.5;
 		vis_pub.publish( marker );
 	}
 }

template<int n>
 void PlannerInterface<n>::visualizePath(const ros::Publisher& vis_pub, const std::vector<planner::WaypointPtr<n>> path)
 {
 	int len = path.size();
 	visualizeTree(vis_pub,path[len-1]);
//start marker
 	visualization_msgs::Marker marker;
 	marker.header.frame_id = "map";
 	marker.header.stamp = ros::Time();
 	marker.type = visualization_msgs::Marker::SPHERE;
 	marker.action = visualization_msgs::Marker::ADD;
 	marker.id = marker_id++;
 	
 	marker.pose.position.x = path[path.size()-1]->coordinates[0];
 	marker.pose.position.y = path[path.size()-1]->coordinates[1];
 	marker.pose.position.z = 0;

 	marker.scale.x = 2;
 	marker.scale.y = 2;
 	marker.scale.z = 2;
 	marker.color.a = 1.0; // Don't forget to set the alpha!
 	marker.color.r = 0.0;
 	marker.color.g = 0.0;
 	marker.color.b = 1.0;
 	vis_pub.publish( marker );


//goal marker

 	marker.id = marker_id++;

 	marker.pose.position.x = path[0]->coordinates[0];
 	marker.pose.position.y = path[0]->coordinates[1];
 	marker.pose.position.z = 0;

 	marker.scale.x = 2;
 	marker.scale.y = 2;
 	marker.scale.z = 2;
 	marker.color.a = 1.0; // Don't forget to set the alpha!
 	marker.color.r = 0.0;
 	marker.color.g = 1.0;
 	marker.color.b = 1.0;
 	vis_pub.publish( marker );

 	marker.type = visualization_msgs::Marker::ARROW;


 	marker.pose.position.x = 0;
 	marker.pose.position.y = 0;
 	marker.pose.position.z = 0;

 	for(int i= path.size()-1; i>0;i--)
 	{
 		marker.id = marker_id++;
  		
  		   geometry_msgs:: Point point;

  		   point.x = path[i]->coordinates[0];
  		   point.y = path[i]->coordinates[1];
  		   point.z = 0;

  		   marker.points.push_back(point);
  		   point.x = path[i-1]->coordinates[0];
  		   point.y = path[i-1]->coordinates[1];
  		   point.z = 0;
  		   marker.points.push_back(point);


  		   marker.scale.x = 0.3/*path[i]->distance((path[i-1]->coordinates))*/;
  		   marker.scale.y = 0.3;
  		   marker.scale.z = 0.7;
  		   marker.color.a = 1.0; // Don't forget to set the alpha!
  		   marker.color.r = 1.0;
  		   marker.color.g = 0.0;
  		   marker.color.b = 0.0;
  		  vis_pub.publish( marker );
  		   marker.points.clear();
 		//publish markers to rviz
 	}
 }

template<int n>
void PlannerInterface<n>::getParams(const ros::NodeHandle &nh)
{
     nh.getParam("/ss_dim",ss_dim);
	 nh.getParam("/maxStepSize", maxStepSize);
	 nh.getParam("/goalTol",goalTol);
	//ss_min
	//ss_max
	//start
	//goal
	//objects


}