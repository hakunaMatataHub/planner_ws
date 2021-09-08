#include <ros/ros.h>

#include "planner_interface.h"


#define DIM 3

 

int main(int argc, char **argv)
{
    
    ros::init(argc, argv, "planner_interface");
   

    

    ros::NodeHandle  nh;

    int dim =0;
    nh.getParam("/ss_dim", dim);
    PlannerInterface<DIM> planner_interface;
    planner_interface.getParams(nh);

    ros::Publisher vis_pub = nh.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );

    planner_interface.getParams(nh);
    planner::RRT<DIM> rrt_planner(planner_interface.maxStepSize,planner_interface.goalTol, planner_interface.ss_min,planner_interface.ss_max);
    rrt_planner.addObstacles(planner_interface.objects);
    planner::State<DIM> start_coordinates(planner_interface.v_start.data());
    planner::State<DIM> goal_coordinates(planner_interface.v_goal.data());
    planner::Waypoint<DIM>* startWp = new planner::Waypoint<DIM>(start_coordinates);
    planner::Waypoint<DIM>* goalWp = new planner::Waypoint<DIM>(goal_coordinates);
    rrt_planner.setStart(startWp);
    rrt_planner.setGoal(goalWp);



    planner_interface.visualizeObstacles(vis_pub);
    //ros::NodeHandle  nh_("~");
    ros::Time prev_time = ros::Time::now();
    ros::Rate rate(10.0); // 10 Hz rate

    
    
    while (ros::ok())
    {
    	std::cout<<"hi i am here 1"<<std::endl;
    	if(rrt_planner.bGoalSet && !rrt_planner.bGoalFound)
    		rrt_planner.plan();
        if(rrt_planner.bGoalFound)
        {
        	    	planner_interface.visualizePath(vis_pub,rrt_planner.path);
        	    	std::cout<<"markers Published !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<std::endl;

        	    	rrt_planner.bGoalSet = false;
        	    	rrt_planner.bGoalFound = false;

        }
     rate.sleep();

    }
        
        return 0;
    }