	
#include "path_planner.h"
#include <Eigen/Geometry>
#include <random>

template <typename T> // Returns a random number in [low, high] 
T randomCoordinate(T low, T high){
    std:: random_device random_device;
    std::mt19937 engine{random_device()};
    std::uniform_real_distribution<double> dist(low, high);
    return dist(engine);
}

namespace planner
{
	template<int n>
	class RRT : public PathPlanner<n>
	{

     public:
     	RRT(double _maxStepSize, double _goalTol, std::vector<double>& ss_min, std::vector<double>& ss_max );

     	void setStart(const WaypointPtr<n> startWp);
     	void setGoal(const WaypointPtr<n> goalWp);
     	WaypointPtr<n> getGoal() const;
     	WaypointPtr<n> getStart() const;


        void savePath();
     	void plan();
     	void sample(State<n>& pnt);
     	bool collisionCheck(const WaypointPtr<n> wp);
     	
     	WaypointPtr<n> findNearestNeighbour(const State<n>& pnt , const WaypointPtr<n> root = NULL, const bool reset= false);
        void fixStepSize(const WaypointPtr<n> wp, State<n>& pnt);
     	//void savePath();
     	
     	std::vector<double> ss_min;
     	std::vector<double> ss_max;
        std::vector<WaypointPtr<n>> path;
        double time; //termination criteria	
        


	};	


	template<int n>
	void RRT<n>::fixStepSize(const WaypointPtr<n> wp, State<n>& pnt)
	{
		if(wp->distance(pnt)<= this->maxStepSize)
			return;
		else
			pnt = wp->coordinates + (((pnt - wp->coordinates)/(pnt-wp->coordinates).norm())*(this->maxStepSize));
	}

	template<int n>
	void RRT<n>::sample(State<n>& pnt)
	{
		int size = pnt.rows();
		for(int i=0;i<size;i++)
		{
			pnt[i] = randomCoordinate(ss_min[i],ss_max[i]);

		}


	}

	template<int n>
	WaypointPtr<n> RRT<n>::findNearestNeighbour(const State<n>& pnt , const WaypointPtr<n> root, const bool reset)
	{
		static double min_distance = 10000000000000000000000; //very large number
        static WaypointPtr<n> wp = NULL;

        if( reset)
        {
        	min_distance = 1000000000000000; //very latge
        	wp = NULL;
        	return wp;
        }
		double distance = root->distance(pnt);
		if(distance<=min_distance)
		{
			min_distance = distance; 
             wp = root;
         }

		if(!(root->nextWps.size()))
		{
			return wp;
		}
		else
		{
			int cnt =root->nextWps.size();
			int i=0;
			while(cnt--)
			{
			    wp = findNearestNeighbour(pnt, root->nextWps[i++]);
			}
			
		}
		
			return wp;
		

	}


    template<int n>
    void RRT<n>::savePath()
    {
    	WaypointPtr<n> head = this->_goalWp;
    	while(head)
    	{
    		this->path.push_back(head);
    		head = head->prevWps[0]; //because in RRT each node has only one parent
    	}
    }
    template<int n>
	void  RRT<n>::plan()
	{
       int counter =0;
	  while(!this->bGoalFound)
	  {
	  	counter++;
	  	//if(counter == 100)
	  	  // bGoalFound =true;
	  	 
	  	     	std::cout<<"hi i am here 2"<<std::endl;

	  	 State<n> pnt;
	  	 sample(pnt);    //generate a random point add bias towards goal
	  	//find nearest neighbour
	  	     	std::cout<<"hi i am here 3 x="<<pnt[0]<<"    y="<<pnt[1]<<std::endl;


	  	 WaypointPtr<n> nearWp = findNearestNeighbour(pnt,this->_startWp);
	  	 findNearestNeighbour(pnt,NULL,true); //resetting
	  	     	std::cout<<"hi i am here 4 x="<<nearWp->coordinates[0]<<"        y="<<nearWp->coordinates[1]<<std::endl;


	  	 fixStepSize(nearWp,pnt);
	  	     	std::cout<<"hi i am here 5 x="<<pnt[0]<<"    y="<<pnt[1]<<std::endl;

         WaypointPtr<n> nextWp= new Waypoint<n>(pnt);

         nextWp->cost = nearWp->cost + nearWp->distance(nextWp->coordinates);
        //add edge and update graph
        nextWp->prevWps.push_back(nearWp);
        if(collisionCheck(nextWp))  
	  	//check for obstacle collison along edge
        {
        	    	std::cout<<"hi i am here 6"<<std::endl;

        	delete nextWp;
        	continue;
        }

        
        nearWp->nextWps.push_back(nextWp);

        std::cout<<"hi i am here 7"<<std::endl;

        if((nextWp->distance(this->_goalWp->coordinates) <= this->goalTol) /*|| counter == 99*/)
        {
        	this->_goalWp->prevWps.push_back(nextWp);
        	this->_goalWp->cost = nextWp->cost + nextWp->distance(this->_goalWp->coordinates);
        	nextWp->nextWps.push_back(this->_goalWp);
        	this->bGoalFound = true;
        	    	std::cout<<"hi i am here 22222222222222222222222"<<std::endl;


        	savePath();
        	    	std::cout<<"hi i am here 1111111111111111111111111"<<std::endl;

        	std::cout<<"Goal Found"<<std::endl;
        }


         	std::cout<<"hi i am here 888"<<std::endl;


	  	//terminate if goal reached within tolerance
	  }
		
	}

	template<int n>
	RRT<n>::RRT(double _maxStepSize, double _goalTol, std::vector<double>& ss_min, std::vector<double>& ss_max)/*:PathPlanner(maxStepSize,goalTol)*/
	{
		PathPlanner<n>::maxStepSize = _maxStepSize;
		PathPlanner<n>::goalTol = _goalTol;
		for(int i=0;i<ss_min.size();i++)
		{
			this->ss_min.push_back(ss_min[i]);
			this->ss_max.push_back(ss_max[i]);
		}
	}


	template<int n>
	void RRT<n>::setStart(const WaypointPtr<n> startWp)
	{
		this->_startWp = startWp;
		this->_startWp->prevWps.push_back(NULL);
		
		
	}

	template<int n>
	void RRT<n>::setGoal(const WaypointPtr<n> goalWp)
	{
		this->_goalWp = goalWp;
		this->bGoalSet = true;

	}

	template<int n>
	WaypointPtr<n> RRT<n>::getStart() const
	{
		return this->_startWp;
	}

	template<int n>
	WaypointPtr<n> RRT<n>::getGoal() const
	{
		return this->_goalWp;
	}

	template<int n>
	bool RRT<n>::collisionCheck(WaypointPtr<n> wp)
	{
		bool collision = false;
		for(int i=0;i<this->objects.size();i++)
		{
			//assuming only circular objects
			ObstaclePtr<n> object = this->objects[i];
			State<n> a = object->vertices[0];
			State<n> b = wp->coordinates;
			State<n> c = (wp->prevWps[0])->coordinates;
			double  perp_dist = abs((((a-c).cross(b-c)).norm())/(b-c).norm());
			if(wp->distance(object->vertices[0]) <= object->radius)
				collision = true;
			if((wp->prevWps[0])->distance(object->vertices[0])<=object->radius)
				collision = true;
			if(perp_dist>=(object->radius))
				collision = false;
			if(perp_dist<=(object->radius) && (((c-a).dot(b-c))*((b-a).dot(b-c)))>0)
				collision = false;
			else if(perp_dist<=(object->radius) && (((c-a).dot(b-c))*((b-a).dot(b-c)))<=0)
				collision = true;

		}

		return collision;
	}

}