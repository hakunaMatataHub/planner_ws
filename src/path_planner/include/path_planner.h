#include <iostream>
#include <random>
#include <vector>
#include <Eigen/Core>// to include eigen header

//typedef Waypoint<int>* Waypointptr;
namespace planner
{
	template<int n>
    using State = Eigen::Matrix<double, n, 1> ;

	template<int n>
	class Waypoint
	{
	public:

		Waypoint(const State<n> coordinates);
		double distance(const State<n>& coordinates);
		State<n> coordinates;
		double cost;

		std::vector<Waypoint<n>*> prevWps;
		std::vector<Waypoint<n>*> nextWps;

	};

	template<int n>
	 Waypoint<n>::Waypoint(const State<n> coordinates)
	{
		this->coordinates = coordinates;
		cost =0;
	}

	template<int n>
	double Waypoint<n>::distance(const State<n>& coordinates)
	{
		State<n> temp = this->coordinates - coordinates;
		return temp.norm();
	}
     
     template<int n>
     using WaypointPtr = Waypoint<n>* ;
     
     template <int n>
	class Object
	{
		

	   public:
	   	Object();
	   	bool bIsStatic;
	    std::vector<State<n>> vertices; //for polygonal obstacles (vertices stroes in cyclic order)
	    double radius;  //radius of smallest circle enclosing the polygon
	};

	template<int n>
	Object<n>::Object()
	{
		bIsStatic = true;
		radius =0;
	}

	template<int n>
	using ObstaclesPtr = std::vector<Object<n>*>;

	template<int n>
	using ObstaclePtr = Object<n>*;
	

	/*template<int n>
	using Waypointptr = Waypoint<n>* ;*/

	template<int n>
	class PathPlanner     //////abstract class
	{
		public:
			PathPlanner(const double maxStepSize, const double goalTol);
			PathPlanner();
			virtual void setGoal(const WaypointPtr<n>) =0;
			virtual WaypointPtr<n> getGoal() const =0;
			virtual void setStart(const WaypointPtr<n>)=0;
			virtual WaypointPtr<n> getStart() const =0;
			virtual void plan() =0;
			virtual void sample(State<n>& pnt) =0;
			virtual bool collisionCheck( const WaypointPtr<n> wp)=0; // makes a spherical approximation
            virtual bool addObstacle(const ObstaclePtr<n> object );
            virtual bool addObstacles(const ObstaclesPtr<n> objects);

		public:
			
            WaypointPtr<n> _goalWp;
            WaypointPtr<n> _startWp;
            

        public:
        	double maxStepSize;
        	double goalTol;
        	bool bGoalSet;
        	bool bCollisionDetection;
        	bool bGoalFound;
        	ObstaclesPtr<n> objects;
        	



	};

	template<int n>
	PathPlanner<n>::PathPlanner(const double maxStepSize, const double goalTol)
	{
		this->maxStepSize = maxStepSize;
		this->goalTol = goalTol;
		bGoalSet = false;
		bCollisionDetection = true;
		bGoalFound = false;
	}

	template<int n>
	PathPlanner<n>::PathPlanner()
	{
		
		bGoalSet = false;
		bCollisionDetection = true;
		bGoalFound = false;
	}


	template<int n>
	bool PathPlanner<n>::addObstacle(const ObstaclePtr<n> object)
	{
		
		this->objects.push_back(object);		

		return true;
	}

	template<int n>
	bool PathPlanner<n>::addObstacles(const ObstaclesPtr<n> objects)
	{
		for(int i=0;i<objects.size();i++)
		{
			addObstacle(objects[i]);

		}

		return true;
	}

}
 