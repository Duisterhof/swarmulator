#ifndef PSO_H
#define PSO_H

#include <stdio.h>
#include <iostream>
#include "controller.h"
#include "auxiliary.h"
#include "omniscient_observer.h"
class laser_ray
{
	public:
		int num_walls; //number of walls that the laser ray intersects with
		float heading; //heading in ENU frame
		Point start_pos; //start position of the ray
		std::vector<std::vector<float>> walls; //all walls that the laser ray intersects with
		std::vector<Point> intersection_points; //intersection points with walls
		std::vector<float> distances;//distances to walls of intersection
		std::vector<std::vector<float>> intersecting_walls; //the walls it's intersecting with
		std::vector<float> intersect_wall; //the wall that it's intersecting with
		float range = 0.0 ; //final outcome, the measured range
		bool wall_following = false;
		// wall following params
		float heading_kp = 4.0;
		float heading_kd = 0.0;
		float heading_ki = 0.0;

		float heading_error = 0.0;
		float old_heading_error = 0.0;
		float heading_error_d = 0.0;
		float heading_error_i = 0.0;
		
		float heading_accumulator = 0.0; // what is finally added to heading
		float desired_laser_distance = 2.5; // desired minimal laser distance when following a wall
		float critical_laser_distance = 0.5; // when this point is reached we should be really really careful
		float engage_laser_distance = 2.7; // the end of the wall following zone, get more than this clearance to get out
		float old_accumulator = 0.0; // old accumulator used to limit the change in accumulation
		
};

class PSO: public Controller
{
public:
	PSO():Controller(){};
	virtual void get_velocity_command(const uint16_t ID, float &psi, float &v_x);
	virtual void animation(const uint16_t ID);
	laser_ray get_laser_reads(laser_ray ray, const uint16_t ID);
	float get_ray_control(laser_ray ray, float dt);
	float get_heading_to_point(Point agent, Point goal);
	bool get_follow_direction(std::vector<float> ranges, float desired_heading, float agent_heading);
	bool get_safe_direction(std::vector<float> ranges, float desired_heading, float threshold, float agent_heading);
	float get_agent_dist(const uint16_t ID1, const uint16_t ID2);
	float laser_headings[4] = {0,M_PI_2,M_PI,3*M_PI_2};
	OmniscientObserver o;
	std::vector<float> prev_x;
	std::vector<float> prev_y;

	float old_accumulator = 0.0; // old accumulators of rays
	float diff_accumulator = 0.0; // difference with old accumulator
	float max_accumulator_increase = 0.1 ; //max [rad] increase in vector, to avoid unstable behavior

	float iteration_start_time = 0.0;
	float local_vx, local_vy;
	float local_psi = 0.0;

	// AI-determined Parameters
	// configuration parameters for PSO
	float rand_p = 0.1;
	float omega = 0.5;
	float phi_p = 0.5;
	float phi_g = 2.5;
	float yaw_incr = 0.1;
	float update_time = 40.0;
	float dist_reached_goal = 0.5;
	bool follow_left = false; 
	bool started_agent_avoid = false;
	float swarm_rerout_time = 3.0;
	float started_swarm_avoid_time = 0.0;
	float swarm_avoidance_thres = 0.8;
	float swarm_avoidance_release = 1.5;
	float k_swarm_avoidance = 5.0;	
	float k_swarm_laser_rep = 5.0;
	float reset_wall_follow_time = 0.0;
	float reset_wall_follow_time_thres = 3.0; //[sec]

	// // wall following params
	// float heading_kp = 3.0;
	// float heading_kd = 30.0;
	// float heading_k_i = 0.05;

	// float heading_error = 0.0;
	// float old_heading_error = 0.0;
	// float heading_error_d = 0.0;
	// float heading_error_i = 0.0;
	
	// float heading_accumulator = 0.0; // what is finally added to heading

	// wall avoiding parameters
	float desired_velocity = 0.5;
	float desired_laser_distance = 1.0; // desired minimal laser distance when following a wall
	float critical_laser_distance = 0.5; // when this point is reached we should be really really careful
	float engage_laser_distance = 1.5; // the end of the wall following zone, get more than this clearance to get out
	float min_laser = desired_laser_distance; //the minimum found laser distance
	int min_laser_idx = 0; // the idx (hence direction) of the laser with lowest value
	float heading_accumulator = 0.0;


	bool wall_following = false;
	Point agent_pos, goal, random_point, other_agent_pos;
	std::vector<laser_ray> laser_rays;
	
};

#endif /*PSO_H*/
