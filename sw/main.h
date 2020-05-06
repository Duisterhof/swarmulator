#ifndef MAIN_H
#define MAIN_H

#include <mutex>
#include <vector>
#include "agent.h"
#include "parameters.hxx" // Auto generated file at compile time
#include "environment.h"
#include "settings.h"

#ifdef ESTIMATOR
#include "pagerank_estimator.h"
extern pagerank_estimator pr;
#endif

extern uint nagents;            // Number of agents in the swarm
extern std::vector<Agent *> s;  // Set up a vector of agents
extern float simtime_seconds;   // Adjusted simulation time (time according to simulation)
extern std::mutex mtx;          // Mutex object
extern bool program_running;    // True if the program is (or should be) running. If false the program shuts down.
extern unique_ptr<parameters_t> param; // XML parameters from conf file
extern float realtimefactor;    // Real time factor of simulation
extern float rangesensor;       // How far each robot can sense
extern Environment environment;
extern string identifier;

#endif /*MAIN_H*/
