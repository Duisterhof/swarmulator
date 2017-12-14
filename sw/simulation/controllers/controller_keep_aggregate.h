#ifndef CONTROLLER_KEEP_AGGREGATE_H
#define CONTROLLER_KEEP_AGGREGATE_H

#include <vector>
#include <stdio.h>
#include <iostream>
#include "controller.h"

#include <map>
#include <fstream>
#include "terminalinfo.h"
#include <sstream>
#include <random>
#include <iterator>
#include "auxiliary.h"

using namespace std;

class Controller_Keep_Aggregate: public Controller
{
  // Map of state-space index to possible action space indexes.
  std::map<int, vector<int>> state_action_matrix;

public:
  Controller_Keep_Aggregate();
  ~Controller_Keep_Aggregate(){};

  float f_attraction(float u, float b);
  float f_repulsion(float u);
  float f_extra(float u);
  float get_attraction_velocity(float u, float b_eq);
  virtual void get_velocity_command(const uint8_t ID, float &v_x, float &v_y);

  float get_preferred_bearing(const vector<float> &bdes, const float v_b);
  bool fill_template(vector<bool> &q, const float b_i, const float u, float dmax);
  void assess_situation(uint8_t ID, vector<bool> &q_old, vector<int> &q_old_ID);
};

#endif /*CONTROLLER_KEEP_AGGREGATE_H*/