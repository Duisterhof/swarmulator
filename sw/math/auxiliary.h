#ifndef AUXILIARY_H
#define AUXILIARY_H

#include <stdlib.h> // qsort
#include <cmath>
#include <vector>
#include <string>

using namespace std;

inline static int bool2int(vector<bool> t)
{
  int n = 0; //initialize
  for (int i = 0; i < 8; i++) {
    n += (int)t[i] * (int)pow(2, 7 - i);
  }
  return n;
}

/* Keeps a value between two bounds */
inline static void keepbounded(float &value, float min, float max)
{
  if (value < min) { value = min; }
  else if (value > max) { value = max; }
}

/* Wraps an angle in radians between -PI and +PI */
inline static void wrapToPi(float &ang)
{
  if (ang >  M_PI) { while (ang >  M_PI) { ang = ang - 2 * M_PI;} }
  else if (ang < -M_PI) { while (ang < -M_PI) { ang = ang + 2 * M_PI;} }
}

inline static void wrapTo2Pi(float &ang)
{
  while (ang > 2 * M_PI) {
    ang = ang - 2 * M_PI;
  }
  while (ang < 0.0) {
    ang = ang + 2 * M_PI;
  }
}

inline static int wraptosequence(int x, int min, int max)
{
  while (x > max) {
    x = x - max;
  }
  while (x < min) {
    x = x + max;
  }
  return x;
}

inline static float wrapToPi_f(float ang)
{
  if (ang > M_PI) {
    while (ang > M_PI) {
      ang = ang - 2 * M_PI;
    }
  } else if (ang < -M_PI) {
    while (ang < -M_PI) {
      ang = ang + 2 * M_PI;
    }
  }
  return ang;
}

inline static float wrapTo2Pi_f(float ang)
{
  if (ang > 2 * M_PI) {
    while (ang > 2 * M_PI) {ang = ang - 2 * M_PI;}
  } else if (ang < 0.0) {
    while (ang < 0.0) {
      ang = ang + 2 * M_PI;
    }
  }
  return ang;
}

inline static float rad2deg(float rad) { return 180.0 / M_PI * rad; }

inline static float deg2rad(float deg) { return M_PI / 180.0 * deg; }

template <typename Iter, typename RandomGenerator>
inline static Iter select_randomly(Iter start, Iter end, RandomGenerator &g)
{
  std::uniform_int_distribution<> dis(0, std::distance(start, end) - 1);
  std::advance(start, dis(g));
  return start;
}

template <typename Iter>
inline static Iter select_randomly(Iter start, Iter end)
{
  static std::random_device rd;
  static std::mt19937 gen(rd());
  return select_randomly(start, end, gen);
}

#endif /*AUXILIARY_H*/