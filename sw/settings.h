#ifndef SETTINGS_H
#define SETTINGS_H

// Options
// #define SEQUENTIAL 5
#define ESTIMATOR
// #define REMAIN_CONNECTED // Check that the swarm remains connected (only if logging)
// #define CHECK_HAPPY // Check whether the global goal is completed  (only if logging)
#define COMMAND_LOCAL 1  // use COMMAND_LOCAL for local commands

/**
 * Noise in relative sensing
 */
// TODO: Make runtime variable
#define NOISE_R 0 // STDEV of noise in range
#define NOISE_B 0 // STDEV of noise in bearing

#endif /*SETTINGS_H*/
