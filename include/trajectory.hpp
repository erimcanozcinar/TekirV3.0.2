#ifndef TRAJECTORY_HPP
#define TRAJECTORY_HPP

#include "Eigen/Dense"
#include <allegro5/allegro.h>
#include <allegro5/allegro_native_dialog.h>
#include "functions.hpp"

/* Declerations */
void xboxController(ALLEGRO_EVENT_QUEUE *event_queue, ALLEGRO_EVENT event, double &Vx_mean, double &Vy_mean, bool &walkEnable, bool &close);
void dualShockController(ALLEGRO_EVENT_QUEUE *event_queue, ALLEGRO_EVENT ev, double (&joyCmd)[22], bool &walkEnable, bool &close);
Eigen::VectorXd comTrajectory(double RealTime, double Ts, double Td, int Nphase, double px, double py, double vx_mean, double vy_mean, double Cz, double Fh, double dt);
Eigen::VectorXd trajGeneration(double RealTime, bool walkEnable, double command_Vx, double command_Vy, double height, double dt);

#endif