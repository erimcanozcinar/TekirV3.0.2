#include "Eigen/Dense"
#include <allegro5/allegro.h>
#include <allegro5/allegro_native_dialog.h>

/* Trajectory variables */
Eigen::VectorXd walkTraj(16);
double Zc = 0.53319176863337994221048177223565;
double Vx_mean = 0.0;					            // Mean velocity of CoM in X-axis
double Vy_mean = 0.0;					            // Mean velocity of CoM in X-axis
int Nphase = 0;						                // Total number of phase(Each Ts + Td is one phase)
double px = 0.0;                                    // X ZMP
double py = 0.0;                                    // Y ZMP
double Ts = 0.28;						            // Single support phase period
double Td = 0.12;						            // Double support phase period
double Fc = 0.15;
double Cx, dCx, ddCx, Xzmp = 0, Yzmp = 0, Kphase;
double Cy, dCy, ddCy;

double Xc = 0.0, Yc = 0.0;
double dXc = 0.0, dYc = 0.0;
double ddXc = 0.0, ddYc = 0.0; 

/* Controller variables */
double Kv = 0.1;
double prev_vx_mean = 0.0, prev_vy_mean = 0.0;
bool walk_enabled = false;
bool can_switch = false;
bool can_stop = false;
double w_start_time = 0.0;

double MIN_BODY_HEIGHT = Zc*0.6;
double MAX_BODY_HEIGHT = 0.58;

/* Declerations */
void xboxController(ALLEGRO_EVENT_QUEUE *event_queue, ALLEGRO_EVENT event, double &Vx_mean, double &Vy_mean, bool &walkEnable, bool &close);
void dualShockController(ALLEGRO_EVENT_QUEUE *event_queue, ALLEGRO_EVENT ev, double (&joyCmd)[22], bool &walkEnable, bool &close);
Eigen::VectorXd comTrajectory(double RealTime, double Ts, double Td, int Nphase, double px, double py, double vx_mean, double vy_mean, double Cz, double Fh, double dt);
Eigen::VectorXd trajGeneration(double RealTime, bool walkEnable, double command_Vx, double command_Vy, double height, double dt);
