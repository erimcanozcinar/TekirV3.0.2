#ifndef TRAJECTORY_HPP
#define TRAJECTORY_HPP

#include "Eigen/Dense"
#include <allegro5/allegro.h>
#include <allegro5/allegro_native_dialog.h>
#include "functions.hpp"

class controller {
    protected:
        double Zc = 0.53319176863337994221048177223565;
        double Kv = 0.1;
        double MIN_BODY_HEIGHT = Zc*0.6;
        double MAX_BODY_HEIGHT = 0.58;
        double Vx_mean = 0.0, Vy_mean = 0.0;
        double cmdZc = Zc;
        double roll = 0.0, pitch = 0.0, yaw = 0.0;
        double decreaseHeight = 0.0, increaseHeight = 0.0;;

    public:
        double joyCmd[22] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.53319176863337994221048177223565, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};;
        bool close = false;
        bool walkEnable = false;

        void xboxController(ALLEGRO_EVENT_QUEUE *event_queue, ALLEGRO_EVENT event, ALLEGRO_JOYSTICK* joyStick);
        void dualShockController(ALLEGRO_EVENT_QUEUE *event_queue, ALLEGRO_EVENT ev);
};

class trajectory {
    protected:
        double prev_vx_mean = 0.0, prev_vy_mean = 0.0;
        bool walk_enabled = false;
        bool can_switch = false;
        bool can_stop = false;
        double w_start_time = 0.0;

        double Zc = 0.53319176863337994221048177223565;
        double Vx_mean = 0.0;					            // Mean velocity of CoM in X-axis
        double Vy_mean = 0.0;					            // Mean velocity of CoM in X-axis
        int Nphase = 0;						                // Total number of phase(Each Ts + Td is one phase)
        double px = 0.0;                                    // X ZMP
        double py = 0.0;                                    // Y ZMP
        double Ts = 0.28;						            // Single support phase period
        double Td = 0.12;						            // Double support phase period
        double Fc = 0.15;
        double Xzmp = 0, Yzmp = 0, Kphase;

        double Cx, dCx, ddCx;
        double Cy, dCy, ddCy;

        double Comx, Comy;
        double Strx = 0.0, Stry = 0.0;

    public:
        double Xcop, Ycop;

        Eigen::Vector3d Footz_L, Footz_R, Footx_L, Footx_R, Footy_L, Footy_R;

        double Xc = 0.0, Yc = 0.0;
        double dXc = 0.0, dYc = 0.0;
        double ddXc = 0.0, ddYc = 0.0;
        double height = Zc;

        Eigen::Vector3d Pfoot_R, Pfoot_L;

        void comTrajectory(double RealTime, double Ts, double Td, int Nphase, double px, double py, double vx_mean, double vy_mean, double Cz, double Fh, double dt);
        void trajGeneration(double RealTime, bool walkEnable, double command_Vx, double command_Vy, double height, double dt);
};

/* Declerations */



#endif