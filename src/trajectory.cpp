#include "trajectory.hpp"
#include "functions.cpp"

void xboxController(ALLEGRO_EVENT_QUEUE *event_queue, ALLEGRO_EVENT event, ALLEGRO_JOYSTICK* joyStick, double (&joyCmd)[22], bool &walkEnable, bool &close)
{
    
    /* Gamepad routines */
    al_wait_for_event_timed(event_queue, &event, 0.0001);
    switch (event.type) {
        case ALLEGRO_EVENT_JOYSTICK_CONFIGURATION:
            printf("Joystick configuration changed.\n");
            break;
        case ALLEGRO_EVENT_JOYSTICK_AXIS:
            // Handle gamepad axis events
            switch (event.joystick.stick) {
                case 0: /* Left Stick */
                    switch (event.joystick.axis) {
                        case 0: /* y axis */
                            Vy_mean = -0.15*event.joystick.pos;
                            Vy_mean = floor(Vy_mean*10000)/10000; /* Truncate to 3 decimal */
                            if(abs(Vy_mean)<0.03) { Vy_mean = 0.0; }
                            joyCmd[1] = Vy_mean;
                            break;
                        case 1: /* x axis */
                            Vx_mean = -Kv*event.joystick.pos;                                
                            Vx_mean = floor(Vx_mean*10000)/10000; /* Truncate to 3 decimal */
                            if(abs(Vx_mean)<0.03) { Vx_mean = 0.0; }
                            joyCmd[0] = Vx_mean;
                            break;
                    }
                    break;                    
                case 1: /* Right Stick */
                    switch (event.joystick.axis) {
                        case 0: /* y axis */
                            std::cout << event.joystick.axis << ": " << event.joystick.pos << std::endl;
                            break;
                        case 1: /* x axis */
                            std::cout << event.joystick.axis << ": " << event.joystick.pos << std::endl;
                            break;
                    }
                    break; 
                case 2: /* D-pad */
                    switch (event.joystick.axis) {
                        case 0: /* D-pad x axis */                    
                            break;                    
                        case 1: /* D-pad y axis */
                            if(event.joystick.pos == -1) /* D-pad Up */
                            {
                                if(Kv > 0.6){Kv = 0.6;}                                
                                else { Kv = Kv + 0.2; }
                                std::cout << "Max. Vel. Set to:" << Kv << std::endl;
                            }
                            else if(event.joystick.pos == 1) /* D-pad Down */
                            {
                                if(Kv <= 0.2){Kv = 0.2;}                                
                                else { Kv = Kv - 0.2; }
                                std::cout << "Max. Vel. Set to:" << Kv << std::endl;
                            }
                            else
                            {
                                Kv = Kv;
                            }
                            break;
                    }
                    break; 
            }
            break;
        case ALLEGRO_EVENT_JOYSTICK_BUTTON_DOWN:
            switch (event.joystick.button){
                case 0:
                    /* Y Button */
                    std::cout << "Y: " << event.joystick.button << std::endl;
                    break;
                case 1:
                    /* B Button */
                    std::cout << "B: " << event.joystick.button << std::endl;
                    break;
                case 2:
                    /* A Button */
                    std::cout << "A: " << event.joystick.button << std::endl;
                    break;
                case 3:
                    /* X Button */
                    std::cout << "X: " << event.joystick.button << std::endl;
                    break;
                case 4:
                    /* LB Button */
                    std::cout << "LB: " << event.joystick.button << std::endl;
                    break;
                case 5:
                    /* RB Button */
                    std::cout << "RB: " << event.joystick.button << std::endl;
                    break;
                case 6:
                    /* LT Button */
                    std::cout << "LT: " << event.joystick.button << std::endl;
                    break;
                case 7:
                    /* RT Button */
                    std::cout << "RT: " << event.joystick.button << std::endl;
                    break;
                case 8:
                    /* Back Button */
                    std::cout << "Back: " << event.joystick.button << std::endl;
                    close = true;
                    break;
                case 9:
                    /* Start Button */
                    if(walkEnable){ 
                        walkEnable = false; 
                    }
                    else{ 
                        walkEnable = true; 
                    }            
                    break;
                case 10:
                    /* Back Button */
                    std::cout << "LSB: " << event.joystick.button << std::endl;
                    break;
                case 11:
                    /* Start Button */
                    std::cout << "RSB: " << event.joystick.button << std::endl;
                    break;
                
                default:
                    break;
            }
            break;
        case ALLEGRO_EVENT_JOYSTICK_BUTTON_UP:
            // Handle gamepad button up events
            // event.joystick.button contains the button ID
            // Add your game logic here
            break;
        // Add more cases for other event types if needed
        default:
            // Handle other event types or do nothing
            break;
    }
}

void dualShockController(ALLEGRO_EVENT_QUEUE *event_queue, ALLEGRO_EVENT ev, double (&joyCmd)[22], bool &walkEnable, bool &close)
{
    ALLEGRO_JOYSTICK *joyStick = al_get_joystick(0);
    ALLEGRO_JOYSTICK_STATE joyState;
    if(joyStick) {al_get_joystick_state(joyStick, &joyState);}

    double Vx_mean = 0.0, Vy_mean = 0.0;
    double cmdZc = Zc; 
    double decreaseHeight = 0.0, increaseHeight = 0.0;;
    double roll = 0.0, pitch = 0.0, yaw = 0.0;

    al_wait_for_event_timed(event_queue, &ev,0.0001);
    switch (ev.type) {
        case ALLEGRO_EVENT_JOYSTICK_CONFIGURATION:
            al_reconfigure_joysticks();
        case ALLEGRO_EVENT_JOYSTICK_AXIS:
            switch (ev.joystick.stick) {
                case 0: /* Left-Stick */                        
                    switch (ev.joystick.axis) {
                        case 0: /* y-axis */
                            // std::cout << "Left stick y-axis: " << ev.joystick.pos << std::endl;
                            Vy_mean = -0.15*ev.joystick.pos;
                            Vy_mean = floor(Vy_mean*10000)/10000; /* Truncate to 3 decimal */
                            if(abs(Vy_mean)<0.03) { Vy_mean = 0.0; }
                            joyCmd[1] = Vy_mean;
                            break;
                        case 1: /* x-axis */
                            // std::cout << "Left stick x-axis: " << ev.joystick.pos << std::endl;
                            Vx_mean = -Kv*ev.joystick.pos;                                
                            Vx_mean = floor(Vx_mean*10000)/10000; /* Truncate to 3 decimal */
                            if(abs(Vx_mean)<0.03) { Vx_mean = 0.0; }
                            joyCmd[0] = Vx_mean;
                            break;
                    }
                    break;                    
                case 1: /* Right stick y-axis and L2 axis */
                    switch (ev.joystick.axis) {
                        case 0: /* L2 axis */
                            cmdZc = mapVal(ev.joystick.pos, 1.0, -1.0, MIN_BODY_HEIGHT, Zc);
                            joyCmd[5] = cmdZc;
                            break;
                        case 1:
                            // std::cout << "Right stick y-axis: " << ev.joystick.pos << std::endl;
                            if(walkEnable) { 
                                yaw = -0.17*ev.joystick.pos;
                                roll = 0.0; 
                            }
                            else {
                                if(joyState.button[4]) { roll = 0.174*ev.joystick.pos; }
                                else { yaw = -0.35*ev.joystick.pos; }                                 
                            }
                            if(abs(yaw) < 0.03) { yaw = 0.0;}
                            if(abs(roll) < 0.03) { roll = 0.0;}
                            joyCmd[2] = yaw;
                            joyCmd[4] = roll;
                            break;
                    }
                    break;
                case 2: /* Right stick x-axis and R2 axis */
                    switch (ev.joystick.axis) {
                        case 0: /* Right stick x-axis */
                            // std::cout << "Right stick x-axis: " << ev.joystick.pos << std::endl;
                            if(!walkEnable) { 
                                pitch = 0.1*ev.joystick.pos;
                            }                               
                            else { 
                                pitch = 0.0;
                            }
                            if(abs(pitch) < 0.03) { pitch = 0.0;}
                            joyCmd[3] = pitch;
                            break;
                        case 1: /* R2 axis */
                            cmdZc = mapVal(ev.joystick.pos, 1.0, -1.0, MAX_BODY_HEIGHT, Zc);
                            joyCmd[5] = cmdZc;
                            break;
                    }
                    break;
                case 3: /* D-pad */
                    switch (ev.joystick.axis) {
                        case 0: /* D-pad R/L */
                            // std::cout << "D-pad R/L: " << ev.joystick.pos << std::endl;
                            break;
                        case 1: /* D-pad U/D */
                            Kv = Kv - 0.1*ev.joystick.pos;
                            if(Kv >= 1.0) { Kv = 1.0;}
                            else if(Kv <= 0.1) { Kv = 0.1; }
                            if(abs(ev.joystick.pos)) { std::cout << "Max. Vel. set to: " << Kv << std::endl; }                                              
                            break;
                    }
                    break;
            }
            break;
        case ALLEGRO_EVENT_JOYSTICK_BUTTON_DOWN:
            switch (ev.joystick.button) {
                // case 0: /* X Button */
                //     std::cout << "X: " << ev.joystick.button << std::endl;
                //     break;
                // case 1: /* Circle Button */
                //     std::cout << "Circle: " << ev.joystick.button << std::endl;
                //     break;
                // case 2: /* Triangle Button */
                //     std::cout << "Triangle: " << ev.joystick.button << std::endl;
                //     break;
                // case 3: /* Square Button */
                //     std::cout << "Square: " << ev.joystick.button << std::endl;
                //     break;
                // case 4: /* L1 Button */
                //     std::cout << "L1: " << ev.joystick.button << std::endl;                                        
                //     break;
                // case 5: /* R1 Button */
                //     std::cout << "R1: " << ev.joystick.button << std::endl;
                //     break;
                case 6: /* L2 Button */
                    // std::cout << "L2: " << ev.joystick.button << std::endl;
                    break;
                case 7: /* R2 Button */
                    // std::cout << "R2: " << ev.joystick.button << std::endl;
                    break;
                case 8: /* Back Button */
                    // std::cout << "Share: " << ev.joystick.button << std::endl;
                    close = true;
                    al_release_joystick(joyStick);
                    break;
                case 9: /* Options */
                    if(walkEnable){ 
                        walkEnable = false;
                        std::cout << "Walking mode disabled!" << std::endl; 
                    }
                    else{ 
                        walkEnable = true;
                        std::cout << "Walking mode enabled!" << std::endl;
                    } 
                    break;
                // case 10: /* PS Button */
                //     std::cout << "PS: " << ev.joystick.button << std::endl;
                //     break;
                // case 11: /* LS Button */
                //     std::cout << "LS: " << ev.joystick.button << std::endl;
                //     break;
                // case 12: /* RS Button */
                //     std::cout << "RS: " << ev.joystick.button << std::endl;
                //     break;
                
                default:
                    break;
            }
            break;
        case ALLEGRO_EVENT_JOYSTICK_BUTTON_UP:
            // Handle gamepad button up events
            // ev.joystick.button contains the button ID
            // std::cout << ev.joystick.button << std::endl;
            // Add your game logic here
            break;
        // Add more cases for other event types if needed
        default:
            // Handle other event types or do nothing
            break;
    }
}

Eigen::VectorXd comTrajectory(double RealTime, double Ts, double Td, int Nphase, double px, double py, double vx_mean, double vy_mean, double Cz, double Fh, double dt)
{    
    double t0 = 0.0; // Beginning of the Universe(or Simulation)
    double t1 = t0 + 0; // Put robot on the ground
    double t2 = t1 + Ts; // First half step
    double tstart_w = t2 + Td; // Walking start
    double tend_w = tstart_w + (Ts + Td) * Nphase; // Walking end
    double t3 = tend_w + Ts; // CoM stoping phase

    double g = 9.81; // Gravitational acceleration
    double w = sqrt(g / Cz); // Natural freq of equivalent pendulum

    Eigen::Vector3d trajComX, trajComY, trajFootz_L, trajFootz_R, trajFootx_L, trajFootx_R, trajFooty_L, trajFooty_R;
    Eigen::VectorXd returnVals(16);
    double Cx, dCx, ddCx, Xcop, Posx_l, Posz_l, Posx_r, Posz_r;
    double Cy, dCy, ddCy, Ycop, Posy_l, Posy_r;

    /* Phase Calculation Start(Each Ts + Td is one phase) */
    int k, kx;
    double Rt4Fr = (RealTime - tstart_w) / (Ts + Td);
    if (FuncInterval(RealTime, t0, tstart_w, dt) == true)
    {
        k = 0;
        kx = 0;
    }
    else if (FuncGreater(RealTime, tend_w, dt) == true)
    {
        k = Nphase - 1;
        kx = floor(k / 2);
    }
    else
    {
        k = floor(Rt4Fr);
        kx = floor(Rt4Fr / 2);
    }
    /* Phase Calculation Start(Each Ts + Td is one phase) */
        
    // Xcom parameters
    double Cxd = (px + vx_mean * Ts / 2);
    double Cx0 = (2 * px - Cxd);
    double dCx0 = w * (px - Cx0) * 1 / tanh(w * Ts / 2);
    double ddCx0 = w * w * (Cx0 - px);
    double d_Cx0 = Cxd;
    double d_dCx0 = dCx0;

    double Kx = d_dCx0 + w * (Cxd - px) * 1 / tanh(w * Td / 2);
    double Strx = 2 * Td * Kx;
    double Cxd0 = Cx0 + Strx / 2;

    // Ycom parameters
    double Cyd = (py + vy_mean * Ts / 2);
    double Cy0 = (2 * py - Cyd);
    double dCy0 = w * (py - Cy0) * 1 / tanh(w * Ts / 2);
    double ddCy0 = w * w * (Cy0 - py);
    double d_Cy0 = Cyd;
    double d_dCy0 = dCy0;

    double Ky = d_dCy0 + w * (Cyd - py) * 1 / tanh(w * Td / 2);
    double Stry = 2 * Td * Ky;
    double Cyd0 = Cy0 + Stry / 2;
    
    /* CoM Trajectory Start */
    if (FuncInterval(RealTime, t0, t1, dt) == true) // Put robot on the ground
    {
        Cx = 0; dCx = 0; ddCx = 0;
        Cy = 0; dCy = 0; ddCy = 0;
    }
    else if (FuncInterval(RealTime, t1, t2, dt) == true) // Initialize CoM position (First half step)
    {
        trajComX = FuncPoly5th(RealTime, t1, t2, 0, 0, 0, Cxd, dCx0, -ddCx0);
        trajComY = FuncPoly5th(RealTime, t1, t2, 0, 0, 0, Cyd, dCy0, -ddCy0);
        Cx = trajComX(0); dCx = trajComX(1); ddCx = trajComX(2);
        Cy = trajComY(0); dCy = trajComY(1); ddCy = trajComY(2);
    }
    else if (FuncInterval(RealTime, t2, tstart_w, dt) == true) // Initialite CoM trajectory (Just before walking starts)
    {
        trajComX = FuncPoly5th(RealTime, t2, tstart_w, Cxd, dCx0, -ddCx0, Cxd0, dCx0, ddCx0);
        trajComY = FuncPoly5th(RealTime, t2, tstart_w, Cyd, dCy0, -ddCy0, Cyd0, dCy0, ddCy0);
        Cx = trajComX(0); dCx = trajComX(1); ddCx = trajComX(2);
        Cy = trajComY(0); dCy = trajComY(1); ddCy = trajComY(2);
    }
    else if (FuncInterval(RealTime, tstart_w, tend_w, dt) == true) // Start walking
    {
        double Rt = RealTime - tstart_w - (Ts + Td) * (k);
        if (FuncInterval(Rt, 0, Ts, dt) == true) // Single Support Phase
        {
            double s_T = Rt;
            Cx = (Cx0 - px) * cosh(w * s_T) + (dCx0 / w) * sinh(w * s_T) + px;
            dCx = w * (Cx0 - px) * sinh(w * s_T) + dCx0 * cosh(w * s_T);
            ddCx = w * w * (Cx0 - px) * cosh(w * s_T) + w * dCx0 * sinh(w * s_T);
            Cy = (Cy0 - py) * cosh(w * s_T) + (dCy0 / w) * sinh(w * s_T) + py;
            dCy = w * (Cy0 - py) * sinh(w * s_T) + dCy0 * cosh(w * s_T);
            ddCy = w * w * (Cy0 - py) * cosh(w * s_T) + w * dCy0 * sinh(w * s_T);
            Cx = Cx + (k + 1) * Strx / 2;
            Cy = Cy + (k + 1) * Stry / 2;
        }
        else if (FuncInterval(Rt, Ts, Ts + Td, dt) == true) // Double Support Phase
        {
            double d_T = Rt - Ts;
            Cx = (d_Cx0 - px) * cosh(w * d_T) + ((d_dCx0 - Kx) / w) * sinh(w * d_T) + Kx * d_T + px;
            dCx = w * (d_Cx0 - px) * sinh(w * d_T) + (d_dCx0 - Kx) * cosh(w * d_T) + Kx;
            ddCx = w * w * (d_Cx0 - px) * cosh(w * d_T) + w * (d_dCx0 - Kx) * sinh(w * d_T);
            Cy = (d_Cy0 - py) * cosh(w * d_T) + ((d_dCy0 - Ky) / w) * sinh(w * d_T) + Ky * d_T + py;
            dCy = w * (d_Cy0 - py) * sinh(w * d_T) + (d_dCy0 - Ky) * cosh(w * d_T) + Ky;
            ddCy = w * w * (d_Cy0 - py) * cosh(w * d_T) + w * (d_dCy0 - Ky) * sinh(w * d_T);
            Cx = Cx + (k + 1) * Strx / 2;
            Cy = Cy + (k + 1) * Stry / 2;
        }
    }
    else if (FuncInterval(RealTime, tend_w, t3, dt) == true) // Stop CoM trajectory(After walking ends)
    {
        double PosX_start = Cxd0 + (k + 1) * Strx / 2;
        double PosX_end = Strx / 2 + (k + 1) * Strx / 2;
        double PosY_start = Cyd0 + (k + 1) * Stry / 2;
        double PosY_end = Stry / 2 + (k + 1) * Stry / 2;
        trajComX = FuncPoly5th(RealTime,  tend_w,  t3,  PosX_start,  dCx0,  ddCx0, PosX_end, 0, 0);
        trajComY = FuncPoly5th(RealTime,  tend_w,  t3,  PosY_start,  dCy0,  ddCy0, PosY_end, 0, 0);
        Cx = trajComX(0); dCx = trajComX(1); ddCx = trajComX(2);
        Cy = trajComY(0); dCy = trajComY(1); ddCy = trajComY(2);
    }
    else
    {
        Cx = Strx / 2 + (k + 1) * Strx / 2; dCx = 0; ddCx = 0;
        Cy = Stry / 2 + (k + 1) * Stry / 2; dCy = 0; ddCy = 0;
    }

    Xcop = Cx - Cz * ddCx / g;
    Ycop = Cy - Cz * ddCy / g;
    /* CoM Trajectory End */

    /* Feet Trajectory Start */
    double Velx_r, Accx_r, Velx_l, Accx_l;
    double Vely_r, Accy_r, Vely_l, Accy_l;
    double Velz_r, Accz_r, Velz_l, Accz_l;
    if (FuncInterval(RealTime, t0, t1, dt) == true) // Put robot on the ground
    {
        Posx_r = 0; Velx_r = 0; Accx_r = 0;
        Posy_r = 0; Vely_r = 0; Accy_r = 0;
        Posz_r = 0; Velz_r = 0; Accz_r = 0;

        Posx_l = 0; Velx_l = 0; Accx_l = 0;
        Posy_l = 0; Vely_l = 0; Accy_l = 0;
        Posz_l = 0; Velz_l = 0; Accz_l = 0;
        
    }
    else if (FuncInterval(RealTime, t1, t2, dt) == true) // Z - axis Foot trajectory initialization(First half step)
    {
        trajFootx_L = FuncPoly5th(RealTime, t1, t2, 0, 0, 0, Strx / 2, 0, 0);
        trajFooty_L = FuncPoly5th(RealTime, t1, t2, 0, 0, 0, Stry / 2, 0, 0);
        trajFootz_L = FuncPoly6th(RealTime, t1, t2, 0, 0, 0, 0, 0, 0, Fh);

        Posx_l = trajFootx_L(0); Velx_l = trajFootx_L(1); Accx_l = trajFootx_L(2);
        Posy_l = trajFooty_L(0); Vely_l = trajFooty_L(1); Accy_l = trajFooty_L(2);
        Posz_l = trajFootz_L(0); Velz_l = trajFootz_L(1); Accz_l = trajFootz_L(2);

        Posx_r = 0; Velx_r = 0; Accx_r = 0;
        Posy_r = 0; Vely_r = 0; Accy_r = 0;
        Posz_r = 0; Velz_r = 0; Accz_r = 0;
        
    }
    else if (FuncInterval(RealTime, t2, tstart_w, dt) == true)
    {
        Posx_r = 0; Velx_r = 0; Accx_r = 0;
        Posy_r = 0; Vely_r = 0; Accy_r = 0;
        Posz_r = 0; Velz_r = 0; Accz_r = 0;

        Posx_l = Strx / 2; Velx_l = 0; Accx_l = 0;
        Posy_l = Stry / 2; Vely_l = 0; Accy_l = 0;
        Posz_l = 0; Velz_l = 0; Accz_l = 0;
        
    }
    else if (FuncInterval(RealTime, tstart_w, tend_w, dt) == true) // Z - axis Foot trajectory afater initialization
    {
        if (((k + 1) % 2) == 0) // Left foot swing, right foot stand
        {
            double ts = tstart_w + (Ts + Td) * k;
            
            trajFootx_L = FuncPoly5th(RealTime, ts, ts + Ts, Strx / 2 + Strx * kx, 0, 0, Strx / 2 + Strx * (kx + 1), 0, 0);
            trajFooty_L = FuncPoly5th(RealTime, ts, ts + Ts, Stry / 2 + Stry * kx, 0, 0, Stry / 2 + Stry * (kx + 1), 0, 0);
            trajFootz_L = FuncPoly6th(RealTime, ts, ts + Ts, 0, 0, 0, 0, 0, 0, Fh);

            trajFootx_R = FuncPoly5th(RealTime, ts, ts + Ts, Strx * (kx + 1), 0, 0, Strx * (kx + 1), 0, 0);
            trajFooty_R = FuncPoly5th(RealTime, ts, ts + Ts, Stry * (kx + 1), 0, 0, Stry * (kx + 1), 0, 0);
            trajFootz_R = FuncPoly6th(RealTime, ts, ts + Ts, 0, 0, 0, 0, 0, 0, 0);

            Posx_l = trajFootx_L(0); Velx_l = trajFootx_L(1); Accx_l = trajFootx_L(2);
            Posy_l = trajFooty_L(0); Vely_l = trajFooty_L(1); Accy_l = trajFooty_L(2);
            Posz_l = trajFootz_L(0); Velz_l = trajFootz_L(1); Accz_l = trajFootz_L(2);
            
            Posx_r = trajFootx_R(0); Velx_r = trajFootx_R(1); Accx_r = trajFootx_R(2);
            Posy_r = trajFooty_R(0); Vely_r = trajFooty_R(1); Accy_r = trajFooty_R(2);
            Posz_r = trajFootz_R(0); Velz_r = trajFootz_R(1); Accz_r = trajFootz_R(2);
        }
        else // Right foot swing, left foot stand
        {
            double ts = tstart_w + (Ts + Td) * k;

            trajFootx_L = FuncPoly5th(RealTime, ts, ts + Ts, Strx / 2 + Strx * kx, 0, 0, Strx / 2 + Strx * kx, 0, 0);
            trajFooty_L = FuncPoly5th(RealTime, ts, ts + Ts, Stry / 2 + Stry * kx, 0, 0, Stry / 2 + Stry * kx, 0, 0);
            trajFootz_L = FuncPoly6th(RealTime, ts, ts + Ts, 0, 0, 0, 0, 0, 0, 0);

            trajFootx_R = FuncPoly5th(RealTime, ts, ts + Ts, Strx * kx, 0, 0, Strx * (kx + 1), 0, 0);
            trajFooty_R = FuncPoly5th(RealTime, ts, ts + Ts, Stry * kx, 0, 0, Stry * (kx + 1), 0, 0);
            trajFootz_R = FuncPoly6th(RealTime, ts, ts + Ts, 0, 0, 0, 0, 0, 0, Fh);

            Posx_l = trajFootx_L(0); Velx_l = trajFootx_L(1); Accx_l = trajFootx_L(2);
            Posy_l = trajFooty_L(0); Vely_l = trajFooty_L(1); Accy_l = trajFooty_L(2);
            Posz_l = trajFootz_L(0); Velz_l = trajFootz_L(1); Accz_l = trajFootz_L(2);

            Posx_r = trajFootx_R(0); Velx_r = trajFootx_R(1); Accx_r = trajFootx_R(2);
            Posy_r = trajFooty_R(0); Vely_r = trajFooty_R(1); Accy_r = trajFooty_R(2);
            Posz_r = trajFootz_R(0); Velz_r = trajFootz_R(1); Accz_r = trajFootz_R(2);
        }
    }
    else if (FuncInterval(RealTime, tend_w, t3, dt) == true)
    {
        if (((k + 1) % 2) == 0)
        {
            trajFootx_R = FuncPoly5th(RealTime, tend_w, t3, Strx * (kx + 1), 0, 0, Strx / 2 + Strx * (kx + 1), 0, 0);
            trajFooty_R = FuncPoly5th(RealTime, tend_w, t3, Stry * (kx + 1), 0, 0, Stry / 2 + Stry * (kx + 1), 0, 0);
            trajFootz_R = FuncPoly6th(RealTime, tend_w, t3, 0, 0, 0, 0, 0, 0, Fh);
            
            Posx_r = trajFootx_R(0); Velx_r = trajFootx_R(1); Accx_r = trajFootx_R(2);
            Posy_r = trajFooty_R(0); Vely_r = trajFooty_R(1); Accy_r = trajFooty_R(2);
            Posz_r = trajFootz_R(0); Velz_r = trajFootz_R(1); Accz_r = trajFootz_R(2);

            Posx_l = Strx / 2 + Strx * (kx + 1); Velx_l = 0; Accx_l = 0;
            Posy_l = Stry / 2 + Stry * (kx + 1); Vely_l = 0; Accy_l = 0;
            Posz_l = 0; Velz_l = 0; Accz_l = 0;
        }
        else
        {
            trajFootx_L = FuncPoly5th(RealTime, tend_w, t3, Strx / 2 + Strx * (kx), 0, 0, Strx + Strx * (kx), 0, 0);
            trajFooty_L = FuncPoly5th(RealTime, tend_w, t3, Stry / 2 + Stry * (kx), 0, 0, Stry + Stry * (kx), 0, 0);
            trajFootz_L = FuncPoly6th(RealTime, tend_w, t3, 0, 0, 0, 0, 0, 0, Fh);

            Posx_l = trajFootx_L(0); Velx_l = trajFootx_L(1); Accx_l = trajFootx_L(2);
            Posy_l = trajFooty_L(0); Vely_l = trajFooty_L(1); Accy_l = trajFooty_L(2);
            Posz_l = trajFootz_L(0); Velz_l = trajFootz_L(1); Accz_l = trajFootz_L(2);

            Posx_r = Strx * (kx + 1); Velx_r = 0; Accx_r = 0;
            Posy_r = Stry * (kx + 1); Vely_r = 0; Accy_r = 0;
            Posz_r = 0; Velz_r = 0; Accz_r = 0;
        }       
    }
    else
    {
        Posz_l = 0; Velz_l = 0; Accz_l = 0;
        Posz_r = 0; Velz_r = 0; Accz_r = 0;
        if (((k + 1) % 2) == 0)
        {
            Posx_l = Strx / 2 + Strx * (kx + 1); Velx_l = 0; Accx_l = 0;
            Posx_r = Strx / 2 + Strx * (kx + 1); Velx_r = 0; Accx_r = 0;
            Posy_l = Stry / 2 + Stry * (kx + 1); Vely_l = 0; Accy_l = 0;
            Posy_r = Stry / 2 + Stry * (kx + 1); Vely_r = 0; Accy_r = 0;
        }
        else
        {
            Posx_l = Strx * (kx + 1); Velx_l = 0; Accx_l = 0;
            Posx_r = Strx * (kx + 1); Velx_r = 0; Accx_r = 0;
            Posy_l = Stry * (kx + 1); Vely_l = 0; Accy_l = 0;
            Posy_r = Stry * (kx + 1); Vely_r = 0; Accy_r = 0;
        }
    }
    /* Feet Trajectory End */

    
    returnVals << Cx, dCx, ddCx, Cy, dCy, ddCy, Posx_r, Posy_r, Posz_r, Posx_l, Posy_l, Posz_l, Strx, Strx/2, Stry, Stry/2;
    return returnVals;
}

Eigen::VectorXd trajGeneration(double RealTime, bool walkEnable, double command_Vx, double command_Vy, double height, double dt)
{
    Eigen::VectorXd outVals(13);
    double footPx_R = 0.0, footPy_R = 0, footPz_R = 0.0;
    double footPx_L = 0.0, footPy_L = 0, footPz_L = 0.0;

    if(!walk_enabled && walkEnable) w_start_time = RealTime;
    walk_enabled = walkEnable;

    if(walk_enabled){
        can_switch = (AreDoubleSame(walkTraj(8),Fc)) || (AreDoubleSame(walkTraj(11),Fc));
    } else {
        can_switch = false;
    }

    if(abs(Vx_mean) > 0 || abs(Vy_mean) > 0){
        can_stop = false;
    } else {
        can_stop = (AreDoubleSame(walkTraj(8),0)) && (AreDoubleSame(walkTraj(11),0));
    }

    if(can_switch && walk_enabled){
        Vx_mean = command_Vx; 
        Vy_mean = command_Vy; 
    }

    if(prev_vx_mean != Vx_mean) Cx += walkTraj(0);
    if(prev_vy_mean != Vy_mean) Cy += walkTraj(3);

    if((prev_vx_mean != Vx_mean) || (prev_vy_mean != Vy_mean)){
        if(AreDoubleSame(walkTraj(8),Fc)){
            w_start_time = RealTime - Ts - Td - Ts/2.0;
            walkTraj = comTrajectory(RealTime-w_start_time, Ts, Td, Nphase, px, py, Vx_mean, Vy_mean, height, Fc, dt);
            if(prev_vx_mean != Vx_mean) Cx -= walkTraj(13);
            if(prev_vy_mean != Vy_mean) Cy -= walkTraj(15);
        }else if(AreDoubleSame(walkTraj(11),Fc)){
            w_start_time = RealTime - Ts - Td - Ts - Td - Ts/2.0 ;
            walkTraj = comTrajectory(RealTime-w_start_time, Ts, Td, Nphase, px, py, Vx_mean, Vy_mean, height, Fc, dt);
            if(prev_vx_mean != Vx_mean) Cx -= walkTraj(12);
            if(prev_vy_mean != Vy_mean) Cy -= walkTraj(14);
        }

        if(prev_vx_mean != Vx_mean) prev_vx_mean = Vx_mean;
        if(prev_vy_mean != Vy_mean) prev_vy_mean = Vy_mean;
    }
    
    if(walk_enabled){
        Nphase += 1;
    }
    else if(!walk_enabled && can_stop){
        Nphase = 0;
    }
    
    walkTraj = comTrajectory(RealTime-w_start_time, Ts, Td, Nphase, px, py, Vx_mean, Vy_mean, height, Fc, dt);
    Xc = walkTraj(0)+Cx; Yc = walkTraj(3)+Cy;
    dXc = walkTraj(1); dYc = walkTraj(4); 
    ddXc = walkTraj(2); ddYc = walkTraj(5);
    footPx_R = walkTraj(6)+Cx; footPy_R = walkTraj(7)+Cy; footPz_R = walkTraj(8);
    footPx_L = walkTraj(9)+Cx; footPy_L = walkTraj(10)+Cy; footPz_L = walkTraj(11);

    outVals << Xc, dXc, ddXc, Yc, dYc, ddYc, height, footPx_R, footPx_L, footPy_R, footPy_L, footPz_R, footPz_L;
    return outVals;
}
