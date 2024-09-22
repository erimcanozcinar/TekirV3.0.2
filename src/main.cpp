#include "main.hpp"

int main(int argc, char** argv) {   
    /* #region: Raisim */
    auto binaryPath = raisim::Path::setFromArgv(argv[0]);
    raisim::World::setActivationKey(binaryPath.getDirectory() + "\\activation.raisim");
    raisim::World world;

    world.setTimeStep(0.001);
    world.setMaterialPairProp("steel", "steel", 0.95, 0.95, 0.001, 0.95, 0.001);
    world.setMaterialPairProp("steel", "rubber", 2, 0.15, 0.001, 2, 0.001);
    auto ground = world.addGround(0, "steel");
    // ground->setAppearance("hidden");

    auto quadruped = world.addArticulatedSystem("/home/erim/RaiSim_Simulations/TekirV3.0.2/rsc/urdf/tekir3mesh_new_V2.urdf");
    quadruped->getCollisionBody("Foot_lf/0").setMaterial("rubber");
    quadruped->getCollisionBody("Foot_rf/0").setMaterial("rubber");
    quadruped->getCollisionBody("Foot_lb/0").setMaterial("rubber");
    quadruped->getCollisionBody("Foot_rb/0").setMaterial("rubber");
    /* #endregion */
    
    /* #region: Create Log file */
    FILE* fp0;
    FILE* fp1;
    fp0 = fopen("/home/erim/RaiSim_Simulations/TekirV3.0.2/Log/dataLog.txt", "w");
    /* #endregion */

    /* #region: Allegro */
    al_init();
    al_install_joystick();
    ALLEGRO_EVENT_QUEUE *event_queue = al_create_event_queue();
    al_register_event_source(event_queue, al_get_joystick_event_source());
    std::cout << "Gamepad routines enabled." << std::endl;
    ALLEGRO_EVENT event;
    /* #endregion */

    /* #region: Init Eigen variables */
    Itorso << 0.410, 0.003, -0.012,
              0.003, 0.908,  0.007,
              -0.012, 0.007, 1.192;

    prevgenVelocity.setZero();
    dQ_LF.setZero(); dQ_RF.setZero(); dQ_LB.setZero(); dQ_RB.setZero();
    Fcon_LF.setZero(); Fcon_RF.setZero(); Fcon_LB.setZero(); Fcon_RB.setZero();

    prev_Pbase.setZero();

    accBody.setZero(); velBody.setZero();
    jPositions.setZero(); jVelocities.setZero(); jAccelerations.setZero(); jffTorques.setZero();

    Kp << 120, 70, 70;
    Kd << 12, 7, 7;

    Tau_LF.setZero(); Tau_RF.setZero(); Tau_LB.setZero(); Tau_RB.setZero();
    
    prevdQcm.setZero(); prevQcm.setZero(); prevQcm_filtered.setZero();

    Fvmc << 0, 0, MASS*GRAVITY, 0, 0, 0;
    /* #endregion */

    /* #region: Init classes*/
    controller jStick;
    trajectory traj;
    /* #endregion */

    /* #region: Initialize Robot */
    traj.trajGeneration(0.0, false, 0.0, 0.0, 0, initComZ, 0.001);
    Pcom << initComX, initComY, initComZ;
    torsoRot << initComRoll, initComPitch, initComYaw;
    Q_LF = fullBodyIKan(traj.Pfoot_LF, Pcom, torsoRot, 1);
    Q_RF = fullBodyIKan(traj.Pfoot_RF, Pcom, torsoRot, 2);
    Q_LB = fullBodyIKan(traj.Pfoot_LB, Pcom, torsoRot, 3);
    Q_RB = fullBodyIKan(traj.Pfoot_RB, Pcom, torsoRot, 4);
    initialConditions << initComX, initComY, initComZ, initQuat_w, initQuat_x, initQuat_y, initQuat_z, Q_LF(0), Q_LF(1), Q_LF(2), Q_RF(0), Q_RF(1), Q_RF(2), Q_LB(0), Q_LB(1), Q_LB(2), Q_RB(0), Q_RB(1), Q_RB(2);
    quadruped->setGeneralizedCoordinate(initialConditions);
    /* #endregion */

    quadruped->setComputeInverseDynamics(true);

    /* #region: Launch raisim server for visualization.Can be visualized on raisimUnity */
    raisim::RaisimServer server(&world);
    server.setMap("default");
    server.focusOn(quadruped);
    server.launchServer();
    raisim::MSLEEP(2000);
    /* #endregion */

    Eigen::VectorXd torqueFromInverseDynamics(quadruped->getDOF());
    torqueFromInverseDynamics.setZero();
    jointTorques.setZero();

    while (!jStick.close) {
        RS_TIMED_LOOP(int(world.getTimeStep()*1e6));
        t = world.getWorldTime();
        dt = world.getTimeStep();        

        /* #region: CONTACT DEFINITION */
        for (auto& contact : quadruped->getContacts()) // LF:3, RF:2, LB:1, RB:0
        {
            if (contact.skip()) continue;
            if (quadruped->getBodyIdx("shank_lf") == contact.getlocalBodyIndex())
            {
                Fcon_LF = -contact.getContactFrame().e().transpose() * contact.getImpulse().e() / dt;
                Pcon_LF = contact.getPosition().e().transpose();
            }
            else if (quadruped->getBodyIdx("shank_rf") == contact.getlocalBodyIndex())
            {
                Fcon_RF = -contact.getContactFrame().e().transpose() * contact.getImpulse().e() / dt;
                Pcon_RF = contact.getPosition().e().transpose();
            }
            else if (quadruped->getBodyIdx("shank_lb") == contact.getlocalBodyIndex())
            {
                Fcon_LB = -contact.getContactFrame().e().transpose() * contact.getImpulse().e() / dt;
                Pcon_LB = contact.getPosition().e().transpose();
            }
            else if (quadruped->getBodyIdx("shank_rb") == contact.getlocalBodyIndex())
            {
                Fcon_RB = -contact.getContactFrame().e().transpose() * contact.getImpulse().e() / dt;
                Pcon_RB = contact.getPosition().e().transpose();
            }
        }
        /* #endregion */

        /* #region: READ ACTUAL DATA */
        genCoordinates = quadruped->getGeneralizedCoordinate().e();
        genVelocity = quadruped->getGeneralizedVelocity().e();
        for (int i = 0; i < 18; i++)
        {
            genAcceleration(i) = Numdiff(genVelocity(i), prevgenVelocity(i), dt);
            prevgenVelocity(i) = genVelocity(i);
        }

        imuRot = quat2euler(genCoordinates(3), genCoordinates(4), genCoordinates(5), genCoordinates(6));
        dimuRot << Numdiff(imuRot(0), prev_imuRot(0), dt), Numdiff(imuRot(1), prev_imuRot(1), dt), Numdiff(imuRot(2), prev_imuRot(2), dt);
        prev_imuRot = imuRot;

        q_LF << genCoordinates(7), genCoordinates(8), genCoordinates(9);
        q_RF << genCoordinates(10), genCoordinates(11), genCoordinates(12);
        q_LB << genCoordinates(13), genCoordinates(14), genCoordinates(15);
        q_RB << genCoordinates(16), genCoordinates(17), genCoordinates(18);
        dq_LF << genVelocity(6), genVelocity(7), genVelocity(8);
        dq_RF << genVelocity(9), genVelocity(10), genVelocity(11);
        dq_LB << genVelocity(12), genVelocity(13), genVelocity(14);
        dq_RB << genVelocity(15), genVelocity(16), genVelocity(17);

        rootOrientation = quat2Rotmat(genCoordinates(3), genCoordinates(4), genCoordinates(5), genCoordinates(6));
        rootAngvelocity << genVelocity(3), genVelocity(4), genVelocity(5);
        rootAngacceleration << genAcceleration(3), genAcceleration(4), genAcceleration(5);

        rootAbsposition << genCoordinates(0), genCoordinates(1), genCoordinates(2);
        rootAbsvelocity << genVelocity(0), genVelocity(1), genVelocity(2);
        rootAbsacceleration << genAcceleration(0), genAcceleration(1), genAcceleration(2);

        jPositions << q_LF(0), q_LF(1), q_LF(2), q_RF(0), q_RF(1), q_RF(2), q_LB(0), q_LB(1), q_LB(2), q_RB(0), q_RB(1), q_RB(2);
        jVelocities << dq_LF(0), dq_LF(1), dq_LF(2), dq_RF(0), dq_RF(1), dq_RF(2), dq_LB(0), dq_LB(1), dq_LB(2), dq_RB(0), dq_RB(1), dq_RB(2);
        jAccelerations << ddQ_LF(0), ddQ_LF(1), ddQ_LF(2), ddQ_RF(0), ddQ_RF(1), ddQ_RF(2), ddQ_LB(0), ddQ_LB(1), ddQ_LB(2), ddQ_RB(0), ddQ_RB(1), ddQ_RB(2);    
        /* #endregion */
        
        /* #region: TRAJECTORY GENERATION WITH JOYSTICK */
        jStick.dualShockController(event_queue, event);
        for(int i=0; i<22; i++)
        {
            cmdJoyF[i] = LPF(jStick.joyCmd[i], pre_cmdJoyF[i], 2*PI*0.2, dt);
            pre_cmdJoyF[i] = cmdJoyF[i];
        }
        cmd_Vx = jStick.joyCmd[0]; cmd_Vy = jStick.joyCmd[1];
        if(jStick.walkEnable) { cmd_yaw = jStick.joyCmd[2]; }
        else { cmd_yaw = cmdJoyF[2]; }
        cmd_pitch = cmdJoyF[3]; cmd_roll = cmdJoyF[4];
        if(t>1){
            traj.trajGeneration(t, jStick.walkEnable, cmd_Vx, cmd_Vy, cmd_yaw, cmdJoyF[5], dt);
        }
        /* #endregion */        
         
        /* #region: DESIRED COM POSITION & ORIENTATION */
        Eigen::Vector3d comcuk = quadruped->getCOM().e() - rootAbsposition;
        Pcom << traj.Xc, traj.Yc, traj.Zc;
        Rcom << 0.00145, 0.003271, 0.01792;
        // Rcom << comcuk(0), comcuk(1), comcuk(2)+traj.Zc;
        torsoRot << cmd_roll, cmd_pitch, traj.Yawc; // Torso Orientation
        /* #endregion */

        /* #region: ANALYTIC INVERSE KINEMATIC */
        pre_dQ_LF = dQ_LF;
        pre_dQ_RF = dQ_RF;
        pre_dQ_LB = dQ_LB;
        pre_dQ_RB = dQ_RB;
        prevQ_LF = Q_LF;
        prevQ_RF = Q_RF;
        prevQ_LB = Q_LB;
        prevQ_RB = Q_RB;
        Q_LF = fullBodyIKan(traj.Pfoot_LF, Pcom, torsoRot, 1);
        Q_RF = fullBodyIKan(traj.Pfoot_RF, Pcom, torsoRot, 2);
        Q_LB = fullBodyIKan(traj.Pfoot_LB, Pcom, torsoRot, 3);
        Q_RB = fullBodyIKan(traj.Pfoot_RB, Pcom, torsoRot, 4);

        dQ_LF << Numdiff(Q_LF(0), prevQ_LF(0), dt), Numdiff(Q_LF(1), prevQ_LF(1), dt), Numdiff(Q_LF(2), prevQ_LF(2), dt);
        dQ_RF << Numdiff(Q_RF(0), prevQ_RF(0), dt), Numdiff(Q_RF(1), prevQ_RF(1), dt), Numdiff(Q_RF(2), prevQ_RF(2), dt);
        dQ_LB << Numdiff(Q_LB(0), prevQ_LB(0), dt), Numdiff(Q_LB(1), prevQ_LB(1), dt), Numdiff(Q_LB(2), prevQ_LB(2), dt);
        dQ_RB << Numdiff(Q_RB(0), prevQ_RB(0), dt), Numdiff(Q_RB(1), prevQ_RB(1), dt), Numdiff(Q_RB(2), prevQ_RB(2), dt);

        ddQ_LF << Numdiff(dQ_LF(0), pre_dQ_LF(0), dt), Numdiff(dQ_LF(1), pre_dQ_LF(1), dt), Numdiff(dQ_LF(2), pre_dQ_LF(2), dt);
        ddQ_RF << Numdiff(dQ_RF(0), pre_dQ_RF(0), dt), Numdiff(dQ_RF(1), pre_dQ_RF(1), dt), Numdiff(dQ_RF(2), pre_dQ_RF(2), dt);
        ddQ_LB << Numdiff(dQ_LB(0), pre_dQ_LB(0), dt), Numdiff(dQ_LB(1), pre_dQ_LB(1), dt), Numdiff(dQ_LB(2), pre_dQ_LB(2), dt);
        ddQ_RB << Numdiff(dQ_RB(0), pre_dQ_RB(0), dt), Numdiff(dQ_RB(1), pre_dQ_RB(1), dt), Numdiff(dQ_RB(2), pre_dQ_RB(2), dt);
        /* #endregion */

        /* #region: VMC CONTROLLER FOR TORSO */
        Rf_LF = fullBodyFK(rootOrientation, 0*Pcom, q_LF, 1);
        Rf_RF = fullBodyFK(rootOrientation, 0*Pcom, q_RF, 2);
        Rf_LB = fullBodyFK(rootOrientation, 0*Pcom, q_LB, 3);
        Rf_RB = fullBodyFK(rootOrientation, 0*Pcom, q_RB, 4);

        dtorsoRot << Numdiff(torsoRot(0), prev_torsoRot(0), dt), Numdiff(torsoRot(1), prev_torsoRot(1), dt), Numdiff(torsoRot(2), prev_torsoRot(2), dt);
        ddtorsoRot << Numdiff(dtorsoRot(0), prev_dtorsoRot(0), dt), Numdiff(dtorsoRot(1), prev_dtorsoRot(1), dt), Numdiff(dtorsoRot(2), prev_dtorsoRot(2), dt);
        prev_torsoRot = torsoRot;
        prev_dtorsoRot = dtorsoRot;

        Wbd << 0, 0, 0;
        angleErr = balanceControl(quat2Rotmat(1,0,0,0), rootOrientation);
        Kpw << 500, 0, 0,
               0, 500, 0,
               0, 0, 500;
        Kdw << 50, 0, 0,
               0, 50, 0,
               0, 0, 50;
        // Kdw = Kpw.sqrt();
        Mvmc =  Itorso*(Kpw*angleErr + Kdw*(Wbd - rootAngvelocity));

        Fvmc(0) = (MASS*traj.ddXc + 50*(traj.Xc-genCoordinates(0)) + sqrt(50)*(traj.dXc - genVelocity(0)));
        Fvmc(1) = (MASS*traj.ddYc + 50*(traj.Yc-genCoordinates(1)) + sqrt(50)*(traj.dYc - genVelocity(1)));
        Fvmc(2) = (50*(traj.Zc-genCoordinates(2)) + sqrt(50)*(0 - genVelocity(2)) + MASS*GRAVITY);
        Fvmc(3) = Mvmc(0);
        Fvmc(4) = Mvmc(1);
        Fvmc(5) = Mvmc(2);
        // RSINFO(comcuk);
        
        Fmatrix = VMC(Rf_LF-Rcom, Rf_RF-Rcom, Rf_LB-Rcom, Rf_RB-Rcom, Fcon_LF, Fcon_RF, Fcon_LB, Fcon_RB, Fvmc, dt);
        // Fmatrix = VMC(traj.Pfoot_LF-Rcom, traj.Pfoot_RF-Rcom, traj.Pfoot_LB-Rcom, traj.Pfoot_RB-Rcom, Fcon_LF, Fcon_RF, Fcon_LB, Fcon_RB, Fvmc, dt);
        // Fmatrix = refForceCalc4(Rf_LF, Rf_RF, Rf_LB, Rf_RB, Q_LF, Q_RF, Q_LB, Q_RB, dt);
        F1cont << Fmatrix(0, 0), Fmatrix(0, 1), Fmatrix(0, 2);
        F2cont << Fmatrix(1, 0), Fmatrix(1, 1), Fmatrix(1, 2);
        F3cont << Fmatrix(2, 0), Fmatrix(2, 1), Fmatrix(2, 2);
        F4cont << Fmatrix(3, 0), Fmatrix(3, 1), Fmatrix(3, 2);

        /* #endregion */
        
        /* #region: INVERSE DYNAMICS */
        jffTorques = funNewtonEuler4Leg(rootAbsacceleration, rootOrientation, rootAngvelocity, rootAngacceleration, jPositions, jVelocities, jAccelerations, F1cont, F2cont, F3cont, F4cont);
        jffTorques2 = funNewtonEuler4Leg(rootAbsacceleration, rootOrientation, rootAngvelocity, rootAngacceleration, genCoordinates.tail(12), genVelocity.tail(12), genAcceleration.tail(12), Fcon_LF, Fcon_RF, Fcon_LB, Fcon_RB);
        /* #endregion */

        /* #region: INVERSE DYNAMICS WITH RAISIM*/
        std::vector<Eigen::Vector3d> axes(quadruped->getDOF()-6);
        for (int j=0; j<quadruped->getDOF()-6; j++)
            axes[j] = quadruped->getJointAxis(j+1).e();

        for (size_t j=1; j<quadruped->getDOF()-5; j++)
            torqueFromInverseDynamics(j+5) = quadruped->getTorqueAtJointInWorldFrame(j).dot(axes[j-1]);
        
        Tau1_JF = JacTranspose_LF(q_LF(0), q_LF(1), q_LF(2), 30*PI/180) * (rootOrientation.transpose() * F1cont);
        Tau2_JF = JacTranspose_RF(q_RF(0), q_RF(1), q_RF(2), -30*PI/180) * (rootOrientation.transpose() * F2cont);
        Tau3_JF = JacTranspose_LB(q_LB(0), q_LB(1), q_LB(2), 30*PI/180) * (rootOrientation.transpose() * F3cont);
        Tau4_JF = JacTranspose_RB(q_RB(0), q_RB(1), q_RB(2), -30*PI/180) * (rootOrientation.transpose() * F4cont);

        JF << 0, 0, 0, 0, 0, 0, Tau1_JF(0), Tau1_JF(1), Tau1_JF(2), Tau2_JF(0), Tau2_JF(1), Tau2_JF(2), Tau3_JF(0), Tau3_JF(1), Tau3_JF(2), Tau4_JF(0), Tau4_JF(1), Tau4_JF(2);
        genAccelerationVec << genAcceleration(0), genAcceleration(1), genAcceleration(2), rootAngacceleration(0), rootAngacceleration(1), rootAngacceleration(2), ddQ_LF(0), ddQ_LF(1), ddQ_LF(2), ddQ_RF(0), ddQ_RF(1), ddQ_RF(2), ddQ_LB(0), ddQ_LB(1), ddQ_LB(2), ddQ_RB(0), ddQ_RB(1), ddQ_RB(2);
        jointTorques = quadruped->getMassMatrix().e() * genAccelerationVec + quadruped->getNonlinearities(world.getGravity()).e() - JF;
        /* #endregion */

        /* #region: PD + ID CONTROLLER */
        for (int i = 0; i < 3; i++)
        {
            // i = 0: Hip AA, i = 1: Hip FE, i = 2: Knee FE
            Tau_LF(i) = Kp(i)*(Q_LF(i) - q_LF(i)) + Kd(i)*(dQ_LF(i) - dq_LF(i)) + jffTorques(i);
            Tau_RF(i) = Kp(i)*(Q_RF(i) - q_RF(i)) + Kd(i)*(dQ_RF(i) - dq_RF(i)) + jffTorques(i+3);
            Tau_LB(i) = Kp(i)*(Q_LB(i) - q_LB(i)) + Kd(i)*(dQ_LB(i) - dq_LB(i)) + jffTorques(i+6);
            Tau_RB(i) = Kp(i)*(Q_RB(i) - q_RB(i)) + Kd(i)*(dQ_RB(i) - dq_RB(i)) + jffTorques(i+9);
            // Tau_LF(i) = Kp(i)*(Q_LF(i) - q_LF(i)) + Kd(i)*(dQ_LF(i) - dq_LF(i)) + torqueFromInverseDynamics(i+6);
            // Tau_RF(i) = Kp(i)*(Q_RF(i) - q_RF(i)) + Kd(i)*(dQ_RF(i) - dq_RF(i)) + torqueFromInverseDynamics(i+9);
            // Tau_LB(i) = Kp(i)*(Q_LB(i) - q_LB(i)) + Kd(i)*(dQ_LB(i) - dq_LB(i)) + torqueFromInverseDynamics(i+12);
            // Tau_RB(i) = Kp(i)*(Q_RB(i) - q_RB(i)) + Kd(i)*(dQ_RB(i) - dq_RB(i)) + torqueFromInverseDynamics(i+15);
        }
        /* #endregion */

        /* #region: SEND COMMEND TO THE ROBOT */
        F << 0, 0, 0, 0, 0, 0, Tau_LF(0), Tau_LF(1), Tau_LF(2), Tau_RF(0), Tau_RF(1), Tau_RF(2), Tau_LB(0), Tau_LB(1), Tau_LB(2), Tau_RB(0), Tau_RB(1), Tau_RB(2);
        quadruped->setGeneralizedForce(F);
        JF << 0, 0, 0, 0, 0, 0, jffTorques2(0), jffTorques2(1), jffTorques2(2), jffTorques2(3), jffTorques2(4), jffTorques2(5),jffTorques2(6), jffTorques2(7), jffTorques2(8), jffTorques2(9), jffTorques2(10), jffTorques2(11);
        // RSWARN(torqueFromInverseDynamics-JF)
        // RSINFO(JF)
        /* #endregion */

        /* #region: PUSH ROBOT */
        Eigen::Vector3d F_ex;
        if (t > 6.13 && t < 6.23)
        {
            F_ex << 0, 100, 0;
        }
        else 
        {
            F_ex << 0, 0, 0;
        }
        //quadruped->setExternalForce(quadruped->getBodyIdx("torso"), F_ex);
        /* #endregion */
                
        // server.setCameraPositionAndLookAt({genCoordinates(0)-1.3,genCoordinates(1),genCoordinates(2)+0.6}, {genCoordinates(0),genCoordinates(1),genCoordinates(2)});
                 
        server.integrateWorldThreadSafe(); 

        /* Log data (fp0:NewtonEulerTorques, fp1:PD_torques, fp2: InvDynTorques ) */
        // fprintf(fp0, "%f %f %f %f %f %f %f %f %f\n", t, traj.Xc, traj.Pfoot_RF(0), traj.Pfoot_RF(2), traj.Pfoot_LF(0), traj.Pfoot_LF(2), traj.Yawc/2, traj.ComYaw/2, traj.aa_LF(0));
        // fprintf(fp0, "%f %f %f %f %f %f %f %f %f %f\n", t, traj.Xc, traj.Footx_LF(0)-traj.Xc, traj.Footy_LF(0), traj.Footz_LF(0), Q_LB(2), dQ_RF(1), dQ_RF(2), dQ_LB(1), dQ_LB(2));
        fprintf(fp0, "%f %f %f %f %f %f %f %f %f %f %f\n", t, imuRot(0), imuRot(1), imuRot(2), rootAngvelocity(0), rootAngvelocity(1), rootAngvelocity(2), Mvmc(0), Mvmc(1), Mvmc(2), F4cont(2));
        // fprintf(fp0, "%f %f %f %f %f %f %f %f %f\n", t, traj.Xc, traj.localStr_LF(0), traj.localStr_LF(1), traj.Footz_R(0), traj.Footx_L(0), traj.Footy_L(0), traj.Footz_L(0), traj.Yaw);
        
    }
    server.killServer();
    fclose(fp0);
    al_destroy_event_queue(event_queue);
    std::cout << "end of simulation" << std::endl;
}