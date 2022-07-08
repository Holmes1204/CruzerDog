#include <FSM/State_Worker/Locomotion_Worker.h>
#include <eigen3/Eigen/Dense>
Locomotion::Locomotion(FSM_data &data_, FSM_topic_control &tpcl) : data_(data_),
																  tpcl_(tpcl)
{

	horizonLength = 10;
	dt = 0.002;
	dtMPC = dt*iterationsBetweenMPC;
	trotting = OffsetDurationGait(horizonLength, Eigen::Vector4<int>(0, 5, 5, 0), Eigen::Vector4<int>(5, 5, 5, 5), "Trotting");
	std::cout << "locomotion initial" << std::endl;
	/*
  if(_controlFSMData->_quadruped->_robotType == RobotType::MINI_CHEETAH){
	cMPCOld = new ConvexMPCLocomotion(_controlFSMData->controlParameters->controller_dt,
		//30 / (1000. * _controlFSMData->controlParameters->controller_dt),
		//22 / (1000. * _controlFSMData->controlParameters->controller_dt),
		27 / (1000. * _controlFSMData->controlParameters->controller_dt),
		_controlFSMData->userParameters);

  }else if(_controlFSMData->_quadruped->_robotType == RobotType::CHEETAH_3){
	cMPCOld = new ConvexMPCLocomotion(_controlFSMData->controlParameters->controller_dt,
		33 / (1000. * _controlFSMData->controlParameters->controller_dt),
		_controlFSMData->userParameters);

  }else{
	assert(false);
  }
	*/

	// Initialize GRF and footstep locations to 0s
	// this->footFeedForwardForces = Mat34<T>::Zero();
	// this->footstepLocations = Mat34<T>::Zero();
	//_wbc_ctrl = new LocomotionCtrl<T>(_controlFSMData->_quadruped->buildModel());
	//_wbc_data = new LocomotionCtrlData<T>();
}

/**
 * Calls the functions to be executed on each control loop iteration.
 */

void Locomotion::run()
{

	//从状态估计器中获得 rpy，xyz world frame
	//  auto& seResult = data_._stateEstimator->getResult();
	//  Eigen::Vector3<double> v_des_robot(_x_vel_des, _y_vel_des, 0);
	// Eigen::Vector3<double> v_des_world =
	// omniMode ? v_des_robot : seResult.rBody.transpose() * v_des_robot;
	// Eigen::Vector3<double> v_robot = seResult.vWorld;
	//运行mpc和wbic
	std::cout << "running locomotion" << std::endl;
	b_run();
	// Call the locomotion control logic for this iteration
	// StateEstimate<T> stateEstimate = this->_data->_stateEstimator->getResult();

	// Contact state logic
	// estimateContact();
	/*
	cMPCOld->run<T>(*this->_data);
	Eigen::Vector3<T> pDes_backup[4];
	Eigen::Vector3<T> vDes_backup[4];
	Eigen::Matrix3<T> Kp_backup[4];
	Eigen::Matrix3<T> Kd_backup[4];

	for(int leg(0); leg<4; ++leg){
	  pDes_backup[leg] = this->_data->_legController->command[leg].pDes;
	  vDes_backup[leg] = this->_data->_legController->command[leg].vDes;
	  Kp_backup[leg] = this->_data->_legController->command[leg].kpCartesian;
	  Kd_backup[leg] = this->_data->_legController->command[leg].kdCartesian;
	}

	if(this->_data->userParameters->use_wbc > 0.9){
	  _wbc_data->pBody_des = cMPCOld->pBody_des;
	  _wbc_data->vBody_des = cMPCOld->vBody_des;
	  _wbc_data->aBody_des = cMPCOld->aBody_des;

	  _wbc_data->pBody_RPY_des = cMPCOld->pBody_RPY_des;
	  _wbc_data->vBody_Ori_des = cMPCOld->vBody_Ori_des;

	  for(size_t i(0); i<4; ++i){
		_wbc_data->pFoot_des[i] = cMPCOld->pFoot_des[i];
		_wbc_data->vFoot_des[i] = cMPCOld->vFoot_des[i];
		_wbc_data->aFoot_des[i] = cMPCOld->aFoot_des[i];
		_wbc_data->Fr_des[i] = cMPCOld->Fr_des[i];
	  }
	  _wbc_data->contact_state = cMPCOld->contact_state;
	  _wbc_ctrl->run(_wbc_data, *this->_data);
	}
	for(int leg(0); leg<4; ++leg){
	  //this->_data->_legController->command[leg].pDes = pDes_backup[leg];
	  this->_data->_legController->command[leg].vDes = vDes_backup[leg];
	  //this->_data->_legController->command[leg].kpCartesian = Kp_backup[leg];
	  this->_data->_legController->command[leg].kdCartesian = Kd_backup[leg];
	}
	*/
}
bool Locomotion::is_finished()
{

	return false;
}
void Locomotion::send()
{

	return;
}
void Locomotion::b_run()
{
	// some first time initialization
	auto &seResult = data_.model_StateEstimate->getResult();

	Eigen::Vector3<double> v_des_robot(_x_vel_des, _y_vel_des, 0);
	Eigen::Vector3<double> v_des_world = seResult.rBody.transpose() * v_des_robot;
	Eigen::Vector3<double> v_robot = seResult.vWorld;

	if (firstRun)
	{
		world_position_desired[0] = seResult.position[0];
		world_position_desired[1] = seResult.position[1];
		world_position_desired[2] = seResult.rpy[2];

		for (int i = 0; i < 4; i++)
		{

			footSwingTrajectories[i].setHeight(0.05);
			footSwingTrajectories[i].setInitialPosition(pFoot[i]);
			footSwingTrajectories[i].setFinalPosition(pFoot[i]);
		}
		firstRun = false;
	}
	// Command Setup
	//_SetupCommand(data_);

	// pick gait
	Gait *gait = &trotting;
	if (_body_height < 0.02)
	{
		_body_height = 0.29;
	}

	// integrate position setpoint

	// pretty_print(v_des_world, std::cout, "v des world");

	// Integral-esque pitche and roll compensation
	if (fabs(v_robot[0]) > .2) // avoid dividing by zero
	{
		rpy_int[1] += dt * (_pitch_des - seResult.rpy[1]) / v_robot[0];
	}
	if (fabs(v_robot[1]) > 0.1)
	{
		rpy_int[0] += dt * (_roll_des - seResult.rpy[0]) / v_robot[1];
	}
	//积分限,对rpy的积分补偿
	rpy_int[0] = fminf(fmaxf(rpy_int[0], -.25), .25);
	rpy_int[1] = fminf(fmaxf(rpy_int[1], -.25), .25);
	rpy_comp[1] = v_robot[0] * rpy_int[1];
	rpy_comp[0] = v_robot[1] * rpy_int[0]; // turn off for pronking
	// world frame 下足端轨迹
	for (int i = 0; i < 4; i++)
	{
		pFoot[i] = seResult.position +
				   seResult.rBody.transpose() * (data_._quadruped->getHipLocation(i) +
												 data_._legController->data[i].p);
	}

	world_position_desired += dt * Eigen::Vector3<double>(v_des_world[0], v_des_world[1], 0);

	// foot placement
	for (int l = 0; l < 4; l++)
		swingTimes[l] = gait->getCurrentSwingTime(dtMPC, l);
	//这几个不知道是什么东西
	double side_sign[4] = {-1, 1, -1, 1};
	double interleave_y[4] = {-0.08, 0.08, 0.02, -0.02};
	// double interleave_gain = -0.13;
	double interleave_gain = -0.2;
	// double v_abs = std::fabs(seResult.vBody[0]);

	double v_abs = std::fabs(v_des_robot[0]);

	for (int i = 0; i < 4; i++)
	{

		if (firstSwing[i])
		{
			swingTimeRemaining[i] = swingTimes[i];
		}
		else
		{
			swingTimeRemaining[i] -= dt;
		}
		// if(firstSwing[i]) {
		// footSwingTrajectories[i].setHeight(.05);
		footSwingTrajectories[i].setHeight(.06);
		Eigen::Vector3<double> offset(0, side_sign[i] * .065, 0);
		Eigen::Vector3<double> pRobotFrame = (data_._quadruped->getHipLocation(i) + offset);
		pRobotFrame[1] += interleave_y[i] * v_abs * interleave_gain;

		double stance_time = gait->getCurrentStanceTime(dtMPC, i);

		Eigen::Vector3<double> des_vel;
		des_vel[0] = _x_vel_des;
		des_vel[1] = _y_vel_des;
		des_vel[2] = 0.0;
		// P_foot findal,足端轨迹的最后值，在world frame下
		Eigen::Vector3<double> Pf = seResult.position + seResult.rBody.transpose() * (des_vel * swingTimeRemaining[i]);

		//这个大概是30cm
		double p_rel_max = 0.3f;

		// Using the estimated velocity is correct
		// Eigen::Vector3<double> des_vel_world = seResult.rBody.transpose() * des_vel;
		double pfx_rel = seResult.vWorld[0] * (.5) * stance_time +
						 .03f * (seResult.vWorld[0] - v_des_world[0]);

		double pfy_rel = seResult.vWorld[1] * .5 * stance_time * dtMPC +
						 .03f * (seResult.vWorld[1] - v_des_world[1]);

		pfx_rel = fminf(fmaxf(pfx_rel, -p_rel_max), p_rel_max);
		pfy_rel = fminf(fmaxf(pfy_rel, -p_rel_max), p_rel_max);
		Pf[0] += pfx_rel;
		Pf[1] += pfy_rel;
		Pf[2] = -0.003;
		// Pf[2] = 0.0;
		footSwingTrajectories[i].setFinalPosition(Pf);
	}

	// calc gait
	iterationCounter++;

	// load LCM leg swing gains
	//此时stance的kp为零，不会因为位置的关系影响脚的回收工作
	Kp << 700, 0, 0,
		0, 700, 0,
		0, 0, 150;
	Kp_stance = 0 * Kp;

	Kd << 7, 0, 0,
		0, 7, 0,
		0, 0, 7;
	Kd_stance = Kd;
	// gait
	Eigen::Vector4<double> contactStates = gait->getContactState();
	Eigen::Vector4<double> swingStates = gait->getSwingState();

	// int *mpcTable = gait->getMpcTable();
	// updateMPCIfNeeded(mpcTable, data_, omniMode);

	//  StateEstimator* se = hw_i->state_estimator;
	Eigen::Vector4<double> se_contactState(0, 0, 0, 0);

	for (int foot = 0; foot < 4; foot++)
	{
		double contactState = contactStates[foot];
		double swingState = swingStates[foot];
		if (swingState > 0) // foot is in swing
		{
			if (firstSwing[foot])
			{
				firstSwing[foot] = false;
				footSwingTrajectories[foot].setInitialPosition(pFoot[foot]);
			}
			footSwingTrajectories[foot].computeSwingTrajectoryBezier(swingState, swingTimes[foot]);

			//      footSwingTrajectories[foot]->updateFF(hw_i->leg_controller->leg_datas[foot].q,
			//                                          hw_i->leg_controller->leg_datas[foot].qd, 0); // velocity dependent friction compensation todo removed
			// hw_i->leg_controller->leg_datas[foot].qd, fsm->main_control_settings.variable[2]);

			Eigen::Vector3<double> pDesFootWorld = footSwingTrajectories[foot].getPosition();
			Eigen::Vector3<double> vDesFootWorld = footSwingTrajectories[foot].getVelocity();
			Eigen::Vector3<double> pDesLeg = seResult.rBody * (pDesFootWorld - seResult.position) - data_._quadruped->getHipLocation(foot);
			Eigen::Vector3<double> vDesLeg = seResult.rBody * (vDesFootWorld - seResult.vWorld);

			// Update for WBC
			pFoot_des[foot] = pDesFootWorld;
			vFoot_des[foot] = vDesFootWorld;
			aFoot_des[foot] = footSwingTrajectories[foot].getAcceleration();

			// Update leg control command regardless of the usage of WBIC
			data_._legController->command[foot].pDes = pDesLeg;
			data_._legController->command[foot].vDes = vDesLeg;
			data_._legController->command[foot].kpCartesian = Kp;
			data_._legController->command[foot].kdCartesian = Kd;
		}
		else // foot is in stance
		{
			firstSwing[foot] = true;

			Eigen::Vector3<double> pDesFootWorld = footSwingTrajectories[foot].getPosition();
			Eigen::Vector3<double> vDesFootWorld = footSwingTrajectories[foot].getVelocity();
			Eigen::Vector3<double> pDesLeg = seResult.rBody * (pDesFootWorld - seResult.position) - data_._quadruped->getHipLocation(foot);
			Eigen::Vector3<double> vDesLeg = seResult.rBody * (vDesFootWorld - seResult.vWorld);
			// cout << "Foot " << foot << " relative velocity desired: " << vDesLeg.transpose() << "\n";

			data_._legController->command[foot].pDes = pDesLeg;
			data_._legController->command[foot].vDes = vDesLeg;
			data_._legController->command[foot].kpCartesian = Kp_stance;
			data_._legController->command[foot].kdCartesian = Kd_stance;
			data_._legController->command[foot].forceFeedForward = f_ff[foot];
			data_._legController->command[foot].kdJoint = Eigen::Matrix3<double>::Identity() * 0.2;

			//      footSwingTrajectories[foot]->updateFF(hw_i->leg_controller->leg_datas[foot].q,
			//                                          hw_i->leg_controller->leg_datas[foot].qd, 0); todo removed
			// hw_i->leg_controller->leg_command[foot].tau_ff += 0*footSwingController[foot]->getTauFF();

			//            cout << "Foot " << foot << " force: " << f_ff[foot].transpose() << "\n";
			se_contactState[foot] = contactState;
			// Update for WBC
			// Fr_des[foot] = -f_ff[foot];
		}
	}
	/*
	// se->set_contact_state(se_contactState);
	data_._stateEstimator->setContactPhase(se_contactState);
	// Update For WBC
	pBody_des[0] = world_position_desired[0];
	pBody_des[1] = world_position_desired[1];
	pBody_des[2] = _body_height;
	vBody_des[0] = v_des_world[0];
	vBody_des[1] = v_des_world[1];
	vBody_des[2] = 0.;
	aBody_des.setZero();
	pBody_RPY_des[0] = 0.;
	pBody_RPY_des[1] = 0.;
	pBody_RPY_des[2] = _yaw_des;
	vBody_Ori_des[0] = 0.;
	vBody_Ori_des[1] = 0.;
	vBody_Ori_des[2] = _yaw_turn_rate;
	contact_state = gait->getContactState();
	// END of WBC Update
	*/
}