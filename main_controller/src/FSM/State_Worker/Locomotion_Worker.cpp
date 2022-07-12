#include <FSM/State_Worker/Locomotion_Worker.h>

Locomotion::Locomotion(FSM_data &data_, FSM_topic_control &tpcl)
	: data_(data_),
	  tpcl_(tpcl),
	  horizonLength(10),
	  dt(0.002),
	  iterationsBetweenMPC(10),
	  trotting(horizonLength, Vec4<int>(0, 2, 5, 7), Vec4<int>(5, 5, 5, 5), "Trotting")
{
	dtMPC = dt * iterationsBetweenMPC;
	mpc_solver = new MPC_SLOVER(1, dtMPC);
	std::cout << "locomotion initial" << std::endl;
	// file.open("path.csv");
	// Initialize GRF and footstep locations to 0s
	// this->footFeedForwardForces = Mat34<T>::Zero();
	this->r_foot_world = Mat34<double>::Zero();
	//_wbc_ctrl = new LocomotionCtrl<T>(_controlFSMData->_quadruped->buildModel());
	//_wbc_data = new LocomotionCtrlData<T>();
}

/**
 * Calls the functions to be executed on each control loop iteration.
 */
void Locomotion::run()
{

	//从状态估计器中获得 rpy，xyz world frame

	b_run();

	/*
	Vec3<T> pDes_backup[4];
	Vec3<T> vDes_backup[4];
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

	tpcl_.unitree_sim_send_cmd();
}

bool Locomotion::is_finished()
{
	if (iterationCounter > 100 * iterationsBetweenMPC)
	{
		file.close();
		return false;
	}

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

	_x_vel_des = 5.0;
	_y_vel_des = 0.0;
	Vec3<double> v_des_robot(_x_vel_des, _y_vel_des, 0);
	Vec3<double> v_des_world = seResult.rBody.transpose() * v_des_robot;
	Vec3<double> v_robot = seResult.vWorld;

	if (firstRun)
	{
		world_position_desired[0] = seResult.position[0];
		world_position_desired[1] = seResult.position[1];
		world_position_desired[2] = seResult.rpy[2];

		for (int i = 0; i < 4; i++)
		{
			// pFoot[i] = seResult.position +
			// 		   seResult.rBody.transpose() * (data_._quadruped->getHipLocation(i) +
			// 										 data_._legController->data[i].p); //此时的p应该是在hipframe下的
			pFoot[i] = data_._legController->data[i].p;
			// footSwingTrajectories[i].setHeight(0.05);
			footSwingTrajectories[i].setHeight(0.05);
			footSwingTrajectories[i].setInitialPosition(pFoot[i]);
			footSwingTrajectories[i].setFinalPosition(pFoot[i]);
			// if(i==0)
			// std::cout << "leg " << i << " :" << pFoot[i].transpose() << std::endl;
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
	gait->setIterations(iterationsBetweenMPC, iterationCounter);
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

	// world frame 下足端轨迹
	for (int i = 0; i < 4; i++)
	{
		// pFoot[i] = seResult.position +
		// 		   seResult.rBody.transpose() *
		// 			   (data_._quadruped->getHipLocation(i) + data_._legController->data[i].p);
		pFoot[i] = data_._legController->data[i].p;
		// // debug
		// {
		// 	static uint32_t dbg_count = 0;
		// 	static int fsd = 1;
		// 	if (dbg_count % 20 == 0)
		// 	{
		// 		system("clear");
		// 		std::cout << "p_foot_world_" << fsd << std::endl
		// 				  << pFoot[fsd].transpose() << std::endl
		// 				  << "postion" << std::endl
		// 				  << seResult.position.transpose() << std::endl
		// 				  << "r_body" << std::endl
		// 				  << seResult.rBody << std::endl
		// 				  << std::endl;
		// 		dbg_count = 0;
		// 	}
		// 	dbg_count++;
		// }
	}

	world_position_desired += dt * Vec3<double>(v_des_world[0], v_des_world[1], 0);
	// foot placement
	for (int l = 0; l < 4; l++)
		swingTimes[l] = gait->getCurrentSwingTime(dtMPC, l);

	double side_sign[4] = {-1, 1, -1, 1};
	// double v_abs = std::fabs(seResult.vBody[0]);
	double v_abs = std::fabs(v_des_robot[0]);

	//轨迹规划在世界坐标系完成，控制在leg frame下完成
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
		// footSwingTrajectories[i].setHeight(.06);
		footSwingTrajectories[i].setHeight(0.05);

		double stance_time = gait->getCurrentStanceTime(dtMPC, i);

		Vec3<double> des_vel;
		des_vel[0] = _x_vel_des;
		des_vel[1] = _y_vel_des;
		des_vel[2] = 0.0;
		// P_foot findal,足端轨迹的最后值，在world frame下
		// Vec3<double> Pf = seResult.position +
		// 							seResult.rBody.transpose() *
		// 								(data_._quadruped->getHipLocation(i) + des_vel * swingTimeRemaining[i]);

		Vec3<double> Pf =Vec3<double>(0.0, 0, -0.3);

		//这个大概是30cm
		// double p_rel_max = 0.15f;
		// // Using the estimated velocity is correct
		// // Vec3<double> des_vel_world = seResult.rBody.transpose() * des_vel;
		// double pfx_rel = seResult.vWorld[0] * (.5) * stance_time +
		// 				 .03f * (seResult.vWorld[0] - v_des_world[0]);

		// double pfy_rel = seResult.vWorld[1] * .5 * stance_time * dtMPC +
		// 				 .03f * (seResult.vWorld[1] - v_des_world[1]);

		// pfx_rel = fminf(fmaxf(pfx_rel, -p_rel_max), p_rel_max);
		// pfy_rel = fminf(fmaxf(pfy_rel, -p_rel_max), p_rel_max);
		// Pf[0] += pfx_rel;
		// Pf[1] += pfy_rel;
		// Pf[2] = -0.2; //一定要慎重
		// Pf[2] = 0.0;
		footSwingTrajectories[i].setFinalPosition(Pf);
		// if(i==0)
		// std::cout << "leg " << i << " :" << Pf.transpose() << std::endl;
	}

	// printf("swing time: %.3f %.3f %.3f %.3f\n", swingTimeRemaining[0], swingTimeRemaining[1], swingTimeRemaining[2], swingTimeRemaining[3]);
	// printf("\n");
	// system("clear");
	//  calc gait
	iterationCounter++;

	//此时stance的kp为零，不会因为位置的关系影响脚的回收工作，这个地方的PID要改
	Kp << 300, 0, 0,
		0, 300, 0,
		0, 0, 300;
	Kp_stance = 0 * Kp;

	Kd << 15, 0, 0,
		0, 15, 0,
		0, 0, 15;
	Kd_stance = Kd;
	// gait

	Vec4<double> contactStates = gait->getContactState();
	Vec4<double> swingStates = gait->getSwingState();

	int *contact_state = gait->getMpcTable();
	// updateMPCIfNeeded(contact_state);
	Vec4<double> se_contactState(0, 0, 0, 0);

	for (int foot = 0; foot < 4; foot++)
	{
		double contactState = contactStates[foot];
		double swingState = swingStates[foot];
		data_._legController->command[foot].zero();
		f_ff[foot] = -seResult.rBody.transpose() * Fr_des.segment(foot * 3, 3);
		if (swingState > 0) // foot is in swing
		{
			if (firstSwing[foot])
			{
				firstSwing[foot] = false;
				Vec3<double> p_init = Vec3<double>(0, 0, -0.3);
				footSwingTrajectories[foot].setInitialPosition(p_init);
			}
			footSwingTrajectories[foot].computeSwingTrajectoryBezier(swingState, swingTimes[foot]);

			Vec3<double> pDesFootWorld = footSwingTrajectories[foot].getPosition();
			Vec3<double> vDesFootWorld = footSwingTrajectories[foot].getVelocity();
			// leg frame 的kp kd 控制
			Vec3<double> pDesLeg = seResult.rBody * (pDesFootWorld - seResult.position) - data_._quadruped->getHipLocation(foot);
			Vec3<double> vDesLeg = seResult.rBody * (vDesFootWorld - seResult.vWorld);

			// Update for WBC
			pFoot_des[foot] = pDesFootWorld;
			vFoot_des[foot] = vDesFootWorld;
			aFoot_des[foot] = footSwingTrajectories[foot].getAcceleration();

			// Update leg control command regardless of the usage of WBIC

			data_._legController->command[foot].zero();
			data_._legController->command[foot].q_Des = data_._quadruped->inverse_kinematic(pDesFootWorld, 0);
			data_._legController->command[foot].kpJoint = Kp;
			data_._legController->command[foot].kdJoint = Kd;

			data_._legController->command[foot].p_Des = pDesLeg;
			data_._legController->command[foot].v_Des = vDesLeg;
			// data_._legController->command[foot].kpCartesian = Kp;
			// data_._legController->command[foot].kdCartesian = Kd;
		}
		else // foot is in stance
		{
			firstSwing[foot] = true;
			Vec3<double> pDesFootWorld = footSwingTrajectories[foot].getPosition();
			Vec3<double> vDesFootWorld = footSwingTrajectories[foot].getVelocity();
			Vec3<double> pDesLeg = seResult.rBody * (pDesFootWorld - seResult.position) - data_._quadruped->getHipLocation(foot);
			Vec3<double> vDesLeg = seResult.rBody * (vDesFootWorld - seResult.vWorld);

			data_._legController->command[foot].zero();
			data_._legController->command[foot].q_Des = data_._quadruped->inverse_kinematic(pDesFootWorld, 0);
			data_._legController->command[foot].kpJoint = Kp_stance;
			data_._legController->command[foot].kdJoint = Kd_stance;

			data_._legController->command[foot].p_Des = pDesLeg;
			data_._legController->command[foot].v_Des = vDesLeg;
			// data_._legController->command[foot].kpCartesian = Kp_stance;
			// data_._legController->command[foot].kdCartesian = Kd_stance;
			// data_._legController->command[foot].force_FF = f_ff[foot];
			// data_._legController->command[foot].kdJoint = Eigen::Matrix3<double>::Identity() * 0.2;
			se_contactState[foot] = contactState;
		}

		// debug
		{

			static uint32_t dbg_count = 0;
			static int i = 0;
			if (dbg_count % 20 == 0)
			{
				system("clear");
				std::cout
					<< "q_Des_0" << std::endl
					<< data_._legController->command[0].q_Des.transpose() << std::endl
					<< "q_Des_1" << std::endl
					<< data_._legController->command[1].q_Des.transpose() << std::endl
					<< "q_Des_2" << std::endl
					<< data_._legController->command[2].q_Des.transpose() << std::endl
					<< "q_Des_3" << std::endl
					<< data_._legController->command[foot].q_Des.transpose() << std::endl
					// << "p" << std::endl
					// << data_._legController->command[foot].p_Des.transpose() << std::endl
					<< std::endl;
				dbg_count = 0;
			}
			dbg_count++;
			// file << data_._legController->command[i].p_Des.transpose() << std::endl;
			// file << data_._legController->data[i].p.transpose() << std::endl;
		}
	}
}

void Locomotion::updateMPCIfNeeded(int *mpcTable)
{
	// iterationsBetweenMPC = 30;
	if ((iterationCounter % iterationsBetweenMPC) == 0)
	{
		auto seResult = data_.model_StateEstimate->getResult();
		double *p = seResult.position.data();

		for (int leg = 0; leg < 4; leg++)
		{
			r_foot_world.col(leg) = seResult.rBody.transpose() * (data_._quadruped->getHipLocation(leg) + data_._legController->data[leg].p);
		}

		Vec12<double> x_0;
		x_0 << seResult.rpy, seResult.position, seResult.omegaWorld, seResult.vWorld;
		Vec3<double> v_des_robot(_x_vel_des, _y_vel_des, 0);
		Vec3<double> v_des_world = seResult.rBody.transpose() * v_des_robot;

		// printf("Position error: %.3f, integral %.3f\n", pxy_err[0], x_comp_integral);
		double *trajAll = new double[12 * horizonLength];
		int current_gait = 0;
		if (current_gait == 4)
		{
			double trajInitial[12] = {
				_roll_des,
				_pitch_des /*-hw_i->state_estimator->se_ground_pitch*/,
				(double)stand_traj[5] /*+(double)stateCommand->data_.stateDes[11]*/,
				(double)stand_traj[0] /*+(double)fsm->main_control_settings.p_des[0]*/,
				(double)stand_traj[1] /*+(double)fsm->main_control_settings.p_des[1]*/,
				(double)_body_height /*fsm->main_control_settings.p_des[2]*/,
				0, 0, 0, 0, 0, 0};

			for (int i = 0; i < horizonLength; i++)
				for (int j = 0; j < 12; j++)
					trajAll[12 * i + j] = trajInitial[j];
		}
		else
		{
			const double max_pos_error = .1;
			double xStart = world_position_desired[0];
			double yStart = world_position_desired[1];

			if (xStart - p[0] > max_pos_error)
				xStart = p[0] + max_pos_error;
			if (p[0] - xStart > max_pos_error)
				xStart = p[0] - max_pos_error;

			if (yStart - p[1] > max_pos_error)
				yStart = p[1] + max_pos_error;
			if (p[1] - yStart > max_pos_error)
				yStart = p[1] - max_pos_error;

			world_position_desired[0] = xStart;
			world_position_desired[1] = yStart;

			double trajInitial[12] = {
				0,		  // 0
				0,		  // 1
				_yaw_des, // 2
				// yawStart,    // 2
				xStart,				  // 3
				yStart,				  // 4
				(double)_body_height, // 5
				0,					  // 6
				0,					  // 7
				_yaw_turn_rate,		  // 8
				v_des_world[0],		  // 9
				v_des_world[1],		  // 10
				0};					  // 11

			for (int i = 0; i < horizonLength; i++)
			{
				for (int j = 0; j < 12; j++)
					trajAll[12 * i + j] = trajInitial[j];

				if (i == 0) // start at current position  TODO consider not doing this
				{
					// trajAll[3] = hw_i->state_estimator->se_pBody[0];
					// trajAll[4] = hw_i->state_estimator->se_pBody[1];
					trajAll[2] = seResult.rpy[2];
				}
				else
				{
					trajAll[12 * i + 2] = trajAll[12 * (i - 1) + 2];
					trajAll[12 * i + 3] = trajAll[12 * (i - 1) + 3] + dtMPC * v_des_world[0];
					trajAll[12 * i + 4] = trajAll[12 * (i - 1) + 4] + dtMPC * v_des_world[1];
				}
			}
		}
		// MPC_calculate here
		mpc_solver->solve_mpc(seResult.rpy(2), x_0, r_foot_world, mpcTable, trajAll, Fr_des);
	}
}
