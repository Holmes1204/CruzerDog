#include <FSM/State_Worker/Locomotion_Worker.h>

Locomotion::Locomotion(FSM_data *data)
	: FSM_State(data, FSM_StateName::LOCOMOTION, "Locomotion"),
	  horizonLength(10),
	  dt(quad::dt),
	  iterationsBetweenMPC(20),
	  trotting(horizonLength, Vec4<int>(0, 5, 5, 0), Vec4<int>(5, 5, 5, 5), "Trotting")
{
	int transition_time = 200;
	dtMPC = dt * iterationsBetweenMPC;
	std::cout << "dt: " << dt << " dtMPC: " << dtMPC << " horizon: " << horizonLength << std::endl;
	// std::cout << "locomotion initial" << std::endl;
	// file.open("path.csv");
	Kp << 50, 0, 0,
		0, 50, 0,
		0, 0, 50;
	Kp_stance = 0 * Kp;

	Kd << .5, 0, 0,
		0, .5, 0,
		0, 0, .5;
	Kd_stance = Kd;
}

Locomotion::~Locomotion()
{
}

void Locomotion::onExit()
{
	std::cout << "Locomotion finished!" << std::endl;
}

void Locomotion::onEnter()
{
	turnOnAllSafetyChecks();
	iterationCounter = 0;
	firstRun = true;
	transition_time = quad::df;
	transitionData.zero();
	for (int i = 0; i < 4; i++)
	{
		firstSwing[i] = true;
	}
	std::cout << "Locomotion start!" << std::endl;
}
FSM_StateName Locomotion::checkTransition()
{
	if (data_->global_state_switch)
	{
		// file.close();
		data_->global_state_switch = 0;
		std::cout << stateString << " to "
				  << "STEADY" << std::endl;
		return FSM_StateName::STEADY;
	}
	else
	{
		return FSM_StateName::LOCOMOTION;
	}
}

// Runs the transition behaviors and returns true when done transitioning
TransitionData Locomotion::transition()
{
	turnOffAllSafetyChecks();
	if (transition_time > 0)
	{
		for (int foot = 0; foot < 4; foot++)
		{
			data_->_legController->command[foot].zero();
			data_->_legController->command[foot].kdJoint = Kd * 5;
		}
		transition_time--;
		// std::cout<<"trsansistioning"<<std::endl;
	}
	else
	{
		transitionData.done = true;
	}
	return transitionData;
}
/**
 * Calls the functions to be executed on each control loop iteration.
 */
void Locomotion::run()
{
	if (firstRun)
	{
		for (int i = 0; i < 4; i++)
		{
			pFoot[i] = data_->_legController->data[i].p;
			footSwingTrajectories[i].setHeight(0.05);
			footSwingTrajectories[i].setInitialPosition(pFoot[i]);
			footSwingTrajectories[i].setFinalPosition(pFoot[i]);
		}
		firstRun = false;
	}
	// pick gait
	Gait *gait = &trotting;
	gait->setIterations(iterationsBetweenMPC, iterationCounter);

	// hip frame 下足端的位置
	for (int i = 0; i < 4; i++)
	{
		pFoot[i] = data_->_legController->data[i].p;
	}

	// foot placement
	for (int l = 0; l < 4; l++)
		swingTimes[l] = gait->getCurrentSwingTime(dtMPC, l);

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
		footSwingTrajectories[i].setHeight(0.05);
		Vec3<double> Pf = Vec3<double>(0.1, 0, -0.3);
		footSwingTrajectories[i].setFinalPosition(Pf);
	}

	// gait
	Vec4<double> contactStates = gait->getContactState();
	Vec4<double> swingStates = gait->getSwingState();
	Vec4<double> se_contactState(0, 0, 0, 0);

	for (int foot = 0; foot < 4; foot++)
	{
		double contactState = contactStates[foot];
		double swingState = swingStates[foot];
		data_->_legController->command[foot].zero();
		Vec3<double> pDesFootWorld;
		Vec3<double> vDesFootWorld;
		if (swingState > 0) // foot is in swing
		{
			if (firstSwing[foot])
			{
				firstSwing[foot] = false;
				Vec3<double> p_init = Vec3<double>(0, 0, -0.3);
				footSwingTrajectories[foot].setInitialPosition(p_init);
			}
			footSwingTrajectories[foot].computeSwingTrajectoryBezier(swingState, swingTimes[foot]);

			pDesFootWorld = footSwingTrajectories[foot].getPosition();
			vDesFootWorld = footSwingTrajectories[foot].getVelocity();
			// leg frame 的kp kd 控制

			data_->_legController->command[foot].zero();
			data_->_legController->command[foot].q_Des = data_->_quadruped->inverse_kinematic(pDesFootWorld, 0);
			data_->_legController->command[foot].kpJoint = Kp;
			data_->_legController->command[foot].kdJoint = Kd;
		}
		else // foot is in stance
		{
			firstSwing[foot] = true;
			pDesFootWorld = footSwingTrajectories[foot].getPosition();
			vDesFootWorld = footSwingTrajectories[foot].getVelocity();
			data_->_legController->command[foot].zero();
			data_->_legController->command[foot].q_Des = data_->_quadruped->inverse_kinematic(pDesFootWorld, 0);
			data_->_legController->command[foot].kpJoint = Kp_stance;
			data_->_legController->command[foot].kdJoint = Kd_stance;
			se_contactState[foot] = contactState;
		}
		// file << pDesFootWorld.transpose() << std::endl
		// 	 << vDesFootWorld.transpose() << std::endl;
	}
	iterationCounter++;
	{
		static uint32_t dbg_count = 0;
		if (dbg_count % 20 == 0)
		{
            system("clear");
            for (int i = 0; i < 4; i++)
            {
                std::cout << "leg" << i << std::endl;
                std::cout << "state" << data_->_legController->motor_enable << std::endl;
                std::cout << "J " << std::endl
                          << data_->_legController->data[i].J << std::endl;
                std::cout << "q " << std::endl
                          << data_->_legController->data[i].q.transpose() << "\n"
                          << "d " << std::endl
                          << data_->_legController->data[i].qd.transpose() << std::endl;
                std::cout << "p " << std::endl
                          << data_->_legController->data[i].p.transpose() << "\n"
                          << "v " << std::endl
                          << data_->_legController->data[i].v.transpose() << std::endl;
                dbg_count = 0;
            }
		}
		dbg_count++;
	}
}
