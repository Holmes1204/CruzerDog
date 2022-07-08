#include "Leg_Control/Gait.h"

// Offset - Duration Gait
OffsetDurationGait::OffsetDurationGait(int nSegment, Eigen::Vector4<int> offsets, Eigen::Vector4<int> durations, const std::string &name) :
  _offsets(offsets.array()),
  _durations(durations.array()),
  _nIterations(nSegment)//horizon length 
{

  _name = name;
  // allocate memory for MPC gait table
  _mpc_table = new int[nSegment * 4];

  _offsetsDouble = offsets.cast<double>() / (double) nSegment;
  _durationsDouble = durations.cast<double>() / (double) nSegment;

  _stance = durations[0];
  _swing = nSegment - durations[0];
}

OffsetDurationGait::~OffsetDurationGait() {
  delete[] _mpc_table;
}


Eigen::Vector4<double> OffsetDurationGait::getContactState() {
  Array4d progress = _phase - _offsetsDouble;

  for(int i = 0; i < 4; i++)
  {
    if(progress[i] < 0) progress[i] += 1.;
    
    if(progress[i] > _durationsDouble[i])
    {
      progress[i] = 0.;
    }
    else
    {
      progress[i] = progress[i] / _durationsDouble[i];
    }
  }

  //printf("contact state: %.3f %.3f %.3f %.3f\n", progress[0], progress[1], progress[2], progress[3]);
  return progress.matrix();
}


Eigen::Vector4<double> OffsetDurationGait::getSwingState()
{
  Array4d swing_offset = _offsetsDouble + _durationsDouble;
  for(int i = 0; i < 4; i++)
    if(swing_offset[i] > 1) swing_offset[i] -= 1.;
  Array4d swing_duration = 1. - _durationsDouble;

  Array4d progress = _phase - swing_offset;

  for(int i = 0; i < 4; i++)
  {
    if(progress[i] < 0) progress[i] += 1.f;
    if(progress[i] > swing_duration[i])
    {
      progress[i] = 0.;
    }
    else
    {
      progress[i] = progress[i] / swing_duration[i];
    }
  }

  //printf("swing state: %.3f %.3f %.3f %.3f\n", progress[0], progress[1], progress[2], progress[3]);
  return progress.matrix();
}



int* OffsetDurationGait::getMpcTable()
{

  //printf("MPC table:\n");
  for(int i = 0; i < _nIterations; i++)
  {
    int iter = (i + _iteration + 1) % _nIterations;
    Array4i progress = iter - _offsets;
    for(int j = 0; j < 4; j++)
    {
      if(progress[j] < 0) progress[j] += _nIterations;
      if(progress[j] < _durations[j])
        _mpc_table[i*4 + j] = 1;
      else
        _mpc_table[i*4 + j] = 0;

      //printf("%d ", _mpc_table[i*4 + j]);
    }
    //printf("\n");
  }



  return _mpc_table;
}


void OffsetDurationGait::setIterations(int iterationsPerMPC, int currentIteration)
{
  _iteration = (currentIteration / iterationsPerMPC) % _nIterations;
  _phase = (double)(currentIteration % (iterationsPerMPC * _nIterations)) / (double) (iterationsPerMPC * _nIterations);
}


int OffsetDurationGait::getCurrentGaitPhase() {
  return _iteration;
}

double OffsetDurationGait::getCurrentSwingTime(double dtMPC, int leg) {
  (void)leg;
  return dtMPC * _swing;
}


double OffsetDurationGait::getCurrentStanceTime(double dtMPC, int leg) {
  (void) leg;
  return dtMPC * _stance;
}

void OffsetDurationGait::debugPrint() {

}
