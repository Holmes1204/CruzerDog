#include "Leg_Control/Gait.h"

// Offset - Duration Gait
OffsetDurationGait::OffsetDurationGait(int nSegment, Vec4<int> offsets,
                                       Vec4<int> durations, const std::string &name)
    : _offsets(offsets.array()),     //支撑相的开始时间，可推导摆动相位
      _durations(durations.array()), // duration 是指支撑相的长度
      _nIterations(nSegment)         // horizon length 也就是将一个步态周期分为n个，为了与MPC的周期相同，所以他俩长度一样
{
  _name = name;
  // allocate memory for MPC gait table
  _mpc_table = new int[nSegment * 4];
  _offsetsDouble = offsets.cast<double>() / (double)nSegment;     //在一个周期内抬放脚的顺序
  _durationsDouble = durations.cast<double>() / (double)nSegment; //每只脚的支撑相周期，在周期步态下这个值相同
  _stance = durations[0];                                         //支撑相迭代次数
  _swing = nSegment - durations[0];                               //悬空相迭代次数
}

OffsetDurationGait::~OffsetDurationGait()
{
  delete[] _mpc_table;
}

Vec4<double> OffsetDurationGait::getContactState()
{
  Array4d progress = _phase - _offsetsDouble; //

  for (int i = 0; i < 4; i++)
  {
    if (progress[i] < 0)
      progress[i] += 1.;

    if (progress[i] > _durationsDouble[i]) //支撑相位的时间
    {
      progress[i] = 0.;
    }
    else
    {
      progress[i] = progress[i] / _durationsDouble[i];
    }
  }
  // printf("phase   state: %.3f\n", _phase);
  // printf("ohters  state: %.3f %.3f %.3f %.3f\n", _offsetsDouble[0], _offsetsDouble[1], _offsetsDouble[2], _offsetsDouble[3]);
  // printf("contact state: %.3f %.3f %.3f %.3f\n", progress[0], progress[1], progress[2], progress[3]);
  // printf("\n");
  return progress.matrix();
}

Vec4<double> OffsetDurationGait::getSwingState()
{
  Array4d swing_offset = _offsetsDouble + _durationsDouble;
  for (int i = 0; i < 4; i++)
    if (swing_offset[i] > 1)
      swing_offset[i] -= 1.;
  Array4d swing_duration = 1. - _durationsDouble;

  Array4d progress = _phase - swing_offset;

  for (int i = 0; i < 4; i++)
  {
    if (progress[i] < 0)
      progress[i] += 1.f;
    if (progress[i] > swing_duration[i])
    {

      progress[i] = 0.;
    }
    else
    {
      progress[i] = progress[i] / swing_duration[i];
    }
  }
  // printf("phase  state: %.3f\n", _phase);
  // printf("ohters state: %.3f %.3f %.3f %.3f\n", swing_offset[0], swing_offset[1], swing_offset[2], swing_offset[3]);
  // printf("swing  state: %.3f %.3f %.3f %.3f\n", progress[0], progress[1], progress[2], progress[3]);
  // printf("\n");

  return progress.matrix();
}
//1代表在各个相位处着地，0代表悬空
int *OffsetDurationGait::getMpcTable()
{
  // printf("MPC table:\n");
  for (int i = 0; i < _nIterations; i++)
  {
    int iter = (i + _iteration + 1) % _nIterations;
    Array4i progress = iter - _offsets;
    for (int j = 0; j < 4; j++)
    {
      if (progress[j] < 0)
        progress[j] += _nIterations;
      if (progress[j] < _durations[j])
        _mpc_table[i * 4 + j] = 1;
      else
        _mpc_table[i * 4 + j] = 0;

      // printf("%d ", _mpc_table[i * 4 + j]);
    }
    // printf("\n");
    // printf("progress:%4d %4d %4d %4d\n", progress[0], progress[1], progress[2], progress[3]);
  }
  // printf("\n");
  return _mpc_table;
}

void OffsetDurationGait::setIterations(int iterationsPerMPC, int currentIteration)
{
  _iteration = (currentIteration / iterationsPerMPC) % _nIterations;                                                   //一个步态周期内的迭代次数
  _phase = (double)(currentIteration % (iterationsPerMPC * _nIterations)) / (double)(iterationsPerMPC * _nIterations); // iterationsPerMPC*一个步态周期内的迭代次数
  // printf("durations : %4d %4d %4d %6.3f\n", _iteration, iterationsPerMPC, currentIteration, _phase);
}

int OffsetDurationGait::getCurrentGaitPhase()
{
  return _iteration;
}

double OffsetDurationGait::getCurrentSwingTime(double dtMPC, int leg)
{
  (void)leg;
  return dtMPC * _swing;
}

double OffsetDurationGait::getCurrentStanceTime(double dtMPC, int leg)
{
  (void)leg;
  return dtMPC * _stance;
}

void OffsetDurationGait::debugPrint()
{
}
