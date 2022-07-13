#ifndef PROJECT_GAIT_H
#define PROJECT_GAIT_H
#include <string>
#include <eigen_types.h>
#include <queue>
using Eigen::Array4d;
using Eigen::Array4i;

class Gait
{
public:
  virtual ~Gait() = default;

  virtual Vec4<double> getContactState() = 0;
  virtual Vec4<double> getSwingState() = 0;
  virtual int *getMpcTable() = 0;
  virtual void setIterations(int iterationsBetweenMPC, int currentIteration) = 0;
  virtual double getCurrentStanceTime(double dtMPC, int leg) = 0;
  virtual double getCurrentSwingTime(double dtMPC, int leg) = 0;
  virtual int getCurrentGaitPhase() = 0;
  virtual void debugPrint() {}

protected:
  std::string _name;
};

class OffsetDurationGait : public Gait
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  OffsetDurationGait(int nSegment, Vec4<int> offset, Vec4<int> durations, const std::string &name);
  ~OffsetDurationGait();
  Vec4<double> getContactState();
  Vec4<double> getSwingState();
  int *getMpcTable();
  void setIterations(int iterationsBetweenMPC, int currentIteration);
  double getCurrentStanceTime(double dtMPC, int leg);
  double getCurrentSwingTime(double dtMPC, int leg);
  int getCurrentGaitPhase();
  void debugPrint();

private:
  int *_mpc_table = NULL;
  Array4i _offsets;         // offset in mpc segments
  Array4i _durations;       // duration of step in mpc segments
  Array4d _offsetsDouble;   // offsets in phase (0 to 1)
  Array4d _durationsDouble; // durations in phase (0 to 1)
  int _stance;
  int _swing;
  int _iteration;
  int _nIterations;
  double _phase;
};

#endif // PROJECT_GAIT_H
