#include <iostream>
#include <Eigen/Dense>


namespace depth_flight_controller
{
  class PathPlanner 
  {
  public:



  protected:



  private:

    static const float target_radius = 3;      % in m
    static const float abs_vel = 2;          % in m/s
    static const float controller_freq = 50;   % in Hz
    static const float super_factor = 4;
    static const float controller_freq = super_factor * controller_freq;
    static const float pi = 3.14159;
}