#include <iostream>
#include <sstream>
#include <vector>
#include <ctime>
#include <chrono>
#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
  this->p_error = 0;
  this->d_error = 0;
  this->i_error = 0;

  is_updated = false;
  total_err = 0;

}

void PID::Init(std::vector<double> &p) {
  std::cout << "AAAAAAAAAA " << p.size();
  Init(p[0], p[1], p[2]);
}

void PID::UpdateError(double cte) {

  if (!is_updated) {
    this->p_error = cte;
    this->prev_clk = std::chrono::high_resolution_clock::now();
    is_updated = true;
    return;
  }

  auto clk = std::chrono::high_resolution_clock::now();
  double dt = std::chrono::duration<double>(clk - this->prev_clk).count();
  this->prev_clk = clk;

  this->d_error = (cte - this->p_error) / dt;

  this->i_error += cte * dt;

  this->p_error = cte;

  this->total_err += cte * cte;

  std::cout << "UpdateError: dt = " << dt << ", p_e = " << this->p_error << ", i_e = " << this->i_error << ", d_e = " << this->d_error << std::endl;
  std::cout << "UpdateError: P = " << this->p_error * this->Kp
            << ", I = " << this->i_error * this->Ki
            << ", D = " << this->d_error * this->Kd
            << std::endl;
  std::cout << ">>>>>> TotalError = " << this->total_err << std::endl;

}

double PID::TotalError() {
  return this->total_err;
}

std::string PID::getParamsStr() {
  ostringstream oss;
  oss << "PID params: Kp = " << Kp << ", Ki = " << Ki << ", Kd = " << Kd << std::endl;
  return oss.str();
}

