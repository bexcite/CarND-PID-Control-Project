#include <iostream>
#include <sstream>
#include <vector>
#include <ctime>
#include <chrono>
#include "PID.h"

using namespace std;

/* =================================================== */
/* === PID implementation ============================ */
/* =================================================== */

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

  std::cout << "UE: dt = " << dt << ", p_e = " << this->p_error << ", i_e = " << this->i_error << ", d_e = " << this->d_error << std::endl;
  std::cout << "UE: P = " << this->p_error * this->Kp
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
  oss << "PID params: Kp = " << Kp << ", Ki = " << Ki << ", Kd = " << Kd;
  return oss.str();
}


/* =================================================== */
/* === Twiddle implementation ======================== */
/* =================================================== */


Twiddle::Twiddle() {
  params_.resize(3);
  dp_.resize(3);
}

void Twiddle::Init(std::vector<double> &params) {
//    params_ = std::vector<double>();
//    params_.push_back(params[0]);
  //params_.assign(params, params+3);
  params_ = params;
  best_params_ = params;
  for (int i = 0; i < n_; ++i) {
    dp_[i] = params[i]/3.0;
  }
  lastError_ = 1000000.0;
  twiddling_started = false;
}

void Twiddle::Init(std::vector<double> const &params, std::vector<double> const &dp) {
  params_ = params;
  best_params_ = params;
  dp_ = dp;
  lastError_ = 1000000.0;
  twiddling_started = false;

}


double Twiddle::next(double error, std::vector<double> &p) {

  if (!twiddling_started) {
    twiddling_started = true;
    lastError_ = error;
    curr_param = 0;
    params_[curr_param] += dp_[curr_param];
    is_up = true;
    p = params_;
    return dpSum();
  }

  if (error < lastError_) {
    lastError_ = error;
    best_params_ = params_;
    dp_[curr_param] *= 1.1;

    // advance to next param
    curr_param = (curr_param + 2) % 3;
    params_[curr_param] += dp_[curr_param];
    is_up = true;
    p = params_;
    return dpSum();
  }

  if (is_up) {
    // try lower
    params_[curr_param] -= 2 * dp_[curr_param];
    is_up = !is_up;
    p = params_;
    return dpSum();
  }

  // return param back and decrease step
  params_[curr_param] += dp_[curr_param];
  dp_[curr_param] *= 0.9;

  // advance to a next param
  curr_param = (curr_param + 2) % 3;
  params_[curr_param] += dp_[curr_param];
  is_up = true;
  p = params_;
  return dpSum();

}

double Twiddle::dpSum() {
  return dp_[0] + dp_[1] + dp_[2];
}

std::vector<double> Twiddle::getParams() {
  return params_;
}

std::vector<double> Twiddle::getBestParams() {
  return best_params_;
}

std::vector<double> Twiddle::getDp() {
  return dp_;
}

double Twiddle::getLastError() {
  return lastError_;
}

