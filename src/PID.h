#ifndef PID_H
#define PID_H

#include <chrono>
#include <vector>

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  bool is_updated;
  std::chrono::high_resolution_clock::time_point prev_clk;

  double total_err;

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);
  void Init(std::vector<double> &p);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  std::string getParamsStr();
};

class Twiddle {
public:

  Twiddle();

  void Init(std::vector<double> &params);

  void Init(std::vector<double> const &params, std::vector<double> const &dp);

  double next(double error, std::vector<double> &p);

  double dpSum();

  std::vector<double> getParams();

  std::vector<double> getBestParams();

  std::vector<double> getDp();

  double getLastError();

private:
  const static int n_ = 3;
  int curr_param;
  bool twiddling_started;
  bool is_up;
  std::vector<double> params_;
  std::vector<double> best_params_;
  std::vector<double> dp_;
  double lastError_;
};


#endif /* PID_H */
