#include <uWS/uWS.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <chrono>
#include "json.hpp"
#include "PID.h"
#include <ctime>
#include <math.h>

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

void save_params(std::vector<double> const &params, std::vector<double> const &dp, double total_error, double cycle_time, double cycle_dist, double throttle) {
  std::string filename = "params.txt";
  std::ofstream dataFile;
  dataFile.open(filename, std::ios::app);
  dataFile << total_error << " | " << params[0] << " " << params[1] << " " << params[2]
           << " | " << dp[0] << " " << dp[1] << " " << dp[2] << " | " << throttle << " | "
           << cycle_time << " " << cycle_dist << "\n";
  dataFile.close();
}

class Twiddle {
public:

  Twiddle() {
    params_.resize(3);
    dp_.resize(3);
  }

  void Init(std::vector<double> &params) {
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

  void Init(std::vector<double> const &params, std::vector<double> const &dp) {
    params_ = params;
    best_params_ = params;
    dp_ = dp;
    lastError_ = 1000000.0;
    twiddling_started = false;

  }


  double next(double error, std::vector<double> &p) {

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

    // advance to next param
    curr_param = (curr_param + 2) % 3;
    params_[curr_param] += dp_[curr_param];
    is_up = true;
    p = params_;
    return dpSum();

  }

  double dpSum() {
    return dp_[0] + dp_[1] + dp_[2];
  }

  std::vector<double> getParams() {
    return params_;
  }

  std::vector<double> getBestParams() {
    return best_params_;
  }

  std::vector<double> getDp() {
    return dp_;
  }

  double getLastError() {
    return lastError_;
  }

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


int main()
{
  uWS::Hub h;



  PID pid;
  // TODO: Initialize the pid variable.

  Twiddle twiddle;

  const bool twiddle_enable = true;

  const double cycle_time_max = 180.0;
  double cycle_dist_max = 2.0; // one full lap 0.71, on sharp turn 0.42


//  std::vector<double> p = {0.2, 0.003, 0.01};
//  std::vector<double> p = {0.1, 0.0001, 0.02};
//  std::vector<double> p = {0.08, 0.01, 0.05}; // good for 0.5

  //std::vector<double> p = {0.08, 0.01, 0.05}; // started - good

//  std::vector<double> p = {0.106667, 0.0133333, 0.0666667}; // found first iter! throttle = 0.5

//  std::vector<double> p = {0.106667, 0.0133333, 0.0666667}; // found second iter! 360.953 0.151667 0.0133333 0.0666667 0.045 0.0054 0.0297 40.0117, thr = 0.5


  // std::vector<double> p = {0.066667, 0.0201409, 0.0506287}; // found third iter! 1340.21 0.066667 0.0201409 0.0506287 0.0288684 0.0032076 0.0176418 45.0127 thr = 0.7

  // std::vector<double> p = {0.110767, 0.0348409, 0.0968287}; // found fourth iter! 570.396 0.110767 0.0348409 0.0968287 0.02541 0.00847 0.0242 45.0033 thr = 0.5

  //std::vector<double> p = {0.164128, 0.0526279, 0.0726287}; // found fifth iter! 601.452 | 0.164128 0.0526279 0.0726287 | 0.0307461 0.009317 0.02662 | 46.8515 0.70037 thr=0.5

  // std::vector<double> p = {0.066667, 0.00151409, 0.0806287}; // found third iter! 1340.21 | 0.066667 0.0201409 0.0506287 | 0.0288684 0.0032076 0.0176418 45.0127 thr = 0.7

  //std::vector<double> p = {0.0377986, 0.00119333, 0.0744459}; // found! thr = 0.8, 2056.95 | 0.0377986 0.00119333 0.0744459 | 0.016876 0.000187512 0.011459 | 24.4711 0.420531

  //std::vector<double> p = {0.0466836, 0.00167769, 0.0744459}; //thr = 0.8, 2965.36 | 0.0466836 0.00167769 0.0744459 | 0.00799102 0.000132636 0.00443945 | 23.33 0.520237   ++++

  //std::vector<double> p = {0.0386926, 0.00167769, 0.0744459}; // thr 0.8, 3264.17 | 0.0386926 0.00167769 0.0744459 | 0.00799102 0.000132636 0.00443945 | 51.8368 1.00072  ++++

  //std::vector<double> p = {0.0618177, 0.00154422, 0.0652491}; // thr 0.8 1941.66 | 0.0618177 0.00154422 0.0652491 | 6.76738e-05 8.35482e-07 3.75965e-05 | 51.4764 1.00031 +++++

  // std::vector<double> p = {0.166818, 0.00154422, 0.170249}; // thr 0.8 583.667 | 0.166818 0.00154422 0.170249 | 0.0605 0.0009 0.055 | 16.6913 0.200166

  //std::vector<double> p = {0.0377986, 0.00119333, 0.0744459}; // thr 0.8, 10539.1 | 0.216818 0.00254422 0.237055 | 0.0236757 0.000526127 0.0263063 | 147.645 1.89792   +++++

  //std::vector<double> dp = {0.01, 0.0004, 0.025};

  //std::vector<double> dp = {0.01, 0.0004, 0.025}; // thr 0.65 1367.83 | 0.0435932 0.00116149 0.0842111 | 0.00140811 4.37995e-05 0.00292145 | 0.65 | 101.046 1.95025  +++

  // thr 0.7 710.522 | 0.0496775 0.00115605 0.0634661 | 0.00305866 0.000114554 0.00787381 | 0.7 | 166.36 3.50002  ++++


  std::vector<double> p = {0.0496775, 0.00115605, 0.0634661};
  std::vector<double> dp = {0.000504997, 1.54745e-05, 0.00106363};

  double throttle = 0.65;
  double dp_thresh = 0.0005;
  double throttle_dist_ratio = 5.0;

  pid.Init(p[0], p[1], p[2]);
  twiddle.Init(p, dp);

  bool is_initialized = false;


  auto prev_clk = std::chrono::high_resolution_clock::now();
  auto cycle_clk = std::chrono::high_resolution_clock::now();

  double prev_speed = 0.0;
  double cycle_dist = 0.0;

  bool fresh_param = true;

  /*
  h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> *ws, char *message, size_t length, uWS::OpCode opCode) {
    ws->send(message, length, opCode);
  });
   */



  h.onMessage([&pid, &p, &dp, &dp_thresh, &throttle_dist_ratio, &is_initialized, &prev_clk, &cycle_clk, &twiddle, &cycle_time_max, &cycle_dist, &cycle_dist_max, &prev_speed, &fresh_param, &throttle](uWS::WebSocket<uWS::SERVER> *ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value = 0.1;

          auto clk = std::chrono::high_resolution_clock::now();
          const double dt = std::chrono::duration<double>(clk - prev_clk).count();
          prev_clk = clk;

          double cycle_dt = std::chrono::duration<double>(clk - cycle_clk).count();

          double cycle_avg_speed = cycle_dist * 3600.0/cycle_dt;

          bool crash = false;
          if (cycle_dist > 0.01) {
            if (pid.TotalError()/cycle_dist > 20000.0) {
              std::cout << "CRASH!!!! - " << pid.TotalError()/cycle_dist << std::endl;
              crash = true;
            }
          } else if (pid.TotalError() > 1000.0) {
            std::cout << "CRASH!!!! - small dist - " << pid.TotalError()/cycle_dist << std::endl;
            crash = true;
          }

          bool userInterrupted = (dt > 1.0);


          // Initialize first time PID params
          if (!is_initialized || userInterrupted || crash) {

            if (!is_initialized) {
              std::cout << "======= INIT ==================" << std::endl;
              p = twiddle.getParams();
              is_initialized = true;
//              fresh_param = true;
            } else if (twiddle_enable) {
              const double cycle_dt = std::chrono::duration<double>(clk - cycle_clk).count();
              save_params(twiddle.getParams(), twiddle.getDp(), pid.TotalError()/(cycle_dist*cycle_dist), cycle_dt, cycle_dist, throttle);

              if (twiddle.next(pid.TotalError()/(cycle_dist*cycle_dist), p) < dp_thresh) {
                p = twiddle.getBestParams();
                dp[0] = p[0] * 0.1;
                dp[1] = p[1] * 0.1;
                dp[2] = p[2] * 0.1;
                twiddle.Init(p, dp);
                throttle += 0.05;
              }

//              if (fresh_param) {
//                fresh_param = !fresh_param;
//              } else {
//                p = twiddle.next(pid.TotalError()/(cycle_dist*cycle_dist));
//                fresh_param = true;
//              }
            }

            pid.Init(p[0], p[1], p[2]);

            std::cout << "INIT PID: " << pid.getParamsStr() << std::endl;

            cycle_clk = clk;
            cycle_dist = 0;

            if (crash) {
              // Restart
              std::string reset_msg = "42[\"reset\",{}]";
              ws->send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
              return;
            }

          }

          cycle_dt = std::chrono::duration<double>(clk - cycle_clk).count();

          cycle_dist += prev_speed * dt / 3600.0; // distance in miles (speed in mph)
          prev_speed = speed;


          std::cout << "DT = " << dt << ", C_DT = " << cycle_dt << ", C_DIST = " << cycle_dist
                    << ", T = " << throttle << ", C_AS = " << cycle_avg_speed << std::endl;

          cycle_dist_max = throttle * throttle_dist_ratio;

          // Finish previous cycle and start a new one
          if (twiddle_enable && cycle_dist > cycle_dist_max) { //cycle_dt > cycle_time_max
            double cycle_error = pid.TotalError()/(cycle_dist*cycle_dist);
            save_params(twiddle.getParams(), twiddle.getDp(), cycle_error, cycle_dt, cycle_dist, throttle);


            if (twiddle.next(cycle_error, p) < dp_thresh) {
              p = twiddle.getBestParams();
              dp[0] = p[0] * 0.1;
              dp[1] = p[1] * 0.1;
              dp[2] = p[2] * 0.1;
              twiddle.Init(p, dp);
              throttle += 0.05;
            }

            pid.Init(p);
            cycle_clk = clk;
            cycle_dist = 0.0;
//            fresh_param = true;
            std::cout << "<<<<< Finish Cycle " << std::endl;
            std::cout << "CYCLE_ERROR = " << cycle_error << std::endl;
            std::cout << "New params: " << pid.getParamsStr() << std::endl;
          }


          pid.UpdateError(cte);


          std::cout << ">> LastBestError = " << twiddle.getLastError() << std::endl;


          if (cycle_dist > 0.01) {
            std::cout << ">> CurrAvgError = " << pid.TotalError() / cycle_dist << std::endl;
          }


          const std::vector<double> dpl = twiddle.getDp();
          std::cout << "Kp = " << pid.Kp << ", Ki = " << pid.Ki << ", Kd = " << pid.Kd
                    << ", (" << dpl[0] << ", " << dpl[1] << ", " << dpl[2] << ") "
                    << twiddle.dpSum() << std::endl;


          steer_value = - pid.Kp * pid.p_error - pid.Ki * pid.i_error - pid.Kd * pid.d_error;


          // sigmoid - to make value from -1 to 1
//          steer_value = 1 / (1 + exp(-steer_value));

          if (steer_value < -1.0) {
            steer_value = -1.0;
          } else if (steer_value > 1.0) {
            steer_value = 1.0;
          }
          
          // DEBUG
          std::cout << "CTE: " << cte << ", Steering: " << steer_value << ", Speed: " << speed
                    << ", Thr: " << throttle << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
//          std::cout << msg << std::endl;
          ws->send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws->send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

/*
  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });
*/

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
//    std::cout << "clock = " << clock() << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
