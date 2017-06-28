#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <cstdio>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"
#include "params.h"

// for convenience
using json = nlohmann::json;
using namespace mpc_params;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          //double psi_unity = j[1]["psi_unity"];
          double v = j[1]["speed"];
          double delta = j[1]["steering_angle"];
          delta *= -1.0; // convert as per World->Unity conventions
          //double accel = j[1]["throttle"];
          
          // Transform telemetry to car coordinate frame
          Eigen::VectorXd ptsx_car = Eigen::VectorXd(ptsx.size());
          Eigen::VectorXd ptsy_car = Eigen::VectorXd(ptsx.size());
          for(size_t i=0; i<ptsx.size(); i++) {
            ptsx_car(i) =  (ptsx[i]-px)*cos(psi) + (ptsy[i]-py)*sin(psi);
            ptsy_car(i) = -(ptsx[i]-px)*sin(psi) + (ptsy[i]-py)*cos(psi);
          }

          // Fit a 3rd degree polynomial to global waypoints
          auto coeffs = polyfit(ptsx_car, ptsy_car, 3);

          // Update state accounding for latency
          v *= 0.44704; // convert from mph to m/s
          psi =  v*delta/Lf*dt_latency;
          px = v*cos(psi)*dt_latency;
          py = v*sin(psi)*dt_latency;
          
          // Calculate the cross track error assuming
          // constant velocity and psi during latency
          double cte = polyeval(coeffs, px);
          // Calculate the orientation error
          double dpy = coeffs[1] + 2*coeffs[2]*px + 3*coeffs[3]*px*px;
          double epsi = -atan(dpy);

          Eigen::VectorXd state(6);
          state << px, py, psi, v, cte, epsi;

          auto vars = mpc.Solve(state, coeffs);
          //for(size_t i=0; i<vars.size(); i++) {
          //  std::cout << vars[i] << " ";
          //}
          //std::cout << std::endl;

          json msgJson;
          // Return steering angle (Unity convention) after normalizing
          // with deg2rad(25) to ensure steering value is in range [-1, 1].
          msgJson["steering_angle"] = -vars[delta_start]/0.436332;
          msgJson["throttle"] = vars[a_start];

          //Display the MPC predicted trajectory 
          vector<double> mpc_x_vals(&vars[x_start],&vars[x_start+N-1]);
          vector<double> mpc_y_vals(&vars[y_start],&vars[y_start+N-1]);

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals(&ptsx_car[0],&ptsx_car[ptsx_car.size()-1]);
          vector<double> next_y_vals(&ptsy_car[0],&ptsy_car[ptsx_car.size()-1]);

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          printf("[steering: %10.6f, throttle: %10.6f\n",
                 float(msgJson["steering_angle"]),float(msgJson["throttle"]));
          
          // Latency
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(100));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
