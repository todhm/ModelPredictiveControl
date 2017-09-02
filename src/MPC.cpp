#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

//Set the timestep length and duration
size_t N = 10;
double dt = 0.01;

size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
size_t delta_start = epsi_start + N;
size_t a_start = delta_start + N - 1;
double ref_v = 60;
const double Lf = 2.67;

// Set weights parameters for the cost function
/*
const double weight_cte = 2.5;
const double weight_epsi = 2.7;
const double  weight_delta =  4.3;
const double  weight_acc = 2.3;
const double  weight_ddelta = 4;
const double  weight_dacc = 2;
**/
const double weight_cte =4.3;
const double weight_epsi = 12.2;
const double  weight_delta =  28;
const double  weight_acc = 1;
const double  weight_ddelta =23;
const double  weight_dacc = 1;

class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
      //Set Cost.
      fg[0] = 0;
      //Set CTE,orientation error and deviation from reference velocity as cost.
      for(int t = 0 ; t<N ; t++){
          fg[0] += weight_cte * CppAD::pow(vars[cte_start +t],2);
          fg[0] += weight_epsi* CppAD::pow(vars[epsi_start + t] ,2);
          fg[0] += CppAD::pow(vars[v_start + t] - ref_v ,2);
      }
      //Minimize the change of actuator varaibles.
      for(int t=0 ; t<N -1 ; t++){
          fg[0] += weight_delta * CppAD::pow(vars[delta_start +t],2);
          fg[0] += weight_acc*CppAD::pow(vars[a_start+t],2);
      
      }
      //Minimize the change of differential of actuator varaible
      for(int t=0; t< N-2; t++){
          fg[0] +=  weight_ddelta * CppAD::pow(vars[delta_start + t+ 1] - vars[delta_start+t],2);
          fg[0] +=  weight_dacc *CppAD::pow(vars[a_start + t +1] - vars[a_start+t],2);
      }
      
      
      //Implementation of MPC
      // Set start varaible.
      fg[1 + x_start] = vars[x_start];
      fg[1 + y_start] = vars[y_start];
      fg[1 + psi_start] = vars[psi_start];
      fg[1 + v_start] = vars[v_start];
      fg[1 + cte_start] = vars[cte_start];
      fg[1 + epsi_start] = vars[epsi_start];
      for(int t = 1 ; t < N -1 ; t++){
         //Set state at time t
         AD<double> x0 = vars[x_start+ t -1];
         AD<double> psi0 =vars[psi_start + t - 1];
         AD<double> v0 = vars[v_start + t -1];
         AD<double> y0 = vars[y_start + t -1];
         AD<double> cte0 = vars[cte_start + t -1];
         AD<double> epsi0 =vars[epsi_start + t -1];
        
         //Set actuator variables.
         AD<double> delta0 = vars[delta_start +t];
         AD<double> a0 = vars[a_start + t ];
        
         //Set state at time t + 1
         AD<double> x1 = vars[x_start +t];
         AD<double> y1 = vars[y_start +t];
         AD<double> psi1 = vars[psi_start + t];
         AD<double> v1 = vars[v_start + t];
         AD<double> cte1 = vars[cte_start + t];
         AD<double> epsi1 = vars[epsi_start+t];
        
         //fitted x and deviation of fitted x.
         AD<double> f0 = coeffs[0] + coeffs[1]*x0 + coeffs[2]*x0*x0 + + coeffs[3]*x0*x0*x0 ;
         AD<double> devfx = coeffs[1] + coeffs[2] * x0 *2 + coeffs[3] * x0*x0 * 3;
        
         fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
         fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
         fg[1 + psi_start + t] = psi1 - (psi0 + (v0/Lf) * delta0  * dt);
         fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
         fg[1 + cte_start + t] = cte1 - (f0 - y0 + v0 * CppAD::sin(epsi0) * dt);
         fg[1 + epsi_start + t] = epsi1 - (psi0 -CppAD::atan(devfx) + (v0/Lf) * delta0* dt);

      }
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  bool ok = true;
  size_t i;
  typedef CPPAD_TESTVECTOR(double) Dvector;


  // Set number of variables.
  size_t n_vars = N * 6 + (N-1) *2;
  //Set the number of constraints
  size_t n_constraints = N * 6;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
    
  for (int i = 0; i < delta_start; i++) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
    }
    

  for (int i = delta_start; i < a_start; i++){
    vars_lowerbound[i] = -0.436332 ;
    vars_upperbound[i] = 0.436332 ;
    }
    
  for(int i = a_start ; i < n_vars ; i++){
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
    }

  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (int i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }
  
  double x= state[0];
  double y = state[1];
  double psi = state[2];
  double v = state[3];
  double cte = state[4];
  double epsi = state[5];
  
  constraints_lowerbound[x_start] =x;
  constraints_lowerbound[y_start] = y;
  constraints_lowerbound[psi_start] =psi;
  constraints_lowerbound[v_start] = v;
  constraints_lowerbound[cte_start] = cte;
  constraints_lowerbound[epsi_start] = epsi;
  
  constraints_upperbound[x_start] =x;
  constraints_upperbound[y_start] = y;
  constraints_upperbound[psi_start] =psi;
  constraints_upperbound[v_start] = v;
  constraints_upperbound[cte_start] = cte;
  constraints_upperbound[epsi_start] = epsi;


  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);

  //
  // NOTE: You don't have to worry about these options
  //
  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          0.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;


  return {solution.x[x_start + 1],   solution.x[y_start + 1],
          solution.x[psi_start + 1], solution.x[v_start + 1],
          solution.x[cte_start + 1], solution.x[epsi_start + 1],
          solution.x[delta_start],   solution.x[a_start]};
}
