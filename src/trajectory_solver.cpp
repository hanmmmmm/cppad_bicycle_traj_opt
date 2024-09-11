#include "cppad_bicycle_traj_opt/trajectory_solver.h"


namespace cppad_bicycle_traj_opt
{
    
ClassTrajectorySolver::ClassTrajectorySolver(size_t N, double dt, double Lf, 
                                            std::vector<std::array<double,2>> obstacles, 
                                            std::array<double,3> goal_pose)
: n_step(N), dt_(dt), Lf_(Lf), obstacles_(obstacles), goal_pose_(goal_pose)
{

    size_t state_dim = 4;  // x, y, theta, v
    size_t cntrl_dim = 2;  // a, delta
    size_t goal_dim = 4;   // x, y, theta, v

    size_t n_var = n_step * (state_dim + cntrl_dim) 
                    + state_dim 
                    + goal_dim;
    
    // Initial all of independent variables to zero.
    Dvector vars(n_var);
    for (size_t i = 0; i < n_var; i++) {
        vars[i] = 0.0;
    }

    // Contents of the independent variables:
    // x0, x1, x2, ..., xN, 
    // y0, y1, y2, ..., yN, 
    // theta0, theta1, theta2, ..., thetaN, 
    // v0, v1, v2, ..., vN
    // a1, a2, ..., aN,
    // delta1, delta2, ..., deltaN,
    // goal_x, goal_y, goal_theta, goal_v
    // the x0 y0 theta0 v0 are the initial state of the vehicle, so they are constants.
    // the goal_x, goal_y, goal_theta, goal_v are also constants.


    // // Define the problem variables
    double x_init = 0.0;
    double y_init = 0.0;
    double theta_init = 0.0;
    double v_init = 4.0;

    double x_goal = goal_pose_[0];
    double y_goal = goal_pose_[1];
    double theta_goal = goal_pose_[2];
    double v_goal = v_init;

    double constraint_v_upper = 10.0;
    double constraint_v_lower = -8.0;

    double constraint_a_upper = 10.0;
    double constraint_a_lower = -10.0;

    double constraint_delta_upper = 0.7;
    double constraint_delta_lower = -0.7;


    Dvector vars_lower_bounds(n_var);
    Dvector vars_upper_bounds(n_var);

    // put init_state into vars and update the bounds
    vars[0] = x_init;
    vars_lower_bounds[0] = x_init;
    vars_upper_bounds[0] = x_init;

    vars[n_step+1] = y_init;
    vars_lower_bounds[n_step+1] = y_init;
    vars_upper_bounds[n_step+1] = y_init;

    vars[2*n_step+2] = theta_init;
    vars_lower_bounds[2*n_step+2] = theta_init;
    vars_upper_bounds[2*n_step+2] = theta_init;

    vars[3*n_step+3] = v_init;
    vars_lower_bounds[3*n_step+3] = v_init;
    vars_upper_bounds[3*n_step+3] = v_init;

    // states constraints. They can be any value, so constraints should be -inf to inf
    for (size_t i=1; i<n_step+1; i++) {
        // x
        vars[i] = 0.0;
        vars_lower_bounds[i] = -1e19;
        vars_upper_bounds[i] = 1e19;
        // y
        vars[i+n_step+1] = 0.0;
        vars_lower_bounds[i+n_step+1] = -1e19;
        vars_upper_bounds[i+n_step+1] = 1e19;
        // theta
        vars[i+2*n_step+2] = 0.0;
        vars_lower_bounds[i+2*n_step+2] = -1e19;
        vars_upper_bounds[i+2*n_step+2] = 1e19;
        // v
        vars[i+3*n_step+3] = 0.0;
        vars_lower_bounds[i+3*n_step+3] = constraint_v_lower;
        vars_upper_bounds[i+3*n_step+3] = constraint_v_upper;
    }

    // control constraints
    auto ctrl_a_idx = (n_step+1)*state_dim;
    for (size_t i=ctrl_a_idx; i<ctrl_a_idx + n_step; i++) {
        vars_lower_bounds[i] = constraint_a_lower;
        vars_upper_bounds[i] = constraint_a_upper;
    }
    auto ctrl_delta_idx = ctrl_a_idx + n_step;
    for (size_t i=ctrl_delta_idx; i<ctrl_delta_idx + n_step; i++) {
        vars_lower_bounds[i] = constraint_delta_lower;
        vars_upper_bounds[i] = constraint_delta_upper;
    }

    // goal state variables are constants
    auto goal_idx = ctrl_delta_idx + n_step;
    vars[goal_idx] = x_goal;
    vars_lower_bounds[goal_idx] = x_goal;
    vars_upper_bounds[goal_idx] = x_goal;

    vars[goal_idx + 1] = y_goal;
    vars_lower_bounds[goal_idx + 1] = y_goal;
    vars_upper_bounds[goal_idx + 1] = y_goal;

    vars[goal_idx + 2] = theta_goal;
    vars_lower_bounds[goal_idx + 2] = theta_goal;
    vars_upper_bounds[goal_idx + 2] = theta_goal;

    vars[goal_idx + 3] = v_goal;
    vars_lower_bounds[goal_idx + 3] = v_goal;
    vars_upper_bounds[goal_idx + 3] = v_goal;


    // put initial guess for the control variables
    for (size_t i = ctrl_a_idx; i < ctrl_a_idx + n_step; i++) {
        vars[i] = 1.23;  // random value for the initial guess
    }
    for (size_t i = ctrl_delta_idx; i < ctrl_delta_idx + n_step; i++) {
        vars[i] = -0.01;
    }

    // guess the states using the guessed control variables
    for (size_t i = 1; i < n_step+1; i++) {
        vars[i] = vars[i-1] + vars[ctrl_a_idx + i-1] * std::cos(vars[2*n_step+2 + i-1]) * dt;
        vars[i+n_step+1] = vars[i+n_step] + vars[ctrl_a_idx + i-1] * std::sin(vars[2*n_step+2 + i-1]) * dt;
        vars[i+2*n_step+2] = vars[i+2*n_step+1] + vars[ctrl_a_idx + i-1] * std::tan(vars[ctrl_delta_idx + i-1]) * dt / Lf;
        vars[i+3*n_step+3] = vars[i+3*n_step+2] + vars[ctrl_a_idx + i-1] * dt;
    }

    auto n_constraints = n_step * state_dim; // each step has 1 dynamics constraint, each dynamics constraint has state_dim equations
    Dvector constraints_lower_bounds(n_constraints);
    Dvector constraints_upper_bounds(n_constraints);

    // vehicle dynamics constraints error should be zero
    for (size_t i = 0; i < n_constraints; i++) {
        constraints_lower_bounds[i] = -0.0;
        constraints_upper_bounds[i] = 0.0;
    }

    FG_eval fg_eval(n_step, dt, Lf, obstacles_);

    // options for IPOPT solver
    std::string options;
    // // Uncomment this if you'd like more print information
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

    auto before_solve = std::chrono::high_resolution_clock::now();

    CppAD::ipopt::solve_result<Dvector> solution;

    CppAD::ipopt::solve<Dvector, FG_eval>(options, vars, 
                                            vars_lower_bounds, vars_upper_bounds, 
                                            constraints_lower_bounds, constraints_upper_bounds, 
                                            fg_eval, solution);

    // done solving, so extract the solution values

    for (size_t i = 0; i < n_step+1; i++) 
    {
        solution_x_.push_back(solution.x[i]);
        solution_y_.push_back(solution.x[n_step + 1 + i]);
        solution_theta_.push_back(solution.x[2*(n_step+1) + i]);
        solution_v_.push_back(solution.x[3*(n_step+1) + i]);
    }

    for (size_t i = 0; i < n_step; i++) 
    {
        solution_a_.push_back(solution.x[(n_step+1)*state_dim + i]);
        solution_delta_.push_back(solution.x[(n_step+1)*state_dim + n_step + i]);
    }
}


std::vector<std::array<double,2>> ClassTrajectorySolver::getSolutionXY()
{
    std::vector<std::array<double,2>> solution_xy;
    for (size_t i = 0; i < n_step+1; i++) 
        solution_xy.push_back({solution_x_[i], solution_y_[i]});
    
    return solution_xy;
}

std::vector<std::array<double,3>> ClassTrajectorySolver::getSolutionXYTheta()
{
    std::vector<std::array<double,3>> solution_xytheta;
    for (size_t i = 0; i < n_step+1; i++) 
        solution_xytheta.push_back({solution_x_[i], solution_y_[i], solution_theta_[i]});
    
    return solution_xytheta;
}

std::vector<double> ClassTrajectorySolver::getSolutionV()
{
    return solution_v_;
}


} // namespace cppad_bicycle_traj_opt