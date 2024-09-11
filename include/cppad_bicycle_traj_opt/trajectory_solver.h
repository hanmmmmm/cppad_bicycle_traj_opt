#ifndef TRAJECTORY_SOLVER_H
#define TRAJECTORY_SOLVER_H

#include <iostream>
#include <cppad/ipopt/solve.hpp>
#include <iomanip>
#include <chrono>




namespace cppad_bicycle_traj_opt
{
using CppAD::AD;


typedef CPPAD_TESTVECTOR(double) Dvector;

class FG_eval 
{
public:

    size_t N_;  // number of steps
    double dt_;  // time step seconds
    double Lf_;  // axle distance, meters

    std::vector<std::array<double,2>> obstacles_;


    FG_eval(size_t N, double dt, double Lf, std::vector<std::array<double,2>> obstacles) 
    : N_(N), dt_(dt), Lf_(Lf), obstacles_(obstacles) 
    {}


    /**
     * @brief Cost function using a modified sigmoid function.
     * 
     * @param x The metric distance between a pose and an obstacle.
     * @return AD<double> The cost value.
     */
    AD<double> cost_sigmoid(AD<double> x)
    {
        auto temp = 0.5 * x / CppAD::sqrt(1.0 + 0.1*x*x) + 0.1001;
        temp = 1.0 / temp - 0.62;
        return temp ;
    }

    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
    
    void operator()(ADvector& fg, const ADvector& vars)
    {
        
        size_t N = N_;
        double dt = dt_;
        double Lf = Lf_;

        // variables
        std::vector<AD<double>> a(N);
        std::vector<AD<double>> delta(N);
        for (size_t i = 0; i < N; i++) 
        {
            size_t a_idx = 4*(N+1) + i;
            a[i] = vars[a_idx];
            delta[i] = vars[a_idx + N];
        }

        AD<double> final_x = vars[N];
        AD<double> final_y = vars[N*2 + 1];
        AD<double> final_theta = vars[N*3 + 2];
        AD<double> final_v = vars[N*4 + 3];
        size_t goal_index = N*4 + 4 + N*2;
        AD<double> goal_x = vars[goal_index];
        AD<double> goal_y = vars[goal_index + 1];
        AD<double> goal_theta = vars[goal_index + 2];
        AD<double> goal_v = vars[goal_index + 3];
        auto final_pose_err = CppAD::pow(final_x - goal_x, 2) + CppAD::pow(final_y - goal_y, 2);
        final_pose_err += 1.0 * CppAD::pow(CppAD::sin(final_theta) - CppAD::sin(goal_theta), 2);
        final_pose_err += 1.0 * CppAD::pow(CppAD::cos(final_theta) - CppAD::cos(goal_theta), 2);
        final_pose_err += CppAD::pow(final_v - goal_v, 2) * 10.0;

        double a_target = 0.0;
        double delta_target = 0.0;

        auto a_err = CppAD::pow(a[0] - a_target, 2);
        for (size_t i = 1; i < N; i++) 
            a_err += CppAD::pow(a[i] - a_target, 2);
        
        auto delta_err = CppAD::pow(delta[0] - delta_target, 2);
        for (size_t i = 1; i < N; i++) 
            delta_err += CppAD::pow(delta[i] - delta_target, 2);
        
        // for each state, calc the cost of obstacle avoidance
        AD<double> obstacle_cost = 0.0;
        for (size_t i=0; i<N; i++)
        {
            for (auto& obs : obstacles_)
            {
                auto obs_x = obs[0];
                auto obs_y = obs[1];
                auto x = vars[1+i];
                auto y = vars[1+i + N+1];
                auto dist = CppAD::sqrt(CppAD::pow(x - obs_x, 2) + CppAD::pow(y - obs_y, 2))+0.0001;
                obstacle_cost += cost_sigmoid(dist);
            }
        }

        // objective function
        fg[0] = 3000.0 * final_pose_err;
        fg[0] += 30.0  * a_err;
        fg[0] += 100.0 * delta_err;
        fg[0] += 140.0 * obstacle_cost;
        
        // motion dynamics constraints for the transition between states
        size_t space = N+1;
        for (size_t i = 0; i<N; i++){
            size_t x_idx = i;
            size_t y_idx = i + space;
            size_t theta_idx = y_idx + space;
            size_t v_idx = theta_idx + space;
            AD<double> x0 = vars[x_idx];
            AD<double> y0 = vars[y_idx];
            AD<double> theta0 = vars[theta_idx];
            AD<double> v0 = vars[v_idx];
            AD<double> x1 = vars[x_idx + 1];
            AD<double> y1 = vars[y_idx + 1];
            AD<double> theta1 = vars[theta_idx + 1];
            AD<double> v1 = vars[v_idx + 1];
            fg[1+i] = x1 - (x0 + v0 * CppAD::cos(theta0) * dt);   // constraint for x
            fg[1+i+N] = y1 - (y0 + v0 * CppAD::sin(theta0) * dt); // constraint for y
            fg[1+i+2*N] = theta1 - (theta0 + v0 * CppAD::tan(delta[i]) * dt / Lf);  // constraint for theta
            fg[1+i+3*N] = v1 - (v0 + a[i] * dt);   // constraint for v
        }

        return;
    }

};


class ClassTrajectorySolver
{
public:
    ClassTrajectorySolver(size_t N, double dt, double Lf, 
                            std::vector<std::array<double,2>> obstacles,
                            std::array<double,3> goal_pose);

    std::vector<std::array<double,2>> getSolutionXY();
    std::vector<std::array<double,3>> getSolutionXYTheta();
    std::vector<double> getSolutionV();

private:
    size_t n_step;  // number of steps in the trajectory
    double dt_;     // time step seconds
    double Lf_;     // axle distance of the vehicle, meters
    std::vector<std::array<double,2>> obstacles_;  // vector of x, y coordinates of obstacles
    std::array<double,3> goal_pose_;  // x, y, theta_rad

private:
    std::vector<double> solution_x_, solution_y_, solution_theta_, solution_v_;
    std::vector<double> solution_a_, solution_delta_;

};


}  // namespace cppad_bicycle_traj_opt

#endif // TRAJECTORY_SOLVER_H