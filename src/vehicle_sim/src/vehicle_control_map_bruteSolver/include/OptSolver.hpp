#pragma once

#include <vector>
#include <functional>
#include <iostream>
#include <cmath>

class OptSolver
{
public:
    OptSolver();
    OptSolver(std::function<double(std::vector<double>)> f_);
    bool solve();
    double getScore();
    std::vector<double> getSolution();
    void setFunction(std::function<double(std::vector<double>)> f_);
    void setGuess(std::vector<double> x0_);
    void setThreshold(double threshold_);
    void setCostThreshold(double threshold_);
    void setStepSize(double step_size_);
    void setMaxIters(uint max_iters_);
    void setMaxGradientNorm(uint max_gradient_);
    void printOutput(bool print_output_);

    std::vector<double> getGuess() const;
    double getThreshold() const;
    double getCostThreshold() const;
    double getStepSize() const;
    uint getMaxIters() const;
    double getMaxGradientNorm() const;

private:
    std::vector<double> perturbX(std::vector<double> x_trial, std::vector<double> k, double factor);
    void calcGradients();
    std::vector<double> Jacobian(const std::vector<double> x_trial);
    bool calcNextBestTrial();
    double score = 1.0e10;
    double threshold = 1e-5;                      // convergence threshold
    double cost_threshold = 1e-5;                      // convergence threshold
    double step_size = 1.0;                       // step size for gradient descent
    double max_gradient_norm = 100.0;             // maximum gradient magnitude
    double h = 1e-3;                              // step size for numerical gradient calculation
    bool converged = false;                       // convergence flag
    bool print_output = false;                    // convergence flag
    uint max_iters = 1000;                        // maximum number of iterations
    std::function<double(std::vector<double>)> f; // optimization function
    std::vector<double> x_trial;                  // trial point
    std::vector<double> lb;                       // lower bounds
    std::vector<double> ub;                       // upper bounds
    std::vector<double> gradients;                // gradients
    std::vector<double> x_opt;                    // optimal point
};
