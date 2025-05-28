#pragma once

#include <vector>
#include <functional>
#include <iostream>
#include <cmath>

class BruteSolver
{
public:
    BruteSolver();
    BruteSolver(std::function<double(std::vector<double>)> f_);
    bool solve();
    double getScore();
    std::vector<double> getSolution();
    void setBounds(std::vector<double> lb_, std::vector<double> ub_);
    void setCostThreshold(double threshold_);
    void setNTrials(uint n_trials_);
    void setNRefinements(uint n_refinements_);
    void printOutput(bool print_output_);
    void calcTrialPoints();

    double getCostThreshold() const;
    uint getMaxIters() const;

private:
    // functions
    void calcTrialPoints();

    // dynamic vars
    std::vector<std::vector<double>> trial_points; // trial points for each refinement
    std::vector<double> opt_trial_point;           // optimal point
    double score = 1.0e10;

    // parameters for optimization
    uint n_trials = 100;                          // number of trials per refinement per state
    uint n_refinements = 10;                      // number of refinement iterations
    std::function<double(std::vector<double>)> f; // optimization function
    std::vector<double> lb;                       // lower bounds
    std::vector<double> ub;                       // upper bounds
    double cost_threshold = 1e-5;                 // convergence threshold
    bool print_output = false;                    // convergence flag
};
