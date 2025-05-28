#pragma once

#include <vector>
#include <functional>
#include <iostream>
#include <cmath>
#include <omp.h>

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
    void setNTrials(std::vector<uint> n_trials_);
    void setNTrials(uint n_trials_);
    void setNRefinements(uint n_refinements_);
    void setReductionRatio(double reduction_ratio_);
    void printOutput(bool print_output_);

    double getCostThreshold() const;
    uint getMaxIters() const;
    std::vector<std::vector<double>> trial_points;

private:
    // functions
    void calcTrialPoints();

    // dynamic vars
    std::vector<double> opt_trial_point; // optimal point
    double score = __FLT_MAX__;

    // parameters for optimization
    std::vector<uint> n_trials;                   // number of trials per refinement per state
    uint n_refinements = 30;                      // number of refinement iterations
    std::function<double(std::vector<double>)> f; // optimization function
    std::vector<double> lb;                       // lower bounds
    std::vector<double> ub;                       // upper bounds
    double cost_threshold = 1e-5;                 // convergence threshold
    bool print_output = false;                    // convergence flag
    double reduction_ratio = 0.5;                 // reduction ratio for bounds

    // dynamic
    std::vector<double> lb_current; // current lower bounds
    std::vector<double> ub_current; // current upper bounds
};
