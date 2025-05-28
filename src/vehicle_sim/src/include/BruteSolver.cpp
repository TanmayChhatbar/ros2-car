#include <BruteSolver.hpp>

BruteSolver::BruteSolver()
{
    BruteSolver(nullptr);
}

BruteSolver::BruteSolver(std::function<double(std::vector<double>)> f_)
{
    f = f_;
}

void BruteSolver::calcTrialPoints()
{

}

bool BruteSolver::solve()
{
    if (f == nullptr)
    {
        std::cerr << "Error: Objective function not set." << std::endl;
        return false;
    }

    // optimize
    bool converged = false;
    
    return converged;
}

std::vector<double> BruteSolver::getSolution(void)
{
    return opt_trial_point;
}

double BruteSolver::getScore()
{
    return score;
}

void BruteSolver::setNTrials(uint n_trials_)
{
    n_trials = n_trials_;
}

void BruteSolver::setNRefinements(uint n_refinements_)
{
    n_refinements = n_refinements_;
}

void BruteSolver::setCostThreshold(double cost_threshold_)
{
    cost_threshold = cost_threshold_;
}

void BruteSolver::printOutput(bool print_output_)
{
    print_output = print_output_;
}

double BruteSolver::getCostThreshold() const
{
    return cost_threshold;
}
