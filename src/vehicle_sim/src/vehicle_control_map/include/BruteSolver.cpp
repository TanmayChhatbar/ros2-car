#include <BruteSolver.hpp>

BruteSolver::BruteSolver()
{
    BruteSolver(nullptr);
}

BruteSolver::BruteSolver(const std::function<double(std::vector<double>)> f_)
{
    f = f_;
}

void BruteSolver::calcTrialPoints()
{
    // calculate vector of trial inputs based on bounds and number of trials
    trial_points.clear();
    uint n = 1;
    for (const auto &n_trial : n_trials)
    {
        n *= n_trial; // total number of trial points
    }
    trial_points.reserve(n);

    // get step size
    std::vector<double> step_size(lb_current.size());
    for (size_t i = 0; i < lb_current.size(); ++i)
    {
        step_size[i] = (ub_current[i] - lb_current[i]) / (double)(n_trials[i] - 1);
    }

    // generate all combinations of trial points
    std::vector<double> trial_point_cur = lb_current;
    trial_points.push_back(trial_point_cur);
    while (true)
    {
        trial_point_cur[0] += step_size[0];
        for (size_t i = 0; i < (lb_current.size() - 1); ++i)
        {
            if (trial_point_cur[i] > ub_current[i])
            {
                trial_point_cur[i] = lb_current[i];
                trial_point_cur[i + 1] += step_size[i + 1];
            }
        }
        if (trial_point_cur.back() > ub_current.back())
        {
            break; // all combinations generated
        }
        trial_points.push_back(trial_point_cur);
    }
}

bool BruteSolver::solve()
{
    if (f == nullptr)
    {
        std::cerr << "Error: Objective function not set." << "\n";
        return false;
    }

    // optimize
    bool converged = false;
    ub_current = ub;
    lb_current = lb;
    uint converged_iter = 0;

    for (uint i = 0; i < n_refinements; ++i)
    {
        if (print_output)
        {
            std::cout << "Iteration:\t" << i + 1 << "\n";
        }

        double last_score = score;
        calcTrialPoints();

        // evaluate all trial points
        double thread_best_score = score;
        std::vector<double> thread_best_trial = opt_trial_point;
#pragma omp parallel for
        for (size_t idx = 0; idx < trial_points.size(); ++idx)
        {
            double trial_score = f(trial_points[idx]);
            if (trial_score < thread_best_score)
            {
                thread_best_score = trial_score;
                thread_best_trial = trial_points[idx];
            }
        }
#pragma omp critical
        {
            if (thread_best_score < score)
            {
                score = thread_best_score;
                opt_trial_point = thread_best_trial;
            }
        }

        // print debug output
        if (print_output)
        {
            std::cout << "\tlb: [";
            for (size_t j = 0; j < lb_current.size() - 1; ++j)
            {
                std::cout << lb_current[j] << ", ";
            }
            std::cout << lb_current.back() << "]";
            std::cout << "\n\tub: [";
            for (size_t j = 0; j < ub_current.size() - 1; ++j)
            {
                std::cout << ub_current[j] << ", ";
            }
            std::cout << ub_current.back() << "]" << "\n";
            std::cout << "\tOpt:\t";
            for (size_t j = 0; j < opt_trial_point.size() - 1; ++j)
            {
                std::cout << opt_trial_point[j] << ", ";
            }
            std::cout << opt_trial_point.back() << "\n";
            std::cout << "\tScore:\t" << score << "\n";
        }

        if (print_output)
        {
            std::cout << "\tΔScore:\t" << std::abs(score - last_score) << "\n";
        }
        // check convergence
        if (std::abs(score - last_score) < cost_threshold)
        {
            if (converged_iter >= 5)
            {
                converged = true;
                if (print_output)
                {
                    std::cout << "Converged with score:\t" << score << "\n";
                }
                break;
            }

            converged_iter++;
        }
        else
        {
            converged_iter = 0;
        }

        // refine bounds
        for (size_t j = 0; j < lb_current.size(); ++j)
        {
            lb_current[j] = opt_trial_point[j] - (opt_trial_point[j] - lb_current[j]) * reduction_ratio;
            ub_current[j] = opt_trial_point[j] + (ub_current[j] - opt_trial_point[j]) * reduction_ratio;
        }
    }
    return converged;
}

std::vector<double> BruteSolver::getSolution(void) const
{
    return opt_trial_point;
}

double BruteSolver::getScore() const
{
    return score;
}

void BruteSolver::setReductionRatio(const double reduction_ratio_)
{
    reduction_ratio = reduction_ratio_;
}

void BruteSolver::setNTrials(const std::vector<uint> n_trials_)
{
    n_trials = n_trials_;
}

void BruteSolver::setNTrials(const uint n_trials_)
{
    n_trials.clear();
    n_trials.resize(lb.size(), n_trials_);
}

void BruteSolver::setNRefinements(const uint n_refinements_)
{
    n_refinements = n_refinements_;
}

void BruteSolver::setBounds(const std::vector<double> lb_, const std::vector<double> ub_)
{
    lb = lb_;
    ub = ub_;
    lb_current = lb_;
    ub_current = ub_;
    n_trials = std::vector<uint>(lb.size(), 30);
}

void BruteSolver::setCostThreshold(const double cost_threshold_)
{
    cost_threshold = cost_threshold_;
}

void BruteSolver::printOutput(const bool print_output_)
{
    print_output = print_output_;
}

double BruteSolver::getCostThreshold() const
{
    return cost_threshold;
}
