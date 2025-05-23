#include <OptSolver.hpp>

#define USE_NORMALIZED_GRADIENTS 0
#define USE_RK4 1

OptSolver::OptSolver()
{
    OptSolver(nullptr);
}

OptSolver::OptSolver(std::function<double(std::vector<double>)> f_)
{
    f = f_;
    score = 0.0;
    x_opt.clear();
    x_trial.clear();
    gradients.clear();
}

void OptSolver::setFunction(std::function<double(std::vector<double>)> f_)
{
    f = f_;
}

void OptSolver::calcGradients()
{
#if USE_RK4
    // using RK4
    // k1
    std::vector<double> x_tmp = x_trial;
    std::vector<double> k1 = Jacobian(x_tmp);
    gradients.resize(x_trial.size());

    // k2
    x_tmp = perturbX(x_trial, k1, 2.0);
    std::vector<double> k2 = Jacobian(x_tmp);

    // k3
    x_tmp = perturbX(x_trial, k2,  2.0);
    std::vector<double> k3 = Jacobian(x_tmp);

    // k4
    x_tmp = perturbX(x_trial, k3, 1.0);
    std::vector<double> k4 = Jacobian(x_tmp);

    // calculate gradients
    for (size_t i = 0; i < x_trial.size(); ++i)
    {
        gradients[i] = (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]) / 6.0;
    }
#else
    gradients = Jacobian(x_trial);
#endif
}

std::vector<double> OptSolver::perturbX(std::vector<double> x_trial, std::vector<double> k, double factor)
{
    std::vector<double> x_tmp = x_trial;
    for (size_t i = 0; i < x_trial.size(); ++i)
    {
        x_tmp[i] = x_trial[i] + h * k[i] / factor;
    }
    return x_tmp;
}

std::vector<double> OptSolver::Jacobian(const std::vector<double> x_trial)
{
    double f_curr = f(x_trial);
    std::vector<double> x_forward = x_trial;
    std::vector<double> J(x_trial.size());
    for (size_t i = 0; i < x_trial.size(); ++i)
    {
        const double orig = x_forward[i];
        x_forward[i] += h;
        J[i] = (f(x_forward) - f_curr) / h;
        x_forward[i] = orig; // restore original value
    }
    return J;
}

bool OptSolver::calcNextBestTrial()
{
#if USE_NORMALIZED_GRADIENTS
    double norm = 0.0;
    for (size_t i = 0; i < gradients.size(); ++i)
    {
        norm += gradients[i] * gradients[i];
    }
    norm = std::sqrt(norm);

    // if gradients very small, near a local optimum
    if (norm < threshold)
    {
        return true;
    }

    // limit norm to avoid too large step size
    if (norm < max_gradient_norm)
    {
        norm = max_gradient_norm;
    }

    // normalize gradients
    for (size_t i = 0; i < gradients.size(); ++i)
    {
        gradients[i] /= norm;
    }
#endif

    // calculate next best trial point
    for (size_t i = 0; i < x_trial.size(); ++i)
    {
        x_trial[i] -= gradients[i] * step_size;
    }
    return false;
}

bool OptSolver::solve()
{
    if (f == nullptr)
    {
        std::cerr << "Error: Objective function not set." << std::endl;
        return false;
    }

    // optimize
    bool converged = false;
    uint n_iter = 0;
    while (!converged && n_iter < max_iters)
    {
        // calculate score
        calcGradients();

        converged = calcNextBestTrial();
        double new_score = f(x_trial);
        if (new_score < score)
        {
            score = new_score;
            x_opt = x_trial;
            if (score < cost_threshold)
            {
                converged = true;
            }
        }

        n_iter++;
        if (print_output && ((n_iter) % 1 == 0 || n_iter == 1))
        {
            std::cout << n_iter << "\t";
            std::cout << score << "\t";
            for (size_t i = 0; i < x_trial.size() - 1; ++i)
            {
                std::cout << x_trial[i] << "\t";
            }
            std::cout << x_trial[x_trial.size() - 1] << std::endl;
        }
    }
    return converged;
}

std::vector<double> OptSolver::getSolution(void)
{
    return x_opt;
}

double OptSolver::getScore()
{
    return score;
}

void OptSolver::setGuess(std::vector<double> x0_)
{
    x_trial = x0_;
    x_opt = x0_;
    score = f(x_trial);
}

void OptSolver::setThreshold(double threshold_)
{
    threshold = threshold_;
}

void OptSolver::setCostThreshold(double cost_threshold_)
{
    cost_threshold = cost_threshold_;
}

void OptSolver::setStepSize(double step_size_)
{
    step_size = step_size_;
}

void OptSolver::setMaxIters(uint max_iters_)
{
    max_iters = max_iters_;
}

void OptSolver::setMaxGradientNorm(uint max_gradient_norm_)
{
    max_gradient_norm = max_gradient_norm_;
}

void OptSolver::printOutput(bool print_output_)
{
    print_output = print_output_;
}

std::vector<double> OptSolver::getGuess() const
{
    return x_trial;
}

double OptSolver::getThreshold() const
{
    return threshold;
}
double OptSolver::getCostThreshold() const
{
    return cost_threshold;
}

double OptSolver::getStepSize() const
{
    return step_size;
}

uint OptSolver::getMaxIters() const
{
    return max_iters;
}

double OptSolver::getMaxGradientNorm() const
{
    return max_gradient_norm;
}
