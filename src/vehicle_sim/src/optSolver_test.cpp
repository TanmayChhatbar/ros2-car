#include <OptSolver.hpp>

double f(std::vector<double> x)
{
    return x[0] * x[0] * 1.0
        + (x[1] - 10.0) * (x[1] - 10.0)
        + (100.0 * x[2] + 100.0) * (100.0 * x[2] + 100.0)
        + 0.0;
}

int main()
{
    OptSolver solver(f);
    std::vector<double> x0 = {1.0, 2.0, 5.0};

    solver.setGuess(x0);
    if (!solver.solve())
    {
        std::cerr << "Solver did not converge." << std::endl;
        return 1;
    }

    std::vector<double> x_opt = solver.getSolution();
    std::cout << "Optimal solution: ";
    for (size_t i = 0; i < x_opt.size()-1; ++i)
    {
        std::cout << x_opt[i] << ",";
    }
    std::cout << x_opt[x_opt.size()-1] << std::endl;
    std::cout << "Objective function value: "
              << solver.getScore() << std::endl;
}
