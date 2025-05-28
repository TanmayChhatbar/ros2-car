#include <BruteSolver.hpp>

double f(std::vector<double> x)
{
    return x[0] * x[0] * 1.0
        + (x[1] - 10.0) * (x[1] - 10.0)
        + (100.0 * x[2] + 100.0) * (100.0 * x[2] + 100.0)
        + 0.0;
}

int main()
{
    BruteSolver solver(f);
    solver.setBounds({-100.0, -100.0, -100.0}, {100.0, 100.0, 100.0});
    solver.setNTrials(30);
    solver.setNRefinements(50);
    solver.printOutput(true);

    if (!solver.solve())
    {
        std::cerr << "Solver did not converge." << "\n";
        return 1;
    }

    // get solution
    std::vector<double> x_opt = solver.getSolution();
    std::cout << "Optimal solution: ";
    for (size_t i = 0; i < x_opt.size()-1; ++i)
    {
        std::cout << x_opt[i] << ",";
    }
    std::cout << x_opt[x_opt.size()-1] << "\n";
    std::cout << "Objective function value: "
              << solver.getScore() << "\n";
}
