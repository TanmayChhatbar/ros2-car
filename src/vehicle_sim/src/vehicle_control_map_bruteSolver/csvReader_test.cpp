#include "CSVReader.hpp"
// compile with 
//      g++ -I./include ./include/*.cpp csvReader_test.cpp -o build/csvreadertest -lm
// run with
//      ./build/csvreadertest

std::vector<std::vector<double>> getPointsToRefine(std::string filename, double score_threshold)
{
    CSVReader reader(filename);
    std::vector<std::vector<double>> trial_points;

    std::vector<std::vector<std::string>> data;
    if (reader.read(data))
    {
        for (uint i = 0; i < data.size(); ++i)
        {
            try
            {
                double score = std::stod(data[i][2]);
                if (score > score_threshold)
                {
                    std::vector trial_point = {std::stod(data[i][0]), std::stod(data[i][1])};
                    trial_points.push_back(trial_point);
                }
            }
            catch (const std::invalid_argument &)
            {
                // do nothing
            }
        }
        reader.close();
    }
    else
    {
        std::cerr << "Failed to read CSV file." << std::endl;
    }
    return trial_points;
}

int main()
{
    // get trial points from CSV file`
    std::string filename = "./build/control_map2.csv";
    double score_threshold = 1.0;
    std::vector<std::vector<double>> trial_points = getPointsToRefine(filename, score_threshold);

    // print the trial points
    std::cout << "Trial points with score > 1.0:\n";
    for (const auto &point : trial_points)
    {
        std::cout << point[0] << ", " << point[1] << "\n";
    }
}
