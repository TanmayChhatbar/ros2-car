#include <iostream>
#include <fstream>
#include <vector>
#include <string>

class CSVWriter
{
public:
    CSVWriter(std::string filename);
    std::ofstream open(std::string filename);
    void write(const std::vector<std::string> &array);
    void write(const std::vector<std::vector<std::string>> &array);
    void write(const std::vector<std::vector<double>> &array);
    void write(const std::vector<double> &array);
    void close();

private:
    std::string filename;
    std::ofstream file;
};
