#include "CSVWriter.hpp"

void writeCSVLine(std::ofstream &file, const std::vector<std::string> &array)
{
    for (size_t i = 0; i < array.size(); i++)
    {
        file << array[i];
        if (i != array.size() - 1)
            file << ",";
    }
    file << std::endl;
}

void writeCSVLine(std::ofstream &file, const std::vector<double> &array)
{
    for (size_t i = 0; i < array.size(); i++)
    {
        file << array[i];
        if (i != array.size() - 1)
            file << ",";
    }
    file << std::endl;
}

std::ofstream openCSVFile(std::string filename)
{
    std::ofstream file(filename);
    if (!file.is_open())
    {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return file;
    }
    throw std::runtime_error("Failed to open file: " + filename);
    return file;
}

void closeCSVFile(std::ofstream &file)
{
    if (file.is_open())
    {
        file.close();
    }
}
