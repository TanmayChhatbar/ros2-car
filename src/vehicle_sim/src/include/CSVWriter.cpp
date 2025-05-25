#include "CSVWriter.hpp"

void CSVWriter::write(const std::vector<std::string> &array)
{
    for (size_t i = 0; i < array.size(); i++)
    {
        file << array[i];
        if (i != array.size() - 1)
            file << ",";
    }
    file << std::endl;
}

void CSVWriter::write(const std::vector<double> &array)
{
    for (size_t i = 0; i < array.size(); i++)
    {
        file << array[i];
        if (i != array.size() - 1)
            file << ",";
    }
    file << std::endl;
}

CSVWriter::CSVWriter(std::string filename)
{
    file = std::ofstream(filename);
    if (!file.is_open())
    {
        throw std::runtime_error("Failed to open file: " + filename);
    }
}

void CSVWriter::close()
{
    if (file.is_open())
    {
        file.close();
    }
}
