#include "CSVReader.hpp"

bool CSVReader::read(std::vector<std::vector<std::string>> &array)
{
    bool success = false;
    if (file.is_open())
    {
        try
        {
            array.clear();
            std::string line;
            while (std::getline(file, line))
            {
                std::vector<std::string> row;
                std::string cell;
                std::stringstream lineStream(line);
                while (std::getline(lineStream, cell, ','))
                {
                    row.push_back(cell);
                }
                array.push_back(row);
            }
        }
        catch (const std::exception &e)
        {
            std::cerr << "Error reading CSV file: " << e.what() << std::endl;
            success = false;
        }
        success = true;
    }
    return success;
}

CSVReader::CSVReader(std::string filename)
{
    file = std::ifstream(filename);
    if (!file.is_open())
    {
        throw std::runtime_error("Failed to open file: " + filename);
    }
}

void CSVReader::close()
{
    if (file.is_open())
    {
        file.close();
    }
}
