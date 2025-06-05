#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>

class CSVReader
{
public:
    CSVReader(std::string filename);
    std::ofstream open(std::string filename);
    bool read(std::vector<std::vector<std::string>> &array);
    void close();

private:
    std::string filename;
    std::ifstream file;
};
