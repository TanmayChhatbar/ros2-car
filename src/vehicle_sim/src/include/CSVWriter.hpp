#include <iostream>
#include <fstream>
#include <vector>
#include <string>

void writeCSVLine(std::ofstream &file, const std::vector<std::string> &array);
void writeCSVLine(std::ofstream &file, const std::vector<double> &array);
std::ofstream openCSVFile(std::string filename);
void closeCSVFile(std::ofstream &file);
