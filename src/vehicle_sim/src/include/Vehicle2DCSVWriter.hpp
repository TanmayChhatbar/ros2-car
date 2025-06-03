#pragma once

#include "Vehicle2DData.hpp"
#include "Vehicle2DConfig.hpp"
#include "VehicleInput.hpp"
#include <iostream>
#include <fstream>

// cSV header and data formatting
void writeCSVHeader(std::ofstream &file);

void writeCSVData(std::ofstream &file, double time, const Vehicle2DData &data,
                  const VehicleInput &input);

void exportVehicle2DConfig(Vehicle2DConfig &Vehicle2DConfig, std::string filename);
