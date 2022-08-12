#include <fstream>
#include <iostream>
#include <Base/SensorDataTypes.h>
#include <Utils/TimeConverter.h>

#include <deque>

using namespace SINS;


ImuData::Ptr ParseNovatelImu(const std::string& line_str) {
    // Split line.
    std::vector<std::string> line_data_vec;
    std::stringstream ss(line_str);
    std::string value_str;
    while (std::getline(ss, value_str, ',')) { line_data_vec.push_back(value_str); }

    for (const auto v : line_data_vec) {
        std::cout << v << std::endl;
    }
    
    // Check sensor name.
    auto sensor_name = line_data_vec[0];
    if (sensor_name != "%RAWIMUSXA") { return nullptr; }

    ImuData::Ptr imu_ptr = std::make_shared<ImuData>();

    // Get time.
    constexpr double kWeekSecond = 7 * 24 * 60 * 60;
    imu_ptr->time = TimeUtil::Gps2Unix(std::stod(line_data_vec[1]) * kWeekSecond + std::stod(line_data_vec[2]));
    
    // Get acc.
    constexpr double kGyroScale = 1.0e-9;
    constexpr double kAccScale = 2.0e-8;
    constexpr double kFreq = 200.0;
    imu_ptr->acc << std::stod(line_data_vec[7]), std::stod(line_data_vec[8]), std::stod(line_data_vec[9]);
    imu_ptr->acc = imu_ptr->acc.eval() * kAccScale;
    imu_ptr->gyro << std::stod(line_data_vec[10]), std::stod(line_data_vec[11]), std::stod(line_data_vec[12]);
    imu_ptr->gyro = imu_ptr->gyro * kGyroScale * kFreq;

    std::cout << "Imu: " << std::fixed << imu_ptr->time << ", Acc" << imu_ptr->acc.transpose() 
              << ", " << imu_ptr->gyro.transpose() << std::endl;
    std::abort();
}

// argv 1: novatel asc file.
int main(int argc, char **argv) {
    const std::string novatel_asc_path = argv[1];
    std::fstream file(novatel_asc_path, std::ios::in);

    std::string line_str;
    std::vector<std::string> line_data_vec;
    while (std::getline(file, line_str)) {
        auto imu_ptr = ParseNovatelImu(line_str);
    }

    return EXIT_SUCCESS;
}