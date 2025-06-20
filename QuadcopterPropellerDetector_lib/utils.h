//
// Created by radio on 24.10.2024.
//

#ifndef PPUTILS_H
#define PPUTILS_H

#include <queue>
#include <functional>
#include <stdexcept>

#include <metavision/sdk/base/utils/timestamp.h>
#include <yaml-cpp/yaml.h>

#include "H5Cpp.h"


// Helper function to retrieve a key and report errors if missing or wrong type.
template<typename T>
T getValue(const std::string& key, const YAML::Node& node) {
    // Check if the key exists.
    if (!node[key]) {
        std::cerr << "Missing key '" << key << "' in YAML configuration." << std::endl;
        throw std::runtime_error("Missing key: " + key);
    }
    try {
        return node[key].as<T>();
    } catch (const YAML::BadConversion &e) {
        // Get the node location information.
        auto mark = node[key].Mark();
        std::cerr << "Error converting key '" << key << "' at line "
                  << mark.line << ", column " << mark.column
                  << ": " << e.what() << std::endl;
        throw;
    }
}

struct Config {
    explicit Config(const YAML::Node& yaml_config)
        : is_analysis(getValue<bool>("is_analysis", yaml_config)),
          is_runtime_analysis(getValue<bool>("is_runtime_analysis", yaml_config)),
          is_quiet(getValue<bool>("is_quiet", yaml_config)),
          simulate_real_time(getValue<bool>("simulate_real_time", yaml_config)),
          width(getValue<unsigned short>("width", yaml_config)),
          height(getValue<unsigned short>("height", yaml_config)),
          mode(getValue<std::string>("mode", yaml_config)),
          fps(getValue<double>("fps", yaml_config)),
          acc(getValue<std::uint32_t>("acc", yaml_config)),
          temporal_stride(getValue<int>("temporal_stride", yaml_config)),
          max_rate(getValue<double>("max_rate", yaml_config)),
          analysis_filepath(getValue<std::string>("analysis_filepath", yaml_config)),
          tensorboard_log_file(getValue<std::string>("tensorboard_log_file", yaml_config)),
          recording_filepath(getValue<std::string>("recording_filepath", yaml_config)),
          alpha_of_burst_std(getValue<double>("alpha_of_burst_std", yaml_config)),
          alpha_interarrival_time(getValue<double>("alpha_interarrival_time", yaml_config)),
          alpha_interarrival_time_window_size(getValue<int>("alpha_interarrival_time_window_size", yaml_config)),
          min_interarrival_time(getValue<double>("min_interarrival_time", yaml_config)),
          max_burst_std_per_cent(getValue<double>("max_burst_std_per_cent", yaml_config)),
          start_us(getValue<Metavision::timestamp>("start_us", yaml_config)),
          end_us(getValue<Metavision::timestamp>("end_us", yaml_config)),
          k_min(getValue<int>("k_min", yaml_config)),
          k_max(getValue<int>("k_max", yaml_config)),
          scale_down_factor(getValue<double>("scale_down_factor", yaml_config)),
          rotation_angle(getValue<double>("rotation_angle", yaml_config)),
          min_E_x_of_std(getValue<double>("min_E_x_of_std", yaml_config)),
          max_E_x_of_std(getValue<double>("max_E_x_of_std", yaml_config)),
          polarity(getValue<char>("polarity", yaml_config)),
          T_min(getValue<int>("T_min", yaml_config)),
          compute_pitch_roll(getValue<bool>("compute_pitch_roll", yaml_config)),
          pitch_roll_estimation_alpha(getValue<double>("pitch_roll_estimation_alpha", yaml_config)),
          pitch_gt(getValue<double>("pitch_gt", yaml_config)),
          pitch_scaler(getValue<double>("pitch_scaler", yaml_config))
    {}

    bool is_analysis, is_quiet, simulate_real_time;
    unsigned short width, height;
    int temporal_stride, k_min, k_max, T_min, alpha_interarrival_time_window_size;

    Metavision::timestamp start_us, end_us;

    double fps, alpha_of_burst_std, alpha_interarrival_time, min_interarrival_time, max_burst_std_per_cent, max_rate;

    std::string mode, analysis_filepath, tensorboard_log_file, recording_filepath;
    std::uint32_t acc;
    double scale_down_factor;
    double min_E_x_of_std;
    double max_E_x_of_std;
    char polarity;
    bool compute_pitch_roll;
    double rotation_angle;
    double pitch_roll_estimation_alpha, pitch_gt, pitch_scaler;
    bool is_runtime_analysis;
};

// -----------------------------------------------------------------------------
// After we finish updating E_x, E_x2 for all blocks, we can compute the standard
// deviation.  The variance = E[x^2] - (E[x])^2.  We'll define a helper function:
inline double computeStdDev(double E_x2, double E_x)
{
    double var = E_x2 - E_x * E_x;
    if (var < 0.0) var = 0.0;  // numerical safety
    return std::sqrt(var);
}

// -----------------------------------------------------------------------------
// We define a function that, given scale k, the maximum scale K, and a base
// forgetting factor alpha0 for the largest square, returns an actual forgetting
// factor alpha_k for scale k.
//
// Interpretation: if we want the largest squares (k=K) to have the "base parameter"
// alpha0, and smaller squares to "forget faster," we can *increase* alpha as k
// decreases. Below is one possible linear mapping:
//
//   alpha(k) = alpha0 + (1 - alpha0) * ( (double)(K - k) / (double)K )
//
// So:
//   - For k=K (largest blocks) => alpha(K) = alpha0
//   - For k=0 (smallest blocks) => alpha(0) = alpha0 + (1 - alpha0) = 1
//
// meaning the smallest blocks use alpha=1 (i.e.\ minimal forgetting) while the
// largest blocks use alpha=alpha0 < 1 (i.e.\ more forgetting).
//
// You can adjust this (e.g., a geometric scaling) to match your preference.
inline double getForgettingFactor(int k, int K, double alpha0) {
    return alpha0;
    // safety checks
    if (K <= 0) {
        return alpha0;
    }
    double fraction = static_cast<double>(K - k) / static_cast<double>(K);
    double alpha_k = alpha0 + (1.0 - alpha0)*fraction;
    if (alpha_k > 1.0) alpha_k = 1.0;
    if (alpha_k < 0.0) alpha_k = 0.0;
    return alpha_k;
}

inline void writeVectorToCSV(const std::vector<long long>& vec, const std::string& filename) {
    std::ofstream outFile(filename);

    if (!outFile.is_open()) {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return;
    }

    for (size_t i = 0; i < vec.size(); ++i) {
        outFile << vec[i];
        if (i < vec.size() - 1) {
            outFile << ",";
        }
    }

    outFile.close();
    std::cout << "Vector written to " << filename << " successfully." << std::endl;
}

inline void writeConfigToH5(H5::Group &group, const Config &config) {
    // Write boolean values
    group.createAttribute("is_analysis", H5::PredType::NATIVE_HBOOL, H5::DataSpace()).write(H5::PredType::NATIVE_HBOOL, &config.is_analysis);
    group.createAttribute("is_quiet", H5::PredType::NATIVE_HBOOL, H5::DataSpace()).write(H5::PredType::NATIVE_HBOOL, &config.is_quiet);
    group.createAttribute("compute_pitch_roll", H5::PredType::NATIVE_HBOOL, H5::DataSpace()).write(H5::PredType::NATIVE_HBOOL, &config.compute_pitch_roll);

    // Write unsigned short values
    group.createAttribute("width", H5::PredType::NATIVE_USHORT, H5::DataSpace()).write(H5::PredType::NATIVE_USHORT, &config.width);
    group.createAttribute("height", H5::PredType::NATIVE_USHORT, H5::DataSpace()).write(H5::PredType::NATIVE_USHORT, &config.height);

    // Write integer values
    group.createAttribute("temporal_stride", H5::PredType::NATIVE_INT, H5::DataSpace()).write(H5::PredType::NATIVE_INT, &config.temporal_stride);
    group.createAttribute("k_min", H5::PredType::NATIVE_INT, H5::DataSpace()).write(H5::PredType::NATIVE_INT, &config.k_min);
    group.createAttribute("k_max", H5::PredType::NATIVE_INT, H5::DataSpace()).write(H5::PredType::NATIVE_INT, &config.k_max);
    group.createAttribute("T_min", H5::PredType::NATIVE_INT, H5::DataSpace()).write(H5::PredType::NATIVE_INT, &config.T_min);
    group.createAttribute("alpha_interarrival_time_window_size", H5::PredType::NATIVE_INT, H5::DataSpace()).write(H5::PredType::NATIVE_INT, &config.alpha_interarrival_time_window_size);
    group.createAttribute("max_rate", H5::PredType::NATIVE_INT, H5::DataSpace()).write(H5::PredType::NATIVE_INT, &config.max_rate);

    // Write long long (timestamp) values
    group.createAttribute("start_us", H5::PredType::NATIVE_LLONG, H5::DataSpace()).write(H5::PredType::NATIVE_LLONG, &config.start_us);
    group.createAttribute("end_us", H5::PredType::NATIVE_LLONG, H5::DataSpace()).write(H5::PredType::NATIVE_LLONG, &config.end_us);

    // Write double values
    group.createAttribute("fps", H5::PredType::NATIVE_DOUBLE, H5::DataSpace()).write(H5::PredType::NATIVE_DOUBLE, &config.fps);
    group.createAttribute("alpha_of_burst_std", H5::PredType::NATIVE_DOUBLE, H5::DataSpace()).write(H5::PredType::NATIVE_DOUBLE, &config.alpha_of_burst_std);
    group.createAttribute("alpha_interarrival_time", H5::PredType::NATIVE_DOUBLE, H5::DataSpace()).write(H5::PredType::NATIVE_DOUBLE, &config.alpha_interarrival_time);
    group.createAttribute("min_interarrival_time", H5::PredType::NATIVE_DOUBLE, H5::DataSpace()).write(H5::PredType::NATIVE_DOUBLE, &config.min_interarrival_time);
    group.createAttribute("max_burst_std_per_cent", H5::PredType::NATIVE_DOUBLE, H5::DataSpace()).write(H5::PredType::NATIVE_DOUBLE, &config.max_burst_std_per_cent);
    group.createAttribute("scale_down_factor", H5::PredType::NATIVE_DOUBLE, H5::DataSpace()).write(H5::PredType::NATIVE_DOUBLE, &config.scale_down_factor);
    group.createAttribute("rotation_angle", H5::PredType::NATIVE_DOUBLE, H5::DataSpace()).write(H5::PredType::NATIVE_DOUBLE, &config.rotation_angle);
    group.createAttribute("pitch_roll_estimation_alpha", H5::PredType::NATIVE_DOUBLE, H5::DataSpace()).write(H5::PredType::NATIVE_DOUBLE, &config.pitch_roll_estimation_alpha);
    group.createAttribute("min_E_x_of_std", H5::PredType::NATIVE_DOUBLE, H5::DataSpace()).write(H5::PredType::NATIVE_DOUBLE, &config.min_E_x_of_std);
    group.createAttribute("max_E_x_of_std", H5::PredType::NATIVE_DOUBLE, H5::DataSpace()).write(H5::PredType::NATIVE_DOUBLE, &config.max_E_x_of_std);

    // Write string values
    group.createAttribute("mode", H5::StrType(H5::PredType::C_S1, H5T_VARIABLE), H5::DataSpace()).write(H5::StrType(H5::PredType::C_S1, H5T_VARIABLE), config.mode);
    group.createAttribute("analysis_filepath", H5::StrType(H5::PredType::C_S1, H5T_VARIABLE), H5::DataSpace()).write(H5::StrType(H5::PredType::C_S1, H5T_VARIABLE), config.analysis_filepath);
    group.createAttribute("tensorboard_log_file", H5::StrType(H5::PredType::C_S1, H5T_VARIABLE), H5::DataSpace()).write(H5::StrType(H5::PredType::C_S1, H5T_VARIABLE), config.tensorboard_log_file);
    group.createAttribute("recording_filepath", H5::StrType(H5::PredType::C_S1, H5T_VARIABLE), H5::DataSpace()).write(H5::StrType(H5::PredType::C_S1, H5T_VARIABLE), config.recording_filepath);

    // Write uint32_t value
    group.createAttribute("acc", H5::PredType::NATIVE_UINT32, H5::DataSpace()).write(H5::PredType::NATIVE_UINT32, &config.acc);

    // Write char value
    group.createAttribute("polarity", H5::PredType::C_S1, H5::DataSpace(H5S_SCALAR)).write(H5::PredType::C_S1, &config.polarity);
}

inline void create_group_if_does_not_exist(H5::Group& h5_group, std::string& group_name)
{
    if (!h5_group.nameExists(group_name))
    {
        h5_group.createGroup(group_name);
    }
}

// Rotates pixel (x, y) around the center of an image of size (width x height)
// by 'angle_deg' degrees (counterclockwise).
inline void
rotatePixel(Metavision::EventCD& ev,
            uint16_t width,
            uint16_t height,
            double angle_deg)
{
    // 1) Convert angle to radians
    double theta = angle_deg * 3.14159265358979323846 / 180.0;
    double cosT  = std::cos(theta);
    double sinT  = std::sin(theta);

    // 2) Compute center of sensor
    double cx = static_cast<double>(width)  / 2.0;
    double cy = static_cast<double>(height) / 2.0;

    // 3) Translate (x, y) so that (cx, cy) is the origin
    double dx = static_cast<double>(ev.x) - cx;
    double dy = static_cast<double>(ev.y) - cy;

    // 4) Apply 2D rotation around origin
    double rx = dx * cosT - dy * sinT;
    double ry = dx * sinT + dy * cosT;

    // 5) Translate back
    rx += cx;
    ry += cy;

    // 6) Clamp and cast to uint16_t
    int ix = static_cast<int>(std::lround(rx));
    int iy = static_cast<int>(std::lround(ry));

    ix = std::max(0, std::min(static_cast<int>(width  - 1), ix));
    iy = std::max(0, std::min(static_cast<int>(height - 1), iy));

    ev.x = static_cast<uint16_t>(ix);
    ev.y = static_cast<uint16_t>(iy);
}

inline double getCameraAngle(const std::string& path) {
    const std::string marker = "camera-angle-";
    size_t startPos = path.find(marker);

    // If the marker is not found, return an empty string or handle error
    if (startPos == std::string::npos) {
        return {};
    }

    // Move startPos to the character right after "camera-angle-"
    startPos += marker.size();

    // Find the next slash (or end of the string if no slash is found)
    size_t slashPos = path.find('/', startPos);
    if (slashPos == std::string::npos) {
        slashPos = path.size();
    }

    // Extract the substring representing the camera angle
    return std::stod(path.substr(startPos, slashPos - startPos));
}

#endif //UTILS_H
