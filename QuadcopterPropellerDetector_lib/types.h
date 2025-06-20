//
// Created by radio on 02.08.2024.
//

#ifndef TYPES_H
#define TYPES_H

#include <tuple>
#include <cstddef>
#include <memory>
#include <string>
#include <metavision/sdk/base/events/event_cd.h>
#include <metavision/sdk/base/utils/callback_id.h>


struct GlobalStats
{
    double pitch;
    double roll;
    double ema_x;
    double ema_y;
    double ema_x2;
    double ema_y2;
    double ema_xy;

    double abs_pitch_error_sum;
    double abs_pitch_error_num;
    double abs_roll_error_sum;
    double pitch_sum;
};

// -----------------------------------------------------------------------------
// A helper struct to store exponential moving statistics for each block.
struct BlockStats {
    double last_timestamp = -1;

    double ewma_interarrival_time = 0.0;
    double ewma_interarrival_time_window = 0.0;

    double E_x_of_std = -1;
    double E_x2_of_std = 0.0;

    double local_burst_detected_sum = 0.0;
    int local_burst_detected_num = 0;
    int T_high = 0;
    bool in_burst = false;

    double burst_timestep_t = 0.0; // aka mean of timesteps of burst t
    double last_timestep_of_burst_t = 0.0; // the last timestep of burst t
    double burst_timestep_t_minus_1 = 0.0; // aka mean of timesteps of burst t-1
    double burst_std_estimated_ratio = 0;

    long long events_propeller_present_num;
    long long events_total_num;

    // std::vector<double> thresholds;
    // std::vector<double> estimated_lambdas;
    std::vector<long long> burst_ts;
    std::vector<long long> burst_distances_us;
    // std::vector<std::pair<double, double>> burst_stds_estimated;
    // std::vector<std::pair<double, double>> burst_detected;
};

// -----------------------------------------------------------------------------
// Detection result: which scale/block was flagged for containing a propeller
struct PropellerDetection {
    int scale;
    int block_m;
    int block_n;
};

// x, y, width, height, ts, correlation
typedef std::tuple<unsigned short, unsigned short, unsigned short, unsigned short, long long, long long, long long> CORRELATION_RESULT_TYPE;
// x, y, width, height, ts_us, distance_us
typedef std::tuple<long long, long long> PEAK_RESULT_TYPE;
// x, y, p, t
typedef std::tuple<unsigned short, unsigned short, short, long long> EVENT_TYPE;

struct MetavisionEvent
{
    MetavisionEvent() : x(0), y(0), p(0), t(0) {}
    MetavisionEvent(unsigned short x, unsigned short y, short p, long long t) : x(x), y(y), p(p), t(t) {}
    unsigned short x, y;
    short p;
    long long t;

    bool operator==(const MetavisionEvent& other) const {
        return (x == other.x) &&
               (y == other.y) &&
               (p == other.p) &&
               (t == other.t);
    }
};


struct DetectionResult
{
    DetectionResult() : x(0), y(0), width(0), height(0), ts(0), correlation(0), peak_distance(0), queue_length(0), polarity_sum(0) {}
    explicit DetectionResult(std::shared_ptr<std::vector<std::vector<std::vector<std::vector<BlockStats>>>>>&& stats) : x(0), y(0), width(0), height(0), ts(0), correlation(0), peak_distance(0), queue_length(0), polarity_sum(0), stats(stats) {}

    std::shared_ptr<std::vector<std::vector<std::vector<std::vector<BlockStats>>>>> stats;
    unsigned short x, y, width, height;
    long long ts, correlation, peak_distance;
    size_t queue_length;
    long long missed_results = 0;
    long long polarity_sum;
    bool is_peak = false;
};

struct MetavisionArrAndEvent {
    MetavisionArrAndEvent() : evp(nullptr), ev(nullptr) {};
    explicit MetavisionArrAndEvent(std::shared_ptr<Metavision::EventCD[]> evp, Metavision::EventCD * ev) : evp(std::move(evp)), ev(ev) {}
    std::shared_ptr<Metavision::EventCD[]> evp;
    Metavision::EventCD * ev;
};


#endif //TYPES_H
