//
// Created by radio on 30.09.2024.
//

#ifndef DETECTOR_H
#define DETECTOR_H
#include <vector>
#include <memory>
#include <cassert>
#include <condition_variable>

#include "TensorBoardLogger.h"
#include <types.h>
#include <utils.h>

#include "H5Cpp.h"
#include <filesystem>
#include <boost/timer/timer.hpp>


class Detector
{
public:
    Detector(TensorBoardLogger &logger, Config & config);


    void addCallback(const std::function<void(std::vector<DetectionResult> &)> &);
    void addEventsAsync(const Metavision::EventCD *, const Metavision::EventCD *);
    [[nodiscard]] size_t jobsCount() const;
    void start(size_t );
    void stop();
    void join();
    void storeRunTime(int_least64_t nanosecond, int_least64_t end_time);

    std::shared_ptr<std::vector<std::vector<std::vector<std::vector<BlockStats>>>>> m_stats;
    std::shared_ptr<std::vector<std::vector<GlobalStats>>> m_global_stats;

    std::vector<long long> timestamps;


    // Standard-deviation thresholds
    double max_burst_std_per_cent;
    double pitch_gt;

    bool isPropellerPresent(BlockStats &stats, double timestamp) const;
private:
    Config &config;
    TensorBoardLogger &logger;


    void initializeAnalysis();
    void updateBlockStats(BlockStats &stats, double timestamp);

    const size_t MAX_CHUNK_SIZE = 50000;
    H5::H5File h5_file;
    std::vector<std::unique_ptr<H5::Group>> channel_groups;

    char m_polarity_allowed;
    int num_polarities_recorded;

    // Base forgetting factor for largest squares: e.g. 0.90
    double alpha_of_burst_std;
    double alpha_interarrival_time;
    double alpha_interarrival_time_window;
    double min_interarrival_time;
    double max_rate;

    int T_min;
    int k_min;

    long long first_ts_us = -1;
    long long last_ts_us = -1;

    unsigned short width = 0;
    unsigned short height = 0;

    bool is_running = false;
    bool stop_processing = false;

    std::vector<std::thread> processing_threads;

    std::vector<DetectionResult> detection_results;
    std::mutex detection_results_mutex;

    std::vector<std::function<void(std::vector<DetectionResult> &correlation_results)>> callbacks;
    std::thread callback_thread;
    std::condition_variable callbacks_cv;

    std::queue<std::tuple<std::shared_ptr<Metavision::EventCD[]>, std::size_t>> jobs;
    std::mutex jobs_mutex;
    std::condition_variable jobs_cv;

    int K = 0;

    void addEvents(std::shared_ptr<Metavision::EventCD[]>&& arr, size_t arr_size);
    void updateGlobalStats(GlobalStats &stats, uint16_t x, uint16_t y);
};



#endif //DETECTOR_H
