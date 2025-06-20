//
// Created by radio on 30.09.2024.
//

#include "Detector.h"

Detector::Detector(TensorBoardLogger& logger, Config& config) :
    config(config), logger(logger),
    alpha_of_burst_std(config.alpha_of_burst_std),
    alpha_interarrival_time(config.alpha_interarrival_time),
    alpha_interarrival_time_window(2.0 / (config.alpha_interarrival_time_window_size + 1)),
    min_interarrival_time(config.min_interarrival_time),
    max_rate(config.max_rate),
    T_min(config.T_min),
    max_burst_std_per_cent(config.max_burst_std_per_cent),
    k_min(config.k_min),
    width(config.width), height(config.height),
    m_polarity_allowed(config.polarity),
    num_polarities_recorded(config.polarity == 's' ? 2 : 1),
    pitch_gt(getCameraAngle(config.recording_filepath))
{
    // -------------------------------------------------------------------------
    // 1) Determine maximum scale K s.t. 2^K <= min(width, height).
    // int min_dim = (config.width < config.height) ? config.width : config.height;
    // K = 0;
    // while ((1 << (K+1)) <= min_dim) {
    //     ++K;
    // }
    K = config.k_max;

    // -------------------------------------------------------------------------
    // 2) Allocate data structures for each scale k in [0..K], storing
    //    ExponentialStats for every block. At scale k, the block size is 2^k,
    //    so we have (width >> k) blocks horizontally, (height >> k) blocks vertically.

    // E.g.: stats[k][n][m].E_x / E_x2
    m_stats = std::make_shared<std::vector<std::vector<std::vector<std::vector<BlockStats>>>>>(num_polarities_recorded);
    for (int p = 0; p < num_polarities_recorded; ++p) {
        (*m_stats)[p].resize(K - k_min + 1);
        for (int k = k_min; k <= K; ++k) {
            int num_blocks_x = std::ceil(config.width / static_cast<double>(1 << k));
            int num_blocks_y = std::ceil(config.height / static_cast<double>(1 << k));
            (*m_stats)[p][k - k_min].resize(num_blocks_y,
                std::vector<BlockStats>(num_blocks_x));
            std::cout << "p: " << p << " k: " << k << " num_blocks_x: " << num_blocks_x << " num_blocks_y: " << num_blocks_y << std::endl;
        }
    }

    m_global_stats = std::make_shared<std::vector<std::vector<GlobalStats>>>(num_polarities_recorded);
    for (int p = 0; p < num_polarities_recorded; ++p) {
        (*m_global_stats)[p].resize(K - k_min + 1);
    }

    if (config.is_analysis) {
        initializeAnalysis();
    }
}

template <typename T>
void write_attribute_to_h5(H5::Group& h5_group, const int x, const int y,
    std::string attribute_name, T attribute_value,
                    const H5::PredType dataset_type = H5::PredType::NATIVE_LLONG)
{
    std::string y_groupname = std::to_string(y);
    std::string x_groupname = std::to_string(x);

    create_group_if_does_not_exist(h5_group, y_groupname);
    auto y_group = h5_group.openGroup(y_groupname);
    create_group_if_does_not_exist(y_group, x_groupname);
    auto x_group = y_group.openGroup(x_groupname);

    x_group.createAttribute(attribute_name, dataset_type, H5::DataSpace()).write(dataset_type, &attribute_value);
}

template <typename T>
void write_to_h5(H5::Group& h5_group,
                        const int x, const int y,
                        std::string group_name,
                        const std::vector<T>* block_data,
                        const size_t MAX_CHUNK_SIZE,
                        const H5::PredType dataset_type = H5::PredType::NATIVE_LLONG)
{
    // Define the data space dimensions
    hsize_t incoming_data_dims[1] = {block_data->size()};
    hsize_t max_dims[1] = {H5S_UNLIMITED};
    const H5::DataSpace initial_dataspace(1, incoming_data_dims, max_dims);

    std::string y_groupname = std::to_string(y);
    std::string x_groupname = std::to_string(x);

    create_group_if_does_not_exist(h5_group, y_groupname);
    auto y_group = h5_group.openGroup(y_groupname);
    create_group_if_does_not_exist(y_group, x_groupname);
    auto x_group = y_group.openGroup(x_groupname);

    // If the datasets do not exist, create them with extendable dimensions
    H5::DSetCreatPropList plist;
    hsize_t chunk_dims[1] = {MAX_CHUNK_SIZE};  // Set chunk size
    plist.setChunk(1, chunk_dims);

    H5::DataSet dataset_ts, dataset_correlation, dataset_is_peak;
    H5::DataSpace filespace_ts;
    hsize_t offset[1];
    hsize_t new_size[1];

    if (!x_group.exists(group_name)) {
        dataset_ts = x_group.createDataSet(group_name, dataset_type, initial_dataspace, plist);
        filespace_ts = dataset_ts.getSpace();
        offset[0] = 0;
    } else {
        dataset_ts = x_group.openDataSet(group_name);
        hsize_t current_size[1];
        filespace_ts = dataset_ts.getSpace();
        filespace_ts.getSimpleExtentDims(current_size, nullptr);
        offset[0] = {current_size[0]};
        new_size[0] = {current_size[0] + incoming_data_dims[0]};
        dataset_ts.extend(new_size);  // Extend the dataset
        filespace_ts = dataset_ts.getSpace();  // Re-fetch the dataspace
    }

    // Select hyperslabs to write the new chunk of data
    filespace_ts.selectHyperslab(H5S_SELECT_SET, incoming_data_dims, offset);

    H5::DataSpace memoryspace(1, incoming_data_dims);
    dataset_ts.write(block_data->data(), dataset_type, memoryspace, filespace_ts);
}

void Detector::initializeAnalysis()
{
    std::string analysis_dir;

    auto h5_flag = H5F_ACC_TRUNC;

    // extract directory from analysis_filepath
    analysis_dir = std::filesystem::path(config.analysis_filepath).parent_path().string();
    // if analysis_dir does not exist, create it
    std::filesystem::create_directories(analysis_dir);

    h5_file = H5::H5File(config.analysis_filepath, h5_flag);
    for (int channel_number = 0; channel_number < num_polarities_recorded; ++channel_number)
    {
        std::string ch_groupname = "ch" + std::to_string(channel_number);
        if (h5_file.nameExists(ch_groupname)) {
            std::cerr << "Group " << ch_groupname << " already exists in " << config.analysis_filepath << std::endl;
            exit(1);
        }
        channel_groups.push_back(std::make_unique<H5::Group>(h5_file.createGroup(ch_groupname)));
    }

    writeConfigToH5(h5_file, config);
}


void Detector::addCallback(
    const std::function<void(std::vector<DetectionResult>& db_results)>& callback)
{
    callbacks.push_back(callback);
}

bool Detector::isPropellerPresent(BlockStats& stats, double timestamp) const
{
    return 100 * stats.burst_std_estimated_ratio < config.max_burst_std_per_cent
                            && stats.E_x_of_std < config.max_E_x_of_std
                            && config.min_E_x_of_std < stats.E_x_of_std
                            && stats.burst_timestep_t + stats.E_x_of_std * (1.0 + config.max_burst_std_per_cent / 100.0) > timestamp;
}

void Detector::addEvents(std::shared_ptr<Metavision::EventCD[]>&& arr, size_t arr_size) {
    if (arr_size == 0) {
        return;
    }

    for (Metavision::EventCD * ev = arr.get(); ev < arr.get() + arr_size; ev += config.temporal_stride) {
        if (m_polarity_allowed == 'p' && ev->p != 1) {
            continue;
        }
        if (m_polarity_allowed == 'n' && ev->p != 0) {
            continue;
        }
        int p = 0;
        if (m_polarity_allowed == 's') {
            p = ev->p;
        }

        uint16_t x = ev->x;
        uint16_t y = ev->y;
        double   t = static_cast<double>(ev->t); // cast to double for safety

        // For each scale k, figure out which block the (x,y) belongs to:
        for (int k = k_min; k <= K; ++k) {
            int m = x >> k;  // floor(x / 2^k)
            int n = y >> k;  // floor(y / 2^k)

            // Update E[x], E[x^2] for that block
            this->updateBlockStats((*m_stats)[p][k - k_min][n][m], t);

            if (config.compute_pitch_roll && isPropellerPresent((*m_stats)[0][k - k_min][n][m], t))
            {
                updateGlobalStats((*m_global_stats)[p][k - k_min], ev->x, ev->y);
            }

            if (config.is_analysis)
            {
                if (isPropellerPresent((*m_stats)[0][k - k_min][n][m], t)
                    || (num_polarities_recorded == 2 && isPropellerPresent((*m_stats)[1][k - k_min][n][m], t)))
                {
                    ++(*m_stats)[p][k - k_min][n][m].events_propeller_present_num;
                }
                ++(*m_stats)[p][k - k_min][n][m].events_total_num;

                auto burst_timesteps = &(*m_stats)[p][k - k_min][n][m].burst_ts;
                auto burst_distances_us = &(*m_stats)[p][k - k_min][n][m].burst_distances_us;
                if (burst_timesteps->size() == MAX_CHUNK_SIZE)
                {
                    write_to_h5(*(channel_groups[p]), m, n, "burst_ts", burst_timesteps, MAX_CHUNK_SIZE);
                    write_to_h5(*(channel_groups[p]), m, n, "burst_distances_us", burst_distances_us, MAX_CHUNK_SIZE);

                    burst_timesteps->clear();
                    burst_distances_us->clear();
                }
            }
            if (config.is_analysis || config.is_runtime_analysis) {
                if (first_ts_us < 0)
                {
                    first_ts_us = ev->t;
                }
                if (last_ts_us < ev->t)
                {
                    last_ts_us = ev->t;
                }
            }
        }
    }

    {
        std::lock_guard<std::mutex> lock(detection_results_mutex);

        detection_results.emplace_back(std::move(m_stats));
    }
    callbacks_cv.notify_one();
}

void Detector::start(size_t num_threads) {
    if (is_running) {
        return;
    }
    is_running = true;

    // Creating worker threads
    for (size_t i = 0; i < num_threads; ++i) {
        processing_threads.emplace_back([this] {
            while (true) {
                std::shared_ptr<Metavision::EventCD[]> arr;
                size_t arr_size;
                {
                    std::unique_lock<std::mutex> lock(jobs_mutex);

                    jobs_cv.wait(lock, [this] {
                        return !jobs.empty() || stop_processing;
                    });

                    if (stop_processing) {
                        return;
                    }

                    std::tie(arr, arr_size) = std::move(jobs.front());
                    jobs.pop();
                }

                // long long total_time = timer.elapsed().wall;
                addEvents(std::move(arr), arr_size);
                // total_time = timer.elapsed().wall - total_time;
                // std::cout << "Total time outside: " << total_time / 1e9 << std::endl;
            }
        });
    }
    // callback worker
    callback_thread = std::thread([this] {
        while (true) {
            {
                std::unique_lock<std::mutex> lock(detection_results_mutex);
                callbacks_cv.wait(lock, [this] {
                    return !detection_results.empty() || stop_processing;
                });
                for (const auto &callback: callbacks) {
                    callback(detection_results);
                }
                detection_results.clear();
            }
            if (stop_processing) {
                break;
            }
        }
    });
}

void Detector::stop() {
    {
        std::unique_lock<std::mutex> lock(jobs_mutex);
        stop_processing = true;
    }

    jobs_cv.notify_all();

    for (auto& thread : processing_threads) {
        thread.join();
    }

    callbacks_cv.notify_all();
    callback_thread.join();

    if (config.is_analysis)
    {
        h5_file.createAttribute("first_ts_us", H5::PredType::NATIVE_LLONG, H5::DataSpace()).write(H5::PredType::NATIVE_LLONG, &first_ts_us);
        h5_file.createAttribute("last_ts_us", H5::PredType::NATIVE_LLONG, H5::DataSpace()).write(H5::PredType::NATIVE_LLONG, &last_ts_us);

        // write out the remainings
        for (int p = 0; p < num_polarities_recorded; ++p) {
            for (int k = m_stats->at(p).size() + k_min - 1; k >= k_min; --k) {
                auto pitch_sum = &(*m_global_stats)[p][k - k_min].pitch_sum;
                auto abs_pitch_error_sum = &(*m_global_stats)[p][k - k_min].abs_pitch_error_sum;
                auto pitch_num = &(*m_global_stats)[p][k - k_min].abs_pitch_error_num;
                auto avg_abs_roll_error = &(*m_global_stats)[p][k - k_min].abs_roll_error_sum;
                write_attribute_to_h5(*(channel_groups[p]), 0, 0, "avg_pitch",
                    *pitch_sum / *pitch_num, H5::PredType::NATIVE_DOUBLE);
                write_attribute_to_h5(*(channel_groups[p]), 0, 0, "abs_pitch_error_avg",
                    *abs_pitch_error_sum / *pitch_num, H5::PredType::NATIVE_DOUBLE);
                write_attribute_to_h5(*(channel_groups[p]), 0, 0, "abs_roll_error_avg",
                    *avg_abs_roll_error / *pitch_num, H5::PredType::NATIVE_DOUBLE);

                for (int y = 0; y < m_stats->at(p)[k - k_min].size(); ++y) {
                    for (int x = 0; x < m_stats->at(p)[k - k_min][y].size(); ++x) {
                        long long events_propeller_present_num = (*m_stats)[p][k - k_min][y][x].events_propeller_present_num;
                        long long events_total_num = (*m_stats)[p][k - k_min][y][x].events_total_num;

                        write_attribute_to_h5(*(channel_groups[p]), x, y, "events_propeller_present_num", events_propeller_present_num);
                        write_attribute_to_h5(*(channel_groups[p]), x, y, "events_total_num", events_total_num);

                        auto burst_timesteps = &(*m_stats)[p][k - k_min][y][x].burst_ts;
                        auto burst_distances_us = &(*m_stats)[p][k - k_min][y][x].burst_distances_us;
                        {
                            write_to_h5(*(channel_groups[p]), x, y, "burst_ts", burst_timesteps, MAX_CHUNK_SIZE);
                            write_to_h5(*(channel_groups[p]), x, y, "burst_distances_us", burst_distances_us, MAX_CHUNK_SIZE);
                            burst_timesteps->clear();
                            burst_distances_us->clear();
                        }
                    }
                }
            }
        }
    }
}

void Detector::addEventsAsync(const Metavision::EventCD *begin, const Metavision::EventCD *end) {
    {
        std::unique_lock<std::mutex> lock(jobs_mutex);
        std::vector<EVENT_TYPE> events;

        size_t num_elements = end - begin;
        std::shared_ptr<Metavision::EventCD[]> copy(new Metavision::EventCD[num_elements], std::default_delete<Metavision::EventCD[]>());
        std::memcpy(copy.get(), begin, num_elements * sizeof(Metavision::EventCD));

        jobs.emplace(copy, num_elements);
    }
    jobs_cv.notify_one();
}

size_t Detector::jobsCount() const {
    return jobs.size();
}

void Detector::join() {
}

void Detector::storeRunTime(int_least64_t start_time, int_least64_t end_time)
{
    int_least64_t runtime_us = (end_time - start_time) / 1000;

    // if h5_file is not open, open
    if (h5_file.getId() == -1)
    {
        // if file exists
        if (std::filesystem::exists(config.analysis_filepath))
        {
            h5_file = H5::H5File(config.analysis_filepath, H5F_ACC_RDWR);
        }
        else
        {
            // File doesn't exist: create a new file
            h5_file = H5::H5File(config.analysis_filepath, H5F_ACC_TRUNC);
            writeConfigToH5(h5_file, config);
            h5_file.createAttribute("first_ts_us", H5::PredType::NATIVE_LLONG, H5::DataSpace()).write(H5::PredType::NATIVE_LLONG, &first_ts_us);
            h5_file.createAttribute("last_ts_us", H5::PredType::NATIVE_LLONG, H5::DataSpace()).write(H5::PredType::NATIVE_LLONG, &last_ts_us);
        }
    }

    // Open (or create) the group "runtime_us"
    H5::Group runtimeGroup;
    if (h5_file.nameExists("runtime_us")) {
        runtimeGroup = h5_file.openGroup("runtime_us");
    } else {
        runtimeGroup = h5_file.createGroup("runtime_us");
    }

    // Dataset name inside the group
    const std::string datasetName = "runtime";
    H5::DataSet dataset;
    bool datasetExists = true;

    // Attempt to open the dataset
    if (!runtimeGroup.nameExists(datasetName)) {
        datasetExists = false;
    }
    else
    {
        dataset = runtimeGroup.openDataSet(datasetName);
    }

    if (!datasetExists) {
        // The dataset does not exist. Create a new 1D dataset with unlimited dimensions.
        hsize_t dims[1]    = { 1 };          // initial size
        hsize_t maxDims[1] = { H5S_UNLIMITED }; // allow unlimited growth

        // Create dataspace with initial dims and max dims
        H5::DataSpace dataspace(1, dims, maxDims);

        // Set dataset creation properties: enable chunking (required for extendible datasets)
        H5::DSetCreatPropList prop;
        hsize_t chunkDims[1] = { 1 };  // choose a chunk size of 1 (adjust as needed)
        prop.setChunk(1, chunkDims);

        // Create the dataset (using unsigned long long type)
        dataset = runtimeGroup.createDataSet(datasetName, H5::PredType::NATIVE_ULLONG, dataspace, prop);

        // Write the first runtime value into the dataset.
        dataset.write(&runtime_us, H5::PredType::NATIVE_ULLONG);
    } else {
        // The dataset exists. Extend it by one element.
        // Get the current dataspace of the dataset.
        H5::DataSpace filespace = dataset.getSpace();
        const int rank = 1;
        hsize_t dims[rank];
        hsize_t maxDims[rank];
        filespace.getSimpleExtentDims(dims, maxDims);

        // Compute new dimensions: extend by one.
        hsize_t newDims[rank] = { dims[0] + 1 };
        dataset.extend(newDims);

        // After extending, get the new filespace.
        H5::DataSpace extendedSpace = dataset.getSpace();

        // Select the hyperslab corresponding to the new element.
        hsize_t offset[rank] = { dims[0] };  // start at the old last element
        hsize_t count[rank]  = { 1 };
        extendedSpace.selectHyperslab(H5S_SELECT_SET, count, offset);

        // Create a memory dataspace for one element.
        H5::DataSpace memspace(1, count);

        // Write the new runtime value into the extended region.
        dataset.write(&runtime_us, H5::PredType::NATIVE_ULLONG, memspace, extendedSpace);
    }

}

void Detector::updateBlockStats(BlockStats &stats, double timestamp) {
    if (stats.last_timestamp < 0) {
        stats.last_timestamp = timestamp;
        return;
    }

    // Compute inter-arrival time
    double interarrival_time = timestamp - stats.last_timestamp;
    stats.last_timestamp = timestamp;

    // Update threshold
    stats.ewma_interarrival_time = alpha_interarrival_time * interarrival_time + (1 - alpha_interarrival_time) * stats.ewma_interarrival_time;

    double lambda_threshold_est = 1.0 / stats.ewma_interarrival_time;
    // stats.thresholds.push_back(lambda_threshold_est);

    stats.ewma_interarrival_time_window = alpha_interarrival_time_window * interarrival_time + (1 - alpha_interarrival_time_window) * stats.ewma_interarrival_time_window;

    // Estimate lambda
    double lambda_est = 1.0 / std::max(stats.ewma_interarrival_time_window, min_interarrival_time);
    lambda_est = std::min(lambda_est, max_rate);
    // stats.estimated_lambdas.push_back(lambda_est);

    // Burst detection
    if (lambda_est > lambda_threshold_est) {
        stats.T_high++;
        if (stats.T_high >= T_min && !stats.in_burst) {
            stats.in_burst = true;
        }
        if (stats.in_burst) {
            stats.local_burst_detected_sum += timestamp;
            stats.local_burst_detected_num++;
            stats.last_timestep_of_burst_t = timestamp;
        }
        // stats.burst_detected.emplace_back(timestamp, lambda_est);
    } else {
        stats.T_high = 0;
        stats.in_burst = false;
        if (stats.local_burst_detected_num > 0) {
            double burst_timestep_mean = stats.local_burst_detected_sum / stats.local_burst_detected_num;
            stats.local_burst_detected_sum = 0;
            stats.local_burst_detected_num = 0;

            stats.burst_timestep_t_minus_1 = stats.burst_timestep_t;
            stats.burst_timestep_t = burst_timestep_mean;

            if (stats.burst_timestep_t_minus_1 > 0.0) {
                double burst_distance = stats.burst_timestep_t - stats.burst_timestep_t_minus_1;

                if (stats.E_x_of_std < 0) {
                    stats.E_x_of_std = burst_distance;
                    stats.E_x2_of_std = burst_distance * burst_distance;
                } else {
                    stats.E_x_of_std = (1.0 - alpha_of_burst_std) * stats.E_x_of_std + alpha_of_burst_std * burst_distance;
                    stats.E_x2_of_std = (1.0 - alpha_of_burst_std) * stats.E_x2_of_std + alpha_of_burst_std * burst_distance * burst_distance;

                    stats.burst_std_estimated_ratio = std::sqrt(stats.E_x2_of_std - stats.E_x_of_std * stats.E_x_of_std) / stats.E_x_of_std;
                }
            }

            if (config.is_analysis)
            {
                if (isPropellerPresent(stats, timestamp))
                {
                    stats.burst_ts.push_back(static_cast<long long>(burst_timestep_mean));
                    stats.burst_distances_us.push_back(static_cast<long long>(stats.burst_timestep_t - stats.burst_timestep_t_minus_1));
                }
                else // reset inter-burst stats
                {
                    stats.burst_timestep_t_minus_1 = -1.0;
                }
            }
        }
    }
}

void Detector::updateGlobalStats(GlobalStats& stats, uint16_t x, uint16_t y)
{
    // Choose an EMA factor alpha in [0,1]. Larger alpha -> faster updates.
    const double alpha = config.pitch_roll_estimation_alpha;

    // 1) Update EMA estimates of first and second moments
    stats.ema_x  = alpha * x         + (1.0 - alpha) * stats.ema_x;
    stats.ema_y  = alpha * y         + (1.0 - alpha) * stats.ema_y;
    stats.ema_x2 = alpha * (x * x)   + (1.0 - alpha) * stats.ema_x2;
    stats.ema_y2 = alpha * (y * y)   + (1.0 - alpha) * stats.ema_y2;
    stats.ema_xy = alpha * (x * y)   + (1.0 - alpha) * stats.ema_xy;

    // 2) Compute mean and (co)variance from EMA values
    double mean_x = stats.ema_x;
    double mean_y = stats.ema_y;
    double Mxx    = stats.ema_x2 - mean_x * mean_x;
    double Myy    = stats.ema_y2 - mean_y * mean_y;
    double Mxy    = stats.ema_xy - mean_x * mean_y;

    // 3) Solve for principal axes of the 2x2 covariance matrix
    //    [ Mxx  Mxy ]
    //    [ Mxy  Myy ]

    // The sum of eigenvalues:
    double trace2 = 0.5 * (Mxx + Myy);
    // The half-difference of eigenvalues:
    double diff2  = 0.5 * std::sqrt((Mxx - Myy)*(Mxx - Myy) + 4.0*Mxy*Mxy);

    // Largest (lambda1) and smallest (lambda2) eigenvalues:
    double lambda1 = trace2 + diff2;
    double lambda2 = trace2 - diff2;

    // 4) Compute pitch and roll.
    //    - pitch = ratio of minor axis to major axis = sqrt(lambda2 / lambda1)
    //    - roll  = orientation of the major axis wrt x-axis

    // Guard against near-zero variance:
    if (lambda1 <= 1e-12 || lambda2 <= 1e-12) {
        stats.pitch = 1.0;  // no shape info => degenerate ellipse
        stats.roll  = 0.0;
    } else {
        // pitch: how "squeezed" the ellipse is
        stats.pitch = std::sqrt(lambda2 / lambda1);

        // roll: angle of the major principal axis (associated w/ lambda1)
        // standard formula for 2x2 covariance eigenvector:
        // angle = 0.5 * atan2(2 Mxy, Mxx - Myy)
        stats.roll = std::abs(0.5 * std::atan2(2.0 * Mxy, (Mxx - Myy)));
    }
    stats.pitch_sum += stats.pitch;
    stats.abs_pitch_error_sum += std::abs(stats.pitch * config.pitch_scaler - pitch_gt);
    stats.abs_roll_error_sum += std::abs((stats.roll * 180.0 / 3.141592) - config.rotation_angle);
    ++stats.abs_pitch_error_num;
}

