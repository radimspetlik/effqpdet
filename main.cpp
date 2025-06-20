#include <iostream>
#include <vector>
#include <tuple>
#include <filesystem>
#include <fstream>
#include <functional>
#include <thread>

#include <boost/program_options.hpp>
#include <boost/timer/timer.hpp>
#include <opencv2/opencv.hpp>
#include <metavision/sdk/base/utils/log.h>
#include <metavision/sdk/core/algorithms/event_buffer_reslicer_algorithm.h>
#include <metavision/sdk/core/algorithms/periodic_frame_generation_algorithm.h>
#include <metavision/sdk/core/algorithms/event_frame_histo_generation_algorithm.h>
#include <metavision/hal/facilities/i_hw_identification.h>
#include <metavision/sdk/driver/camera.h>
#include <metavision/sdk/ui/utils/window.h>
#include <metavision/sdk/ui/utils/event_loop.h>
#include <yaml-cpp/yaml.h>

#include "TensorBoardLogger.h"
#include "QuadcopterPropellerDetector_lib/utils.h"
#include "utils.h"
#include "const.h"
#include "Detector.h"


int width = 8;
int frame_idx = 0;
std::string str_idx;
std::vector<cv::Mat> frames = std::vector<cv::Mat>(10000);

void initialize_detector_visual_mode(Config &config,
                                     Metavision::Window * window,
                                     Metavision::PeriodicFrameGenerationAlgorithm &frame_gen, std::shared_ptr<Detector> &detector)
{
    frame_gen.set_output_callback([&, window, detector](Metavision::timestamp mtv_ts, cv::Mat& frame)
    {
        // cv::putText(frame, std::to_string(mtv_ts), cv::Point(100, 100), cv::FONT_HERSHEY_SIMPLEX, 2.0, cv::Scalar(255, 255, 255));

        float font_scale = 0.3;
        int k_min = config.k_min;

        // for (int k = k_min; k < detector->m_stats->size() + k_min; ++k) {
        for (int p = 0; p < detector->m_stats->size(); ++p) {
            for (int k = detector->m_stats->at(p).size() + k_min - 1; k >= k_min; --k) {
                bool detection_any = false;
                for (int row = 0; row < detector->m_stats->at(p)[k - k_min].size(); ++row) {
                    for (int column = 0; column < detector->m_stats->at(p)[k - k_min][row].size(); ++column) {
                        auto stats = detector->m_stats->at(p)[k - k_min][row][column];
                        if (row == 0 && column == 0 && k == k_min)
                        {
                            std::stringstream stream;
                            stream << std::fixed << std::setprecision(0) << "p: " << (detector->m_global_stats->at(p)[k - k_min].pitch * 180.0 / 3.141592) << " r: " << (detector->m_global_stats->at(p)[k - k_min].roll * 180.0 / 3.141592);
                            double font_height = cv::getTextSize("p: P", cv::FONT_HERSHEY_SIMPLEX, font_scale * 3, 1, nullptr).height;
                            cv::putText(frame, stream.str(), cv::Point((1 << k) * column, (1 << k) * row + font_height * (k - k_min + 1)), cv::FONT_HERSHEY_SIMPLEX, font_scale * 3, cv::Scalar(0, 0, 255));
                        }
                        if (stats.burst_std_estimated_ratio == 0.0) {
                            continue;
                        }
                        // if (row != 5 || column != 6) {
                            // continue;
                        // }
                        int x = (1 << k) * column;
                        int y = (1 << k) * row;

                        if (detector->isPropellerPresent(stats, mtv_ts)) {
                            detection_any = true;
                            if (k == k_min) {
                                double detection_strength = 255 * (1.0 - stats.burst_std_estimated_ratio / config.max_burst_std_per_cent);
                                cv::rectangle(frame, cv::Rect(x, y, 1 << k, 1 << k),
                                              cv::Scalar(0, static_cast<int>(detection_strength), 0), 2.0);
                            }
                        }

                        // double std = computeStdDevOfAboveExs(stats);
                        // cv::rectangle(frame, cv::Rect(x, y, 1 << k, 1 << k), cv::Scalar(0, 0, 255), k_rel + 0.5);
                        // double font_height = cv::getTextSize(std::to_string(std), cv::FONT_HERSHEY_SIMPLEX, font_scale, 1, nullptr).height;
                        // one decimal place
                        // std::stringstream stream;
                        // stream << std::fixed << std::setprecision(0) << std;

                        // int r = int(255 * k_rel);
                        // if (k == k_min) {
                        //     cv::putText(frame, stream.str(), cv::Point(x, y + font_height * (k - k_min + 1)), cv::FONT_HERSHEY_SIMPLEX, font_scale, cv::Scalar(0, r, 255));
                        //     stream.str("");
                        //     stream << std::fixed << std::setprecision(0) << detector->m_stats->at(k - k_min)[row][column].E_x_std;
                        //     cv::putText(frame, stream.str(), cv::Point(x, y + font_height * (k - k_min + 2) * 1.1), cv::FONT_HERSHEY_SIMPLEX, font_scale, cv::Scalar(0, 50, 255));
                        //     stream.str("");
                        //     stream << std::fixed << std::setprecision(0) << detector->m_stats->at(k - k_min)[row][column].E_x_std_2;
                        //     cv::putText(frame, stream.str(), cv::Point(x, y + font_height * (k - k_min + 3) * 1.1), cv::FONT_HERSHEY_SIMPLEX, font_scale, cv::Scalar(0, 100, 255));
                        //     std::string ss = "0";
                        //     if (detector->m_stats->at(k - k_min)[row][column].E_x_std < detector->m_stats->at(k - k_min)[row][column].E_x_std_2)
                        //         ss = "1";
                        //     cv::putText(frame, ss, cv::Point(x, y + font_height * (k - k_min + 4) * 1.1), cv::FONT_HERSHEY_SIMPLEX, font_scale, cv::Scalar(0, 255, 255));
                            // double ratio = std / computeStdDevOfStdDev(detector->m_stats->at(k - k_min + 1)[row >> 1][column >> 1]);
                            // double ratio = computeStdDevOfAboveExs(detector->m_stats->at(k - k_min + 1)[row >> 1][column >> 1]);
                            // stream.str("");
                            // stream << std::fixed << std::setprecision(0) << ratio;
                            // s = stream.str();
                            // cv::putText(frame, s, cv::Point(x, y + font_height * (k - k_min + 2)), cv::FONT_HERSHEY_SIMPLEX, font_scale, cv::Scalar(0, 0, 255));

                            // ratio = detector->m_stats->at(k - k_min + 1)[row >> 1][column >> 1].above_E_x_x2__E_x2_t;
                            // stream.str("");
                            // stream << std::fixed << std::setprecision(1) << ratio;
                            // s = stream.str();
                            // cv::putText(frame, s, cv::Point(x, y + font_height * (k - k_min + 3)), cv::FONT_HERSHEY_SIMPLEX, font_scale, cv::Scalar(100, 0, 255));

                            // if (detector->ratio_threshold_low < ratio && ratio < detector->ratio_threshold_high) {
                            //     cv::rectangle(frame, cv::Rect(x, y, 1 << k, 1 << k), cv::Scalar(0, 255, 0), k_rel + 0.5);
                            // }
                        // }
                    }
                }
                if (config.compute_pitch_roll && detection_any) {
                    const auto &gstats = detector->m_global_stats->at(p)[k - k_min];
                    double mean_x = gstats.ema_x;
                    double mean_y = gstats.ema_y;
                    double Mxx    = gstats.ema_x2 - mean_x * mean_x;
                    double Myy    = gstats.ema_y2 - mean_y * mean_y;
                    double Mxy    = gstats.ema_xy - mean_x * mean_y;
                    double trace2 = 0.5 * (Mxx + Myy);
                    double diff2  = 0.5 * std::sqrt((Mxx - Myy) * (Mxx - Myy) + 4.0 * Mxy * Mxy);
                    double lambda1 = trace2 + diff2;
                    double lambda2 = trace2 - diff2;
                    if (lambda1 > 0.0 && lambda2 > 0.0) {
                        double major = std::sqrt(lambda1);
                        double minor = std::sqrt(lambda2);
                        cv::Point center(static_cast<int>(mean_x), static_cast<int>(mean_y));
                        cv::Size axes(static_cast<int>(major), static_cast<int>(minor));
                        double angle = gstats.roll * 180.0 / 3.141592;
                        cv::ellipse(frame, center, axes, angle, 0.0, 360.0, cv::Scalar(255, 0, 0), 2);
                    }
                }
            }
        }
        window->show(frame);

        // create leading zeros for frame_idx
        // str_idx = std::to_string(frame_idx++);
        // if (str_idx.length() < width)
        // {
        //     str_idx.insert(0, width - str_idx.length(), '0');
        // }
        //
        // cv::imwrite("c:/Users/radio/data/periodic/videos/" + str_idx + ".png", frame);
    });
}

bool pathExists(hid_t id, const std::string& path)
{
    return H5Lexists( id, path.c_str(), H5P_DEFAULT ) > 0;
}


void load_start_and_end_us_from_rpm_alignment_if_available(Config& config)
{
    // if there exists file with the same name, in the subdirectory "rpm_alignment" of the recording_filepath directory, load the file
    // and set config.start_us and config.end_us to the third value of the second line and to the fourth value of the last line, respectively
    std::string recording_dir = std::filesystem::path(config.recording_filepath).parent_path().string();
    std::string recording_filename = std::filesystem::path(config.recording_filepath).filename().string();
    std::string recording_filename_no_ext = recording_filename.substr(0, recording_filename.find_last_of("."));
    std::string recording_dir_rpm_alignment = recording_dir + "/rpm_alignment";
    std::string recording_filepath_rpm_alignment = recording_dir_rpm_alignment + "/" + recording_filename_no_ext + ".csv";
    std::cout << recording_filepath_rpm_alignment << std::endl;
    if (std::filesystem::exists(recording_filepath_rpm_alignment))
    {
        std::ifstream rpm_alignment_file(recording_filepath_rpm_alignment);
        std::string line;
        std::vector<std::string> lines;
        while (std::getline(rpm_alignment_file, line))
        {
            lines.push_back(line);
        }
        std::istringstream ss;
        std::string token;
        if (lines.size() >= 1000)
        {
            ss = std::istringstream(lines[1000]);
            std::getline(ss, token, ',');
            std::getline(ss, token, ',');
            std::getline(ss, token, ',');
            config.start_us = std::stoll(token);
        }
        if (lines.size() >= 1000)
        {
            ss = std::istringstream(lines[lines.size() - 1000]);
            std::getline(ss, token, ',');
            std::getline(ss, token, ',');
            std::getline(ss, token, ',');
            config.end_us = std::stoll(token);
        }
        std::cout << "Loaded from rpm_alignment csv start_us: " << config.start_us << ", end_us: " << config.end_us << std::endl;
    }
}


int main(int argc, char *argv[]) {
    std::string config_filepath = "config.yaml";
    if (argc >= 2) {
        config_filepath = argv[1];
    }
    YAML::Node yaml_config;
    try {
        yaml_config = YAML::LoadFile(config_filepath);
    } catch (const YAML::BadFile &e) {
        std::cout << e.what() << std::endl;
        exit(-1);
    } catch (const YAML::ParserException &e) {
        std::cout << e.what() << std::endl;
        exit(-1);
    }

    Config config = Config(yaml_config);

    if (config.start_us == 0 && config.end_us == 0)
    {
        load_start_and_end_us_from_rpm_alignment_if_available(config);
    }

    TensorBoardLogger logger(config.tensorboard_log_file);

    Metavision::Camera camera;
    try {
        if (!config.recording_filepath.empty()) {
            Metavision::FileConfigHints file_config_hints;
            file_config_hints.real_time_playback(config.simulate_real_time);
            file_config_hints.time_shift(false);
            camera = Metavision::Camera::from_file(config.recording_filepath, file_config_hints);
        } else {
            camera = Metavision::Camera::from_first_available();
        }
    } catch (Metavision::CameraException &e) {
        // MV_LOG_ERROR() << e.what();
        return 2;
    }

    const int camera_width  = camera.geometry().width();
    const int camera_height = camera.geometry().height();
    const int npixels       = camera_width * camera_height;

    // now we can create our frame generator using previous variables
    auto frame_gen = Metavision::PeriodicFrameGenerationAlgorithm(camera_width, camera_height, config.acc, config.fps,
        Metavision::ColorPalette::Dark);

    long long first_timestep = -1;
    long long last_timestep = -1;
    camera.cd().add_callback([&](const Metavision::EventCD *begin, const Metavision::EventCD *end) {
        if (first_timestep == -1) {
            first_timestep = begin->t;
        }
        // periodic_phenomena_processor->addEventsAsync(begin, end);
        last_timestep = (end - 1)->t;
    });

    auto detector = std::make_shared<Detector>(logger, config);

    // we add the callback that will pass the events to the frame generator
    camera.cd().add_callback([&](const Metavision::EventCD *begin, const Metavision::EventCD *end) {
        if (config.scale_down_factor == 1.0 && config.rotation_angle == 0)
        {
            frame_gen.process_events(begin, end);
            detector->addEventsAsync(begin, end);
        } else
        {
            // clone array, sample randomly to reduce the number of events to 25%
            std::vector<Metavision::EventCD> events;
            events.reserve(end - begin);
            for (auto it = begin; it != end; ++it) {
                if (rand() % static_cast<int>(config.scale_down_factor * config.scale_down_factor) == 0) {
                    events.push_back(*it);
                }
            }

            // change events to x / sdf, y / sdf
            for (auto &event : events) {
                if (config.rotation_angle != 0)
                {
                    rotatePixel(event, camera_width, camera_height, config.rotation_angle);
                }

                event.x /= config.scale_down_factor;
                event.y /= config.scale_down_factor;
            }
            frame_gen.process_events(events.data(), events.data() + events.size());
            detector->addEventsAsync(events.data(), events.data() + events.size());
        }
    });

    bool should_stop = false;
    if (config.end_us > 0)
    {
        camera.cd().add_callback([&](const Metavision::EventCD *begin, const Metavision::EventCD *end) {
            if (begin->t >= config.end_us)
            {
                should_stop = true;
            }
        });
    }


    Metavision::Window * window = nullptr;
    if (config.mode == CONFIG_MODE_VISUAL)
    {
        window = new Metavision::Window("Recording", camera_width, camera_height, Metavision::BaseWindow::RenderMode::BGR);
        window->set_keyboard_callback(
        [&, window](Metavision::UIKeyEvent key, int scancode, Metavision::UIAction action, int mods)
        {
            if (action == Metavision::UIAction::RELEASE &&
                (key == Metavision::UIKeyEvent::KEY_ESCAPE || key == Metavision::UIKeyEvent::KEY_Q))
            {
                window->set_close_flag();
            }
        });
        initialize_detector_visual_mode(
            config, window,
            frame_gen, detector);
    }

    boost::timer::auto_cpu_timer timer;
    auto start_time = timer.elapsed().wall;

    detector->start(1);

    if (config.start_us > 0)
    {
        while (!camera.offline_streaming_control().seek(config.start_us))
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
    camera.start();

    // keep running until the camera is off, the recording is finished or the escape key was pressed
    while (camera.is_running() && (window == nullptr || !window->should_close()) && !should_stop) {
        if (config.mode == CONFIG_MODE_VISUAL)
        {
            // we poll events (keyboard, mouse etc.) from the system with a 20ms sleep to avoid using 100% of a CPU's core
            // and we push them into the window where the callback on the escape key will ask the windows to close
            static constexpr std::int64_t kSleepPeriodMs = 20;
            Metavision::EventLoop::poll_and_dispatch(kSleepPeriodMs);
        }
    }
    camera.stop();

    if (!config.is_quiet)
        std::cout << "remaining jobs count: " << detector->jobsCount() << std::endl;

    while (detector->jobsCount() > 0) {
        if (!config.is_quiet)
        {
            std::cout << detector->jobsCount() << std::flush;
            std::cout << "\r"; // Move cursor one line up
        }
        if (config.mode == CONFIG_MODE_VISUAL)
        {
            static constexpr std::int64_t kSleepPeriodMs = 20;
            Metavision::EventLoop::poll_and_dispatch(kSleepPeriodMs);
        }
    }

    auto end_time = timer.elapsed().wall;

    if (config.is_runtime_analysis)
    {
        detector->storeRunTime(start_time, end_time);
    }
    detector->join();
    detector->stop();

    delete window;

    // for (int i = 0; i < frame_idx; ++i)
    // {
    //     str_idx = std::to_string(i);
    //     if (str_idx.length() < width)
    //     {
    //         str_idx.insert(0, width - str_idx.length(), '0');
    //     }
    //     cv::imwrite("c:/Users/radio/data/periodic/videos/" + str_idx + ".png", frames[i]);
    // }

    return 0;
}
