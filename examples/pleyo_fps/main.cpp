#include <iostream>
#include <fstream>
#include <vector>
#include <chrono>
#include <thread>
#include <string>
#include <mutex>

// Include the standard header
#include <k4a/k4a.h>

using namespace std;

// Helper to get serial number
static string get_serial(k4a_device_t device)
{
    size_t serial_number_length = 0;
    if (k4a_device_get_serialnum(device, NULL, &serial_number_length) != K4A_BUFFER_RESULT_TOO_SMALL)
    {
        return "Unknown_Serial";
    }
    vector<char> serial_number(serial_number_length);
    if (k4a_device_get_serialnum(device, serial_number.data(), &serial_number_length) != K4A_BUFFER_RESULT_SUCCEEDED)
    {
        return "Unknown_Serial";
    }
    return string(serial_number.data());
}

int main(int argc, char **argv)
{
    string log_file_path = (argc >= 2) ? argv[1] : "";
    bool should_log_to_file = !log_file_path.empty();
    mutex log_mutex;

    uint32_t device_count = k4a_device_get_installed_count();
    if (device_count == 0)
    {
        cout << "No devices found" << endl;
        return 0;
    }

    // Configuration setup (WFOV Binned = 512x512)
    k4a_device_configuration_t config;
    config.color_format = (k4a_image_format_t)0;         // K4A_IMAGE_FORMAT_COLOR_MJPEG
    config.color_resolution = (k4a_color_resolution_t)2; // K4A_COLOR_RESOLUTION_1080P
    config.depth_mode = (k4a_depth_mode_t)3;             // K4A_DEPTH_MODE_WFOV_2X2BINNED
    config.camera_fps = (k4a_fps_t)2;                    // K4A_FRAMES_PER_SECOND_30
    config.synchronized_images_only = false;
    config.depth_delay_off_color_usec = 0;
    config.wired_sync_mode = (k4a_wired_sync_mode_t)0;
    config.subordinate_delay_off_master_usec = 0;
    config.disable_streaming_indicator = false;

    const chrono::milliseconds connect_delay(200);
    const chrono::milliseconds close_delay(100);

    ofstream log_file;
    if (should_log_to_file)
    {
        log_file.open(log_file_path, ios::out | ios::trunc);
    }

    auto log_line = [&](const string &line) {
        lock_guard<mutex> lock(log_mutex);
        cout << line << endl;
        if (should_log_to_file && log_file.is_open())
        {
            log_file << line << endl;
        }
    };

    // 1. Process each camera (in parallel)
    vector<thread> workers;
    workers.reserve(device_count);

    for (uint32_t i = 0; i < device_count; i++)
    {
        workers.emplace_back([&, i]() {
            k4a_device_t device = NULL;
            if (K4A_RESULT_SUCCEEDED != k4a_device_open(i, &device))
            {
                log_line("[WARN] Failed to open camera index " + to_string(i));
                return;
            }

            log_line("[INFO] Camera " + to_string(i) + " connected. Waiting for stabilization...");
            this_thread::sleep_for(connect_delay);

            if (K4A_RESULT_SUCCEEDED != k4a_device_start_cameras(device, &config))
            {
                log_line("[WARN] Failed to start cameras for index " + to_string(i));
                k4a_device_close(device);
                return;
            }
            k4a_device_start_imu(device);

            string serial = get_serial(device);
            k4a_capture_t capture = NULL;
            uint64_t last_ts_color = 0, last_ts_depth = 0, last_ts_ir = 0;

            // --- FLUSH BUFFER ---
            // We empty the accumulated frames in the USB buffer to ensure real-time measurement
            log_line("[INFO] Flushing buffer for camera " + serial + "...");
            for (int flush = 0; flush < 20; flush++)
            {
                if (k4a_device_get_capture(device, &capture, 0) == K4A_WAIT_RESULT_SUCCEEDED)
                {
                    k4a_capture_release(capture);
                }
            }

            // 10 batches of 10 unique frames
            for (int batch = 1; batch <= 10; batch++)
            {
                int count_color = 0, count_depth = 0, count_ir = 0;

                // Wait for one fresh frame before starting the timer
                if (k4a_device_get_capture(device, &capture, 2000) == K4A_WAIT_RESULT_SUCCEEDED)
                {
                    k4a_capture_release(capture);
                }

                auto start_time = chrono::high_resolution_clock::now();

                while (count_color < 10 || count_depth < 10 || count_ir < 10)
                {
                    if (k4a_device_get_capture(device, &capture, 1000) == K4A_WAIT_RESULT_SUCCEEDED)
                    {

                        // Independent Check for Color
                        k4a_image_t img_color = k4a_capture_get_color_image(capture);
                        if (img_color != NULL)
                        {
                            uint64_t ts = k4a_image_get_device_timestamp_usec(img_color);
                            if (ts != last_ts_color)
                            {
                                count_color++;
                                last_ts_color = ts;
                            }
                            k4a_image_release(img_color);
                        }

                        // Independent Check for Depth
                        k4a_image_t img_depth = k4a_capture_get_depth_image(capture);
                        if (img_depth != NULL)
                        {
                            uint64_t ts = k4a_image_get_device_timestamp_usec(img_depth);
                            if (ts != last_ts_depth)
                            {
                                count_depth++;
                                last_ts_depth = ts;
                            }
                            k4a_image_release(img_depth);
                        }

                        // Independent Check for Infrared
                        k4a_image_t img_ir = k4a_capture_get_ir_image(capture);
                        if (img_ir != NULL)
                        {
                            uint64_t ts = k4a_image_get_device_timestamp_usec(img_ir);
                            if (ts != last_ts_ir)
                            {
                                count_ir++;
                                last_ts_ir = ts;
                            }
                            k4a_image_release(img_ir);
                        }
                        k4a_capture_release(capture);
                    }
                    else
                    {
                        break;
                    }
                }

                auto end_time = chrono::high_resolution_clock::now();
                double duration = chrono::duration<double>(end_time - start_time).count();

                double fps_color = (duration > 0) ? (double)count_color / duration : 0;
                double fps_depth = (duration > 0) ? (double)count_depth / duration : 0;
                double fps_ir = (duration > 0) ? (double)count_ir / duration : 0;

                string log_entry = "Camera: " + serial + " | Batch: " + to_string(batch) +
                                   " | Color: " + to_string(fps_color) + " FPS | Depth: " + to_string(fps_depth) +
                                   " FPS | IR: " + to_string(fps_ir) + " FPS";

                log_line("[LOG] " + log_entry);

                // IMU data heartbeat
                k4a_imu_sample_t imu_sample;
                k4a_device_get_imu_sample(device, &imu_sample, 0);
            }

            k4a_device_stop_imu(device);
            k4a_device_stop_cameras(device);
            log_line("[INFO] Streams stopped for camera " + serial + ". Waiting before close...");
            this_thread::sleep_for(close_delay);
            k4a_device_close(device);
        });
    }

    for (auto &worker : workers)
    {
        if (worker.joinable())
        {
            worker.join();
        }
    }

    if (should_log_to_file)
        log_file.close();

    return 0;
}
