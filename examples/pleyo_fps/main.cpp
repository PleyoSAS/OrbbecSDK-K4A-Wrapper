#include <iostream>
#include <fstream>
#include <vector>
#include <chrono>
#include <thread>
#include <string>

// Include the standard header
#include <k4a/k4a.h>

using namespace std;

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

    uint32_t device_count = k4a_device_get_installed_count();
    if (device_count == 0)
    {
        cout << "No devices found" << endl;
        return 0;
    }

    vector<k4a_device_t> devices(device_count);

    // Configuration: Color 1080p MJPG, Depth WFOV Binned (512x512)
    k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    config.color_format = (k4a_image_format_t)0;         // MJPEG
    config.color_resolution = (k4a_color_resolution_t)2; // 1080P
    config.depth_mode = (k4a_depth_mode_t)1;             // WFOV_2X2BINNED (512x512)
    config.camera_fps = (k4a_fps_t)2;                    // 30 FPS
    config.synchronized_images_only = true;              // Ensures aligned frames

    // 1. Start all cameras and IMU
    for (uint32_t i = 0; i < device_count; i++)
    {
        if (K4A_RESULT_SUCCEEDED != k4a_device_open(i, &devices[i]))
            continue;

        // Start Cameras
        if (K4A_RESULT_SUCCEEDED != k4a_device_start_cameras(devices[i], &config))
        {
            k4a_device_close(devices[i]);
            devices[i] = NULL;
            continue;
        }

        // Start IMU
        k4a_device_start_imu(devices[i]);
    }

    ofstream log_file;
    if (should_log_to_file)
    {
        log_file.open(log_file_path, ios::out | ios::trunc);
    }

    // 2. Process each camera
    for (uint32_t i = 0; i < device_count; i++)
    {
        if (devices[i] == NULL)
            continue;
        string serial = get_serial(devices[i]);
        k4a_capture_t capture = NULL;

        // Internal tracking for unique timestamps per stream
        uint64_t last_ts_color = 0, last_ts_depth = 0, last_ts_ir = 0;

        // Bypass loading frame
        if (k4a_device_get_capture(devices[i], &capture, 5000) == 0)
        {
            k4a_capture_release(capture);
        }

        for (int batch = 1; batch <= 6; batch++)
        {
            int valid_frames = 0;
            auto start = chrono::high_resolution_clock::now();

            while (valid_frames < 10)
            {
                if (k4a_device_get_capture(devices[i], &capture, 1000) == 0)
                {
                    // Extract images to check timestamps
                    k4a_image_t img_color = k4a_capture_get_color_image(capture);
                    k4a_image_t img_depth = k4a_capture_get_depth_image(capture);
                    k4a_image_t img_ir = k4a_capture_get_ir_image(capture);

                    uint64_t ts_color = k4a_image_get_device_timestamp_usec(img_color);
                    uint64_t ts_depth = k4a_image_get_device_timestamp_usec(img_depth);
                    uint64_t ts_ir = k4a_image_get_device_timestamp_usec(img_ir);

                    // Check if frames are new
                    if (ts_color != last_ts_color && ts_depth != last_ts_depth && ts_ir != last_ts_ir)
                    {
                        valid_frames++;
                        last_ts_color = ts_color;
                        last_ts_depth = ts_depth;
                        last_ts_ir = ts_ir;
                    }

                    k4a_image_release(img_color);
                    k4a_image_release(img_depth);
                    k4a_image_release(img_ir);
                    k4a_capture_release(capture);
                }
                else
                {
                    break;
                }
            }

            auto end = chrono::high_resolution_clock::now();
            double duration = chrono::duration<double>(end - start).count();
            double fps = (duration > 0) ? (double)valid_frames / duration : 0;

            // Log output (simultaneous for Color/Depth/IR since synchronized)
            string out = "Cam: " + serial + " | Batch: " + to_string(batch) + " | FPS: " + to_string(fps);
            cout << "[LOG] " << out << endl;
            if (should_log_to_file && log_file.is_open())
                log_file << out << endl;

            // IMU Data check (Log only a sample)
            k4a_imu_sample_t imu_sample;
            if (k4a_device_get_imu_sample(devices[i], &imu_sample, 0) == K4A_WAIT_RESULT_SUCCEEDED)
            {
                string imu_out = "Cam: " + serial + " | IMU Accel X: " + to_string(imu_sample.acc_sample.xyz.x);
                if (batch == 1)
                    cout << "[IMU] " << imu_out << endl; // Just a sample log
            }
        }
    }

    if (should_log_to_file)
        log_file.close();

    // 3. Cleanup
    for (uint32_t i = 0; i < device_count; i++)
    {
        if (devices[i])
        {
            k4a_device_stop_imu(devices[i]);
            k4a_device_stop_cameras(devices[i]);
            k4a_device_close(devices[i]);
        }
    }
    return 0;
}