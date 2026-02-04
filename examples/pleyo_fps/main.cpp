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

    // Configuration using manual integer values to avoid undeclared identifiers
    k4a_device_configuration_t config;
    config.color_format = (k4a_image_format_t)0;         // K4A_IMAGE_FORMAT_COLOR_MJPEG
    config.color_resolution = (k4a_color_resolution_t)2; // K4A_COLOR_RESOLUTION_1080P
    config.depth_mode = (k4a_depth_mode_t)2;             // K4A_DEPTH_MODE_NFOV_UNBINNED
    config.camera_fps = (k4a_fps_t)2;                    // K4A_FRAMES_PER_SECOND_30
    config.synchronized_images_only = true;
    config.depth_delay_off_color_usec = 0;
    config.wired_sync_mode = (k4a_wired_sync_mode_t)0; // K4A_WIRED_SYNC_MODE_STANDALONE
    config.subordinate_delay_off_master_usec = 0;
    config.disable_streaming_indicator = false;

    // 1. Start all cameras
    for (uint32_t i = 0; i < device_count; i++)
    {
        if (K4A_RESULT_SUCCEEDED != k4a_device_open(i, &devices[i]))
            continue;
        if (K4A_RESULT_SUCCEEDED != k4a_device_start_cameras(devices[i], &config))
        {
            k4a_device_close(devices[i]);
            devices[i] = NULL;
        }
    }

    ofstream log_file;
    if (should_log_to_file)
    {
        log_file.open(log_file_path, ios::out | ios::app);
    }

    // 2. Process each camera for FPS
    for (uint32_t i = 0; i < device_count; i++)
    {
        if (devices[i] == NULL)
            continue;
        string serial = get_serial(devices[i]);
        k4a_capture_t capture = NULL;

        int frames = 0;
        const int frames_to_test = 30;

        // Wait for the very first frame to bypass loading/cold-start time
        k4a_wait_result_t wait_result = k4a_device_get_capture(devices[i], &capture, 5000);
        if (wait_result != K4A_WAIT_RESULT_SUCCEEDED)
        {
            cerr << "Failed to receive first frame for camera " << serial << endl;
            continue;
        }
        k4a_capture_release(capture);

        // Start timer AFTER the first frame is received
        auto start = chrono::high_resolution_clock::now();

        while (frames < frames_to_test)
        {
            if (k4a_device_get_capture(devices[i], &capture, 1000) == K4A_WAIT_RESULT_SUCCEEDED)
            {
                frames++;
                k4a_capture_release(capture);
            }
            else
            {
                break;
            }
        }

        auto end = chrono::high_resolution_clock::now();
        double duration = chrono::duration<double>(end - start).count();
        double fps = (duration > 0) ? (double)frames / duration : 0;

        if (should_log_to_file && log_file.is_open())
        {
            log_file << "Camera: " << serial << " | FPS: " << fps << endl;
        }
        else
        {
            cout << "[LOG] " << serial << ": " << fps << " FPS" << endl;
        }
    }

    if (should_log_to_file)
    {
        log_file.close();
    }

    // 3. Shut down cameras
    for (uint32_t i = 0; i < device_count; i++)
    {
        if (devices[i])
        {
            k4a_device_stop_cameras(devices[i]);
            k4a_device_close(devices[i]);
        }
    }

    return 0;
}