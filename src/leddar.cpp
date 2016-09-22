#include <ros/ros.h>
#include <ros/console.h>
#include <angles/angles.h>
#include <leddar/ScanConfig.h>
#include <sensor_msgs/LaserScan.h>
#include <dynamic_reconfigure/server.h>

#include <cstddef>
#include <string>
#include <stdexcept>
#include <boost/format.hpp>

#include <LeddarC.h>
#include <LeddarProperties.h>

#define MAX_AMPLITUDE 1024

// Leddar handle.
static LeddarHandle handle = NULL;

// Leddar specifications.
static std::string frame;
static double max_range;
static double field_of_view;
static int beam_count;
static double angle_min;
static double angle_max;
static double angle_increment;
static double scan_time;

// ROS publisher.
ros::Publisher pub;

static void handle_detections(void *handle) 
{
    // Get the number of available detections
    unsigned int count = LeddarGetDetectionCount(handle);

    // Acquire detections from Leddar.
    LdDetection detections[count];
    int code = LeddarGetDetections(handle, detections, count);
    if (code != LD_SUCCESS)
    {
        ROS_ERROR("failed to get detections: %d", code);
        return;
    }

    // Construct LaserScan message.
    sensor_msgs::LaserScan msg;
    msg.header.frame_id = frame;
    msg.header.stamp = ros::Time::now();

    // Set up field of view.
    msg.angle_min = angle_min;
    msg.angle_max = angle_max;
    msg.angle_increment = angle_increment;
    msg.range_min = 0.0;
    msg.range_max = max_range;
    msg.scan_time = scan_time;

    // Load detection and amplitudes into message.
    msg.ranges = sensor_msgs::LaserScan::_ranges_type(beam_count, max_range + 1);
    msg.intensities = sensor_msgs::LaserScan::_intensities_type(beam_count, 0.0);
    for (int i = 0; i < count; i++)
        if (detections[i].mSegment < beam_count) {
            if (detections[i].mDistance < msg.ranges[detections[i].mSegment])
            {
                msg.ranges[detections[i].mSegment] = detections[i].mDistance;
                msg.intensities[detections[i].mSegment] = detections[i].mAmplitude / MAX_AMPLITUDE;
            }
        }
        else
            ROS_ERROR("invalid segment: %d", detections[i].mSegment);

    // Publish and keep going.
    pub.publish(msg);
}

double getDoubleProperty(void *handle, LdProperties property_id, int index = 0)
{
    double value;
    int code = LeddarGetProperty(handle, property_id, index, &value);
    if (code != LD_SUCCESS)
        throw std::runtime_error("failed to get property: " + (boost::format("0x%08x") % code).str() + std::to_string(code));
    return value;
}

int getIntProperty(void *handle, LdProperties property_id, int index = 0)
{
    double value;
    int code = LeddarGetProperty(handle, property_id, index, &value);
    if (code != LD_SUCCESS)
        throw std::runtime_error("failed to get property: " + (boost::format("0x%08x") % code).str() + std::to_string(code));
    return static_cast<int>(value);
}

std::string getTextProperty(void *handle, LdProperties property_id)
{
    char value[256];
    int code = LeddarGetTextProperty(handle, property_id, 0, value);
    if (code != LD_SUCCESS)
        throw std::runtime_error("failed to get property: " + (boost::format("0x%08x") % code).str() + std::to_string(code));
    return value;
}

void configure_callback(leddar::ScanConfig &config, uint32_t level) 
{
    ROS_INFO("Reconfiguring...");

    // Set relative intensity of LEDs.
    ROS_DEBUG("INTENSITY: %d", config.intensity);
    LeddarSetProperty(handle, PID_LED_INTENSITY, 0, config.intensity);
    
    // Set automatic LED intensity.
    ROS_DEBUG("ACQ OPTIONS: %d", config.acq_options);
    LeddarSetProperty(handle, PID_ACQ_OPTIONS, 0, config.acq_options);

    // Set number of accumulations to perform.
    ROS_DEBUG("ACCUMULATIONS: %d", config.accumulations);
    LeddarSetProperty(handle, PID_ACCUMULATION_EXPONENT, 0, config.accumulations);

    // Set number of oversamplings to perform between base samples.
    ROS_DEBUG("OVERSAMPLING: %d", config.oversampling);
    LeddarSetProperty(handle, PID_OVERSAMPLING_EXPONENT, 0, config.oversampling);

    // Set number of base samples acquired.
    ROS_DEBUG("BASE SAMPLES: %d", config.base_point_count);
    LeddarSetProperty(handle, PID_BASE_POINT_COUNT, 0, config.base_point_count);

    // Set offset to increase detection threshold.
    ROS_DEBUG("THRESHOLD OFFSET: %d", config.threshold_offset);
    LeddarSetProperty(handle, PID_THRESHOLD_OFFSET, 0, config.threshold_offset);

    // Write changes to Leddar.
    LeddarWriteConfiguration(handle);

    // Update scan time
    scan_time = (1 << config.accumulations) * (1 << config.oversampling - 1) / 12800.0;
}

static void connect(LeddarHandle handle) 
{
    char connectionType[10] = "USB";
    char addresses[256] = "USB";
    LeddarU32 count = sizeof(addresses);

    LeddarListSensors(addresses, &count);
    if (count == 0)
        throw std::runtime_error("could not find leddar tech usb device");

    int code = LeddarConnect(handle, connectionType, addresses);
    if (code != LD_SUCCESS)
        throw std::runtime_error("Failed to connect to " + std::string(addresses) + " with code: " + std::to_string(code));

    ROS_INFO("Connected to %s", addresses);
}


int main(int argc, char** argv)
 {
    int code;
    int status = 0;

    try 
    {

        // Initialize node.
        ros::init(argc, argv, "leddar");
        ros::NodeHandle nh("~");

        // Initialize publisher.
        pub = nh.advertise<sensor_msgs::LaserScan>(std::string("scan"), 1);

        // Initialize Leddar handle.
        handle = LeddarCreate();

        // Connect to Leddar Tech device.
        connect(handle);

        // Get Leddar specifications.
        if (!nh.hasParam("range"))
            throw std::runtime_error("~range parameter not set");
        if (!nh.hasParam("frame"))
            throw std::runtime_error("~frame parameter not set");

        nh.getParam("range", max_range);
        nh.getParam("frame", frame);

        beam_count = getIntProperty(handle, PID_SEGMENT_COUNT);
        field_of_view = getDoubleProperty(handle, PID_FIELD_OF_VIEW);
        angle_min = angles::from_degrees(-field_of_view / 2.0);
        angle_max = angles::from_degrees(field_of_view / 2.0);
        angle_increment = angles::from_degrees(field_of_view / beam_count);

        // Set up dynamic_reconfigure server and callback.
        dynamic_reconfigure::Server<leddar::ScanConfig> server;
        dynamic_reconfigure::Server<leddar::ScanConfig>::CallbackType f;
        f = boost::bind(&configure_callback, _1, _2);
        server.setCallback(f);

        bool transfer_started = false;
        while (ros::ok())
        {
            // Enable data transfer is disabled
            if (!transfer_started)
            {
                ROS_INFO("starting data transfers");
                code = LeddarStartDataTransfer(handle, LDDL_DETECTIONS);
                if (code != LD_SUCCESS)
                    throw std::runtime_error("failed to start detect transfers: " + std::to_string(code));
                transfer_started = true;
            }

            // Wait for detection to be available
            code = LeddarWaitForData(handle, 160);
            if (code != LD_SUCCESS)
            {
                ROS_INFO("timeout, stopping data transfers");
                code = LeddarStopDataTransfer(handle);
                if (code != LD_SUCCESS)
                    throw std::runtime_error("failed to stop detect transfers: " + std::to_string(code));
                transfer_started = false;
            }

            // Publish the detections
            handle_detections(handle);

            ros::spinOnce();
        }
    }
    catch (std::runtime_error& e)
    {
        ROS_FATAL("%s", e.what());
        status = 1;
    }

    // Clean up.
    LeddarStopDataTransfer(handle);
    LeddarDisconnect(handle);
    LeddarDestroy(handle);

    return status;
}
