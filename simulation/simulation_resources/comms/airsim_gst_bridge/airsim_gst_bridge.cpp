// airsim_gst_bridge
// -----------------------------------------------------------------------------
// AirSim camera -> H.264/RTP -> udp:<ip>:5600 shim.
//
// This is the AirSim counterpart of comms/gz_gst_bridge: it produces the EXACT
// same RTP/H.264 stream on udp:<ip>:5600 that yolo_py's x86_64 simulation
// pipeline already ingests (see aircraft/.../yolo_py/yolo_node.py, the
// "udpsrc port=5600 ... rtph264depay ... avdec_h264" pipeline). yolo_py needs
// no changes -- only the *source* of the frames moves from Gazebo to AirSim.
//
// Frames are received as a ROS 2 sensor_msgs/Image, published by the
// TEVV-Airsim-ROS2-Bridge for an AirSim camera (e.g. /Drone1/front_center/image).
// The encode/RTP/udpsink half is copied verbatim from gz_gst_bridge so the wire
// format and yolo_py decoder stay identical.
//
// Usage (mirrors gz_gst_bridge's positional CLI):
//   airsim_gst_bridge <ros_image_topic> <ip> [port=5600] [framerate=8] [--ros-args ...]
// -----------------------------------------------------------------------------

#include <iostream>
#include <string>
#include <memory>
#include <cstring>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <gst/gst.h>
#include <gst/app/gstappsrc.h>

// Global GStreamer objects (mirrors gz_gst_bridge structure).
GstElement *pipeline = nullptr;
GstElement *appsrc = nullptr;
bool configured = false;

// Command line args
std::string IMAGE_TOPIC = "/Drone1/front_center/image";
std::string TARGET_IP = "127.0.0.1";
int TARGET_PORT = 5600;
int FRAMERATE = 8;  // matches sensor_config.yaml camera_intrinsics.update_rate

// Map a ROS 2 sensor_msgs/Image encoding to a GStreamer raw-video format.
// videoconvert downstream handles the conversion to whatever the encoder wants.
static const char *gst_format_for_encoding(const std::string &encoding) {
    if (encoding == "rgb8")  return "RGB";
    if (encoding == "bgr8")  return "BGR";
    if (encoding == "rgba8") return "RGBA";
    if (encoding == "bgra8") return "BGRA";
    if (encoding == "mono8") return "GRAY8";
    return "RGB";  // AirSim Scene captures are RGB by default
}

void on_frame(const sensor_msgs::msg::Image::SharedPtr msg) {
    if (!appsrc) return;

    // Configure caps on the first frame from the image geometry/encoding.
    if (!configured) {
        GstCaps *caps = gst_caps_new_simple("video/x-raw",
            "format", G_TYPE_STRING, gst_format_for_encoding(msg->encoding),
            "width", G_TYPE_INT, static_cast<int>(msg->width),
            "height", G_TYPE_INT, static_cast<int>(msg->height),
            "framerate", GST_TYPE_FRACTION, FRAMERATE, 1,
            NULL);
        gst_app_src_set_caps(GST_APP_SRC(appsrc), caps);
        gst_caps_unref(caps);

        std::cout << "Configured GStreamer for " << msg->width << "x" << msg->height
                  << " (" << msg->encoding << ")" << std::endl;
        configured = true;
    }

    // Copy the pixel data from the ROS 2 message into a GStreamer buffer.
    gsize size = msg->data.size();
    GstBuffer *buffer = gst_buffer_new_allocate(NULL, size, NULL);
    if (buffer) {
        gst_buffer_fill(buffer, 0, msg->data.data(), size);
        GstFlowReturn ret = gst_app_src_push_buffer(GST_APP_SRC(appsrc), buffer);
        if (ret != GST_FLOW_OK) {
            g_warning("Error pushing buffer to appsrc");
        }
    } else {
        g_warning("Failed to allocate GStreamer buffer");
    }
}

bool check_nvidia_encoder() {
    GstElementFactory *factory = gst_element_factory_find("nvh264enc");
    if (factory) {
        gst_object_unref(factory);
        return true;
    }
    return false;
}

int main(int argc, char **argv) {
    if (argc < 3) {
        std::cerr << "Usage: ./airsim_gst_bridge <ros_image_topic> <ip> [port] [framerate] [--ros-args ...]\n";
        return 1;
    }
    // Parse positional args before ROS sees them.
    IMAGE_TOPIC = argv[1];
    TARGET_IP = argv[2];
    if (argc > 3 && std::strncmp(argv[3], "--", 2) != 0) TARGET_PORT = std::stoi(argv[3]);
    if (argc > 4 && std::strncmp(argv[4], "--", 2) != 0) FRAMERATE = std::stoi(argv[4]);

    // Initialize GStreamer and ROS 2 (rclcpp ignores the leading positional args).
    gst_init(&argc, &argv);
    rclcpp::init(argc, argv);

    // Build the pipeline string -- IDENTICAL encode/RTP/udpsink to gz_gst_bridge,
    // so the udp:5600 stream is byte-compatible with yolo_py's decoder.
    std::string ip_port = "host=" + TARGET_IP + " port=" + std::to_string(TARGET_PORT);
    std::string pipeline_str;
    if (check_nvidia_encoder()) {
        std::cout << "Using NVIDIA GPU Encoder (nvh264enc)\n";
        pipeline_str = "appsrc name=airsim_source ! queue max-size-buffers=1 leaky=downstream ! "
                       "videoconvert ! "
                       "nvh264enc preset=low-latency-hq zerolatency=true rc-mode=cbr bitrate=2048 qp-min=15 qp-max=35 gop-size=30 ! "
                       "rtph264pay config-interval=1 mtu=1400 ! udpsink sync=false " + ip_port;
    } else {
        std::cout << "Using CPU Encoder (x264enc)\n";
        pipeline_str = "appsrc name=airsim_source ! queue max-size-buffers=1 leaky=downstream ! "
                       "videoconvert ! "
                       "x264enc speed-preset=ultrafast tune=zerolatency bitrate=2048 key-int-max=30 ! "
                       "rtph264pay config-interval=1 mtu=1400 ! udpsink sync=false " + ip_port;
    }

    GError *error = nullptr;
    pipeline = gst_parse_launch(pipeline_str.c_str(), &error);
    if (!pipeline) {
        std::cerr << "Failed to create pipeline: " << (error ? error->message : "unknown") << std::endl;
        return -1;
    }

    appsrc = gst_bin_get_by_name(GST_BIN(pipeline), "airsim_source");
    g_object_set(appsrc, "format", GST_FORMAT_TIME, NULL);
    g_object_set(appsrc, "is-live", TRUE, NULL);
    g_object_set(appsrc, "do-timestamp", TRUE, NULL);
    g_object_set(appsrc, "leaky-type", 2, NULL);  // drop old frames
    g_object_set(appsrc, "max-bytes", 0, NULL);
    g_object_set(appsrc, "max-buffers", 2, NULL);
    gst_element_set_state(pipeline, GST_STATE_PLAYING);

    // The GStreamer pipeline runs in its own threads; rclcpp::spin drives the
    // image callback. No GLib main loop needed (unlike gz_gst_bridge, which had
    // no other event loop to block on).
    auto node = std::make_shared<rclcpp::Node>("airsim_gst_bridge");
    auto sub = node->create_subscription<sensor_msgs::msg::Image>(
        IMAGE_TOPIC, rclcpp::SensorDataQoS(), on_frame);

    std::cout << "Streaming " << IMAGE_TOPIC << " to " << TARGET_IP << ":" << TARGET_PORT
              << " at " << FRAMERATE << " FPS." << std::endl;

    rclcpp::spin(node);

    // Cleanup
    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(appsrc);
    gst_object_unref(pipeline);
    rclcpp::shutdown();
    return 0;
}
