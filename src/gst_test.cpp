#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <gst/app/gstappsink.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include "ffmpeg_image_transport_msgs/msg/ffmpeg_packet.hpp"
#include <fstream>
#include <thread>

class GstTest : public rclcpp::Node, public std::enable_shared_from_this<GstTest> {
public:
  GstTest(int argc, char* argv[], const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node("gst_test", options), error(NULL) {
      gst_init(&argc, &argv);

      pipeline = gst_parse_launch(
        "appsrc name=appsrc-video is-live=true ! queue ! videoconvert ! "
        "video/x-raw,format=I420,width=1920,height=1080,framerate=30/1 ! "
        "openh264enc ! h264parse ! video/x-h264,stream-format=byte-stream,alignment=au ! "
        "appsink name=appsink-video emit-signals=TRUE sync=FALSE",
        &error
      );

      if (error) {
        g_printerr("Error parsing pipeline: %s\n", error->message);
        g_clear_error(&error);
        return;
      }

      appsrc = gst_bin_get_by_name(GST_BIN(pipeline), "appsrc-video");
      appsink = gst_bin_get_by_name(GST_BIN(pipeline), "appsink-video");
      g_signal_connect(appsink, "new-sample", G_CALLBACK(on_new_sample), this);
      gst_object_unref(appsink);

      // Configure appsrc caps
      GstCaps *caps = gst_caps_new_simple(
        "video/x-raw",
        "format", G_TYPE_STRING, "I420",
        "width", G_TYPE_INT, 1920,
        "height", G_TYPE_INT, 1080,
        "framerate", GST_TYPE_FRACTION, 30, 1,
        NULL);
      gst_app_src_set_caps(GST_APP_SRC(appsrc), caps);
      gst_caps_unref(caps);

      gst_element_set_state(pipeline, GST_STATE_PLAYING);

      loop = g_main_loop_new(NULL, FALSE);
      bus = gst_element_get_bus(pipeline);
      gst_bus_add_watch(bus, (GstBusFunc)bus_call, loop);

      image_sub = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/image_raw", 10, std::bind(&GstTest::image_callback, this, std::placeholders::_1));

      // Publisher for the FFMPEGPacket
      ffmpeg_pub = this->create_publisher<ffmpeg_image_transport_msgs::msg::FFMPEGPacket>("ffmpeg_packet", 10);

      std::thread([this]() { g_main_loop_run(loop); }).detach();
    }

  ~GstTest() {
      gst_element_set_state(pipeline, GST_STATE_NULL);
      gst_object_unref(pipeline);
      gst_object_unref(bus);
      g_main_loop_unref(loop);
  }

private:
  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat bgr_frame = cv_ptr->image;
    cv::Mat yuv_frame;
    cv::cvtColor(bgr_frame, yuv_frame, cv::COLOR_BGR2YUV_I420);

    GstBuffer* buffer = gst_buffer_new_allocate(NULL, yuv_frame.total() * yuv_frame.elemSize(), NULL);
    GstMapInfo map;
    gst_buffer_map(buffer, &map, GST_MAP_WRITE);
    memcpy(map.data, yuv_frame.data, yuv_frame.total() * yuv_frame.elemSize());
    gst_buffer_unmap(buffer, &map);

    GstFlowReturn ret;
    g_signal_emit_by_name(appsrc, "push-buffer", buffer, &ret);
    gst_buffer_unref(buffer);

    if (ret != GST_FLOW_OK) {
      RCLCPP_ERROR(this->get_logger(), "Error pushing buffer to appsrc: %d", ret);
    }
  }

  static GstFlowReturn on_new_sample(GstElement *sink, gpointer user_data) {
    GstTest *self = static_cast<GstTest*>(user_data);

    GstSample *sample;
    GstBuffer *buffer;
    GstMapInfo map;

    sample = gst_app_sink_pull_sample(GST_APP_SINK(sink));
    buffer = gst_sample_get_buffer(sample);

    if (gst_buffer_map(buffer, &map, GST_MAP_READ)) {
        // Publish the buffer data
        self->publish_ffmpeg_packet(map.data, map.size);
        gst_buffer_unmap(buffer, &map);
    }

    gst_sample_unref(sample);
    return GST_FLOW_OK;
  }

  static gboolean bus_call(GstBus *bus, GstMessage *msg, gpointer data) {
    GMainLoop *loop = (GMainLoop *) data;

    switch (GST_MESSAGE_TYPE(msg)) {
        case GST_MESSAGE_ERROR: {
            GError *err;
            gchar *debug;
            gst_message_parse_error(msg, &err, &debug);
            g_printerr("Error: %s\n", err->message);
            g_error_free(err);
            g_free(debug);
            g_main_loop_quit(loop);
            break;
        }
        default:
            break;
    }
    return TRUE;
  }

  void publish_ffmpeg_packet(const guint8* data, gsize size) {
    auto packet = ffmpeg_image_transport_msgs::msg::FFMPEGPacket();

    packet.header.stamp = this->get_clock()->now();
    packet.header.frame_id = "camera_frame";
    packet.width = 1920;
    packet.height = 1080;
    packet.encoding = "h264";
    packet.pts = 0; // Set to appropriate presentation timestamp
    packet.flags = 0; // Set to appropriate flags
    packet.is_bigendian = false;
    packet.data = std::vector<uint8_t>(data, data + size);

    ffmpeg_pub->publish(packet);
  }

  GstElement *pipeline, *appsrc, *appsink;
  GError *error;
  GstBus *bus;
  GMainLoop *loop;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;
  rclcpp::Publisher<ffmpeg_image_transport_msgs::msg::FFMPEGPacket>::SharedPtr ffmpeg_pub;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto test = std::make_shared<GstTest>(argc, argv);
  rclcpp::spin(test);
  rclcpp::shutdown();
  return 0;
}
