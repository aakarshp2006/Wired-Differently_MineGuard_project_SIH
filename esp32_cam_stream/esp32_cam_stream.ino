/*
  ================================================================
  esp32_cam_stream.ino
  ----------------------------------------------------------------
  MineGuard - Smart Mine Vehicle Safety & Monitoring System
  ESP32-CAM (AI Thinker board, OV2640 sensor) Streaming Module
  ================================================================

  ROLE OF THIS DEVICE IN THE MINEGUARD ARCHITECTURE:

      ESP32-CAM  --(Wi-Fi MJPEG stream)-->  Laptop (Python + OpenCV)
                                                   |
                                                   v
                                          YOLO Object Detection
                                                   |
                                                   v
                                            Fog Detection
                                                   |
                                                   v
                                     MineGuard Integration System

  This firmware ONLY captures and serves camera images. It does
  NOT run any AI/YOLO inference on-device. All heavy processing
  happens later on a laptop that pulls frames from this device
  over Wi-Fi.

  ENDPOINTS:
      GET /          -> simple HTML status/info page with links
      GET /stream    -> live MJPEG video stream (multipart/x-mixed-replace)
      GET /capture   -> single JPEG snapshot

  HARDWARE:
      Board    : AI Thinker ESP32-CAM
      Camera   : OV2640
      Uploader : ESP32-CAM-MB programmer

  Requires the "esp32" board package (Espressif Arduino core) to be
  installed in the Arduino IDE / arduino-cli. Uses the built-in
  esp_camera + esp_http_server libraries that ship with that core.
  ================================================================
*/

#include <WiFi.h>
#include "esp_camera.h"
#include "esp_http_server.h"
#include "esp_timer.h"

// ================================================================
// >>> WI-FI CREDENTIALS - EDIT THESE BEFORE UPLOADING <<<
// ================================================================
const char* WIFI_SSID     = "......";     // Normal ESP32's Wi-Fi hotspot (Member 1+2 board)
const char* WIFI_PASSWORD = ".....";
// ================================================================

// ----------------------------------------------------------------
// AI THINKER ESP32-CAM PIN MAP (OV2640)
// Do not change unless you are using a different board variant.
// ----------------------------------------------------------------
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM        35
#define Y8_GPIO_NUM        34
#define Y7_GPIO_NUM        39
#define Y6_GPIO_NUM        36
#define Y5_GPIO_NUM        21
#define Y4_GPIO_NUM        19
#define Y3_GPIO_NUM        18
#define Y2_GPIO_NUM         5
#define VSYNC_GPIO_NUM     25
#define HREF_GPIO_NUM      23
#define PCLK_GPIO_NUM      22

// ----------------------------------------------------------------
// MJPEG multipart stream framing
// ----------------------------------------------------------------
#define PART_BOUNDARY "mineguardframe"
static const char* STREAM_CONTENT_TYPE =
    "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* STREAM_PART =
    "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

httpd_handle_t camera_httpd = NULL;

// ================================================================
// HTTP HANDLER: "/"  -> simple status/info page
// ================================================================
static esp_err_t index_handler(httpd_req_t* req) {
  httpd_resp_set_type(req, "text/html");

  String ip = WiFi.localIP().toString();
  String html =
      "<html><head><title>MineGuard ESP32-CAM</title></head><body>"
      "<h2>MineGuard ESP32-CAM Module</h2>"
      "<p>Status: <b>Online</b></p>"
      "<p>Live stream: <a href=\"/stream\">/stream</a></p>"
      "<p>Single snapshot: <a href=\"/capture\">/capture</a></p>"
      "<img src=\"/stream\" style=\"max-width:100%;\">"
      "</body></html>";

  return httpd_resp_send(req, html.c_str(), html.length());
}

// ================================================================
// HTTP HANDLER: "/capture" -> single JPEG snapshot
// ================================================================
static esp_err_t capture_handler(httpd_req_t* req) {
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("[CAM] Capture failed: fb_get() returned NULL");
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }

  httpd_resp_set_type(req, "image/jpeg");
  httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");

  esp_err_t res = httpd_resp_send(req, (const char*)fb->buf, fb->len);

  esp_camera_fb_return(fb);
  return res;
}

// ================================================================
// HTTP HANDLER: "/stream" -> continuous MJPEG stream
// ================================================================
static esp_err_t stream_handler(httpd_req_t* req) {
  camera_fb_t* fb = NULL;
  esp_err_t res = ESP_OK;
  char part_buf[64];

  res = httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
  if (res != ESP_OK) {
    return res;
  }

  Serial.println("[CAM] Stream client connected");

  while (true) {
    fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("[CAM] Stream error: fb_get() returned NULL");
      res = ESP_FAIL;
    } else {
      if (fb->format != PIXFORMAT_JPEG) {
        // Should not happen since we configure JPEG output, but guard anyway.
        Serial.println("[CAM] Stream error: frame is not JPEG format");
        esp_camera_fb_return(fb);
        res = ESP_FAIL;
      }
    }

    // Send the multipart boundary
    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY));
    }

    // Send the per-frame header (content type + length)
    if (res == ESP_OK) {
      size_t hlen = snprintf(part_buf, sizeof(part_buf), STREAM_PART, fb->len);
      res = httpd_resp_send_chunk(req, part_buf, hlen);
    }

    // Send the actual JPEG bytes
    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, (const char*)fb->buf, fb->len);
    }

    if (fb) {
      esp_camera_fb_return(fb);
      fb = NULL;
    }

    if (res != ESP_OK) {
      // Client likely disconnected, or a real error occurred.
      break;
    }
  }

  Serial.println("[CAM] Stream client disconnected");
  return res;
}

// ================================================================
// Start the HTTP camera server and register all endpoints
// ================================================================
void startCameraServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;
  config.ctrl_port = 32768;
  config.max_uri_handlers = 8;

  httpd_uri_t index_uri = {
      .uri = "/",
      .method = HTTP_GET,
      .handler = index_handler,
      .user_ctx = NULL
  };

  httpd_uri_t capture_uri = {
      .uri = "/capture",
      .method = HTTP_GET,
      .handler = capture_handler,
      .user_ctx = NULL
  };

  httpd_uri_t stream_uri = {
      .uri = "/stream",
      .method = HTTP_GET,
      .handler = stream_handler,
      .user_ctx = NULL
  };

  Serial.printf("[HTTP] Starting camera server on port %d\n", config.server_port);

  if (httpd_start(&camera_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(camera_httpd, &index_uri);
    httpd_register_uri_handler(camera_httpd, &capture_uri);
    httpd_register_uri_handler(camera_httpd, &stream_uri);
    Serial.println("[HTTP] Camera server started successfully");
  } else {
    Serial.println("[HTTP] ERROR: Failed to start camera server");
  }
}

// ================================================================
// Camera hardware initialization
// ================================================================
bool initCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0       = Y2_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;   // JPEG output required for MJPEG streaming

  // Frame size / quality: chosen as a stable middle ground between
  // streaming smoothness, Wi-Fi bandwidth, and usefulness for the
  // downstream YOLO pipeline on the laptop.
  if (psramFound()) {
    config.frame_size   = FRAMESIZE_VGA;   // 640x480
    config.jpeg_quality  = 12;             // lower number = higher quality
    config.fb_count      = 2;              // double-buffer when PSRAM is available
  } else {
    config.frame_size   = FRAMESIZE_QVGA;  // 320x240 (fallback, no PSRAM)
    config.jpeg_quality  = 15;
    config.fb_count      = 1;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("[CAM] ERROR: Camera init failed with error 0x%x\n", err);
    return false;
  }

  Serial.println("[CAM] Camera initialized successfully");
  return true;
}

// ================================================================
// Wi-Fi connection
// ================================================================
bool connectWiFi() {
  Serial.println();
  Serial.printf("[WiFi] Connecting to SSID: %s\n", WIFI_SSID);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  const unsigned long WIFI_TIMEOUT_MS = 20000;
  unsigned long startAttempt = millis();

  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - startAttempt > WIFI_TIMEOUT_MS) {
      Serial.println("[WiFi] ERROR: Connection timed out");
      return false;
    }
    delay(300);
    Serial.print(".");
  }

  Serial.println();
  Serial.println("[WiFi] Connected successfully");
  return true;
}

// ================================================================
// SETUP
// ================================================================
void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  delay(1000);

  Serial.println();
  Serial.println("========================================");
  Serial.println(" MineGuard ESP32-CAM Streaming Module");
  Serial.println("========================================");

  // Slightly reduce brownout-related resets during camera init on some boards.
  // (Left out intentionally to keep this sketch dependency-free; use a
  // stable 5V/2A supply through the ESP32-CAM-MB instead.)

  if (!initCamera()) {
    Serial.println("[SYSTEM] HALTED: Camera initialization failed. Check wiring/power.");
    while (true) {
      delay(1000); // Halt here — nothing useful can run without the camera.
    }
  }

  if (!connectWiFi()) {
    Serial.println("[SYSTEM] HALTED: Wi-Fi connection failed. Check credentials/signal.");
    while (true) {
      delay(1000);
    }
  }

  startCameraServer();

  IPAddress ip = WiFi.localIP();
  Serial.println();
  Serial.println("[SYSTEM] MineGuard ESP32-CAM is ready.");
  Serial.print("[SYSTEM] IP Address: ");
  Serial.println(ip);
  Serial.println();
  Serial.println("Open these URLs in a browser or point the laptop's OpenCV client at:");
  Serial.print("  Status page : http://");
  Serial.print(ip);
  Serial.println("/");
  Serial.print("  Live stream : http://");
  Serial.print(ip);
  Serial.println("/stream");
  Serial.print("  Snapshot    : http://");
  Serial.print(ip);
  Serial.println("/capture");
  Serial.println();
}

// ================================================================
// LOOP
// The HTTP server runs in its own background task, so loop() has
// nothing to do. A small delay keeps the idle task fed.
// ================================================================
void loop() {
  delay(10000);
}
