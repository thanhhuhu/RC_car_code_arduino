#include <thanhhuhu-project-1_inferencing.h>
#include "edge-impulse-sdk/dsp/image/image.hpp"
#include <WebServer.h>
#include <WiFi.h>
#include "esp_camera.h"

const char* ssid     = "185";
const char* password = "88223834";

WebServer server(80);

float  result_clear    = 0;
float  result_humans   = 0;
float  result_obstacle = 0;
String result_label    = "unknown";
unsigned long inference_ms = 0;

uint8_t* cached_jpg     = nullptr;
size_t   cached_jpg_len = 0;

#define INFERENCE_INTERVAL 5000
unsigned long lastInference = 0;
bool inferenceRunning = false;

#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

#define EI_CAMERA_RAW_FRAME_BUFFER_COLS  320
#define EI_CAMERA_RAW_FRAME_BUFFER_ROWS  240
#define EI_CAMERA_FRAME_BYTE_SIZE        3

static bool  debug_nn       = false;
static bool  is_initialised = false;
uint8_t*     snapshot_buf;

static camera_config_t camera_config = {
  .pin_pwdn     = PWDN_GPIO_NUM,
  .pin_reset    = RESET_GPIO_NUM,
  .pin_xclk     = XCLK_GPIO_NUM,
  .pin_sscb_sda = SIOD_GPIO_NUM,
  .pin_sscb_scl = SIOC_GPIO_NUM,
  .pin_d7 = Y9_GPIO_NUM, .pin_d6 = Y8_GPIO_NUM,
  .pin_d5 = Y7_GPIO_NUM, .pin_d4 = Y6_GPIO_NUM,
  .pin_d3 = Y5_GPIO_NUM, .pin_d2 = Y4_GPIO_NUM,
  .pin_d1 = Y3_GPIO_NUM, .pin_d0 = Y2_GPIO_NUM,
  .pin_vsync = VSYNC_GPIO_NUM,
  .pin_href  = HREF_GPIO_NUM,
  .pin_pclk  = PCLK_GPIO_NUM,
  // ✅ OV3660 cần xclk 20MHz — giữ nguyên
  .xclk_freq_hz  = 20000000,
  .ledc_timer    = LEDC_TIMER_0,
  .ledc_channel  = LEDC_CHANNEL_0,
  .pixel_format  = PIXFORMAT_JPEG,
  .frame_size    = FRAMESIZE_QVGA,
  .jpeg_quality  = 8,
  .fb_count      = 1,
  .fb_location   = CAMERA_FB_IN_PSRAM,
  .grab_mode     = CAMERA_GRAB_WHEN_EMPTY,
};

bool ei_camera_init(void);
bool ei_camera_capture(uint32_t w, uint32_t h, uint8_t* buf);
static int ei_camera_get_data(size_t offset, size_t length, float* out_ptr);

void updateCachedFrame() {
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) return;
  if (cached_jpg) { free(cached_jpg); cached_jpg = nullptr; }
  cached_jpg = (uint8_t*)malloc(fb->len);
  if (cached_jpg) {
    memcpy(cached_jpg, fb->buf, fb->len);
    cached_jpg_len = fb->len;
  }
  esp_camera_fb_return(fb);
}

void handleSnapshot() {
  if (!cached_jpg) updateCachedFrame();
  if (!cached_jpg) { server.send(500, "text/plain", "No image"); return; }
  server.sendHeader("Cache-Control", "no-cache");
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send_P(200, "image/jpeg", (const char*)cached_jpg, cached_jpg_len);
}

const char PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>ESP32-CAM AI</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{background:#0d0f14;color:#e8ecf4;font-family:monospace;display:flex;flex-direction:column;align-items:center;padding:16px;gap:10px}
h2{color:#00e5a0;letter-spacing:3px;font-size:13px;margin-top:6px}
.card{width:100%;max-width:440px;background:#161920;border:1px solid rgba(255,255,255,.08);border-radius:10px;padding:14px}
.lbl{font-size:9px;color:#6b7080;letter-spacing:2px;margin-bottom:10px}
.cam-wrap{position:relative;width:100%;max-width:440px;aspect-ratio:4/3;background:#000;border-radius:10px;overflow:hidden}
#cam{width:100%;height:100%;object-fit:cover;display:block}
.cam-badge{position:absolute;top:8px;left:8px;font-size:9px;padding:3px 8px;border-radius:4px;letter-spacing:1px;background:rgba(0,229,160,.15);border:1px solid #00e5a0;color:#00e5a0}
.cam-ai{position:absolute;top:8px;right:8px;font-size:10px;font-weight:bold;padding:4px 10px;border-radius:4px;letter-spacing:1px;transition:all .3s}
.ai-clear   {background:rgba(0,229,160,.2);border:1px solid #00e5a0;color:#00e5a0}
.ai-humans  {background:rgba(55,138,221,.2);border:1px solid #378ADD;color:#378ADD}
.ai-obstacle{background:rgba(255,77,106,.2);border:1px solid #ff4d6a;color:#ff4d6a}
.ai-unknown {background:rgba(107,112,128,.2);border:1px solid #6b7080;color:#6b7080}
.cam-bottom{position:absolute;bottom:8px;left:8px;right:8px;display:flex;justify-content:space-between}
.cam-info{font-size:9px;color:rgba(255,255,255,.5);letter-spacing:1px}
.row{display:flex;align-items:center;margin-bottom:10px}
.row:last-child{margin-bottom:0}
.name{font-size:11px;min-width:70px}
.bw{flex:1;height:6px;background:#1e2230;border-radius:3px;margin:0 10px;overflow:hidden}
.bar{height:100%;border-radius:3px;width:0%;transition:width .5s}
.pct{font-size:11px;font-weight:bold;min-width:44px;text-align:right}
.c1{background:#00e5a0}.p1{color:#00e5a0}
.c2{background:#378ADD}.p2{color:#378ADD}
.c3{background:#ff4d6a}.p3{color:#ff4d6a}
.status{text-align:center;padding:12px;border-radius:8px;font-size:20px;font-weight:bold;transition:all .3s}
.s-clear   {color:#00e5a0;background:rgba(0,229,160,.1);border:1px solid rgba(0,229,160,.3)}
.s-humans  {color:#378ADD;background:rgba(55,138,221,.1);border:1px solid rgba(55,138,221,.3)}
.s-obstacle{color:#ff4d6a;background:rgba(255,77,106,.1);border:1px solid rgba(255,77,106,.3)}
.s-unknown {color:#6b7080;background:rgba(107,112,128,.1);border:1px solid rgba(107,112,128,.2)}
.meta{font-size:10px;color:#6b7080;text-align:center}
.dot{display:inline-block;width:6px;height:6px;border-radius:50%;background:#00e5a0;margin-right:6px;animation:blink 1s infinite}
@keyframes blink{0%,100%{opacity:1}50%{opacity:.2}}
.countdown{font-size:9px;color:#6b7080;text-align:right;margin-top:6px}
</style></head><body>
<h2>ESP32-CAM · AI VIEWER</h2>
<div class="cam-wrap">
  <img id="cam" src="/snapshot"/>
  <div class="cam-badge">● LIVE</div>
  <div class="cam-ai ai-unknown" id="cam-ai">--</div>
  <div class="cam-bottom">
    <span class="cam-info" id="time-info">--</span>
    <span class="cam-info" id="ms-info">--</span>
  </div>
</div>
<div class="card">
  <div class="lbl">KẾT QUẢ PHÂN TÍCH</div>
  <div class="row"><span class="name">Clear</span><div class="bw"><div class="bar c1" id="bc"></div></div><span class="pct p1" id="vc">0%</span></div>
  <div class="row"><span class="name">Humans</span><div class="bw"><div class="bar c2" id="bh"></div></div><span class="pct p2" id="vh">0%</span></div>
  <div class="row"><span class="name">Obstacle</span><div class="bw"><div class="bar c3" id="bo"></div></div><span class="pct p3" id="vo">0%</span></div>
  <div class="countdown" id="next">AI cập nhật sau: --</div>
</div>
<div class="card">
  <div class="lbl">TRẠNG THÁI</div>
  <div class="status s-unknown" id="st">--</div>
</div>
<div class="meta"><span class="dot"></span><span id="ping">Đang kết nối...</span></div>
<script>
const cls={clear:'s-clear',humans:'s-humans',obstacle:'s-obstacle'};
const aic={clear:'ai-clear',humans:'ai-humans',obstacle:'ai-obstacle'};
let lastAiUpdate=0, aiInterval=3000;
function refreshCam(){
  const img=new Image();
  img.onload=()=>{document.getElementById('cam').src=img.src;};
  img.src='/snapshot?'+Date.now();
}
async function updateAI(){
  const t=Date.now();
  try{
    const d=await(await fetch('/ai')).json();
    const f=v=>Math.round(v*100)+'%';
    document.getElementById('bc').style.width=f(d.clear);
    document.getElementById('bh').style.width=f(d.humans);
    document.getElementById('bo').style.width=f(d.obstacle);
    document.getElementById('vc').textContent=f(d.clear);
    document.getElementById('vh').textContent=f(d.humans);
    document.getElementById('vo').textContent=f(d.obstacle);
    const best=Object.entries({clear:d.clear,humans:d.humans,obstacle:d.obstacle}).reduce((a,b)=>b[1]>a[1]?b:a);
    const lbl=best[0],pct=f(best[1]);
    const st=document.getElementById('st');
    st.textContent=lbl.toUpperCase()+' '+pct;
    st.className='status '+(cls[lbl]||'s-unknown');
    const badge=document.getElementById('cam-ai');
    badge.textContent=lbl.toUpperCase()+' '+pct;
    badge.className='cam-ai '+(aic[lbl]||'ai-unknown');
    document.getElementById('time-info').textContent=new Date().toLocaleTimeString('vi',{hour12:false});
    document.getElementById('ms-info').textContent='AI: '+d.ms+'ms';
    document.getElementById('ping').textContent='Cam: 500ms · AI: '+d.ms+'ms · PING: '+(Date.now()-t)+'ms';
    aiInterval=d.ms+500; lastAiUpdate=Date.now();
  }catch(e){document.getElementById('ping').textContent='Mất kết nối...';}
}
function updateCountdown(){
  const remain=Math.max(0,aiInterval-(Date.now()-lastAiUpdate));
  document.getElementById('next').textContent='AI cập nhật sau: '+(remain/1000).toFixed(1)+'s';
}
refreshCam(); updateAI();
setInterval(refreshCam,200);
setInterval(updateAI,5000);
setInterval(updateCountdown,500);
</script></body></html>
)rawliteral";

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  Serial.print("WiFi");
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.println("\nIP: " + WiFi.localIP().toString());

  if (!ei_camera_init()) Serial.println("Camera FAILED!");
  else Serial.println("Camera OK!");

  updateCachedFrame();

  server.on("/", []() { server.send_P(200, "text/html", PAGE); });
  server.on("/snapshot", handleSnapshot);
  server.on("/ai", []() {
    String json = "{";
    json += "\"clear\":"    + String(result_clear,    5) + ",";
    json += "\"humans\":"   + String(result_humans,   5) + ",";
    json += "\"obstacle\":" + String(result_obstacle, 5) + ",";
    json += "\"ms\":"       + String(inference_ms);
    json += "}";
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(200, "application/json", json);
  });
  server.begin();
  Serial.println("Vao: http://" + WiFi.localIP().toString());
}

void loop() {
  server.handleClient();

  static unsigned long lastFrame = 0;
  if (millis() - lastFrame >= 200) {
    lastFrame = millis();
    if (!inferenceRunning) updateCachedFrame();
  }

  if (millis() - lastInference >= INFERENCE_INTERVAL && !inferenceRunning) {
    lastInference = millis();
    inferenceRunning = true;

    snapshot_buf = (uint8_t*)malloc(
      EI_CAMERA_RAW_FRAME_BUFFER_COLS *
      EI_CAMERA_RAW_FRAME_BUFFER_ROWS *
      EI_CAMERA_FRAME_BYTE_SIZE);

    if (snapshot_buf) {
      ei::signal_t signal;
      signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
      signal.get_data = &ei_camera_get_data;

      if (ei_camera_capture(EI_CLASSIFIER_INPUT_WIDTH,
                            EI_CLASSIFIER_INPUT_HEIGHT, snapshot_buf)) {
        ei_impulse_result_t result = { 0 };
        if (run_classifier(&signal, &result, debug_nn) == EI_IMPULSE_OK) {
          inference_ms = result.timing.dsp + result.timing.classification;
          float maxVal = 0;
          for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
            String lbl = String(ei_classifier_inferencing_categories[i]);
            float  val = result.classification[i].value;
            if (lbl == "clear")    result_clear    = val;
            if (lbl == "humans")   result_humans   = val;
            if (lbl == "obstacle") result_obstacle = val;
            if (val > maxVal) { maxVal = val; result_label = lbl; }
          }
          Serial.printf("AI: %s %.0f%% (%lums)\n",
            result_label.c_str(), maxVal*100, inference_ms);
        }
      }
      free(snapshot_buf);
    }
    inferenceRunning = false;
  }
}

// ═══════════════════════════════════════════════
// ═══════════════════════════════════════════════
bool ei_camera_init(void) {
  if (is_initialised) return true;
  esp_err_t err = esp_camera_init(&camera_config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: 0x%x\n", err);
    return false;
  }

  sensor_t* s = esp_camera_sensor_get();
  if (s) {
    s->set_vflip(s, 1);
    s->set_hmirror(s, 0);

    // Màu sắc — tăng sáng và nét
    s->set_brightness(s, 2);      // tối đa: -2~2
    s->set_contrast(s, 2);        // tối đa: -2~2
    s->set_saturation(s, 1);
    s->set_sharpness(s, 2);       // tối đa: -2~2
    s->set_special_effect(s, 0);
    s->set_colorbar(s, 0);

    // Auto exposure & gain
    s->set_whitebal(s, 1);
    s->set_awb_gain(s, 1);
    s->set_wb_mode(s, 0);
    s->set_exposure_ctrl(s, 1);
    s->set_aec2(s, 1);
    s->set_gain_ctrl(s, 1);
    s->set_gainceiling(s, GAINCEILING_8X); 

    // Correction
    s->set_bpc(s, 1);
    s->set_wpc(s, 1);
    s->set_raw_gma(s, 1);
    s->set_lenc(s, 1);
    s->set_dcw(s, 1);
  }

  is_initialised = true;
  return true;
}

bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t* out_buf) {
  if (!is_initialised) return false;
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) return false;
  bool ok = fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, snapshot_buf);
  esp_camera_fb_return(fb);
  if (!ok) return false;
  if (img_width  != EI_CAMERA_RAW_FRAME_BUFFER_COLS ||
      img_height != EI_CAMERA_RAW_FRAME_BUFFER_ROWS) {
    ei::image::processing::crop_and_interpolate_rgb888(
      out_buf,
      EI_CAMERA_RAW_FRAME_BUFFER_COLS,
      EI_CAMERA_RAW_FRAME_BUFFER_ROWS,
      out_buf, img_width, img_height);
  }
  return true;
}

static int ei_camera_get_data(size_t offset, size_t length, float* out_ptr) {
  size_t pixel_ix = offset * 3, out_ptr_ix = 0;
  while (length != 0) {
    out_ptr[out_ptr_ix] = (snapshot_buf[pixel_ix+2] << 16)
                        + (snapshot_buf[pixel_ix+1] << 8)
                        +  snapshot_buf[pixel_ix];
    out_ptr_ix++; pixel_ix += 3; length--;
  }
  return 0;
}

#if !defined(EI_CLASSIFIER_SENSOR) || EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_CAMERA
#error "Invalid model for current sensor"
#endif
