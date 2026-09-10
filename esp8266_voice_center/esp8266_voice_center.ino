// =====================================================================
//  智能家居语音环境中心 — ESP8266 语音 + 云端侧
//  NodeMCU V3 + INMP441 麦克风 + HENOPI-98357(MAX98357A) 功放
//  架构：ESP 做 录音 + UDP 收发 + 播放 + 控制 51 + MQTT 巴法云
//        电脑端做 讯飞 ASR 识别 + TTS 合成
//  语音闭环：按住按键说话 → UDP 发音频给电脑 → 电脑 ASR 识别 → UDP 回传文字
//            → 意图解析 → 控制 51 + 电脑 TTS 合成 → TCP 回传音频 → 播放
//  云端：MQTT 上报 env（温湿度/光照/灯态）到巴法云 + 订阅 cmd 远程控制
// =====================================================================

// ---------- 配置（按需修改） ----------
#define WIFI_SSID       "你的WiFi名"
#define WIFI_PASS       "你的WiFi密码"

// 巴法云 MQTT······
#define BEMFA_UID       "你的巴法云私钥"   // 私钥
#define BEMFA_SERVER    "bemfa.com"
#define BEMFA_PORT      9501
#define TOPIC_ENV       "env"    // 上报主题（纯主题名，不带 UID）
#define TOPIC_CMD       "cmd"    // 控制主题（纯主题名，不带 UID）

// ---------- 引脚 ----------
#define PIN_BTN     D3   // 语音触发按键，低电平有效
#define PIN_51_RX   D2   // 软串口 RX（接 51 TX）
#define PIN_51_TX   D1   // 软串口 TX（接 51 RX）

// ---------- 常量 ----------
#define SAMPLE_RATE    16000
#define CHUNK_SAMPLES  320
#define UART_BAUD      9600
#define UART51_ENABLED 1      // 接 51 后设 1（接收传感器上报）
#define MAX_RECORD_MS  8000   // 单次录音上限

// ---------- UDP 端口 ----------
const uint16_t PC_AUDIO_PORT = 5005;  // ESP → 电脑（录音音频）
const uint16_t ESP_TEXT_PORT  = 5006;  // 电脑 → ESP（识别文字）
const uint16_t ESP_TTS_PORT   = 5007;  // 电脑 → ESP（TTS 音频）
const uint16_t PC_ENV_PORT    = 5009;  // ESP → 电脑（环境数据，供语音查询）

// ---------- 包含 ----------
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <SoftwareSerial.h>
#include <PubSubClient.h>
#include "core_esp8266_i2s.h"

// ---------- 全局 ----------
SoftwareSerial s51(PIN_51_RX, PIN_51_TX);
WiFiUDP udpAudio;    // 发录音音频给电脑
WiFiUDP udpText;     // 收电脑回传的识别文字
WiFiServer ttsServer(ESP_TTS_PORT);   // 收电脑回传的 TTS 音频（TCP，自带流控）

WiFiClient espClient;
PubSubClient mqtt(espClient);
uint32_t g_lastEnvPublish = 0;
uint32_t g_lastWifiTry = 0;

const IPAddress PC_IP(192, 168, 1, 100);   // 电脑 IP（改成电脑实际 IP）

struct {
  float temp = 0;
  float hum  = 0;
  int   light = 0;
  int   lamp  = 0;
  int   autoMode = 0;
  int   dist = 0;
  int   alarm = 0;
  int   fan = 0;
} g_env;

enum VoiceState { ST_IDLE, ST_RECORDING };
VoiceState g_state = ST_IDLE;
uint32_t g_recStart = 0;

// =====================================================================
//  I2S 录音/播放（内核 I2S，半双工分时切换）
// =====================================================================
void i2sBeginPlayback() { i2s_rxtx_begin(false, true); i2s_set_rate(SAMPLE_RATE); }
bool i2sBeginRecording() {
  bool ok = i2s_rxtx_begin(true, false);
  if (ok) i2s_set_rate(SAMPLE_RATE);
  return ok;
}   

int readPCM(int16_t* buf, int maxSamples) {
  int n = 0;
  while (n < maxSamples && i2s_rx_available() > 0) {
    int16_t l, r;
    if (i2s_read_sample(&l, &r, false)) buf[n++] = l;
    else break;
  }
  return n;
}

// LED 闪灯（D4/GPIO2）
void blinkLed(int times) {
  pinMode(D4, OUTPUT);
  for (int i = 0; i < times; i++) {
    digitalWrite(D4, LOW);  delay(150);   // 亮
    digitalWrite(D4, HIGH); delay(150);   // 灭
  }
}

// =====================================================================
//  与 51 软串口
// =====================================================================
void uart51Setup() { s51.begin(UART_BAUD); }
void dbg(const String& s) { s51.print(s); }
void dbgln(const String& s) { s51.println(s); }

void parseFrame(const String& f) {
  int i1 = f.indexOf(':');
  if (i1 < 0) return;
  char type = f[0];
  if (type == 'D') {
    int i2 = f.indexOf(':', i1 + 1);
    if (i2 > 0) { g_env.temp = f.substring(i1 + 1, i2).toFloat(); g_env.hum = f.substring(i2 + 1).toFloat(); }
  } else if (type == 'L') {
    g_env.light = f.substring(i1 + 1).toInt();
  } else if (type == 'A') {
    g_env.lamp = f.substring(i1 + 1).toInt();
  } else if (type == 'R') {
    g_env.dist = f.substring(i1 + 1).toInt();
  } else if (type == 'W') {
    g_env.alarm = f.substring(i1 + 1).toInt();
  } else if (type == 'F') {
    g_env.fan = f.substring(i1 + 1).toInt();
  }
}

void uart51Poll() {
  static String line;
  while (s51.available()) {
    char c = (char)s51.read();
    if (c == '\n') { parseFrame(line); line = ""; }
    else if (c != '\r') line += c;
  }
}

void uart51Send(const char* cmd) { s51.println(cmd); }

// =====================================================================
//  MQTT 巴法云
// =====================================================================
// 指令消息 → 51 命令（兼容 JSON 与纯文本两种）
String cmdFromPayload(const String& m) {
  String dev, act;
  if (m.startsWith("{")) {
    // JSON: {"dev":"lamp","act":"on"}
    int p = m.indexOf("\"dev\"");
    if (p >= 0) {
      p = m.indexOf(':', p) + 1;
      int q1 = m.indexOf('"', p);
      int q2 = m.indexOf('"', q1 + 1);
      if (q1 >= 0 && q2 > q1) dev = m.substring(q1 + 1, q2);
    }
    p = m.indexOf("\"act\"");
    if (p >= 0) {
      p = m.indexOf(':', p) + 1;
      int q1 = m.indexOf('"', p);
      int q2 = m.indexOf('"', q1 + 1);
      if (q1 >= 0 && q2 > q1) act = m.substring(q1 + 1, q2);
    }
  } else {
    // 纯文本: "lamp:on" / "fan:on" / "auto:1"
    int c = m.indexOf(':');
    if (c > 0) { dev = m.substring(0, c); act = m.substring(c + 1); }
    else { dev = m; }
  }
  dev.trim(); act.trim();

  if (dev == "lamp") {
    if (act == "on")  return "C:lamp:on";
    if (act == "off") return "C:lamp:off";
  } else if (dev == "fan") {
    if (act == "on")  return "C:fan:on";
    if (act == "off") return "C:fan:off";
  } else if (dev == "auto") {
    if (act == "1" || act == "on")  return "C:auto:1";
    if (act == "0" || act == "off") return "C:auto:0";
  }
  return "";
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String msg;
  msg.reserve(length);
  for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];

  String cmd = cmdFromPayload(msg);
  if (cmd.length() > 0) {
    uart51Send(cmd.c_str());
    if (cmd == "C:auto:1")       g_env.autoMode = 1;
    else if (cmd == "C:auto:0")  g_env.autoMode = 0;
    else if (cmd == "C:lamp:on") g_env.lamp = 1;
    else if (cmd == "C:lamp:off") g_env.lamp = 0;
    else if (cmd == "C:fan:on")  g_env.fan = 1;
    else if (cmd == "C:fan:off") g_env.fan = 0;
  }
}

void mqttConnect() {
  if (mqtt.connected()) return;
  if (mqtt.connect(BEMFA_UID)) {
    mqtt.subscribe(TOPIC_CMD);   // 巴法云主题=纯名，不带 UID、不带 #
  }
}

void mqttPublishEnv() {
  String p = "{\"t\":" + String((int)g_env.temp) +
             ",\"h\":" + String((int)g_env.hum) +
             ",\"light\":" + String(g_env.light) +
             ",\"lamp\":" + String(g_env.lamp) +
             ",\"auto\":" + String(g_env.autoMode) +
             ",\"dist\":" + String(g_env.dist) +
             ",\"alarm\":" + String(g_env.alarm) +
             ",\"fan\":" + String(g_env.fan) + "}";
  mqtt.publish(TOPIC_ENV, p.c_str());
}

// 把当前环境数据发一份给电脑（供语音查询温湿度）
void sendEnvToPc() {
  String e = "E:" + String((int)g_env.temp) + ":" + String((int)g_env.hum) +
             ":" + String(g_env.light) + ":" + String(g_env.lamp) +
             ":" + String(g_env.autoMode);
  udpAudio.beginPacket(PC_IP, PC_ENV_PORT);
  udpAudio.write((const uint8_t*)e.c_str(), e.length());
  udpAudio.endPacket();
}

// =====================================================================
//  UDP 接收：等电脑回传识别文字
// =====================================================================
bool waitTextResult(String& text, uint32_t timeoutMs) {
  // 丢弃积压的旧指令，避免读到上一句
  while (udpText.parsePacket() > 0) {
    while (udpText.read() != -1) { }
  }
  uint32_t t0 = millis();
  while (millis() - t0 < timeoutMs) {
    int sz = udpText.parsePacket();
    if (sz > 0) {
      char buf[256];
      int n = udpText.read(buf, sizeof(buf) - 1);
      if (n > 0) {
        buf[n] = 0;
        text = buf;
        return true;
      }
    }
    uart51Poll();      // 阻塞期间仍接收 51 上报
    mqtt.loop();       // 保持 MQTT 心跳与命令接收
    yield();
    delay(5);
  }
  return false;
}

// =====================================================================
//  TCP 接收：边收边播 TTS 音频（TCP 自带流控，阻塞写 I2S 会反压 PC）
// =====================================================================
// 从已建立的 TCP 连接播放 TTS 音频（阻塞，直到播完或超时）
void playTtsStream(WiFiClient& client, uint32_t timeoutMs) {
  static int16_t chunk[513];   // 512 样本 + 1 字节 spare 空间
  uint8_t* bytes = (uint8_t*)chunk;
  int spare = 0;               // 残留的单个字节数（0 或 1）
  uint32_t lastPoll = 0;       // 上次处理串口/MQTT 的时间（节流用）

  i2sBeginPlayback();
  uint32_t t1 = millis();
  while (client.connected() || client.available()) {
    int n = client.read(bytes + spare, sizeof(chunk) - spare);
    if (n > 0) {
      t1 = millis();
      int total = spare + n;
      int samples = total / 2;
      if (samples > 0) {
        i2s_write_buffer_mono(chunk, samples);   // 阻塞写入，自然限速
      }
      spare = total & 1;                     // 是否残留单字节
      if (spare) bytes[0] = bytes[total - 1]; // 残留字节移到开头，下次拼接
    } else {
      // 节流：每 50ms 才处理一次串口/MQTT，降低对 I2S 时序的干扰
      if (millis() - lastPoll > 50) {
        lastPoll = millis();
        uart51Poll();
        mqtt.loop();
      }
      yield();
    }
    if (millis() - t1 > timeoutMs) break;   // 长时间无数据则结束
  }

  // 等 DMA 把剩余音频播完再 end
  uint32_t t2 = millis();
  while (!i2s_is_empty() && millis() - t2 < 500) { yield(); delay(1); }
  client.stop();
  i2s_end();
}

void receiveAndPlayTTS(uint32_t timeoutMs) {
  // 丢弃之前积压的旧连接，避免播上一句的音频
  WiFiClient stale = ttsServer.available();
  while (stale) {
    stale.stop();
    stale = ttsServer.available();
  }

  // 等 PC 的 TCP 连接
  uint32_t t0 = millis();
  WiFiClient client = ttsServer.available();
  while (!client && millis() - t0 < timeoutMs) {
    client = ttsServer.available();
    uart51Poll();
    mqtt.loop();
    yield();
    delay(1);
  }
  if (!client) return;   // 没等到连接
  playTtsStream(client, timeoutMs);
}

// 非阻塞检查：若有 PC 推送的 TTS（如报警播报），立即播放
void checkPushTTS() {
  WiFiClient client = ttsServer.available();
  if (!client) return;
  playTtsStream(client, 15000);
}

// =====================================================================
//  setup / loop
// =====================================================================
void setup() {
  Serial.begin(9600);          // I2S 时钟依赖 UART0 9600 分频
  delay(3000);
  pinMode(PIN_BTN, INPUT_PULLUP);
  uart51Setup();

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  {
    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) delay(500);
  }
  if (WiFi.status() != WL_CONNECTED) {
    blinkLed(8);   // WiFi 连接失败：D4 快闪 8 下提示（loop 里会持续重连）
  }

  udpAudio.begin(5008);        // 本地端口（发音频用）
  udpText.begin(ESP_TEXT_PORT); // 收识别文字
  ttsServer.begin();            // TCP 收 TTS 音频

  mqtt.setServer(BEMFA_SERVER, BEMFA_PORT);
  mqtt.setCallback(mqttCallback);
  mqtt.setKeepAlive(60);
  mqttConnect();

  // 用 D4 灯指示 MQTT 状态（串口被 I2S 时钟干扰无法用，只能看灯）
  if (mqtt.connected()) {
    blinkLed(2);          // 闪 2 下 = MQTT 已连上（巴法云应显示在线）
  } else {
    int rc = mqtt.state();
    if (rc == -2 || rc == -4) blinkLed(5);   // 闪 5 下 = 连不上服务器（没外网/超时）
    else                      blinkLed(6);   // 闪 6 下 = 其他（5=UID 鉴权失败）
  }

  dbgln("[boot] ready");
}

void loop() {
#if UART51_ENABLED
  uart51Poll();
#endif

  // WiFi 断线重连（每 5 秒尝试一次）
  if (WiFi.status() != WL_CONNECTED && millis() - g_lastWifiTry > 5000) {
    g_lastWifiTry = millis();
    WiFi.reconnect();
  }

  // MQTT 保活
  if (!mqtt.connected()) {
    mqttConnect();
  } else {
    mqtt.loop();
  }

  // 每 2s：上报 env 到巴法云
  if (millis() - g_lastEnvPublish > 2000) {
    g_lastEnvPublish = millis();
    if (mqtt.connected()) mqttPublishEnv();
  }

  bool pressed = (digitalRead(PIN_BTN) == LOW);

  if (g_state == ST_IDLE) {
    if (pressed) {
      dbgln("[btn]");
      blinkLed(1);
      g_state = ST_RECORDING;
      i2sBeginRecording();
      g_recStart = millis();
      sendEnvToPc();             // 回传环境数据给电脑（供语音查询）
      blinkLed(2);               // 提示：开始说话
    } else {
      checkPushTTS();            // 空闲时检查 PC 推送的报警播报
    }
  }

  if (g_state == ST_RECORDING) {
    bool timeout = (millis() - g_recStart > MAX_RECORD_MS);
    if (!pressed || timeout) {
      i2s_end();               // 停止录音
      delay(50);

      // 1. 等电脑回传指令文字（如 C:lamp:on）
      String cmd;
      if (waitTextResult(cmd, 15000)) {
        dbgln("[cmd] " + cmd);
        // 2. 直接转发指令给 51（并同步 auto/灯 状态，供 MQTT 上报）
        if (cmd != "C:none") {
          uart51Send(cmd.c_str());
          if (cmd == "C:auto:1")       g_env.autoMode = 1;
          else if (cmd == "C:auto:0")  g_env.autoMode = 0;
          else if (cmd == "C:lamp:on") g_env.lamp = 1;
          else if (cmd == "C:lamp:off") g_env.lamp = 0;
          else if (cmd == "C:fan:on")  g_env.fan = 1;
          else if (cmd == "C:fan:off") g_env.fan = 0;
        }
        // 3. 等电脑回传 TTS 音频并播放
        receiveAndPlayTTS(15000);
      } else {
        dbgln("[cmd] timeout");
      }

      g_state = ST_IDLE;
    } else {
      // 录音循环：readPCM → UDP 发音频给电脑（实时流式，不阻塞）
      static int16_t buf[CHUNK_SAMPLES];
      int n = readPCM(buf, CHUNK_SAMPLES);
      if (n > 0) {
        udpAudio.beginPacket(PC_IP, PC_AUDIO_PORT);
        udpAudio.write((const uint8_t*)buf, n * 2);
        udpAudio.endPacket();
      }
    }
  }

  delay(1);
}
