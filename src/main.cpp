#include <M5Unified.h>
#include <SD.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <WebSocketsClient.h>
#include <stdarg.h>

static constexpr size_t CHUNK = 2048; // 1回の読み書きサンプル数（安定性向上）
static constexpr int REC_SEC = 5;
static const char *PCM_PATH = "/mic.pcm";
static uint32_t GLOBAL_SAMPLERATE = 16000;
static constexpr float REC_GAIN = 3.0f; // 録音ゲイン（音声レベル向上）
static constexpr float PLAY_GAIN = 1.0f; // 再生ゲイン（スピーカー出力向上）

// WiFi設定
const char* ssid = "Buffalo-G-2220";
const char* password = "7hur6atkyvkwv";
const char* serverURL = "http://192.168.11.18:3000";
const char* websocketHost = "192.168.11.18";
const int websocketPort = 3000;
const char* websocketPath = "/pcm/stream";

// ストリーミング状態管理
bool streamingActive = false;
bool headerSent = false;
bool micInitialized = false;
unsigned long streamStartTime = 0;
WebSocketsClient webSocket;

int16_t buf[CHUNK];

// ーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーー
// ロギングユーティリティ
// ーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーーー
static void LOGF(const char* tag, const char* fmt, ...) {
  char line[256];
  va_list args;
  va_start(args, fmt);
  vsnprintf(line, sizeof(line), fmt, args);
  va_end(args);
  Serial.printf("[%10lu] %-6s | %s\n", millis(), tag, line);
}

static void logNetworkInfo() {
  LOGF("NET", "SSID: %s", WiFi.SSID().c_str());
  LOGF("NET", "RSSI: %d dBm", WiFi.RSSI());
  LOGF("NET", "IP: %s", WiFi.localIP().toString().c_str());
  LOGF("NET", "GW: %s", WiFi.gatewayIP().toString().c_str());
  LOGF("NET", "MASK: %s", WiFi.subnetMask().toString().c_str());
  LOGF("NET", "DNS: %s", WiFi.dnsIP().toString().c_str());
}

static void logWsTarget(const char* host, int port, const char* path) {
  IPAddress ip;
  bool resolved = WiFi.hostByName(host, ip);
  if (resolved) {
    LOGF("WS", "Target: %s:%d%s (resolved %s)", host, port, path, ip.toString().c_str());
  } else {
    LOGF("WS", "Target: %s:%d%s (DNS resolve failed)", host, port, path);
  }
}

// TCPポート到達性の簡易チェック（SYN/ACK 成否）
static bool tcpProbe(const char* host, uint16_t port, uint32_t timeoutMs = 3000) {
  WiFiClient client;
  uint32_t t0 = millis();
  bool ok = client.connect(host, port, timeoutMs);
  uint32_t dt = millis() - t0;
  LOGF("TCP", "connect %s:%u => %s (%lums)", host, (unsigned)port, ok ? "OK" : "FAIL", dt);
  if (ok) client.stop();
  return ok;
}

// シンプルなハイパスフィルタ (DC成分除去)
class SimpleHighPassFilter {
private:
  float prev_input = 0.0f;
  float prev_output = 0.0f;
  float alpha = 0.995f; // カットオフ ≈ 80Hz @ 16kHz
public:
  int16_t process(int16_t input) {
    float fin = (float)input;
    float output = alpha * (prev_output + fin - prev_input);
    prev_input = fin;
    prev_output = output;
    if (output > 32767.0f) return 32767;
    if (output < -32768.0f) return -32768;
    return (int16_t)output;
  }
};

// シンプルなローパスフィルタ (高周波ノイズ除去)
class SimpleLowPassFilter {
private:
  float prev_output = 0.0f;
  float alpha = 0.8f; // カットオフ ≈ 1.6kHz @ 16kHz (緩やかなローパス)
public:
  int16_t process(int16_t input) {
    float fin = (float)input;
    prev_output = alpha * prev_output + (1 - alpha) * fin;
    if (prev_output > 32767.0f) return 32767;
    if (prev_output < -32768.0f) return -32768;
    return (int16_t)prev_output;
  }
};

SimpleHighPassFilter hpf_rec, hpf_play;
SimpleLowPassFilter lpf_rec, lpf_play;

struct __attribute__((packed)) PcmHeader
{
  char magic[4];        // "PCM1"
  uint32_t sample_rate; // 16000
  uint8_t channels;     // 1
  uint8_t bits;         // 16
  uint16_t frame_samps; // 320 (=20ms@16k)
  uint16_t reserved;    // 0
};

// ヘッダーサイズを確認（コンパイル時）
// Note: sizeof(PcmHeader) should be 16 bytes

void sendPcmHeader() {
  if (headerSent) {
    Serial.println("Header already sent, skipping");
    return;
  }
  
  if (!webSocket.isConnected()) {
    Serial.println("WebSocket not connected, cannot send header");
    return;
  }
  
  LOGF("WS", "Creating PCM header...");
  
  // バイト配列として直接ヘッダーを構築
  uint8_t header[16];
  
  // Magic "PCM1" (4 bytes)
  header[0] = 'P';
  header[1] = 'C';
  header[2] = 'M';
  header[3] = '1';
  
  // Sample rate (4 bytes, little endian)
  uint32_t sampleRate = GLOBAL_SAMPLERATE;
  header[4] = sampleRate & 0xFF;
  header[5] = (sampleRate >> 8) & 0xFF;
  header[6] = (sampleRate >> 16) & 0xFF;
  header[7] = (sampleRate >> 24) & 0xFF;
  
  // Channels (1 byte)
  header[8] = 1;
  
  // Bits (1 byte)
  header[9] = 16;
  
  // Frame samples (2 bytes, little endian)
  uint16_t frameSamps = CHUNK;
  header[10] = frameSamps & 0xFF;
  header[11] = (frameSamps >> 8) & 0xFF;
  
  // Reserved (2 bytes)
  header[12] = 0;
  header[13] = 0;
  
  // Padding to 16 bytes
  header[14] = 0;
  header[15] = 0;
  
  LOGF("WS", "Sending header: Magic=PCM1, Rate=%d, Ch=1, Bits=16, Size=16", GLOBAL_SAMPLERATE);
  
  // ヘッダーの16進ダンプを表示
  Serial.print("Header bytes: ");
  for (int i = 0; i < 16; i++) {
    Serial.printf("%02x ", header[i]);
  }
  Serial.println();
  
  // 複数回送信試行
  bool sent = false;
  for (int retry = 0; retry < 3; retry++) {
    LOGF("WS", "Sending header attempt %d...", retry + 1);
    if (webSocket.sendBIN(header, 16)) {
      sent = true;
      LOGF("WS", "Header send successful on attempt %d", retry + 1);
      break;
    }
    delay(100);
    LOGF("WS", "Header send retry %d failed", retry + 1);
  }
  
  if (sent) {
    headerSent = true;
    M5.Display.setCursor(10, 240);
    M5.Display.printf("Header sent (16 bytes)");
    LOGF("WS", "PCM Header sent successfully: 16 bytes");
  } else {
    LOGF("WS", "Failed to send PCM header");
  }
}

void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
  switch(type) {
    case WStype_DISCONNECTED:
      LOGF("WS", "Disconnected");
      M5.Display.setCursor(10, 220);
      M5.Display.println("WS Disconnected");
      streamingActive = false;
      headerSent = false;
      break;
      
    case WStype_CONNECTED:
      LOGF("WS", "Connected payload: %s", (const char*)payload);
      M5.Display.setCursor(10, 220);
      M5.Display.printf("WS Connected: %s", payload);
      Serial.printf("WebSocket connected to: %s\n", payload);
      Serial.println("WebSocket connection established");
      break;
      
    case WStype_TEXT:
      LOGF("WS", "Text: %.*s", (int)length, (const char*)payload);
      M5.Display.setCursor(10, 240);
      M5.Display.printf("WS Text: %s", payload);
      Serial.printf("WebSocket text: %s\n", payload);
      break;
      
    case WStype_ERROR:
      LOGF("WS", "Error (%u bytes)%s", (unsigned)length, length ? ": see serial" : "");
      if (length > 0) {
        Serial.print("[WS ERROR PAYLOAD] ");
        for (size_t i = 0; i < length; ++i) Serial.printf("%02X ", payload[i]);
        Serial.println();
      }
      M5.Display.setCursor(10, 260);
      M5.Display.println("WS Error");
      streamingActive = false;
      headerSent = false;
      Serial.println("WebSocket error occurred");
      break;
      
    case WStype_PING:
      LOGF("WS", "PING received");
      break;
    case WStype_PONG:
      LOGF("WS", "PONG received");
      break;

    case WStype_BIN:
      LOGF("WS", "Binary data received: %u bytes", (unsigned)length);
      break;
      
    default:
      LOGF("WS", "Event: %d", type);
      break;
  }
}

bool startStreaming() {
  if (streamingActive) return false;
  
  if (WiFi.status() != WL_CONNECTED) {
    M5.Display.setCursor(10, 220);
    M5.Display.println("WiFi not connected");
    return false;
  }
  
  Serial.println("------- Starting streaming session -------");
  M5.Display.setCursor(10, 200);
  M5.Display.println("Connecting WebSocket...");
  logNetworkInfo();
  logWsTarget(websocketHost, websocketPort, websocketPath);
  // 事前にTCP到達性をチェック（FW/隔離の切り分け）
  tcpProbe(websocketHost, websocketPort, 2000);
  
  webSocket.begin(websocketHost, websocketPort, websocketPath);
  webSocket.onEvent(webSocketEvent);
  webSocket.setReconnectInterval(3000);
  // ハートビート: 15sごとにPING、3s待ってPONGが無ければ2回で切断
  webSocket.enableHeartbeat(15000, 3000, 2);
  // ログ用にUAを付与（サーバ側で識別しやすくする）
  webSocket.setExtraHeaders("User-Agent: M5Stack-PCM-Streamer\r\n");
  
  streamingActive = true;
  headerSent = false;
  streamStartTime = millis();
  
  LOGF("WS", "Connecting to %s:%d%s", websocketHost, websocketPort, websocketPath);
  LOGF("WS", "Waiting for WebSocket connection...");
  
  return true;
}

void stopStreaming() {
  if (!streamingActive) return;
  
  webSocket.disconnect();
  streamingActive = false;
  headerSent = false;
  micInitialized = false;  // マイク初期化フラグもリセット
  
  // マイクの状態もリセット
  M5.Mic.end();
  
  M5.Display.setCursor(10, 200);
  M5.Display.println("Stream Stopped");
  Serial.println("Streaming stopped");
  Serial.println("------- Streaming session ended -------");
}

bool streamPcmData()
{
  if (!streamingActive) return false;
  
  // WebSocket接続確認
  if (!webSocket.isConnected()) {
    M5.Display.setCursor(10, 240);
    M5.Display.println("WS not connected");
    LOGF("WS", "send aborted: ws not connected");
    return false;
  }
  
  // ヘッダー送信完了確認
  if (!headerSent) {
    // ヘッダーがまだ送信されていない場合は音声データ送信を待つ
    return false;
  }
  
  // マイク初期化（初回のみ）
  if (!micInitialized) {
    M5.Mic.setSampleRate(GLOBAL_SAMPLERATE);
    if (!M5.Mic.begin()) {
      M5.Display.setCursor(10, 240);
      M5.Display.println("Mic init failed");
      return false;
    }
    micInitialized = true;
    M5.Display.setCursor(10, 260);
    M5.Display.println("Mic initialized");
  }
  
  // 音声データを録音して送信
  if (M5.Mic.record(buf, CHUNK, GLOBAL_SAMPLERATE)) {
    // フィルタとゲイン適用
    for (size_t i = 0; i < CHUNK; ++i) {
      int16_t sample = buf[i];
      
      // ハイパスフィルタでDC成分除去
      sample = hpf_rec.process(sample);
      
      // ローパスフィルタで高周波ノイズ除去
      sample = lpf_rec.process(sample);
      
      // ゲイン適用（音声レベル向上）
      float gained = (float)sample * REC_GAIN;
      
      // ソフトクリッピング（歪みを最小化）
      if (gained > 32767.0f) {
        gained = 32767.0f - (gained - 32767.0f) * 0.3f;
        if (gained > 32767.0f) gained = 32767.0f;
      }
      if (gained < -32768.0f) {
        gained = -32768.0f - (gained + 32768.0f) * 0.3f;
        if (gained < -32768.0f) gained = -32768.0f;
      }
      
      buf[i] = (int16_t)gained;
    }
    
    // WebSocket経由で送信
    if (webSocket.isConnected()) {
      size_t dataSize = CHUNK * sizeof(int16_t);
      webSocket.sendBIN((uint8_t*)buf, dataSize);
      
      // デバッグ情報（最初の数回だけ）
      static int debugCount = 0;
      if (debugCount < 5) {
        LOGF("WS", "Audio data sent: %d bytes (chunk %d)", (int)dataSize, debugCount + 1);
        debugCount++;
      }
      
      // 画面簡易メータ
      long peak = 0;
      for (size_t i = 0; i < CHUNK; ++i)
        peak = std::max<long>(peak, std::abs((int)buf[i]));
      int w = M5.Display.width() - 20;
      int bar = std::min(w, int((float)peak / 32767.f * w));
      M5.Display.fillRect(10, 60, w, 30, TFT_DARKGREY);
      M5.Display.fillRect(10, 60, bar, 30, streamingActive ? TFT_RED : TFT_GREEN);
    } else {
      M5.Display.setCursor(10, 260);
      M5.Display.println("WS not connected");
      LOGF("WS", "disconnected during audio send");
      return false;
    }
  }
  
  return true;
}

bool connectWiFi()
{
  M5.Display.setCursor(10, 180);
  M5.Display.println("Connecting WiFi...");
  
  WiFi.begin(ssid, password);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    M5.Display.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    M5.Display.setCursor(10, 200);
    M5.Display.printf("Connected: %s", WiFi.localIP().toString().c_str());
    return true;
  } else {
    M5.Display.setCursor(10, 200);
    M5.Display.println("WiFi Failed");
    return false;
  }
}

bool uploadPcmToServer(const char *path)
{
  File f = SD.open(path, FILE_READ);
  if (!f) {
    M5.Display.setCursor(10, 220);
    M5.Display.println("File open failed");
    return false;
  }
  
  size_t fileSize = f.size();
  uint8_t* buffer = (uint8_t*)malloc(fileSize);
  if (!buffer) {
    f.close();
    M5.Display.setCursor(10, 220);
    M5.Display.println("Memory alloc failed");
    return false;
  }
  
  f.read(buffer, fileSize);
  f.close();
  
  HTTPClient http;
  http.begin(String(serverURL) + "/pcm/upload");
  http.addHeader("Content-Type", "application/octet-stream");
  
  M5.Display.setCursor(10, 220);
  M5.Display.println("Uploading...");
  
  int httpResponseCode = http.POST(buffer, fileSize);
  
  free(buffer);
  
  if (httpResponseCode > 0) {
    String response = http.getString();
    M5.Display.setCursor(10, 240);
    if (httpResponseCode == 200) {
      M5.Display.println("Upload SUCCESS");
    } else {
      M5.Display.printf("Upload failed: %d", httpResponseCode);
    }
  } else {
    M5.Display.setCursor(10, 240);
    M5.Display.printf("HTTP Error: %d", httpResponseCode);
  }
  
  http.end();
  return httpResponseCode == 200;
}

uint32_t measureMicRate(uint32_t want_sr)
{
  static int16_t tmp[1024];
  const size_t N = 1024;
  uint32_t cnt = 0;
  uint32_t t0 = millis();
  while (millis() - t0 < 1000)
  {
    if (M5.Mic.record(tmp, N, want_sr))
      cnt += N;
  }
  return cnt; // ≒ 実サンプルレート
}

bool record_to_pcm(const char *path)
{

  M5.Mic.setSampleRate(GLOBAL_SAMPLERATE);
  if (!M5.Mic.begin())
    return false;

  File f = SD.open(path, FILE_WRITE);
  if (!f)
    return false;

  // 軽量ヘッダを書き込む（固定長16B）
  PcmHeader h{};
  memcpy(h.magic, "PCM1", 4);
  h.sample_rate = GLOBAL_SAMPLERATE;
  h.channels = 1;
  h.bits = 16;
  h.frame_samps = 320; // 任意（参考情報）
  f.write((uint8_t *)&h, sizeof(h));

  uint32_t target_samples = GLOBAL_SAMPLERATE * REC_SEC;
  uint32_t collected = 0;
  uint32_t t0 = millis();

  while (collected < target_samples)
  {
    if (M5.Mic.record(buf, CHUNK, GLOBAL_SAMPLERATE))
    {
      size_t n = (collected + CHUNK > target_samples) ? (target_samples - collected) : CHUNK;
      
      // フィルタとゲイン適用
      for (size_t i = 0; i < n; ++i) {
        int16_t sample = buf[i];
        
        // ハイパスフィルタでDC成分除去
        sample = hpf_rec.process(sample);
        
        // ローパスフィルタで高周波ノイズ除去
        sample = lpf_rec.process(sample);
        
        // ゲイン適用（音声レベル向上）
        float gained = (float)sample * REC_GAIN;
        
        // ソフトクリッピング（歪みを最小化）
        if (gained > 32767.0f) {
          gained = 32767.0f - (gained - 32767.0f) * 0.3f; // ソフトリミッター
          if (gained > 32767.0f) gained = 32767.0f;
        }
        if (gained < -32768.0f) {
          gained = -32768.0f - (gained + 32768.0f) * 0.3f; // ソフトリミッター
          if (gained < -32768.0f) gained = -32768.0f;
        }
        
        buf[i] = (int16_t)gained;
      }
      
      f.write((uint8_t *)buf, n * sizeof(int16_t));
      collected += n;

      // 画面簡易メータ
      long peak = 0;
      for (size_t i = 0; i < n; ++i)
        peak = std::max<long>(peak, std::abs((int)buf[i]));
      int w = M5.Display.width() - 20;
      int bar = std::min(w, int((float)peak / 32767.f * w));
      M5.Display.fillRect(10, 60, w, 30, TFT_DARKGREY);
      M5.Display.fillRect(10, 60, bar, 30, TFT_GREEN);
      M5.Display.setCursor(10, 100);
      M5.Display.printf("REC %d/%d s   ", (millis() - t0) / 1000, REC_SEC);
    }
    M5.update();
  }

  f.close();
  M5.Mic.end();
  return true;
}

bool play_pcm(const char *path)
{
  File f = SD.open(path, FILE_READ);
  if (!f)
    return false;
  if (f.size() < (int)sizeof(PcmHeader))
  {
    f.close();
    return false;
  }

  // 軽量ヘッダを読む
  PcmHeader h{};
  f.read((uint8_t *)&h, sizeof(h));
  if (memcmp(h.magic, "PCM1", 4) != 0)
  {
    f.close();
    return false;
  }
  if (h.bits != 16 || h.channels != 1)
  {
    f.close();
    return false;
  }
  uint32_t sr = h.sample_rate ? h.sample_rate : GLOBAL_SAMPLERATE;

  auto spkcfg = M5.Speaker.config();
  spkcfg.sample_rate = sr;          // ← ここ重要
  M5.Speaker.config(spkcfg);
  if (!M5.Speaker.begin()) { f.close(); return false; }
  M5.Speaker.setVolume(255);

  size_t r;
  while ((r = f.read((uint8_t *)buf, sizeof(buf))) > 0)
  {
    // r はバイト数。16bitモノラルとしてそのまま出す
    size_t samples = r / sizeof(int16_t);
    
    // フィルタとゲイン適用
    for (size_t i = 0; i < samples; ++i) {
      int16_t sample = buf[i];
      
      // ハイパスフィルタでDC成分除去
      sample = hpf_play.process(sample);
      
      // ローパスフィルタで高周波ノイズ除去
      sample = lpf_play.process(sample);
      
      // 再生ゲイン適用（スピーカー出力向上）
      float gained = (float)sample * PLAY_GAIN;
      
      // ソフトクリッピング（スピーカー歪み防止）
      if (gained > 32767.0f) {
        gained = 32767.0f - (gained - 32767.0f) * 0.3f; // ソフトリミッター
        if (gained > 32767.0f) gained = 32767.0f;
      }
      if (gained < -32768.0f) {
        gained = -32768.0f - (gained + 32768.0f) * 0.3f; // ソフトリミッター
        if (gained < -32768.0f) gained = -32768.0f;
      }
      
      buf[i] = (int16_t)gained;
    }
    
    M5.Speaker.playRaw(buf, samples, sr, /*stereo=*/false, /*channels=*/1);
    // だいたいの再生時間ぶん待機
    uint32_t ms = (samples * 1000) / sr;
    delay(ms);
    M5.update();
  }

  delay(100);
  M5.Speaker.end();
  f.close();
  return true;
}

void setup()
{
  auto cfg = M5.config();
  M5.begin(cfg);

  M5.Display.setRotation(1);
  M5.Display.fillScreen(TFT_BLACK);
  M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
  M5.Display.setTextSize(2);
  M5.Display.setCursor(10, 10);
  M5.Display.println("A: REC 5s -> /mic.pcm");
  M5.Display.setCursor(10, 30);
  M5.Display.println("B: PLAY /mic.pcm");
  M5.Display.setCursor(10, 50);
  M5.Display.println("C: STREAM Toggle");

  Serial.begin(115200);

  // ヘッダーサイズの実行時確認
  Serial.printf("PcmHeader size: %d bytes (expected: 16)\n", sizeof(PcmHeader));
  if (sizeof(PcmHeader) != 16) {
    Serial.println("WARNING: PcmHeader size is not 16 bytes!");
  }

  if (M5.Mic.begin())
  {
    delay(100);
    uint32_t m = measureMicRate(GLOBAL_SAMPLERATE); // ここが ≈18432Hz になる個体あり
    M5.Mic.end();
    // 実測値が16000Hzに近い場合のみ採用（異常値を避ける）
    if (m >= 15000 && m <= 17000) {
      GLOBAL_SAMPLERATE = m;
    } else {
      GLOBAL_SAMPLERATE = 16000; // デフォルト値を維持
    }
    Serial.printf("Measured mic rate ≈ %u Hz\n", GLOBAL_SAMPLERATE);
    M5.Display.setCursor(10, 160);
    M5.Display.printf("Mic rate: %u Hz", GLOBAL_SAMPLERATE);
  }

  // SDカード初期化（Core2は内部で SPI が設定されているので CS=4）
  if (!SD.begin(GPIO_NUM_4))
  {
    M5.Display.setCursor(10, 140);
    M5.Display.println("SD init FAILED");
  }
  else
  {
    M5.Display.setCursor(10, 140);
    M5.Display.println("SD init OK");
  }
  
  // WiFi接続
  connectWiFi();
}

void loop()
{
  M5.update();
  if (M5.BtnA.wasPressed())
  {
    M5.Display.fillRect(0, 140, M5.Display.width(), 40, TFT_BLACK);
    M5.Display.setCursor(10, 140);
    M5.Display.println("Recording...");
    bool ok = record_to_pcm(PCM_PATH);
    M5.Display.setCursor(10, 160);
    M5.Display.println(ok ? "Saved: /mic.pcm" : "REC FAILED");
  }
  if (M5.BtnB.wasPressed())
  {
    M5.Display.fillRect(0, 140, M5.Display.width(), 40, TFT_BLACK);
    M5.Display.setCursor(10, 140);
    M5.Display.println("Playing...");
    bool ok = play_pcm(PCM_PATH);
    M5.Display.setCursor(10, 160);
    M5.Display.println(ok ? "Play DONE" : "PLAY FAILED");
  }
  if (M5.BtnC.wasPressed())
  {
    M5.Display.fillRect(0, 140, M5.Display.width(), 120, TFT_BLACK);
    
    if (!streamingActive) {
      // ストリーミング開始
      M5.Display.setCursor(10, 140);
      M5.Display.println("Starting stream...");
      
      if (WiFi.status() != WL_CONNECTED) {
        M5.Display.setCursor(10, 160);
        M5.Display.println("WiFi not connected");
        M5.Display.setCursor(10, 180);
        M5.Display.println("Reconnecting...");
        connectWiFi();
      }
      
      if (WiFi.status() == WL_CONNECTED) {
        startStreaming();
      }
    } else {
      // ストリーミング終了
      M5.Display.setCursor(10, 140);
      M5.Display.println("Stopping stream...");
      stopStreaming();
      M5.Mic.end();
    }
  }
  
  // WebSocket処理
  if (streamingActive) {
    webSocket.loop();
    
    // WebSocket接続確認とヘッダー送信（フォールバック）
    if (webSocket.isConnected() && !headerSent) {
      // 接続から1秒経過後、または即座にヘッダー送信
      if (millis() - streamStartTime > 1000 || millis() - streamStartTime > 200) {
        Serial.println("Fallback: Sending header in main loop");
        sendPcmHeader();
        delay(100); // ヘッダー送信後少し待つ
      }
    }
    
    streamPcmData();
  }
  
  delay(streamingActive ? 1 : 5); // ストリーミング中は高頻度更新
}
