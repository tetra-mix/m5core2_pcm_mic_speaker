#include <M5Unified.h>
#include <SD.h>
#include <WiFi.h>

static constexpr size_t CHUNK = 2048; // 1回の読み書きサンプル数（安定性向上）
static constexpr int REC_SEC = 5;
static const char *PCM_PATH = "/mic.pcm";
static uint32_t GLOBAL_SAMPLERATE = 16000;
static constexpr float REC_GAIN = 3.0f; // 録音ゲイン（音声レベル向上）
static constexpr float PLAY_GAIN = 1.0f; // 再生ゲイン（スピーカー出力向上）

int16_t buf[CHUNK];

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

  Serial.begin(115200);

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
  delay(5);
}
