#include <Arduino.h>
#include <M5Unified.h>
#include <SD.h>
#include <SPI.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

/* =========================================================
   TinyGPS++ 互換っぽい “最小” 自力実装（RMC/GGAのみ）
   - encode(c) で1文字ずつ投入
   - 受理した文から location/date/time/speed/altitude/satellites を更新
   - distanceBetween() はハバースイン
   ========================================================= */
class TinyGPSPlus {
public:
  struct Location {
    double _lat = 0.0, _lng = 0.0;
    double lat() const { return _lat; }
    double lng() const { return _lng; }
  } location;

  struct Date {
    int _year = 0, _month = 0, _day = 0;
    int year()  const { return _year;  }
    int month() const { return _month; }
    int day()   const { return _day;   }
  } date;

  struct Time {
    int _hour = 0, _minute = 0, _second = 0;
    int hour()   const { return _hour;   }
    int minute() const { return _minute; }
    int second() const { return _second; }
  } time;

  struct Speed {
    double _kmph = 0.0;
    double kmph() const { return _kmph; }
  } speed;

  struct Altitude {
    double _meters = 0.0;
    double meters() const { return _meters; }
  } altitude;

  struct Satellites {
    int _value = 0;
    int value() const { return _value; }
  } satellites;

  bool encode(char c) {
    if (c == '\r') return false;

    if (c == '$') {
      _idx = 0;
      _buf[_idx++] = c;
      _buf[_idx] = '\0';
      return false;
    }

    if (_idx < (int)sizeof(_buf) - 1) {
      _buf[_idx++] = c;
      _buf[_idx] = '\0';
    }

    if (c == '\n') {
      parseLine(_buf);
      _idx = 0;
      return true;
    }
    return false;
  }

  static double distanceBetween(double lat1, double lon1, double lat2, double lon2) {
    // ハバースイン（m）
    const double R = 6371000.0;
    const double d2r = 0.017453292519943295; // pi/180

    double p1 = lat1 * d2r;
    double p2 = lat2 * d2r;
    double dp = (lat2 - lat1) * d2r;
    double dl = (lon2 - lon1) * d2r;

    double a = sin(dp * 0.5) * sin(dp * 0.5) +
               cos(p1) * cos(p2) * sin(dl * 0.5) * sin(dl * 0.5);
    double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
    return R * c;
  }

private:
  char _buf[160];
  int  _idx = 0;

  static uint8_t hex2byte(const char* p) {
    auto h = [](char ch) -> uint8_t {
      if (ch >= '0' && ch <= '9') return ch - '0';
      if (ch >= 'A' && ch <= 'F') return ch - 'A' + 10;
      if (ch >= 'a' && ch <= 'f') return ch - 'a' + 10;
      return 0;
    };
    return (uint8_t)((h(p[0]) << 4) | h(p[1]));
  }

  static double nmeaToDeg(const char* s) {
    // ddmm.mmmm or dddmm.mmmm → deg
    if (!s || !*s) return 0.0;
    double v = atof(s);
    int deg = (int)(v / 100.0);
    double minutes = v - (deg * 100.0);
    return (double)deg + minutes / 60.0;
  }

  void parseTime_hhmmss(const char* s) {
    if (!s || strlen(s) < 6) return;
    // hhmmss( .sss は無視)
    char hh[3] = { s[0], s[1], 0 };
    char mm[3] = { s[2], s[3], 0 };
    char ss[3] = { s[4], s[5], 0 };
    time._hour   = atoi(hh);
    time._minute = atoi(mm);
    time._second = atoi(ss);
  }

  void parseDate_ddmmyy(const char* s) {
    if (!s || strlen(s) < 6) return;
    char dd[3] = { s[0], s[1], 0 };
    char mm[3] = { s[2], s[3], 0 };
    char yy[3] = { s[4], s[5], 0 };
    int d = atoi(dd);
    int m = atoi(mm);
    int y = atoi(yy);
    // 80-99 は 1900台、それ以外は 2000台に寄せる
    int full = (y >= 80) ? (1900 + y) : (2000 + y);
    date._day = d;
    date._month = m;
    date._year = full;
  }

  void setLatLon(const char* lat, const char* ns, const char* lon, const char* ew) {
    if (!lat || !*lat || !lon || !*lon) return;

    double la = nmeaToDeg(lat);
    double lo = nmeaToDeg(lon);

    if (ns && *ns == 'S') la = -la;
    if (ew && *ew == 'W') lo = -lo;

    location._lat = la;
    location._lng = lo;
  }

  void parseRMC(char** f, int n) {
    // $..RMC, time, status, lat, N/S, lon, E/W, speed(knots), course, date, ...
    if (n < 10) return;
    if (!f[2] || f[2][0] != 'A') return; // A=valid
    parseTime_hhmmss(f[1]);
    setLatLon(f[3], f[4], f[5], f[6]);

    double knots = (f[7] && *f[7]) ? atof(f[7]) : 0.0;
    speed._kmph = knots * 1.852;

    parseDate_ddmmyy(f[9]);
  }

  void parseGGA(char** f, int n) {
    // $..GGA, time, lat, N/S, lon, E/W, fixq, sats, hdop, alt(m), ...
    if (n < 10) return;
    parseTime_hhmmss(f[1]);
    setLatLon(f[2], f[3], f[4], f[5]);

    satellites._value = (f[7] && *f[7]) ? atoi(f[7]) : satellites._value;
    altitude._meters  = (f[9] && *f[9]) ? atof(f[9]) : altitude._meters;
  }

  void parseLine(char* line) {
    if (!line || line[0] != '$') return;

    // チェックサム検証
    char* body = line + 1;                 // '$'の次
    char* ast  = strchr(body, '*');
    if (!ast || (ast - body) <= 0) return;

    uint8_t cs = 0;
    for (char* p = body; p < ast; ++p) cs ^= (uint8_t)(*p);

    if (strlen(ast) < 3) return;           // "*hh"
    uint8_t sent = hex2byte(ast + 1);
    if (cs != sent) return;

    *ast = '\0'; // ここで文末を切る（チェックサム以降無視）

    // CSV分割
    char* fields[24];
    int nf = 0;
    char* save = nullptr;
    for (char* tok = strtok_r(body, ",", &save); tok && nf < 24; tok = strtok_r(nullptr, ",", &save)) {
      fields[nf++] = tok;
    }
    if (nf <= 0) return;

    // type末尾3文字で判定（GPRMC/GNRMC/GLRMC等をまとめて拾う）
    const char* type = fields[0];
    size_t len = strlen(type);
    if (len < 3) return;
    const char* suf = type + (len - 3);

    if (strcmp(suf, "RMC") == 0) {
      parseRMC(fields, nf);
    } else if (strcmp(suf, "GGA") == 0) {
      parseGGA(fields, nf);
    }
  }
};

/* =========================================================
   元コードのグローバル
   ========================================================= */
TinyGPSPlus gps;

File file;
String fname = "/LAP_log.csv";

int YEAR, MONTH, DAY, HOUR, MINUTE, SECOND, LapCount, SatVal, BestLapNum;

float LAT0 = 35.3698692322f, LONG0 = 138.9336547852f;
float LAT, LONG, KMPH, TopSpeed, ALTITUDE, distanceToMeter0, BeforeTime;
float LAP, LAP1, LAP2, LAP3, LAP4, LAP5, BestLap = 99999.0f, AverageLap, Sprit;

bool LAPCOUNTNOW, LAPRADchange;
float LAPRAD = 5.0f;  // ラップ計測トリガー半径(m)
long lastdulation;

/* =========================================================
   差分描画用キャッシュ＆ヘルパ
   ========================================================= */
struct UiCache {
  char timeLine[32]    = "";
  char sat[8]          = "";
  char lapPanelKey[32] = "";
  char delta[16]       = "";
  uint16_t deltaBg     = 0xFFFF;
  char elapsed[8]      = "";
  char bestKey[24]     = "";
  char avgKey[24]      = "";
  char speed[16]       = "";
  char dist[16]        = "";
  char lapRad[8]       = "";
  int barAvgW          = -1;
  int barBestW         = -1;
};
static UiCache ui;

static void cacheCopy(char* dst, size_t n, const char* src) {
  if (!dst || n == 0) return;
  snprintf(dst, n, "%s", src ? src : "");
}

static bool drawTextIfChanged(int x, int y, int w, int h,
                              uint16_t bg, uint16_t fg, int size,
                              const char* text, char* cache, size_t cacheN,
                              bool force = false)
{
  if (!force && text && cache && strcmp(text, cache) == 0) return false;

  M5.Display.fillRect(x, y, w, h, bg);
  M5.Display.setTextColor(fg);
  M5.Display.setTextSize(size);
  M5.Display.setCursor(x, y);
  M5.Display.print(text ? text : "");
  cacheCopy(cache, cacheN, text);
  return true;
}

static int clampi(int v, int lo, int hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static void drawStaticUI() {
  M5.Display.fillScreen(BLACK);

  // 下段の固定ラベル
  M5.Display.setTextSize(1);

  M5.Display.setCursor(15, 228);
  M5.Display.setTextColor(ORANGE);
  M5.Display.print("SET ");
  M5.Display.setTextColor(CYAN);
  M5.Display.print("Zero");
  M5.Display.setTextColor(ORANGE);
  M5.Display.print("-Point");

  M5.Display.setTextColor(ORANGE);
  M5.Display.setCursor(120, 228);
  M5.Display.print("Rad= ");

  M5.Display.setTextColor(ORANGE);
  M5.Display.setCursor(230, 228);
  M5.Display.print("Lap Count");

  // GPSラベル
  M5.Display.setTextColor(CYAN);
  M5.Display.setTextSize(1);
  M5.Display.setCursor(245, 5);
  M5.Display.print("G P S:");

  // 前ラップ背景（黄色帯）
  M5.Display.fillRect(0, 20, 320, 59, YELLOW);

  // 経過時間枠
  M5.Display.drawRoundRect(180, 80, 140, 50, 10, WHITE);
  M5.Display.setTextColor(WHITE);
  M5.Display.setTextSize(2);
  M5.Display.setCursor(300, 110);
  M5.Display.print("s");

  // バー枠
  M5.Display.drawRect(10, 200, 300, 25, WHITE);

  // Best/Average ラベル（値は差分描画）
  M5.Display.setTextColor(CYAN);
  M5.Display.setTextSize(2);
  M5.Display.setCursor(20, 145);
  M5.Display.print("Best");

  M5.Display.setTextColor(PINK);
  M5.Display.setTextSize(2);
  M5.Display.setCursor(20, 175);
  M5.Display.print("Average");
}

/* =========================================================
   既存関数プロトタイプ
   ========================================================= */
void ReadGPS();
void CountLAP();
void showvalue(int dulation);
void writeData();

/* =========================================================
   setup / loop（loopは使わない）
   ========================================================= */
void setup()
{
  Serial.begin(115200);
  Serial2.begin(115200);

  auto cfg = M5.config();
  M5.begin(cfg);

  SD.begin();

  M5.Speaker.end();

  M5.Display.setBrightness(255);
  M5.Display.setTextColor(WHITE);
  M5.Display.setTextSize(1);
  M5.Display.setCursor(10, 10);
  M5.Display.print("Start");

  LapCount = 0;

  file = SD.open(fname, FILE_APPEND);
  if (file) {
    file.println("LAPCount,LapTime,TopSpeed,YYYY/MM/DD/Hour:Minute:Second");
    file.close();
  }

  // 固定UIは1回だけ描画
  drawStaticUI();

  // ===== loop() を使わず setup内で回す =====
  for (;;) {
    M5.update();   // 入力更新（レスポンス改善）

    ReadGPS();
    CountLAP();
    showvalue(100);

    delay(1);      // ESP32系の詰まり/WDT対策（yieldでも可）
  }
}

void loop()
{
  // 使わない
}

/* =========================================================
   GPS読み取り＆状態更新
   ========================================================= */
void ReadGPS()
{
  // GPSデコード
  while (Serial2.available()) {
    char c = (char)Serial2.read();
    gps.encode(c);
    Serial.print(c);
  }

  // GPSデータ展開
  LAT = (float)gps.location.lat();
  LONG = (float)gps.location.lng();
  YEAR = gps.date.year();
  MONTH = gps.date.month();
  DAY = gps.date.day();
  HOUR = gps.time.hour();
  MINUTE = gps.time.minute();
  SECOND = gps.time.second();
  KMPH = (float)gps.speed.kmph();
  ALTITUDE = (float)gps.altitude.meters();
  distanceToMeter0 = (float)TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), LAT0, LONG0);
  SatVal = gps.satellites.value();

  // ラップ計測中の最高速度
  if (TopSpeed < KMPH) {
    TopSpeed = KMPH;
  }

  // JST変換（簡易：日付繰り上げのみ。月末処理などは元コード同様未対応）
  HOUR += 9;
  if (HOUR >= 24) {
    DAY += HOUR / 24;
    HOUR = HOUR % 24;
  }

  // 相対距離原点設定（BtnA）
  if (M5.BtnA.isPressed()) {
    LAT0 = LAT;
    LONG0 = LONG;
    distanceToMeter0 = (float)TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), LAT0, LONG0);
  }

  // LAPRAD変更（BtnB）
  if (!M5.BtnB.isPressed() && LAPRADchange == true) {
    LAPRADchange = false;
  }

  if (M5.BtnB.isPressed() && LAPRADchange == false) {
    if (LAPRAD == 50) {
      LAPRAD = 0;
    }
    LAPRAD += 5;
    LAPRADchange = true;
  }
}

/* =========================================================
   ラップ計測
   ========================================================= */
void CountLAP()
{
  if (distanceToMeter0 >= LAPRAD && !M5.BtnC.isPressed() && LAPCOUNTNOW == true) {
    LAPCOUNTNOW = false;
  }

  // 4ラップ計測（元コードそのまま）
  if (((distanceToMeter0 != 0 && distanceToMeter0 <= LAPRAD) || M5.BtnC.isPressed())
      && LAPCOUNTNOW == false
      && ((millis() - BeforeTime) / 1000) > 10)
  {
    if (LapCount > 0) {
      LAP5 = LAP4;
      LAP4 = LAP3;
      LAP3 = LAP2;
      LAP2 = LAP1;
      LAP1 = LAP;

      LAP = (millis() - BeforeTime) / 1000;
      BeforeTime = millis();

      if (LAP < BestLap) {
        BestLap = LAP;
        BestLapNum = LapCount;
      }
      writeData();

      Sprit += LAP;
      if (LapCount > 1) {
        AverageLap = Sprit / LapCount;
      }
    } else {
      BeforeTime = millis();
    }

    LapCount++;
    LAPCOUNTNOW = true;
  }
}

/* =========================================================
   差分描画（変更があった場所だけ更新）
   ========================================================= */
void showvalue(int dulation) {
  if (millis() <= lastdulation + dulation) return;
  lastdulation = millis();

  char buf[64];

  // ===== 時刻表示 =====
  snprintf(buf, sizeof(buf), "%04d/%02d/%02d %02d:%02d:%02d",
           YEAR, MONTH, DAY, HOUR, MINUTE, SECOND);

  drawTextIfChanged(
    0, 0, 320, 18,
    BLACK, WHITE, 2,
    buf, ui.timeLine, sizeof(ui.timeLine)
  );

  // ===== 衛星数 =====
  snprintf(buf, sizeof(buf), "%d", SatVal);
  drawTextIfChanged(
    285, 1, 35, 18,
    BLACK, CYAN, 2,
    buf, ui.sat, sizeof(ui.sat)
  );

  // ===== LAPRAD 数字部分 =====
  snprintf(buf, sizeof(buf), "%.0f", LAPRAD);
  drawTextIfChanged(
    165, 228, 40, 12,
    BLACK, 47072, 1,
    buf, ui.lapRad, sizeof(ui.lapRad)
  );

  // ===== 前ラップ表示（黄色帯：キーが変わった時だけ更新）=====
  char key[32];
  if (LapCount > 1) {
    snprintf(key, sizeof(key), "L%d:%.3f", LapCount - 1, LAP);
  } else if (LapCount == 1) {
    float t = (millis() - BeforeTime) / 1000.0f;
    snprintf(key, sizeof(key), "L1:%.3f", t);
  } else {
    snprintf(key, sizeof(key), "L0");
  }

  if (strcmp(key, ui.lapPanelKey) != 0) {
    cacheCopy(ui.lapPanelKey, sizeof(ui.lapPanelKey), key);

    M5.Display.fillRect(0, 20, 320, 59, YELLOW);
    M5.Display.setTextColor(BLACK);

    if (LapCount > 1) {
      M5.Display.setTextSize(3);
      M5.Display.setCursor(15, 30);
      M5.Display.print(LapCount - 1);
      M5.Display.print(">");

      M5.Display.setTextSize(6);
      M5.Display.print(LAP, 3);
    } else if (LapCount == 1) {
      M5.Display.setTextSize(3);
      M5.Display.setCursor(15, 30);
      M5.Display.print(LapCount);
      M5.Display.print(">");

      M5.Display.setTextSize(6);
      M5.Display.print((millis() - BeforeTime) / 1000.0f, 3);
    }
  }

  // ===== タイム差（色と値が変わった時だけ）=====
  float d = (LapCount > 1) ? (LAP - LAP1) : 0.0f;
  char dstr[16];
  if (d > 0) snprintf(dstr, sizeof(dstr), "+%.1f", d);
  else       snprintf(dstr, sizeof(dstr), "%.1f", d);

  uint16_t bg = (d <= 0) ? BLUE : RED;

  if (bg != ui.deltaBg || strcmp(dstr, ui.delta) != 0) {
    ui.deltaBg = bg;
    cacheCopy(ui.delta, sizeof(ui.delta), dstr);

    M5.Display.fillRect(1, 80, 178, 50, bg);

    M5.Display.setTextSize(4);
    M5.Display.setTextColor(BLACK);
    M5.Display.setCursor(10, 92);
    M5.Display.print(dstr);

    M5.Display.setTextColor(WHITE);
    M5.Display.setCursor(8, 90);
    M5.Display.print(dstr);
  }

  // ===== 経過時間 =====
  int elapsed = (int)((millis() - BeforeTime) / 1000.0f);
  snprintf(buf, sizeof(buf), "%d", elapsed);
  drawTextIfChanged(
    190, 90, 105, 30,
    BLACK, WHITE, 4,
    buf, ui.elapsed, sizeof(ui.elapsed)
  );

  // ===== Best =====
  char bestKey[24];
  if (BestLap != 99999) snprintf(bestKey, sizeof(bestKey), "(%d)%.3f", BestLapNum, BestLap);
  else                  snprintf(bestKey, sizeof(bestKey), "NONE");

  if (strcmp(bestKey, ui.bestKey) != 0) {
    cacheCopy(ui.bestKey, sizeof(ui.bestKey), bestKey);

    M5.Display.fillRect(0, 135, 320, 35, BLACK);

    if (BestLap != 99999) {
      M5.Display.setTextColor(CYAN);
      M5.Display.setTextSize(2);
      M5.Display.setCursor(20, 145);
      M5.Display.print("Best(");
      M5.Display.print(BestLapNum);
      M5.Display.print(")");

      M5.Display.setCursor(120, 140);
      M5.Display.setTextSize(3);
      M5.Display.print("> ");
      M5.Display.print(BestLap);
    } else {
      M5.Display.setTextColor(CYAN);
      M5.Display.setTextSize(2);
      M5.Display.setCursor(20, 145);
      M5.Display.print("Best");
    }
  }

  // ===== Average =====
  char avgKey[24];
  if (AverageLap != 0) snprintf(avgKey, sizeof(avgKey), "%.3f", AverageLap);
  else                snprintf(avgKey, sizeof(avgKey), "NONE");

  if (strcmp(avgKey, ui.avgKey) != 0) {
    cacheCopy(ui.avgKey, sizeof(ui.avgKey), avgKey);

    M5.Display.fillRect(0, 170, 320, 28, BLACK);

    M5.Display.setTextColor(PINK);
    M5.Display.setTextSize(2);
    M5.Display.setCursor(20, 175);
    M5.Display.print("Average");

    if (AverageLap != 0) {
      M5.Display.setCursor(120, 170);
      M5.Display.setTextSize(3);
      M5.Display.print("> ");
      M5.Display.print(AverageLap);
    }
  }

  // ===== バー（毎秒変化しやすい）=====
  float tsec = (millis() - BeforeTime) / 1000.0f;

  int wAvg = 0;
  if (AverageLap > 0) {
    float r = (AverageLap - tsec) / AverageLap;
    wAvg = clampi((int)(300.0f * r), 0, 300);
  }

  int wBest = 0;
  if (BestLap != 99999) {
    float r = (BestLap - tsec) / BestLap;
    wBest = clampi((int)(300.0f * r), 0, 300);
  }

  bool barsChanged = (wAvg != ui.barAvgW) || (wBest != ui.barBestW);
  if (barsChanged) {
    ui.barAvgW = wAvg;
    ui.barBestW = wBest;

    M5.Display.fillRect(10, 200, 300, 25, BLACK);
    if (wAvg > 0)  M5.Display.fillRect(10, 200, wAvg, 25, PINK);
    if (wBest > 0) M5.Display.fillRect(10, 200, wBest, 25, CYAN);
    M5.Display.drawRect(10, 200, 300, 25, WHITE);
  }

  // ===== 時速・距離（バー更新で塗られるので必要なら強制再描画）=====
  snprintf(buf, sizeof(buf), "%.1f km/h", KMPH);
  drawTextIfChanged(
    20, 205, 130, 18,
    BLACK, WHITE, 2,
    buf, ui.speed, sizeof(ui.speed),
    barsChanged
  );

  snprintf(buf, sizeof(buf), "%.1f m", distanceToMeter0);
  drawTextIfChanged(
    160, 205, 150, 18,
    BLACK, WHITE, 2,
    buf, ui.dist, sizeof(ui.dist),
    barsChanged
  );
}

/* =========================================================
   SD書き込み（元コード準拠）
   ========================================================= */
void writeData() {
  file = SD.open(fname , FILE_APPEND);
  if (!file) return;

  file.print((String)LapCount + ",");
  file.print((String)LAP + ",");
  file.print((String)TopSpeed + ",");
  file.println((String)YEAR + "/" + (String)MONTH + "/" + (String)DAY + "-" + (String)HOUR + ":" + (String)MINUTE + ":" + (String)SECOND + ",");
  file.close();

  TopSpeed = 0; // 最高速度をリセット
}
