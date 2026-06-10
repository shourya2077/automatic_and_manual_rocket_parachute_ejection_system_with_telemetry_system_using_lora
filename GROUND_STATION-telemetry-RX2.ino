  /*
  * ============================================================
  *  ROCKET PARACHUTE EJECTION SYSTEM   — GROUND STATION
  *  Hardware: ESP32 + LoRa SX1278
  *  WiFi: Access Point  |  Web UI at http://192.168.4.1
  * ============================================================
  *  LoRa wiring (same as rocket):
  *    NSS → GPIO5   SCK → GPIO18   MISO → GPIO19   MOSI → GPIO23
  *    RST → GPIO14  DIO0 → GPIO2   VCC → 3.3V  GND → GND
  *
  *  MUST match rocket LoRa settings exactly.
  *
  
  */

  #include <WiFi.h>
  #include <WebServer.h>
  #include <SPI.h>
  #include <LoRa.h>
  #include <ArduinoJson.h>
  #include <math.h>

  const char* AP_SSID     = "RocketGS";
  const char* AP_PASSWORD = "ground123";

  // ── LoRa ─────────────────────────────────────────────────────
  #define LORA_SS    5
  #define LORA_RST   14
  #define LORA_DIO0  2
  #define LORA_FREQ       433E6
  #define LORA_SF         7       // MUST match rocket
  #define LORA_BW         125E3
  #define LORA_CR         5
  #define LORA_TX_POWER   17
  #define LORA_SYNC_WORD  0xB4

  // ── Ground-side telemetry log (for CSV download) ─────────────
  #define GS_LOG_SIZE 500
  struct GsLog {
    uint32_t t;
    float alt, maxAlt, netAccel, vertAccel, accelX, accelY, accelZ;
    int   rssi;
  };
  GsLog gsLog[GS_LOG_SIZE];
  int   gsLogCount = 0;
  bool  gsLogging  = false;
  unsigned long gsLogStart = 0;

  // ── Remote log download state ─────────────────────────────────
  #define REMOTE_LOG_SIZE 500
  struct RemoteEntry { uint32_t t; float alt,net,vert,ax,ay,az; };
  RemoteEntry remoteLog[REMOTE_LOG_SIZE];
  int  remoteLogCount   = 0;
  int  remoteLogExpected= 0;
  bool remoteLogActive  = false;
  bool remoteLogDone    = false;

  // ── Last ACK from rocket ─────────────────────────────────────
  char lastAckCmd[16] = "";
  bool lastAckOk      = false;
  unsigned long lastAckMs = 0;

  // ── Telemetry state ──────────────────────────────────────────
  struct Telemetry {
    float alt=0, maxAlt=0, pressure=0, temp=0;
    bool  armed=false, deployed=false, apogee=false;
    int   falling=0;
    float netAccel=1.0f, vertAccel=0.0f, tilt=0.0f;
    bool  freeFall=false, mpuApogee=false;
    int   accelFall=0;
    float mpuTemp=0;
    bool  mpuOK=false, bmpOK=false;
    int   fusionMode=1;
    unsigned long freeHeap=0;
    bool  recording=false;
    int   logCount=0;
    float accelX=0, accelY=0, accelZ=1.0f;
    int   rssi=0;
    float snr=0;
    unsigned long lastPacketMs=0;
    bool  linkOK=false;
  } tele;

  // ── Ground settings cache ────────────────────────────────────
  struct RocketSettings {
    int   servo1Angle=90, servo2Angle=90;
    bool  autoEnabled=false;
    int   apogeeMargin=200;
    int   activeServo=1;
    float seaLevel=1013.25f;
    int   fusionMode=1;
    int   freeFallThresh=300;
  } rocketCfg;

  WebServer server(80);

  // ── Send command with optional retries ───────────────────────
  void sendCommand(const String& cmd, int retries=3) {
    for (int i=0; i<retries; i++) {
      LoRa.beginPacket();
      LoRa.print(cmd);
      LoRa.endPacket();
      Serial.printf("[CMD TX] %s (attempt %d)\n", cmd.c_str(), i+1);
      if (retries>1) delay(50);
    }
  }

  // ── Parse telemetry packet (24 fields) ───────────────────────
  // T,alt,maxAlt,press,temp,armed,deployed,apogee,falling,
  //   netAcc,vertAcc,tilt,freeFall,mpuApogee,accelFall,
  //   mpuTemp,mpuOK,bmpOK,fusionMode,heap,rec,logCnt,
  //   ax,ay,az
  void parseTelemetry(const String& pkt, int rssi, float snr) {
    if (!pkt.startsWith("T,")) return;
    String s = pkt.substring(2);

    // Split all tokens into array
    float v[24];
    int idx=0, start=0;
    while (idx<24) {
      int comma = s.indexOf(',', start);
      String tok = (comma<0) ? s.substring(start) : s.substring(start,comma);
      tok.trim();
      v[idx++] = tok.toFloat();
      if (comma<0) break;
      start = comma+1;
    }
    Serial.printf("[PARSE] %d fields\n", idx);
    // Accept packet if we got at least 21 fields (ax/ay/az optional, older fw)
    if (idx < 21) {
      Serial.println("[PARSE] Too short, dropped");
      return;
    }
    tele.alt        = v[0];   tele.maxAlt    = v[1];
    tele.pressure   = v[2];   tele.temp      = v[3];
    tele.armed      = (int)v[4]; tele.deployed= (int)v[5];
    tele.apogee     = (int)v[6]; tele.falling = (int)v[7];
    tele.netAccel   = v[8];   tele.vertAccel = v[9];
    tele.tilt       = v[10];  tele.freeFall  = (int)v[11];
    tele.mpuApogee  = (int)v[12]; tele.accelFall=(int)v[13];
    tele.mpuTemp    = v[14];  tele.mpuOK     = (int)v[15];
    tele.bmpOK      = (int)v[16]; tele.fusionMode=(int)v[17];
    tele.freeHeap   = (unsigned long)v[18];
    tele.recording  = (int)v[19]; tele.logCount=(int)v[20];
    if (idx >= 24) {
      tele.accelX = v[21]; tele.accelY = v[22]; tele.accelZ = v[23];
    } else {
      // Estimate from tilt+vert for older firmware
      float tz = tele.vertAccel + 1.0f;
      float tr = tele.tilt * 3.14159f / 180.0f;
      tele.accelX = sinf(tr)*0.707f;
      tele.accelY = sinf(tr)*0.707f;
      tele.accelZ = tz;
    }
    tele.rssi = rssi; tele.snr = snr;
    tele.lastPacketMs = millis(); tele.linkOK = true;

    // Accumulate into ground log if logging
    if (gsLogging && gsLogCount < GS_LOG_SIZE) {
      gsLog[gsLogCount++] = {
        (uint32_t)(millis()-gsLogStart),
        tele.alt, tele.maxAlt, tele.netAccel, tele.vertAccel,
        tele.accelX, tele.accelY, tele.accelZ, rssi
      };
    }
  }

  // ── Parse ACK packet ─────────────────────────────────────────
  void parseAck(const String& pkt) {
    // Format: A,CMD,ok
    if (!pkt.startsWith("A,")) return;
    String s = pkt.substring(2);
    int comma = s.indexOf(',');
    if (comma < 0) return;
    strncpy(lastAckCmd, s.substring(0,comma).c_str(), 15);
    lastAckOk  = s.substring(comma+1).toInt() == 1;
    lastAckMs  = millis();
    Serial.printf("[ACK] cmd=%s ok=%d\n", lastAckCmd, lastAckOk);
  }

  // ── Parse log count / entry / end ────────────────────────────
  void parseLogPacket(const String& pkt) {
    if (pkt.startsWith("LC,")) {
      remoteLogExpected = pkt.substring(3).toInt();
      remoteLogCount    = 0;
      remoteLogActive   = true;
      remoteLogDone     = false;
      Serial.printf("[LOG] Expecting %d entries\n", remoteLogExpected);
    }
    else if (pkt.startsWith("L,") && remoteLogActive && remoteLogCount<REMOTE_LOG_SIZE) {
      // L,t,alt,net,vert,ax,ay,az
      String s=pkt.substring(2);
      float v[7]; int idx=0,start=0;
      while(idx<7){
        int c=s.indexOf(',',start);
        String tok=(c<0)?s.substring(start):s.substring(start,c);
        v[idx++]=tok.toFloat();
        if(c<0) break;
        start=c+1;
      }
      if(idx>=7){
        remoteLog[remoteLogCount++]={(uint32_t)v[0],v[1],v[2],v[3],v[4],v[5],v[6]};
      }
    }
    else if (pkt=="LE") {
      remoteLogActive=false; remoteLogDone=true;
      Serial.printf("[LOG] Download complete: %d entries\n", remoteLogCount);
    }
  }

  // ── Web: telemetry JSON ───────────────────────────────────────
  void handleTelemetry() {
    if (millis()-tele.lastPacketMs>3000) tele.linkOK=false;
    StaticJsonDocument<900> doc;
    doc["alt"]       = tele.alt;      doc["maxAlt"]   = tele.maxAlt;
    doc["pressure"]  = tele.pressure; doc["temp"]     = tele.temp;
    doc["armed"]     = tele.armed;    doc["deployed"] = tele.deployed;
    doc["apogee"]    = tele.apogee;   doc["falling"]  = tele.falling;
    doc["netAccel"]  = serialized(String(tele.netAccel,3));
    doc["vertAccel"] = serialized(String(tele.vertAccel,3));
    doc["tilt"]      = serialized(String(tele.tilt,1));
    doc["freeFall"]  = tele.freeFall; doc["mpuApogee"]= tele.mpuApogee;
    doc["accelFall"] = tele.accelFall;doc["mpuTemp"]  = serialized(String(tele.mpuTemp,1));
    doc["mpuOK"]     = tele.mpuOK;    doc["bmpOK"]    = tele.bmpOK;
    doc["fusionMode"]= tele.fusionMode;doc["freeHeap"]= tele.freeHeap;
    doc["recording"] = tele.recording;doc["logCount"] = tele.logCount;
    doc["accelX"]    = serialized(String(tele.accelX,3));
    doc["accelY"]    = serialized(String(tele.accelY,3));
    doc["accelZ"]    = serialized(String(tele.accelZ,3));
    doc["rssi"]      = tele.rssi;     doc["snr"]      = serialized(String(tele.snr,1));
    doc["linkOK"]    = tele.linkOK;
    doc["auto"]      = rocketCfg.autoEnabled;
    doc["gsHeap"]    = ESP.getFreeHeap();
    doc["gsLogCount"]= gsLogCount;
    doc["gsLogging"] = gsLogging;
    // ACK status
    doc["lastAckCmd"]= lastAckCmd;
    doc["lastAckOk"] = lastAckOk;
    doc["lastAckAge"]= (millis()-lastAckMs)/1000;
    doc["remoteLogDone"]   = remoteLogDone;
    doc["remoteLogCount"]  = remoteLogCount;
    String out; serializeJson(doc,out);
    server.send(200,"application/json",out);
  }

  // ── Web: command handlers ─────────────────────────────────────
  void handleArm()    { sendCommand("ARM");    server.send(200,"application/json","{\"ok\":true,\"msg\":\"ARM sent x3\"}"); }
  void handleDisarm() { sendCommand("DISARM"); server.send(200,"application/json","{\"ok\":true,\"msg\":\"DISARM sent\"}"); }
  void handleFire()   {
    // Fire is critical — send 5 times
    sendCommand("FIRE",5);
    server.send(200,"application/json","{\"ok\":true,\"msg\":\"FIRE sent x5\"}");
  }
  void handleReset()  { sendCommand("RESET");  server.send(200,"application/json","{\"ok\":true,\"msg\":\"RESET sent\"}"); }
  void handleTare()   { sendCommand("TARE");   server.send(200,"application/json","{\"ok\":true,\"msg\":\"TARE sent\"}"); }
  void handleCalMPU() { sendCommand("CALMPU"); server.send(200,"application/json","{\"ok\":true,\"msg\":\"CALMPU sent\"}"); }
  void handlePing()   { sendCommand("PING",1); server.send(200,"application/json","{\"ok\":true,\"msg\":\"PING sent\"}"); }
  void handleRecordStart(){
    gsLogging=true; gsLogStart=millis(); gsLogCount=0;
    sendCommand("RECSTART");
    server.send(200,"application/json","{\"ok\":true,\"msg\":\"REC started (rocket+ground)\"}");
  }
  void handleRecordStop(){
    gsLogging=false;
    sendCommand("RECSTOP");
    server.send(200,"application/json","{\"ok\":true,\"msg\":\"REC stopped\"}");
  }
  void handleGetLog(){
    // Ground station CSV download (packets buffered here on ground)
    String csv="time_ms,alt_m,maxAlt_m,netAccel_g,vertAccel_g,accelX,accelY,accelZ,rssi_dBm\n";
    for(int i=0;i<gsLogCount;i++){
      csv+=String(gsLog[i].t)+","+
          String(gsLog[i].alt,2)+","+
          String(gsLog[i].maxAlt,2)+","+
          String(gsLog[i].netAccel,3)+","+
          String(gsLog[i].vertAccel,3)+","+
          String(gsLog[i].accelX,3)+","+
          String(gsLog[i].accelY,3)+","+
          String(gsLog[i].accelZ,3)+","+
          String(gsLog[i].rssi)+"\n";
    }
    server.sendHeader("Content-Disposition","attachment; filename=ground_log.csv");
    server.send(200,"text/csv",csv);
  }
  void handleGetRemoteLog(){
    // Remote (rocket) log received over LoRa
    if(!remoteLogDone){
      server.send(503,"application/json","{\"ok\":false,\"msg\":\"Remote log not downloaded yet. Use GETLOG button.\"}");
      return;
    }
    String csv="time_ms,alt_m,netAccel_g,vertAccel_g,accelX,accelY,accelZ\n";
    for(int i=0;i<remoteLogCount;i++){
      csv+=String(remoteLog[i].t)+","+
          String(remoteLog[i].alt,2)+","+
          String(remoteLog[i].net,3)+","+
          String(remoteLog[i].vert,3)+","+
          String(remoteLog[i].ax,3)+","+
          String(remoteLog[i].ay,3)+","+
          String(remoteLog[i].az,3)+"\n";
    }
    server.sendHeader("Content-Disposition","attachment; filename=rocket_log.csv");
    server.send(200,"text/csv",csv);
  }
  void handleRequestRemoteLog(){
    // Tell rocket to transmit its log over LoRa
    remoteLogDone=false; remoteLogCount=0; remoteLogActive=false;
    sendCommand("GETLOG",1);
    server.send(200,"application/json","{\"ok\":true,\"msg\":\"GETLOG sent — wait for transfer\"}");
  }

  void handleSettings(){
    if(!server.hasArg("plain")){server.send(400,"application/json","{\"ok\":false}");return;}
    String body=server.arg("plain");
    StaticJsonDocument<256> doc;
    if(deserializeJson(doc,body)){server.send(400,"application/json","{\"ok\":false,\"msg\":\"Bad JSON\"}");return;}
    if(doc.containsKey("servo1Angle"))    rocketCfg.servo1Angle  =doc["servo1Angle"];
    if(doc.containsKey("servo2Angle"))    rocketCfg.servo2Angle  =doc["servo2Angle"];
    if(doc.containsKey("autoEnabled"))    rocketCfg.autoEnabled  =doc["autoEnabled"];
    if(doc.containsKey("apogeeMargin"))   rocketCfg.apogeeMargin =doc["apogeeMargin"];
    if(doc.containsKey("activeServo"))    rocketCfg.activeServo  =doc["activeServo"];
    if(doc.containsKey("seaLevel"))       rocketCfg.seaLevel     =doc["seaLevel"];
    if(doc.containsKey("fusionMode"))     rocketCfg.fusionMode   =doc["fusionMode"];
    if(doc.containsKey("freeFallThresh")) rocketCfg.freeFallThresh=doc["freeFallThresh"];
    sendCommand("SET:"+body);
    server.send(200,"application/json","{\"ok\":true,\"msg\":\"Settings forwarded to rocket\"}");
  }
  void handleGetSettings(){
    StaticJsonDocument<192> doc;
    doc["servo1Angle"]   =rocketCfg.servo1Angle;
    doc["servo2Angle"]   =rocketCfg.servo2Angle;
    doc["autoEnabled"]   =rocketCfg.autoEnabled;
    doc["apogeeMargin"]  =rocketCfg.apogeeMargin;
    doc["activeServo"]   =rocketCfg.activeServo;
    doc["seaLevel"]      =rocketCfg.seaLevel;
    doc["fusionMode"]    =rocketCfg.fusionMode;
    doc["freeFallThresh"]=rocketCfg.freeFallThresh;
    String out; serializeJson(doc,out);
    server.send(200,"application/json",out);
  }
  void handleTestServo(){
    if(!server.hasArg("plain")){server.send(400,"application/json","{\"ok\":false}");return;}
    StaticJsonDocument<64> doc;
    if(deserializeJson(doc,server.arg("plain"))){server.send(400,"application/json","{\"ok\":false}");return;}
    int sv =constrain((int)(doc["servo"]|1),1,2);
    int ang=constrain((int)(doc["angle"]|45),0,180);
    sendCommand("TEST:"+String(sv)+","+String(ang));
    server.send(200,"application/json","{\"ok\":true,\"msg\":\"Servo test sent\"}");
  }

  // ─────────────────────────────────────────────────────────────
  //  Web UI
  // ─────────────────────────────────────────────────────────────
  const char INDEX_HTML[] PROGMEM = R"rawliteral(
  <!DOCTYPE html>
  <html lang="en">
  <head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1">
  <title>ROCKET GS v4</title>
  <style>
  @import url('https://fonts.googleapis.com/css2?family=Share+Tech+Mono&family=Orbitron:wght@400;700;900&display=swap');
  :root{
    --bg:#080a0d;--panel:#0f1216;--border:#1a2030;--accent:#ff6600;
    --green:#00ff88;--yellow:#ffcc00;--red:#ff2244;--blue:#00aaff;
    --purple:#cc44ff;--cyan:#00eeff;
    --text:#c0cad8;--dim:#505870;
    --mono:'Share Tech Mono',monospace;--head:'Orbitron',monospace;
  }
  *{box-sizing:border-box;margin:0;padding:0}
  body{background:var(--bg);color:var(--text);font-family:var(--mono);overflow-x:hidden;}
  header{background:var(--panel);border-bottom:2px solid var(--accent);padding:10px 14px;
    display:flex;align-items:center;justify-content:space-between;position:sticky;top:0;z-index:100;flex-wrap:wrap;gap:6px;}
  .logo{font-family:var(--head);font-size:.9rem;font-weight:900;letter-spacing:3px;color:var(--accent);}
  .logo span{color:var(--dim);font-size:.6rem;font-weight:400;}
  .sbar{display:flex;gap:8px;align-items:center;font-size:.6rem;flex-wrap:wrap;}
  .dot{width:8px;height:8px;border-radius:50%;background:var(--dim);transition:.3s;flex-shrink:0;}
  .dot.on{background:var(--green);box-shadow:0 0 6px var(--green);}
  .dot.warn{background:var(--yellow);box-shadow:0 0 6px var(--yellow);}
  .dot.danger{background:var(--red);box-shadow:0 0 6px var(--red);}
  .dot.pur{background:var(--purple);box-shadow:0 0 6px var(--purple);}
  .rssibar{height:6px;width:70px;background:#1a2030;border-radius:3px;overflow:hidden;display:inline-block;vertical-align:middle;}
  .rssifill{height:100%;border-radius:3px;transition:width .4s;}
  .grid{display:grid;grid-template-columns:1fr 1fr;gap:10px;padding:10px;max-width:920px;margin:0 auto;}
  @media(max-width:540px){.grid{grid-template-columns:1fr;}}
  .card{background:var(--panel);border:1px solid var(--border);border-radius:6px;padding:12px;}
  .card.full{grid-column:1/-1;}
  .card h3{font-family:var(--head);font-size:.55rem;letter-spacing:2px;color:var(--dim);
    margin-bottom:9px;border-bottom:1px solid var(--border);padding-bottom:5px;}
  .card h3.imu{color:#6633aa;}
  .bignum{font-family:var(--head);font-size:2rem;font-weight:900;color:var(--green);letter-spacing:1px;line-height:1;}
  .bignum.warn{color:var(--yellow);} .bignum.danger{color:var(--red);}
  .unit{font-size:.7rem;color:var(--dim);margin-left:3px;}
  .lbl{font-size:.6rem;color:var(--dim);margin-top:3px;}
  .row{display:flex;justify-content:space-between;padding:4px 0;border-bottom:1px solid var(--border);font-size:.68rem;}
  .row:last-child{border:none;}
  .val{color:var(--green);}  .val.c{color:var(--cyan);}
  .val.w{color:var(--yellow);} .val.d{color:var(--red);}
  .gwrap{display:grid;grid-template-columns:1fr 1fr 1fr;gap:5px;margin-top:5px;}
  .gauge{background:#0a0d10;border:1px solid var(--border);border-radius:4px;padding:7px;text-align:center;}
  .gauge .gv{font-family:var(--head);font-size:.82rem;color:var(--cyan);}
  .gauge .gv.neg{color:#ff6688;}
  .gauge .gl{font-size:.5rem;color:var(--dim);margin-top:2px;}
  .bwrap{margin:5px 0 3px;height:9px;background:#0a0d10;border-radius:5px;overflow:hidden;border:1px solid var(--border);}
  .bfill{height:100%;border-radius:5px;transition:width .2s;}
  .btn{display:block;width:100%;padding:10px;border:none;border-radius:4px;
    font-family:var(--head);font-size:.63rem;letter-spacing:2px;cursor:pointer;
    transition:.15s;margin-bottom:7px;font-weight:700;}
  .btn:active{transform:scale(.97);} .btn:last-child{margin-bottom:0;}
  .b-arm{background:var(--yellow);color:#000;}
  .b-arm.armed{background:var(--green);color:#000;}
  .b-dis{background:var(--border);color:var(--text);}
  .b-fire{background:var(--red);color:#fff;font-size:.72rem;letter-spacing:3px;}
  .b-fire:disabled{opacity:.3;cursor:not-allowed;}
  .b-tare{background:var(--blue);color:#000;}
  .b-rst{background:#141414;color:var(--text);border:1px solid var(--border);}
  .b-rec{background:#0c1910;color:var(--green);border:1px solid var(--green);}
  .b-rec.active{background:var(--red);color:#fff;border-color:var(--red);}
  .b-save{background:var(--accent);color:#fff;}
  .b-test{background:#1a1200;color:var(--yellow);border:1px solid var(--yellow);}
  .b-cal{background:#130a1f;color:var(--purple);border:1px solid var(--purple);}
  .b-dl{background:#001824;color:var(--blue);border:1px solid var(--blue);}
  .b-ping{background:#0a1408;color:var(--green);border:1px solid var(--green);}
  .alert{padding:7px 10px;border-radius:4px;font-size:.67rem;margin-bottom:7px;display:none;}
  .alert.show{display:block;}
  .alert.ok{background:#0c1f14;border:1px solid var(--green);color:var(--green);}
  .alert.err{background:#1f0c11;border:1px solid var(--red);color:var(--red);}
  .badge{display:inline-block;padding:1px 7px;border-radius:8px;font-size:.56rem;
    font-family:var(--head);margin-left:5px;vertical-align:middle;}
  .badge.ok{background:#0c1f14;color:var(--green);}
  .badge.err{background:#1f0c11;color:var(--red);}
  .badge.warn{background:#1a1200;color:var(--yellow);}
  .badge.info{background:#001a2a;color:var(--blue);}
  .ack-box{background:#05080a;border:1px solid #1a3050;border-radius:4px;padding:6px 10px;
    font-size:.63rem;margin-top:6px;display:flex;justify-content:space-between;}
  .inprow{display:flex;justify-content:space-between;align-items:center;padding:5px 0;font-size:.68rem;}
  .inprow label{color:var(--dim);}
  input[type=range]{width:90px;accent-color:var(--accent);}
  input[type=number]{background:#0a0d10;border:1px solid var(--border);color:var(--green);
    padding:4px 6px;width:78px;font-family:var(--mono);font-size:.68rem;border-radius:3px;}
  select{background:#0a0d10;border:1px solid var(--border);color:var(--green);
    padding:4px 8px;font-family:var(--mono);font-size:.68rem;border-radius:3px;}
  .vbadge{background:#0a0d10;border:1px solid var(--border);color:var(--green);
    padding:2px 7px;border-radius:3px;min-width:36px;text-align:center;font-size:.68rem;}
  .toggle{display:flex;align-items:center;gap:8px;cursor:pointer;}
  .toggle input{display:none;}
  .ttrack{width:36px;height:18px;background:var(--border);border-radius:9px;position:relative;transition:.2s;}
  .toggle input:checked~.ttrack{background:var(--green);}
  .ttrack::after{content:'';position:absolute;width:14px;height:14px;border-radius:50%;
    background:#fff;top:2px;left:2px;transition:.2s;}
  .toggle input:checked~.ttrack::after{left:20px;}
  canvas{width:100%;display:block;background:#0a0d10;border-radius:4px;}
  #altChart{height:110px;} #imuChart{height:80px;} #rssiChart{height:55px;}
  .tring{width:54px;height:54px;border-radius:50%;border:3px solid var(--border);
    position:relative;margin:0 auto;background:#0a0d10;}
  .tdot{width:10px;height:10px;border-radius:50%;background:var(--cyan);
    position:absolute;top:50%;left:50%;transform:translate(-50%,-50%);
    transition:.3s;box-shadow:0 0 6px var(--cyan);}
  .tctr{width:4px;height:4px;border-radius:50%;background:var(--dim);
    position:absolute;top:50%;left:50%;transform:translate(-50%,-50%);}
  .stale{position:fixed;top:0;left:0;right:0;bottom:0;background:rgba(0,0,0,.75);
    z-index:200;display:none;align-items:center;justify-content:center;flex-direction:column;gap:16px;}
  .stale.show{display:flex;}
  .stalebox{background:#1a0008;border:2px solid var(--red);border-radius:8px;
    padding:22px 28px;text-align:center;font-family:var(--head);}
  .stalebox h2{color:var(--red);font-size:.9rem;letter-spacing:3px;}
  .stalebox p{color:var(--dim);font-size:.62rem;margin-top:8px;}
  @keyframes flashRed{0%,100%{background:var(--panel)}50%{background:#3a0010}}
  .flash{animation:flashRed .4s ease 3;}
  .tare-progress{height:4px;background:var(--border);border-radius:2px;margin-top:6px;overflow:hidden;}
  .tare-fill{height:100%;background:var(--blue);border-radius:2px;transition:width .2s;}
  </style>
  </head>
  <body>
  <div class="stale" id="staleOverlay">
    <div class="stalebox">
      <h2>⚠ LINK LOST</h2>
      <p>No telemetry &gt;3s. Check LoRa antennas + rocket power.</p>
    </div>
  </div>
  <header>
    <div class="logo">GND STATION<span>  v4 · LoRa SF7 433MHz</span></div>
    <div class="sbar">
      <div class="dot" id="lDot"></div><span id="lLbl">LINK</span>
      <div class="dot" id="aDot"></div><span>ARM</span>
      <div class="dot" id="rDot"></div><span>REC</span>
      <div class="dot danger" id="dDot" style="opacity:.2"></div><span>DEP</span>
      <div class="dot pur" id="mDot" style="opacity:.3"></div><span>MPU</span>
      <div class="dot" id="bDot" style="opacity:.3"></div><span>BMP</span>
      <div class="dot" id="fDot" style="opacity:.2"></div><span>FF</span>
      &nbsp;|&nbsp;
      <span style="color:var(--cyan);">RSSI:</span>
      <span id="rssiV" style="color:var(--cyan);">--</span>
      <div class="rssibar"><div class="rssifill" id="rssiBar" style="width:0%"></div></div>
      <span style="color:var(--dim);">SNR:<span id="snrV">--</span>dB</span>
    </div>
  </header>

  <div class="grid">

    <!-- ALTITUDE -->
    <div class="card">
      <h3>▲ ALTITUDE</h3>
      <div class="bignum" id="altN">0.00</div><span class="unit">m AGL</span>
      <div class="lbl">CURRENT</div>
      <div style="margin-top:9px;">
        <div class="row"><span>MAX</span><span class="val" id="maxA">0.0 m</span></div>
        <div class="row"><span>PRESSURE</span><span class="val" id="pres">-- hPa</span></div>
        <div class="row"><span>TEMP (BMP)</span><span class="val" id="tmp">-- °C</span></div>
        <div class="row"><span>APOGEE</span><span class="val" id="apoSt">--</span></div>
        <div class="row"><span>FALL COUNT</span><span class="val" id="fallC">0</span></div>
      </div>
    </div>

    <!-- SYSTEM -->
    <div class="card">
      <h3>⚙ SYSTEM</h3>
      <div class="row"><span>ARMED</span><span id="armSt" class="val">NO</span></div>
      <div class="row"><span>DEPLOYED</span><span id="depSt" class="val">NO</span></div>
      <div class="row"><span>AUTO</span><span id="autoSt" class="val">--</span></div>
      <div class="row"><span>FUSION</span><span id="fusSt" class="val">--</span></div>
      <div class="row"><span>MPU APOGEE</span><span id="mpuASt" class="val">--</span></div>
      <div class="row"><span>RECORDING</span><span id="recSt" class="val">NO</span></div>
      <div class="row"><span>ROCKET LOG</span><span id="logC" class="val">0</span></div>
      <div class="row"><span>GS LOG</span><span id="gsLogC" class="val">0</span></div>
      <div class="row"><span>ROCKET HEAP</span><span id="heap" class="val">--</span></div>
      <div style="margin-top:7px;font-size:.62rem;color:var(--dim);" id="sysMsg">Waiting for LoRa link...</div>
      <!-- ACK status -->
      <div class="ack-box">
        <span style="color:var(--dim);">LAST ACK</span>
        <span id="ackInfo" style="color:var(--cyan);">—</span>
      </div>
      <!-- Tare progress -->
      <div class="tare-progress"><div class="tare-fill" id="tareFill" style="width:0%"></div></div>
    </div>

    <!-- ALTITUDE CHART -->
    <div class="card full">
      <h3>📈 ALTITUDE (last 80 samples)</h3>
      <canvas id="altChart"></canvas>
    </div>

    <!-- LORA LINK -->
    <div class="card full">
      <h3>📡 LoRa LINK · SF7 · BW125 · 433MHz · SyncWord=0xB4</h3>
      <div style="display:flex;gap:10px;flex-wrap:wrap;align-items:center;">
        <div style="flex:1;min-width:150px;">
          <div style="font-family:var(--head);font-size:1.4rem;color:var(--green);" id="rssiLg">-- dBm</div>
          <div class="row"><span>SNR</span><span class="val" id="snrLg">-- dB</span></div>
          <div class="row"><span>PACKETS RX</span><span class="val" id="pktC">0</span></div>
          <div class="row"><span>LINK</span><span id="linkSt" class="val">SEARCHING</span></div>
        </div>
        <div style="flex:2;min-width:180px;">
          <canvas id="rssiChart"></canvas>
          <div style="font-size:.58rem;color:var(--dim);margin-top:3px;">RSSI history (dBm)</div>
        </div>
      </div>
    </div>

    <!-- IMU -->
    <div class="card full">
      <h3 class="imu">🔷 MPU-6500 IMU</h3>
      <div style="display:flex;gap:7px;align-items:center;margin-bottom:9px;flex-wrap:wrap;">
        <span id="mpuBadge" class="badge err">MPU FAIL</span>
        <span id="bmpBadge" class="badge err">BMP FAIL</span>
        <span id="ffBadge"  class="badge warn" style="display:none;">⚡ FREE-FALL</span>
        <span id="maBadge"  class="badge warn" style="display:none;">▼ MPU APOGEE</span>
      </div>
      <div class="gwrap">
        <div class="gauge"><div class="gv" id="gAX">0.000</div><div class="gl">AX (g)</div></div>
        <div class="gauge"><div class="gv" id="gAY">0.000</div><div class="gl">AY (g)</div></div>
        <div class="gauge"><div class="gv" id="gAZ">1.000</div><div class="gl">AZ (g)</div></div>
      </div>
      <div style="margin-top:5px;">
        <div style="font-size:.57rem;color:var(--dim);margin-bottom:2px;">NET |a| (g) — 0 to 2g</div>
        <div class="bwrap"><div class="bfill" id="netBar" style="width:50%;background:linear-gradient(90deg,var(--green),var(--yellow));"></div></div>
        <div style="display:flex;justify-content:space-between;font-size:.59rem;color:var(--dim);">
          <span>0g</span><span id="netV" style="color:var(--cyan);">1.000 g</span><span>2g</span>
        </div>
      </div>
      <div style="margin-top:5px;">
        <div style="font-size:.57rem;color:var(--dim);margin-bottom:2px;">VERT ACCEL (g) — -1 to +1</div>
        <div class="bwrap"><div class="bfill" id="vertBar" style="width:50%;background:linear-gradient(90deg,var(--blue),var(--cyan));"></div></div>
        <div style="display:flex;justify-content:space-between;font-size:.59rem;color:var(--dim);">
          <span>-1g</span><span id="vertV" style="color:var(--cyan);">0.000 g</span><span>+1g</span>
        </div>
      </div>
      <div style="display:flex;gap:10px;margin-top:9px;flex-wrap:wrap;">
        <div style="flex:1;min-width:120px;">
          <div style="font-size:.57rem;color:var(--dim);margin-bottom:3px;">TILT</div>
          <div style="display:flex;align-items:center;gap:9px;">
            <div class="tring"><div class="tdot" id="tDot"></div><div class="tctr"></div></div>
            <div>
              <div style="font-family:var(--head);font-size:1.2rem;color:var(--cyan);" id="tiltV">0.0°</div>
              <div style="font-size:.57rem;color:var(--dim);">from axis</div>
            </div>
          </div>
        </div>
        <div style="flex:1;min-width:110px;">
          <div class="row"><span>MPU TEMP</span><span class="val c" id="mpuT">--</span></div>
          <div class="row"><span>FALL CNT</span><span class="val c" id="afC">0</span></div>
          <div class="row"><span>FREE FALL</span><span id="ffSt" class="val">NO</span></div>
        </div>
      </div>
      <div style="margin-top:8px;">
        <div style="font-size:.57rem;color:var(--dim);margin-bottom:3px;">NET ACCEL HISTORY</div>
        <canvas id="imuChart"></canvas>
      </div>
    </div>

    <!-- FLIGHT CONTROL -->
    <div class="card">
      <h3>🚀 FLIGHT CONTROL</h3>
      <div id="al1" class="alert"></div>
      <button class="btn b-ping"  onclick="doPing()">📡 PING ROCKET</button>
      <button class="btn b-tare"  onclick="doTare()">◉ TARE + CAL MPU</button>
      <button class="btn b-arm"   id="armBtn" onclick="doArm()">▶ ARM SYSTEM</button>
      <button class="btn b-dis"   onclick="doDisarm()">■ DISARM</button>
      <button class="btn b-fire"  id="fireBtn" onclick="doFire()" disabled>🔥 FIRE DEPLOY</button>
      <button class="btn b-rst"   onclick="doReset()">↺ RESET ALL</button>
    </div>

    <!-- RECORDING + DOWNLOAD -->
    <div class="card">
      <h3>💾 DATA</h3>
      <div id="al2" class="alert"></div>
      <button class="btn b-rec"   id="recBtn" onclick="toggleRec()">⏺ START RECORDING</button>
      <button class="btn b-cal"   onclick="doCalMPU()">🔷 CAL MPU ONLY</button>
      <hr style="border-color:var(--border);margin:8px 0;">
      <div style="font-size:.62rem;color:var(--dim);margin-bottom:6px;">GROUND STATION LOG (buffered here)</div>
      <a href="/gslog" style="text-decoration:none;">
        <button class="btn b-dl">⬇ DOWNLOAD GS LOG CSV</button>
      </a>
      <div style="font-size:.62rem;color:var(--dim);margin:8px 0 4px;">ROCKET LOG (transfer via LoRa)</div>
      <button class="btn b-dl" onclick="doGetRemoteLog()">📡 REQUEST ROCKET LOG</button>
      <div id="remoteLogStatus" style="font-size:.6rem;color:var(--dim);margin-top:4px;">Not downloaded yet</div>
      <a id="rlogLink" href="/remotelog" style="text-decoration:none;display:none;">
        <button class="btn b-dl" style="margin-top:4px;">⬇ DOWNLOAD ROCKET LOG CSV</button>
      </a>
      <div style="font-size:.6rem;color:var(--dim);margin-top:6px;">
        GS log: <span id="gsLogCnt">0</span>/500 · Rocket log: <span id="remLogCnt">0</span> entries
      </div>
    </div>

    <!-- SETTINGS -->
    <div class="card full">
      <h3>⚡ SETTINGS</h3>
      <div id="al3" class="alert"></div>
      <div class="inprow"><label>ACTIVE SERVO</label>
        <select id="activeServo"><option value="1">SERVO 1 (GPIO25)</option><option value="2">SERVO 2 (GPIO26)</option></select>
      </div>
      <div class="inprow"><label>SERVO 1 ANGLE</label>
        <div style="display:flex;gap:7px;align-items:center;">
          <input type="range" id="s1R" min="0" max="180" value="90" oninput="document.getElementById('s1V').textContent=this.value+'°'">
          <span class="vbadge" id="s1V">90°</span>
        </div>
      </div>
      <div class="inprow"><label>SERVO 2 ANGLE</label>
        <div style="display:flex;gap:7px;align-items:center;">
          <input type="range" id="s2R" min="0" max="180" value="90" oninput="document.getElementById('s2V').textContent=this.value+'°'">
          <span class="vbadge" id="s2V">90°</span>
        </div>
      </div>
      <div class="inprow"><label>AUTO DEPLOY</label>
        <label class="toggle"><input type="checkbox" id="autoChk"><div class="ttrack"></div><span id="autoLbl">OFF</span></label>
      </div>
      <div class="inprow" style="flex-direction:column;align-items:flex-start;gap:5px;">
        <label style="color:var(--purple);font-size:.62rem;letter-spacing:1px;">🔷 FUSION MODE</label>
        <select id="fusionMode" style="width:100%;">
          <option value="0">0 — BMP280 ONLY</option>
          <option value="1" selected>1 — FUSION: BMP + MPU (recommended)</option>
          <option value="2">2 — MPU PRIMARY (free-fall triggers)</option>
        </select>
        <div style="font-size:.57rem;color:var(--dim);">
          Mode 1: both sensors must agree. Mode 0/2: single sensor (less reliable).
        </div>
      </div>
      <div class="inprow"><label>FREE-FALL THRESHOLD (g×1000)</label>
        <input type="number" id="ffThresh" value="300" min="50" max="900">
      </div>
      <div class="inprow"><label>APOGEE MARGIN (cm)</label>
        <input type="number" id="apoMargin" value="200" min="10" max="5000">
      </div>
      <div class="inprow"><label>SEA LEVEL (hPa)</label>
        <input type="number" id="seaLvl" value="1013.25" step="0.25" min="950" max="1080">
      </div>
      <button class="btn b-save" style="margin-top:9px;" onclick="saveSettings()">💾 SAVE + SEND TO ROCKET</button>
      <h3 style="margin-top:12px;">🔧 SERVO TEST</h3>
      <div class="inprow"><label>SERVO</label>
        <select id="tSv"><option value="1">SERVO 1</option><option value="2">SERVO 2</option></select>
      </div>
      <div class="inprow"><label>ANGLE</label>
        <div style="display:flex;gap:7px;align-items:center;">
          <input type="range" id="tAng" min="0" max="180" value="45" oninput="document.getElementById('tAngV').textContent=this.value+'°'">
          <span class="vbadge" id="tAngV">45°</span>
        </div>
      </div>
      <button class="btn b-test" onclick="testServo()">▶ TEST (1.5s pulse)</button>
    </div>

  </div>

  <script>
  const POLL=300;
  let altH=[],accH=[],rssiH=[];
  let recording=false,pktCount=0;
  let tareing=false,tareStart=0;
  const FL=['BMP ONLY','BMP+MPU','MPU PRIMARY'];

  async function poll(){
    try{
      const r=await fetch('/telemetry');if(!r.ok)return;
      const d=await r.json();ui(d);
      document.getElementById('lDot').className='dot on';
      document.getElementById('lLbl').textContent='LIVE';
    }catch(e){
      document.getElementById('lDot').className='dot danger';
      document.getElementById('lLbl').textContent='ERR';
    }
  }

  function ui(d){
    recording=d.recording;
    const lk=d.linkOK;
    document.getElementById('staleOverlay').className='stale'+(lk?'':' show');
    document.getElementById('lDot').className='dot '+(lk?'on':'danger');
    document.getElementById('linkSt').textContent=lk?'ACTIVE':'NO SIGNAL';
    document.getElementById('linkSt').className='val '+(lk?'':'d');

    // RSSI
    const rssi=d.rssi||0,snr=parseFloat(d.snr)||0;
    document.getElementById('rssiLg').textContent=rssi+' dBm';
    document.getElementById('rssiV').textContent=rssi;
    document.getElementById('snrV').textContent=snr.toFixed(1);
    document.getElementById('snrLg').textContent=snr.toFixed(1)+' dB';
    const rp=Math.min(Math.max((rssi+120)/80*100,0),100);
    const rb=document.getElementById('rssiBar');
    rb.style.width=rp+'%';
    rb.style.background=rp>60?'var(--green)':rp>30?'var(--yellow)':'var(--red)';
    if(lk){pktCount++;document.getElementById('pktC').textContent=pktCount;}

    // Altitude
    document.getElementById('altN').textContent=parseFloat(d.alt).toFixed(2);
    document.getElementById('maxA').textContent=parseFloat(d.maxAlt).toFixed(2)+' m';
    document.getElementById('pres').textContent=parseFloat(d.pressure).toFixed(1)+' hPa';
    document.getElementById('tmp').textContent=parseFloat(d.temp).toFixed(1)+' °C';
    document.getElementById('apoSt').textContent=d.apogee?'✓ DETECTED':'Monitoring';
    document.getElementById('fallC').textContent=d.falling;

    // System
    document.getElementById('armSt').textContent=d.armed?'YES':'NO';
    document.getElementById('depSt').textContent=d.deployed?'DEPLOYED!':'NO';
    document.getElementById('autoSt').textContent=d.auto?'ON':'OFF';
    document.getElementById('fusSt').textContent=FL[d.fusionMode]||'--';
    if(d.fusionMode!==undefined){const fsel=document.getElementById('fusionMode');if(fsel&&fsel.value!=String(d.fusionMode)){fsel.value=d.fusionMode;}rocketCfg_fusionMode=d.fusionMode;}
    if(d.auto!==undefined){const achk=document.getElementById('autoChk');if(achk&&achk.checked!==d.auto){achk.checked=d.auto;document.getElementById('autoLbl').textContent=d.auto?'ON':'OFF';}}
    document.getElementById('mpuASt').textContent=d.mpuApogee?'✓ YES':'Waiting';
    document.getElementById('recSt').textContent=d.recording?'ON':'OFF';
    document.getElementById('logC').textContent=d.logCount;
    document.getElementById('gsLogC').textContent=d.gsLogCount||0;
    document.getElementById('heap').textContent=(d.freeHeap/1024).toFixed(1)+' KB';
    document.getElementById('gsLogCnt').textContent=d.gsLogCount||0;
    document.getElementById('remLogCnt').textContent=d.remoteLogCount||0;

    // ACK display
    if(d.lastAckCmd&&d.lastAckCmd.length>0){
      const age=d.lastAckAge||0;
      document.getElementById('ackInfo').textContent=
        d.lastAckCmd+' → '+(d.lastAckOk?'✓ OK':'✗ FAIL')+' ('+age+'s ago)';
      document.getElementById('ackInfo').style.color=d.lastAckOk?'var(--green)':'var(--red)';
    }

    // Remote log done
    if(d.remoteLogDone){
      document.getElementById('remoteLogStatus').textContent='✓ Ready ('+d.remoteLogCount+' entries)';
      document.getElementById('rlogLink').style.display='block';
    }

    // Tare progress bar (non-blocking tare takes ~6s on rocket)
    if(tareing){
      const elapsed=Date.now()-tareStart;
      const pct=Math.min(elapsed/7000*100,100);
      document.getElementById('tareFill').style.width=pct+'%';
      if(pct>=100){tareing=false;document.getElementById('tareFill').style.width='0%';}
    }

    // IMU
    const ax=parseFloat(d.accelX),ay=parseFloat(d.accelY),az=parseFloat(d.accelZ);
    const net=parseFloat(d.netAccel),vert=parseFloat(d.vertAccel),tilt=parseFloat(d.tilt);
    function gv(id,v){const e=document.getElementById(id);e.textContent=v.toFixed(3);e.className='gv'+(v<0?' neg':'');}
    gv('gAX',ax);gv('gAY',ay);gv('gAZ',az);
    document.getElementById('netBar').style.width=Math.min(net/2*100,100)+'%';
    document.getElementById('netV').textContent=net.toFixed(3)+' g';
    document.getElementById('vertBar').style.width=Math.min(Math.max((vert+1)/2*100,0),100)+'%';
    document.getElementById('vertV').textContent=(vert>=0?'+':'')+vert.toFixed(3)+' g';

    // Tilt dot
    const td=document.getElementById('tDot');
    const mr=17,tr2=Math.min(tilt/45,1)*mr;
    const tdx=(net>0.1)?(ax/net):0,tdy=(net>0.1)?(ay/net):0;
    td.style.transform=`translate(calc(-50% + ${tdx*tr2}px),calc(-50% + ${tdy*tr2}px))`;
    document.getElementById('tiltV').textContent=tilt.toFixed(1)+'°';
    document.getElementById('mpuT').textContent=parseFloat(d.mpuTemp).toFixed(1)+' °C';
    document.getElementById('afC').textContent=d.accelFall;
    const ffs=document.getElementById('ffSt');
    ffs.textContent=d.freeFall?'YES ⚡':'NO';
    ffs.className='val '+(d.freeFall?'d':'');

    // Badges / dots
    document.getElementById('mpuBadge').textContent=d.mpuOK?'MPU OK':'MPU FAIL';
    document.getElementById('mpuBadge').className='badge '+(d.mpuOK?'ok':'err');
    document.getElementById('mDot').style.opacity=d.mpuOK?'1':'.2';
    document.getElementById('bmpBadge').textContent=d.bmpOK?'BMP OK':'BMP FAIL';
    document.getElementById('bmpBadge').className='badge '+(d.bmpOK?'ok':'err');
    document.getElementById('bDot').className='dot '+(d.bmpOK?'on':'danger');
    document.getElementById('ffBadge').style.display=d.freeFall?'inline-block':'none';
    document.getElementById('maBadge').style.display=d.mpuApogee?'inline-block':'none';
    document.getElementById('fDot').className='dot '+(d.freeFall?'danger':'');
    document.getElementById('fDot').style.opacity=d.freeFall?'1':'.2';
    document.getElementById('aDot').className='dot'+(d.armed?' warn':'');
    document.getElementById('rDot').className='dot'+(d.recording?' on':'');
    document.getElementById('dDot').style.opacity=d.deployed?'1':'.2';

    document.getElementById('fireBtn').disabled=!(d.armed&&!d.deployed&&lk);
    const ab=document.getElementById('armBtn');
    if(d.armed){ab.textContent='✓ ARMED';ab.className='btn b-arm armed';}
    else{ab.textContent='▶ ARM SYSTEM';ab.className='btn b-arm';}
    const rb2=document.getElementById('recBtn');
    rb2.textContent=d.recording?'⏹ STOP RECORDING':'⏺ START RECORDING';
    rb2.className='btn b-rec'+(d.recording?' active':'');

    if(d.deployed&&!document.getElementById('altN').dataset.fl){
      document.querySelector('.grid').classList.add('flash');
      document.getElementById('altN').dataset.fl='1';
      document.getElementById('sysMsg').textContent='🔥 DEPLOYED!';
      document.getElementById('sysMsg').style.color='var(--red)';
      setTimeout(()=>document.querySelector('.grid').classList.remove('flash'),1600);
    }
    if(!d.deployed) delete document.getElementById('altN').dataset.fl;

    altH.push(parseFloat(d.alt));if(altH.length>80)altH.shift();
    accH.push(net);if(accH.length>80)accH.shift();
    rssiH.push(rssi);if(rssiH.length>80)rssiH.shift();
    chart('altChart',altH,'#00ff88',null,'rgba(0,255,136,0.06)');
    chart('imuChart',accH,'#00eeff',2,'rgba(0,238,255,0.05)');
    rssiChart();
  }

  function chart(id,data,color,fixMax,fill){
    const c=document.getElementById(id);const ctx=c.getContext('2d');
    c.width=c.offsetWidth;c.height=c.offsetHeight;
    const w=c.width,h=c.height;ctx.clearRect(0,0,w,h);
    if(data.length<2)return;
    const mn=Math.min(...data);
    const mx=fixMax!=null?(fixMax||Math.max(...data)||1):(Math.max(...data)||1);
    const range=(mx-mn)||1;
    ctx.strokeStyle='#1a2030';ctx.lineWidth=1;
    for(let i=0;i<=4;i++){const y=h-(i/4)*h;ctx.beginPath();ctx.moveTo(0,y);ctx.lineTo(w,y);ctx.stroke();
      ctx.fillStyle='#384050';ctx.font='8px monospace';ctx.fillText((mn+(i/4)*range).toFixed(2),2,y-2);}
    ctx.beginPath();ctx.strokeStyle=color;ctx.lineWidth=2;ctx.shadowColor=color;ctx.shadowBlur=4;
    data.forEach((v,i)=>{const x=(i/(data.length-1))*w;const y=h-((v-mn)/range)*(h-10)-5;
      i===0?ctx.moveTo(x,y):ctx.lineTo(x,y);});
    ctx.stroke();ctx.shadowBlur=0;ctx.lineTo(w,h);ctx.lineTo(0,h);ctx.closePath();
    ctx.fillStyle=fill;ctx.fill();
  }
  function rssiChart(){
    const c=document.getElementById('rssiChart');if(!c)return;
    const ctx=c.getContext('2d');c.width=c.offsetWidth;c.height=c.offsetHeight;
    const w=c.width,h=c.height,data=rssiH;ctx.clearRect(0,0,w,h);if(data.length<2)return;
    ctx.beginPath();ctx.strokeStyle='#00aaff';ctx.lineWidth=2;ctx.shadowColor='#00aaff';ctx.shadowBlur=3;
    data.forEach((v,i)=>{const x=(i/(data.length-1))*w;const y=h-((v+130)/90)*(h-4)-2;
      i===0?ctx.moveTo(x,y):ctx.lineTo(x,y);});
    ctx.stroke();ctx.shadowBlur=0;ctx.lineTo(w,h);ctx.lineTo(0,h);ctx.closePath();
    ctx.fillStyle='rgba(0,170,255,0.05)';ctx.fill();
  }

  async function api(url,method='POST',body=null){
    const o={method,headers:{'Content-Type':'application/json'}};
    if(body)o.body=JSON.stringify(body);
    const r=await fetch(url,o);return r.json();
  }
  function alert_(id,msg,ok){
    const e=document.getElementById(id);
    e.textContent=msg;e.className='alert show '+(ok?'ok':'err');
    setTimeout(()=>e.className='alert',5000);
  }

  async function doPing(){const d=await api('/ping');alert_('al1',d.msg,d.ok);}
  async function doArm(){const d=await api('/arm');alert_('al1',d.msg,d.ok);}
  async function doDisarm(){const d=await api('/disarm');alert_('al1',d.msg,d.ok);}
  async function doFire(){
    if(!confirm('🔥 FIRE via LoRa? Sends FIRE command x5 to rocket.'))return;
    const d=await api('/fire');alert_('al1',d.msg,d.ok);
  }
  async function doReset(){if(!confirm('Send RESET?'))return;const d=await api('/reset');altH=[];accH=[];alert_('al1',d.msg,d.ok);}
  async function doTare(){
    const d=await api('/tare');alert_('al1',d.msg,d.ok);
    altH=[];accH=[];
    tareing=true;tareStart=Date.now();  // start progress bar
  }
  async function doCalMPU(){const d=await api('/calibrateMPU');alert_('al2',d.msg,d.ok);}
  async function toggleRec(){const d=await api(recording?'/recordStop':'/recordStart');alert_('al2',d.msg,d.ok);}
  async function doGetRemoteLog(){
    document.getElementById('remoteLogStatus').textContent='⏳ Requesting...';
    document.getElementById('rlogLink').style.display='none';
    const d=await api('/requestRemoteLog');alert_('al2',d.msg,d.ok);
  }
  async function saveSettings(){
    const body={
      servo1Angle:parseInt(document.getElementById('s1R').value),
      servo2Angle:parseInt(document.getElementById('s2R').value),
      autoEnabled:document.getElementById('autoChk').checked,
      apogeeMargin:parseInt(document.getElementById('apoMargin').value),
      activeServo:parseInt(document.getElementById('activeServo').value),
      seaLevel:parseFloat(document.getElementById('seaLvl').value),
      fusionMode:parseInt(document.getElementById('fusionMode').value),
      freeFallThresh:parseInt(document.getElementById('ffThresh').value),
    };
    const d=await api('/settings','POST',body);alert_('al3',d.msg,d.ok);
  }
  async function testServo(){
    const sv=parseInt(document.getElementById('tSv').value);
    const ang=parseInt(document.getElementById('tAng').value);
    const btn=document.querySelector('.b-test');
    btn.disabled=true;btn.textContent='▶ SENDING...';
    try{
      const d=await api('/testServo','POST',{servo:sv,angle:ang});
      alert_('al3',d.ok?('Servo'+sv+' → '+ang+'° sent'):'Error: '+d.msg,d.ok);
      let t=1.5;const iv=setInterval(()=>{t=Math.max(0,t-.1);
        btn.textContent='↺ '+t.toFixed(1)+'s';
        if(t<=0){clearInterval(iv);btn.disabled=false;btn.textContent='▶ TEST (1.5s pulse)';}
      },100);
    }catch(e){btn.disabled=false;btn.textContent='▶ TEST (1.5s pulse)';alert_('al3','Error',false);}
  }
  async function loadSettings(){
    try{
      const d=await api('/settings','GET');
      document.getElementById('s1R').value=d.servo1Angle;document.getElementById('s1V').textContent=d.servo1Angle+'°';
      document.getElementById('s2R').value=d.servo2Angle;document.getElementById('s2V').textContent=d.servo2Angle+'°';
      document.getElementById('autoChk').checked=d.autoEnabled;
      document.getElementById('autoLbl').textContent=d.autoEnabled?'ON':'OFF';
      document.getElementById('apoMargin').value=d.apogeeMargin;
      document.getElementById('activeServo').value=d.activeServo;
      document.getElementById('seaLvl').value=d.seaLevel;
      document.getElementById('fusionMode').value=d.fusionMode;
      document.getElementById('ffThresh').value=d.freeFallThresh;
    }catch(e){}
  }
  document.getElementById('autoChk').addEventListener('change',function(){
    document.getElementById('autoLbl').textContent=this.checked?'ON':'OFF';
  });
  window.addEventListener('resize',()=>{
    chart('altChart',altH,'#00ff88',null,'rgba(0,255,136,0.06)');
    chart('imuChart',accH,'#00eeff',2,'rgba(0,238,255,0.05)');
    rssiChart();
  });
  loadSettings();
  setInterval(poll,POLL);
  poll();
  </script>
  </body>
  </html>
  )rawliteral";

  void handleRoot(){server.send_P(200,"text/html",INDEX_HTML);}

  void setup(){
    Serial.begin(115200);
    delay(300);
    Serial.println("\n=== GROUND STATION v4.0 ===");
    SPI.begin(18,19,23,LORA_SS);
    LoRa.setPins(LORA_SS,LORA_RST,LORA_DIO0);
    if(!LoRa.begin(LORA_FREQ)){Serial.println("[LORA] FAIL");while(1)delay(1000);}
    LoRa.setSpreadingFactor(LORA_SF);
    LoRa.setSignalBandwidth(LORA_BW);
    LoRa.setCodingRate4(LORA_CR);
    LoRa.setTxPower(LORA_TX_POWER);
    LoRa.setSyncWord(LORA_SYNC_WORD);
    LoRa.enableCrc();
    Serial.println("[LORA] Ground station ready");
    WiFi.softAP(AP_SSID,AP_PASSWORD);
    Serial.printf("[WiFi] AP=%s  IP=%s\n",AP_SSID,WiFi.softAPIP().toString().c_str());
    server.on("/",              HTTP_GET,  handleRoot);
    server.on("/telemetry",     HTTP_GET,  handleTelemetry);
    server.on("/arm",           HTTP_POST, handleArm);
    server.on("/disarm",        HTTP_POST, handleDisarm);
    server.on("/fire",          HTTP_POST, handleFire);
    server.on("/reset",         HTTP_POST, handleReset);
    server.on("/tare",          HTTP_POST, handleTare);
    server.on("/calibrateMPU",  HTTP_POST, handleCalMPU);
    server.on("/ping",          HTTP_POST, handlePing);
    server.on("/settings",      HTTP_GET,  handleGetSettings);
    server.on("/settings",      HTTP_POST, handleSettings);
    server.on("/testServo",     HTTP_POST, handleTestServo);
    server.on("/recordStart",   HTTP_POST, handleRecordStart);
    server.on("/recordStop",    HTTP_POST, handleRecordStop);
    server.on("/gslog",         HTTP_GET,  handleGetLog);
    server.on("/remotelog",     HTTP_GET,  handleGetRemoteLog);
    server.on("/requestRemoteLog",HTTP_POST,handleRequestRemoteLog);
    server.begin();
    Serial.println("[HTTP] Ready at http://192.168.4.1");
  }

  void loop(){
    server.handleClient();
    int pktSize=LoRa.parsePacket();
    if(pktSize>0){
      String pkt="";
      while(LoRa.available()) pkt+=(char)LoRa.read();
      int rssi=LoRa.packetRssi();
      float snr=LoRa.packetSnr();
      Serial.printf("[RX] rssi=%d snr=%.1f len=%d\n",rssi,(double)snr,pktSize);
      if(pkt.startsWith("T,"))       parseTelemetry(pkt,rssi,snr);
      else if(pkt.startsWith("A,"))  parseAck(pkt);
      else if(pkt.startsWith("LC,")) parseLogPacket(pkt);
      else if(pkt.startsWith("L,"))  parseLogPacket(pkt);
      else if(pkt=="LE")             parseLogPacket(pkt);
      else Serial.printf("[RX] Unknown: %s\n",pkt.c_str());
    }
  }
