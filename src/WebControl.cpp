#include "WebControl.h"

#include <Arduino.h>
#include <WebServer.h>
#include <WiFi.h>

#include "HardwareConfig.h"
#include "RobotContext.h"
#include "WifiCredentials.h"

namespace {
WebServer server(80);

const char page[] PROGMEM = R"HTML(<!DOCTYPE html>
<html><head>
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Robot Control</title>
<style>
body{font-family:Arial;background:#111;color:#fff;text-align:center;padding:20px}
h2{margin-bottom:6px}
.card{background:#1a1a1a;border:1px solid #333;border-radius:12px;padding:16px;margin:12px auto;max-width:360px}
input[type=number]{width:110px;padding:8px;margin:6px;border-radius:8px;border:1px solid #444;background:#222;color:#fff;text-align:center;font-size:15px}
input[type=range]{width:200px;margin:8px}
.btn-update{padding:10px 28px;background:#00ffcc;border:none;border-radius:10px;font-size:15px;font-weight:bold;cursor:pointer;margin-top:6px}
.dpad{display:grid;grid-template-columns:repeat(3,70px);grid-template-rows:repeat(3,60px);gap:8px;justify-content:center;margin:10px auto}
.dbtn{padding:0;border:none;border-radius:10px;font-size:22px;cursor:pointer;background:#1e3a5f;color:#7ab8f5;font-weight:bold}
.dbtn:active{background:#2563eb;color:#fff}
.stopbtn{background:#3a1a1a;color:#f87171}
.stopbtn:active{background:#b91c1c;color:#fff}
.empty{visibility:hidden}
.spd-row{display:flex;align-items:center;justify-content:center;gap:10px;margin:8px 0}
label{font-size:13px;color:#aaa}
#spd-val{font-size:14px;font-weight:bold;color:#00ffcc;min-width:40px;text-align:left}
</style></head><body>
<h2>Self-Balancing Robot</h2>
<p style="color:#888;font-size:13px;margin-bottom:4px">IP: {{IP}}</p>
<div class="card">
<b>PID Tuning</b><br><br>
<form action="/set">
<label>Kp</label><br><input type="number" step="0.1" name="kp" value="{{KP}}"><br>
<label>Ki</label><br><input type="number" step="0.1" name="ki" value="{{KI}}"><br>
<label>Kd</label><br><input type="number" step="0.1" name="kd" value="{{KD}}"><br>
<label>Setpoint</label><br><input type="number" step="0.1" name="sp" value="{{SP}}"><br>
<label>Drive Tilt (derajat)</label><br>
<input type="number" step="0.01" name="tilt" value="{{TILT}}"><br><br>
<button class="btn-update" type="submit">Update PID</button>
</form></div>
<div class="card">
<b>Kontrol Arah</b><br>
<div class="spd-row">
<label>Kecepatan belok</label>
<input type="range" min="0" max="200" value="{{TURN}}" id="spd" oninput="document.getElementById('spd-val').textContent=this.value">
<span id="spd-val">{{TURN}}</span>
</div>
<div class="dpad">
<div class="empty"></div>
<button class="dbtn" ontouchstart="send('fwd')" onmousedown="send('fwd')" ontouchend="send('stop')" onmouseup="send('stop')">&#9650;</button>
<div class="empty"></div>
<button class="dbtn" ontouchstart="send('left')" onmousedown="send('left')" ontouchend="send('stop')" onmouseup="send('stop')">&#9664;</button>
<button class="dbtn stopbtn" onclick="send('stop')">&#9632;</button>
<button class="dbtn" ontouchstart="send('right')" onmousedown="send('right')" ontouchend="send('stop')" onmouseup="send('stop')">&#9654;</button>
<div class="empty"></div>
<button class="dbtn" ontouchstart="send('bwd')" onmousedown="send('bwd')" ontouchend="send('stop')" onmouseup="send('stop')">&#9660;</button>
<div class="empty"></div>
</div></div>
<script>
function send(cmd){
  var spd=document.getElementById('spd').value;
  fetch('/drive?cmd='+cmd+'&turn='+spd);
}
</script></body></html>)HTML";

void handleRoot() {
  String html = FPSTR(page);
  html.replace("{{IP}}", WiFi.localIP().toString());
  html.replace("{{KP}}", String(robot.kp, 2));
  html.replace("{{KI}}", String(robot.ki, 2));
  html.replace("{{KD}}", String(robot.kd, 2));
  html.replace("{{SP}}", String(robot.setpointBase, 2));
  html.replace("{{TILT}}", String(robot.driveSetpointOffset, 2));
  html.replace("{{TURN}}", String(robot.turnOffset));
  server.send(200, "text/html", html);
}

void handleSet() {
  if (server.hasArg("kp")) {
    String value = server.arg("kp");
    value.trim();
    if (value.length()) robot.kp = value.toFloat();
  }
  if (server.hasArg("ki")) {
    String value = server.arg("ki");
    value.trim();
    if (value.length()) {
      robot.ki = value.toFloat();
      robot.pidIntegral = 0.0f;
    }
  }
  if (server.hasArg("kd")) {
    String value = server.arg("kd");
    value.trim();
    if (value.length()) robot.kd = value.toFloat();
  }
  if (server.hasArg("sp")) {
    String value = server.arg("sp");
    value.trim();
    if (value.length()) robot.setpointBase = value.toFloat();
  }
  if (server.hasArg("tilt")) {
    String value = server.arg("tilt");
    value.trim();
    if (value.length()) robot.driveSetpointOffset = value.toFloat();
  }

  robot.setpoint = robot.setpointBase;
  Serial.printf("WEB PID -> Kp=%.2f Ki=%.2f Kd=%.2f SP=%.2f Tilt=%.2f\n",
                robot.kp, robot.ki, robot.kd, robot.setpointBase,
                robot.driveSetpointOffset);
  server.sendHeader("Location", "/");
  server.send(303);
}

void handleDrive() {
  if (server.hasArg("turn")) {
    String value = server.arg("turn");
    value.trim();
    if (value.length()) {
      robot.turnOffset = constrain(value.toInt(), 0, hardware::dutyMax);
    }
  }

  if (server.hasArg("cmd")) {
    const String command = server.arg("cmd");
    if (command == "fwd") {
      robot.driveCommand = DriveCommand::Forward;
      robot.setpoint = robot.setpointBase + robot.driveSetpointOffset;
    } else if (command == "bwd") {
      robot.driveCommand = DriveCommand::Backward;
      robot.setpoint = robot.setpointBase - robot.driveSetpointOffset;
    } else if (command == "left") {
      robot.driveCommand = DriveCommand::Left;
      robot.setpoint = robot.setpointBase;
    } else if (command == "right") {
      robot.driveCommand = DriveCommand::Right;
      robot.setpoint = robot.setpointBase;
    } else {
      robot.driveCommand = DriveCommand::Stop;
      robot.setpoint = robot.setpointBase;
    }

    Serial.printf("DRIVE: %s  setpoint=%.2f  turn=%d\n",
                  command.c_str(), robot.setpoint, robot.turnOffset);
  }
  server.send(200, "text/plain", "OK");
}
}  // namespace

void beginWebControl() {
  WiFi.begin(wifiSsid, wifiPassword);
  Serial.print("Connecting WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nConnected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  server.on("/", handleRoot);
  server.on("/set", handleSet);
  server.on("/drive", handleDrive);
  server.begin();
  Serial.println("Web server started");
}

void handleWebControl() { server.handleClient(); }
