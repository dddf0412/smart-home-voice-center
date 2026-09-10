# -*- coding: utf-8 -*-
"""
智能家居环境中心 —— 一体化主程序（三合一）
  1. 网页控制台：实时数值卡片 + 温度/湿度/光照曲线 + 实体控制按钮
  2. 数据存档：订阅巴法云 env，每一笔带时间戳写入 env_log.csv
  3. 语音交互：讯飞 ASR 识别 + 意图解析 + TTS 播报（后台线程）

依赖：pip install flask paho-mqtt websocket-client
运行：python main.py   （浏览器打开 http://127.0.0.1:8080）
"""
import base64
import csv
import hashlib
import hmac
import json
import os
import random
import select
import socket
import sys
import threading
import time
import urllib.parse
import warnings
import wave
import webbrowser
from collections import deque
from datetime import datetime
from time import mktime
from wsgiref.handlers import format_date_time

import paho.mqtt.client as mqtt
import websocket
from flask import Flask, jsonify, render_template, request

warnings.filterwarnings("ignore", category=DeprecationWarning)

# ============================ 配置 ============================
# 巴法云
UID = "你的巴法云私钥"
BEMFA_SERVER = "bemfa.com"
BEMFA_PORT = 9501
TOPIC_ENV = "env"          # 订阅（纯主题名）
TOPIC_CMD = "cmd"          # 发布控制指令（纯主题名）

# 讯飞
APPID = "你的讯飞AppID"
APIKEY = "你的讯飞APIKey"
APISECRET = "你的讯飞APISecret"
IAT_HOST = "iat-api.xfyun.cn"
IAT_PATH = "/v2/iat"
TTS_HOST = "tts-api.xfyun.cn"
TTS_PATH = "/v2/tts"

# 语音相关端口
PC_AUDIO_PORT = 5005   # 收 ESP 录音
ESP_TEXT_PORT = 5006   # 回传指令文字
ESP_TTS_PORT = 5007    # 回传 TTS 音频
PC_ENV_PORT = 5009     # 收 ESP 环境数据（语音查询用）
SAMPLE_RATE = 16000

LOG_FILE = "env_log.csv"
HISTORY_MAX = 300      # 网页缓存 300 笔 ≈ 10 分钟

# ============================ 数据缓存 + CSV ============================
history = deque(maxlen=HISTORY_MAX)
voice_log = deque(maxlen=20)   # 最近语音交互记录（20 条 = 10 次对话，供网页显示）

# 页面关闭自动退出：连续无请求则结束进程（CSV 每笔已即时落盘，退出即保存）
start_time = time.time()
last_activity = time.time()
GRACE_PERIOD = 10     # 启动后 10s 内不因无请求退出（留时间给浏览器打开）
IDLE_TIMEOUT = 10     # 连续 10s 无请求（页面已关）则退出


def on_mqtt_connect(client, userdata, flags, rc):
    if rc == 0:
        client.subscribe(TOPIC_ENV)
        print("已连接巴法云，订阅 env")
    else:
        print("MQTT 连接失败 rc=", rc)


last_alarm = 0   # 报警状态，用于检测报警跳变


def on_mqtt_message(client, userdata, msg):
    global last_alarm
    try:
        d = json.loads(msg.payload.decode("utf-8"))
        row = {
            "time": datetime.now().strftime("%H:%M:%S"),
            "t": d.get("t", 0), "h": d.get("h", 0),
            "light": d.get("light", 0), "dist": d.get("dist", 0),
            "lamp": d.get("lamp", 0), "auto": d.get("auto", 0),
            "alarm": d.get("alarm", 0), "fan": d.get("fan", 0),
        }
        history.append(row)
        with open(LOG_FILE, "a", newline="", encoding="utf-8-sig") as f:
            csv.writer(f).writerow(
                [row["time"], row["t"], row["h"], row["light"], row["lamp"], row["auto"], row["dist"], row["alarm"], row["fan"]])

        # 报警联动：报警刚触发时异步 TTS 播报
        if row["alarm"] == 1 and last_alarm == 0:
            print("报警触发！温湿度/距离/灯故障异常")
            threading.Thread(target=alarm_tts, daemon=True).start()
        last_alarm = row["alarm"]
    except Exception as e:
        print("解析失败:", e)


def alarm_tts():
    """报警触发时向 ESP 推送语音播报（尽力而为）"""
    audio = tts("警报，检测到环境异常，请及时处理")
    if not audio:
        return
    if not esp_ip:
        print("（ESP IP 未知，跳过报警播报）")
        return
    try:
        t = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        t.settimeout(10)
        t.connect((esp_ip, ESP_TTS_PORT))
        t.sendall(audio)
        t.close()
        print("已向 ESP 推送报警语音")
    except Exception as e:
        print("报警语音推送失败:", e)


mqttc = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1, client_id=UID)   # 同 UID 多连接共存
mqttc.on_connect = on_mqtt_connect
mqttc.on_message = on_mqtt_message

# ============================ Flask 网页 ============================
def resource_path(relative):
    """兼容 PyInstaller 打包后的资源路径"""
    base = getattr(sys, "_MEIPASS", os.path.dirname(os.path.abspath(__file__)))
    return os.path.join(base, relative)


app = Flask(__name__, template_folder=resource_path("templates"))


@app.before_request
def track_activity():
    global last_activity
    last_activity = time.time()


@app.route("/")
def index():
    return render_template("index.html")


@app.route("/api/data")
def api_data():
    return jsonify(list(history))


@app.route("/api/control", methods=["POST"])
def api_control():
    dev = request.json.get("dev", "")
    act = request.json.get("act", "")
    allowed = {"lamp": ("on", "off"), "fan": ("on", "off"), "auto": ("1", "0")}
    if dev not in allowed or act not in allowed[dev]:
        return jsonify({"ok": False, "msg": "非法指令"}), 400
    cmd = f"{dev}:{act}"
    mqttc.publish(TOPIC_CMD, cmd)
    print("已下发:", cmd)
    return jsonify({"ok": True, "cmd": cmd})


@app.route("/api/voice")
def api_voice():
    return jsonify(list(voice_log))


@app.route("/api/shutdown", methods=["POST", "GET"])
def shutdown():
    print("收到关闭信号，退出，日志已保存到", LOG_FILE)
    os._exit(0)


def watchdog():
    """网页关闭后自动退出：超过 IDLE_TIMEOUT 无任何请求则结束进程"""
    while True:
        time.sleep(2)
        if time.time() - start_time > GRACE_PERIOD and time.time() - last_activity > IDLE_TIMEOUT:
            print("页面已关闭（无请求超时），自动退出，日志已保存到", LOG_FILE)
            os._exit(0)


# ============================ 语音（讯飞 ASR/TTS）============================
def build_url(host, path):
    date = format_date_time(mktime(datetime.now().timetuple()))
    origin = f"host: {host}\ndate: {date}\nGET {path} HTTP/1.1"
    sig = base64.b64encode(
        hmac.new(APISECRET.encode(), origin.encode(), hashlib.sha256).digest()).decode()
    auth_origin = (f'api_key="{APIKEY}", algorithm="hmac-sha256", '
                   f'headers="host date request-line", signature="{sig}"')
    auth = base64.b64encode(auth_origin.encode()).decode()
    return f"wss://{host}{path}?authorization={urllib.parse.quote(auth)}" \
           f"&date={urllib.parse.quote(date)}&host={host}"


def asr(pcm):
    ws = websocket.create_connection(build_url(IAT_HOST, IAT_PATH), timeout=10)

    def send_frame(status, audio_b64):
        ws.send(json.dumps({
            "common": {"app_id": APPID},
            "business": {"language": "zh_cn", "domain": "iat", "accent": "mandarin", "vad_eos": 3000},
            "data": {"status": status, "format": "audio/L16;rate=16000", "encoding": "raw", "audio": audio_b64},
        }))

    send_frame(0, "")
    for i in range(0, len(pcm), 1280):
        send_frame(1, base64.b64encode(pcm[i:i + 1280]).decode())
    send_frame(2, "")

    result = ""
    while True:
        try:
            msg = ws.recv()
        except Exception:
            break
        if not msg:
            break
        data = json.loads(msg)
        if data.get("code") != 0:
            break
        for o in data.get("data", {}).get("result", {}).get("ws", []):
            for cw in o.get("cw", []):
                result += cw.get("w", "")
        if data.get("data", {}).get("result", {}).get("ls", False):
            break
    ws.close()
    return result


def tts(text):
    ws = websocket.create_connection(build_url(TTS_HOST, TTS_PATH), timeout=10)
    ws.send(json.dumps({
        "common": {"app_id": APPID},
        "business": {"aue": "raw", "auf": "audio/L16;rate=16000", "vcn": "xiaoyan", "tte": "UTF8",
                     "volume": 70, "speed": 45, "pitch": 50},
        "data": {"status": 2, "text": base64.b64encode(text.encode()).decode()},
    }))
    audio = b""
    while True:
        try:
            msg = ws.recv()
        except Exception:
            break
        if not msg:
            break
        data = json.loads(msg)
        if data.get("code") != 0:
            break
        a = data.get("data", {}).get("audio", "")
        if a:
            audio += base64.b64decode(a)
        if data.get("data", {}).get("status", 0) == 2:
            break
    ws.close()
    return audio


def intent_parse(text, env):
    t = env["temp"]; h = env["hum"]
    if "开灯" in text:
        return "C:lamp:on", random.choice(["好的，已开灯", "灯已打开", "好的，为您开灯"])
    if "关灯" in text:
        return "C:lamp:off", random.choice(["好的，已关灯", "灯已关闭", "好的，为您关灯"])
    if "风扇" in text or "窗" in text:
        if "关" in text:
            return "C:fan:off", random.choice(["好的，已关风扇", "风扇已关闭", "好的，为您关风扇"])
        return "C:fan:on", random.choice(["好的，已开风扇", "风扇已打开", "好的，为您开风扇"])
    if "自动" in text:
        if "关" in text:
            return "C:auto:0", random.choice(["已关闭自动模式", "好的，自动模式已关闭"])
        return "C:auto:1", random.choice(["已开启自动模式", "好的，自动模式已开启"])
    if "温度" in text or "几度" in text or "多少度" in text:
        return "C:none", random.choice([f"现在温度{t:.0f}度", f"当前室温{t:.0f}度", f"温度是{t:.0f}度"])
    if "湿度" in text or "潮" in text:
        return "C:none", random.choice([f"现在湿度{h:.0f}%", f"当前湿度{h:.0f}%", f"湿度是{h:.0f}%"])
    return "C:none", random.choice(["我没有听懂，请再说一次", "抱歉，能再说一遍吗", "这个指令我还没学会"])


def save_wav(filename, pcm):
    w = wave.open(filename, "wb")
    w.setnchannels(1)
    w.setsampwidth(2)
    w.setframerate(SAMPLE_RATE)
    w.writeframes(pcm)
    w.close()


def parse_env(data, env):
    try:
        s = data.decode()
        if not s.startswith("E:"):
            return
        parts = s[2:].split(":")
        env["temp"] = float(parts[0])
        env["hum"] = float(parts[1])
        if len(parts) > 2: env["light"] = int(parts[2])
        if len(parts) > 3: env["lamp"] = int(parts[3])
        if len(parts) > 4: env["auto"] = int(parts[4])
    except Exception:
        pass


esp_ip = None


def voice_loop():
    """语音桥接主循环（后台线程运行）"""
    global esp_ip
    audio_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    audio_sock.bind(("0.0.0.0", PC_AUDIO_PORT))
    env_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    env_sock.bind(("0.0.0.0", PC_ENV_PORT))

    env = {"temp": 0.0, "hum": 0.0, "light": 0, "lamp": 0, "auto": 0}
    print(f"语音服务启动：音频 {PC_AUDIO_PORT} / 环境 {PC_ENV_PORT}")

    pcm = b""
    while True:
        r, _, _ = select.select([audio_sock, env_sock], [], [], 2.0)
        if not r:
            if not pcm:
                continue
            text = asr(pcm)
            print("识别结果:", repr(text))
            pcm = b""
            voice_log.append({"time": datetime.now().strftime("%H:%M:%S"), "type": "asr", "text": text})
            cmd, reply = intent_parse(text, env)
            print("意图:", cmd, "| 回复:", reply)
            voice_log.append({"time": datetime.now().strftime("%H:%M:%S"), "type": "reply", "text": reply})
            if esp_ip:
                audio_sock.sendto(cmd.encode(), (esp_ip, ESP_TEXT_PORT))
            audio = tts(reply)
            if audio:
                save_wav("tts_reply.wav", audio)
                print(f"TTS 合成 {len(audio)} 字节")
            if audio and esp_ip:
                try:
                    t = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    t.settimeout(10)
                    t.connect((esp_ip, ESP_TTS_PORT))
                    t.sendall(audio)
                    t.close()
                except Exception as e:
                    print("TCP 回传失败:", e)
            continue

        for s in r:
            if s is env_sock:
                data, _ = s.recvfrom(65535)
                parse_env(data, env)
            else:
                chunk, addr = s.recvfrom(65535)
                if esp_ip is None:
                    esp_ip = addr[0]
                    print("ESP 已连接:", esp_ip)
                pcm += chunk


# ============================ 入口 ============================
def main():
    if not os.path.exists(LOG_FILE):
        with open(LOG_FILE, "w", newline="", encoding="utf-8-sig") as f:
            csv.writer(f).writerow(["时间", "温度", "湿度", "光照", "灯", "自动", "距离", "报警", "风扇"])

    mqttc.connect(BEMFA_SERVER, BEMFA_PORT, 60)
    mqttc.loop_start()                                        # MQTT 后台线程
    threading.Thread(target=voice_loop, daemon=True).start()  # 语音后台线程
    threading.Thread(target=watchdog, daemon=True).start()    # 页面关闭自动退出
    print("网页控制台: http://127.0.0.1:8080")
    # 延迟 2s 等服务器就绪后，自动用默认浏览器打开控制台
    threading.Timer(2.0, lambda: webbrowser.open("http://127.0.0.1:8080")).start()
    app.run(host="0.0.0.0", port=8080, debug=False)           # Flask 主线程


if __name__ == "__main__":
    main()
