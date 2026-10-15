#!/usr/bin/env python3
import json
import threading
import time
from collections import deque
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

import rospy
from diagnostic_msgs.msg import DiagnosticArray
from mavros_msgs.msg import State, PositionTarget
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Log
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from uav_control_fsm.msg import uav_command


lock = threading.RLock()
seen = {}
counts = {}
events = deque(maxlen=200)
started = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

flight = {
    "connected": False,
    "armed": False,
    "mode": "未知",
}

bridge = ""
prearm = "未知"
system_state = "未知"
arm_seen = False
disarm_seen = False
motor_seen = False
motor_stopped = False
last_arm = None
last_result = ""
last_error = ""
last_outcome = ""
last_check = ""
coordinate_audit = {}
bench_report = {}
active_run_id = ""
model_bench_report = {}
last_messages = {}


def event(text, kind="info"):
    events.append({
        "time": datetime.now().strftime("%H:%M:%S"),
        "text": text,
        "kind": kind,
    })


def touch(key):
    seen[key] = time.monotonic()
    counts[key] = counts.get(key, 0) + 1


def receive(key):
    def callback(msg):
        with lock:
            first = key not in seen
            touch(key)
            if key == "command":
                last_messages[key] = "来源=%s；XYZ=(%.3f,%.3f,%.3f)m；yaw=%.2f°" % (
                    msg._connection_header.get("callerid", "unknown"),
                    msg.pos_ref.x, msg.pos_ref.y, msg.pos_ref.z, msg.yaw_ref)
            elif key == "setpoint":
                last_messages[key] = "来源=%s；XYZ=(%.3f,%.3f,%.3f)m；yaw=%.2f°；mask=%d" % (
                    msg._connection_header.get("callerid", "unknown"),
                    msg.position.x, msg.position.y, msg.position.z,
                    msg.yaw * 180 / 3.141592653589793, msg.type_mask)
            if first:
                event("首次收到：" + key)
    return callback


def on_state(msg):
    global last_arm, arm_seen, disarm_seen

    with lock:
        touch("state")
        flight.update(
            connected=bool(msg.connected),
            armed=bool(msg.armed),
            mode=msg.mode,
        )

        if last_arm is None or last_arm != msg.armed:
            if msg.armed:
                arm_seen = True
                event("飞控已解锁；不等同于电机实际转动", "warn")
            elif last_arm is True:
                disarm_seen = True
                event("飞控已从解锁转为上锁", "ok")
            else:
                event("初始飞控状态：未解锁")

            last_arm = msg.armed


def on_diagnostics(msg):
    global prearm, system_state

    with lock:
        for status in msg.status:
            if status.name == "uav1/mavros: System":
                touch("prearm")
                prearm = "未知"
                for item in status.values:
                    if item.key.startswith("pre-arm check status"):
                        prearm = item.value

            if status.name == "uav1/mavros: Heartbeat":
                touch("system")
                for item in status.values:
                    if item.key == "System status":
                        system_state = item.value


def on_bridge(msg):
    global bridge, last_check

    with lock:
        touch("bridge")
        bridge = msg.data
        detail = bridge.split("detail=", 1)[-1]

        if detail != last_check:
            last_check = detail
            event(
                "桥接：" + detail,
                "warn" if "rejected" in detail else "info",
            )


def on_log(msg):
    global last_result, last_error, last_outcome

    if msg.name.rstrip("/").split("/")[-1] != "vla_direct_bridge":
        return

    with lock:
        if msg.msg.startswith("HTTP success"):
            touch("http")
            last_result = msg.msg
            event(msg.msg, "ok")

        elif msg.msg.startswith("request id="):
            last_error = msg.msg
            last_outcome = "rejected"
            event(msg.msg, "warn")

        elif msg.msg.startswith("target preview frame="):
            last_outcome = "accepted"


def on_candidate(msg):
    with lock:
        touch("candidate")
        p = msg.pose.position
        seen["candidate_text"] = (
            "%s：x=%.3f，y=%.3f，z=%.3f m\n"
            "估算坐标，尚未完成标定验证"
            % (msg.header.frame_id, p.x, p.y, p.z)
        )


def on_preview(msg):
    global last_outcome

    with lock:
        touch("preview")
        last_outcome = "accepted"


def on_audit(msg):
    global coordinate_audit
    try:
        data = json.loads(msg.data)
        if data.get("run_id") != active_run_id:
            return
        with lock:
            coordinate_audit = data
            touch("audit")
    except (ValueError, TypeError):
        pass


def on_bench(msg):
    global bench_report
    try:
        data = json.loads(msg.data)
        if data.get("run_id") != active_run_id:
            return
        with lock:
            bench_report = data
            touch("bench")
            event("台架诊断：%s；不是模型目标" % data.get("status"), "info")
    except (ValueError, TypeError):
        pass


def on_model_bench(msg):
    global model_bench_report
    try:
        data = json.loads(msg.data)
        if data.get("run_id") != active_run_id:
            return
        with lock:
            model_bench_report = data
            touch("model_bench")
    except (ValueError, TypeError):
        pass


def snapshot():
    with lock:
        now = time.monotonic()

        def age(key):
            return now - seen[key] if key in seen else None

        def fresh(key, limit=3.0):
            value = age(key)
            return value is not None and value < limit

        cards = []

        def card(title, state, text):
            cards.append({
                "title": title,
                "state": state,
                "text": text,
            })

        def stream(title, key):
            value = age(key)
            text = "尚未收到"
            if value is not None:
                text = "已收 %d 条；距上次 %.1f 秒" % (
                    counts[key], value
                )
            if key in last_messages:
                text += "\n" + last_messages[key]
            passed = bench_report.get("status") == "passed" and key in ("command", "setpoint")
            if passed:
                text += "\n当前轮次台架诊断已通过 %d 项；目标来源为诊断脚本，不是模型。" % len(bench_report.get("cases", []))
            card(title, "ok" if fresh(key) or passed else "wait", text)

        stream("1 · 相机图像到 NX", "image")
        stream("2 · Odin 位姿到 NX", "odin")
        stream("3 · MAVROS 位姿到 NX", "odom")

        card(
            "4 · 桥接运行",
            "ok" if fresh("bridge") else "wait",
            bridge or "等待 /vla/bridge_status",
        )

        card(
            "5 · 主机推理结果返回 NX",
            "ok" if "http" in seen else "wait",
            (
                "本次监控已观察到 %d 次 HTTP 成功\n"
                "距最近成功 %.1f 秒\n%s"
                % (counts["http"], age("http"), last_result)
            ) if "http" in seen else "等待桥接 HTTP success 日志",
        )

        card(
            "6 · 候选目标转换",
            "warn" if "candidate" in seen else "wait",
            seen.get("candidate_text", "等待 /vla/target_candidate"),
        )

        card(
            "7 · 最近一次目标检查",
            "ok" if last_outcome == "accepted" else
            "warn" if last_outcome == "rejected" else "wait",
            (
                "通过代码中的目标检查；不代表坐标已标定"
                if last_outcome == "accepted"
                else last_error or "尚无检查结果"
            ),
        )

        stream("8 · 控制器输入 /move_command", "command")
        stream("9 · 发往 MAVROS 的 setpoint", "setpoint")

        card(
            "10 · 飞控当前状态",
            "ok" if fresh("state") and flight["connected"] else "wait",
            "连接=%s；解锁=%s；模式=%s%s" % (
                flight["connected"],
                flight["armed"],
                flight["mode"],
                "" if fresh("state") else "；数据未收到或已过期",
            ),
        )

        card(
            "11 · 解锁前检查",
            "wait" if not fresh("prearm") else
            "ok" if prearm.lower() == "ok" else "warn",
            "%s；系统状态=%s%s" % (
                prearm,
                system_state,
                "" if fresh("prearm") and fresh("system")
                else "（部分诊断过期或未收到）",
            ),
        )

        card(
            "12 · 本次监控的解锁记录",
            "ok" if arm_seen else "wait",
            "观察到 armed=True：%s\n之后观察到上锁：%s"
            % (arm_seen, disarm_seen),
        )

        card(
            "13 · 电机实际转动",
            "ok" if motor_seen else "wait",
            "人工确认转动：%s\n人工确认停转：%s\n不是转速遥测"
            % (motor_seen, motor_stopped),
        )

        card(
            "14 · 模型坐标验证",
            "warn" if coordinate_audit else "wait",
            ("服务器 action/action_ori 数学一致性：%s；误差 %.3g\n"
             "输入 cm/deg → 输出 cm/rad → 控制 m/deg。\n"
             "Odin→MAVROS 当前为配对估算；物理轴向和实飞效果尚未标定。" % (
                coordinate_audit.get("server_composition_verified"),
                coordinate_audit.get("max_composition_error", 0.))) if coordinate_audit else "等待坐标审计",
        )
        card(
            "15 · 未解锁 OFFBOARD 台架测试",
            "ok" if bench_report.get("status") == "passed" else "wait",
            ("状态=%s；通过 %d/%d 项；曾解锁=%s；恢复模式=%s\n"
             "实际话题端到端测试；不代表模型导航成功。" % (
                bench_report.get("status"),
                sum(bool(c.get("passed")) for c in bench_report.get("cases", [])),
                len(bench_report.get("cases", [])), bench_report.get("armed_ever"),
                bench_report.get("restored_mode", "等待"))) if bench_report else "尚未执行台架诊断",
        )
        card(
            "16 · 真实模型限幅台架传递",
            "ok" if model_bench_report.get("status") == "passed" else "wait",
            ("状态=%s；通过 %d/%d 次模型目标传递；曾解锁=%s；恢复=%s\n"
             "指令=%s；目标限幅 XY≤5cm、Z≤1cm、yaw≤3°。\n"
             "模型→控制器→MAVROS 已测试；机身固定，未验证飞行反馈或到椅子终点。" % (
                model_bench_report.get("status"),
                sum(bool(c.get("passed")) for c in model_bench_report.get("cases", [])),
                len(model_bench_report.get("cases", [])),
                model_bench_report.get("armed_ever"), model_bench_report.get("restored_mode", "等待"),
                model_bench_report.get("instruction"))) if model_bench_report else "尚未执行真实模型台架测试",
        )
        return {
            "model_bench_report": model_bench_report,
            "run_id": active_run_id,
            "instruction": rospy.get_param("/vla_direct_bridge/active_instruction", ""),
            "coordinate_audit": coordinate_audit,
            "bench_report": bench_report,
            "started": started,
            "cards": cards,
            "events": list(events),
            "preview": "preview=True" in bridge,
            "updated": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        }


HTML = r"""<!doctype html>
<html lang="zh-CN">
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>无人机链路检查</title>
<style>
*{box-sizing:border-box}
body{
 margin:0;background:#0b1220;color:#e5edf9;
 font:15px system-ui,sans-serif
}
main{max-width:1280px;margin:auto;padding:28px}
h1{font-size:27px;margin:0 0 10px}
.sub{color:#9dadc5;line-height:1.7}
.banner{
 padding:16px;border:1px solid #394964;background:#182337;
 border-radius:12px;margin:20px 0;line-height:1.8
}
.toolbar{display:flex;gap:10px;flex-wrap:wrap;margin:18px 0}
button{
 border:1px solid #52617c;border-radius:8px;background:#23334e;
 color:white;padding:11px 16px;cursor:pointer
}
.grid{
 display:grid;
 grid-template-columns:repeat(auto-fit,minmax(280px,1fr));
 gap:13px
}
.card{
 background:#131f32;border:1px solid #293a53;
 border-top:4px solid #64748b;border-radius:12px;padding:17px
}
.card.ok{border-top-color:#34d399}
.card.warn{border-top-color:#fbbf24}
.title{font-weight:650;margin-bottom:12px}
.body{
 color:#b8c7dc;font-size:13px;line-height:1.8;
 white-space:pre-wrap;overflow-wrap:anywhere
}
#connection{color:#34d399}
#events{
 background:#111c2d;border:1px solid #293a53;border-radius:12px;
 padding:16px;max-height:420px;overflow:auto
}
.event{
 padding:9px 0;border-bottom:1px solid #23324a;
 overflow-wrap:anywhere;font:13px monospace
}
.event.ok{color:#6ee7b7}
.event.warn{color:#fcd34d}
</style>
<main>
<h1>无人机实时链路检查</h1>
<div class="sub">
只读监控 · 测试由独立台架脚本执行；面板不发布指令 ·
<span id="connection">正在连接面板</span>
</div>

<div class="banner">
绿色表示该项已收到证据；黄色表示拒绝或需要人工核实；
灰色表示未观察到或数据已过期。<br>
HTTP 成功 ≠ 目标通过检查 ≠ 飞控执行目标 ≠ 电机实际转动。<br>
模型仍处于预览时，台架诊断也可单独产生 move_command；请看来源。
普通 setpoint 可能由悬停等控制逻辑产生，不能归因于模型。<br>
只记录本面板启动之后的事件。
人工按钮只记录观察结果，不控制飞机。
</div>

<div class="sub" id="summary"></div>
<div class="toolbar">
<button onclick="mark('motor_seen')">记录：我亲眼看到电机转动</button>
<button onclick="mark('motor_stopped')">记录：我亲眼确认电机停转</button>
<button onclick="downloadReport()">导出本次 JSON 记录</button>
</div>

<div class="grid" id="cards"></div>
<h2>最近事件</h2>
<div id="events"></div>
</main>

<script>
let report=null;

function element(tag,cls,text){
 const e=document.createElement(tag);
 e.className=cls;
 e.textContent=text;
 return e;
}

async function refresh(){
 try{
  const r=await fetch('/api',{
   cache:'no-store',
   signal:AbortSignal.timeout(4000)
  });
  if(!r.ok)throw Error(r.status);
  report=await r.json();

  const connection=document.getElementById('connection');
  connection.textContent='面板连接正常 · '+report.updated;
  connection.style.color='#34d399';

  document.getElementById('summary').textContent=
   '监控开始：'+report.started+
   ' ｜ 桥接报告预览模式：'+report.preview;

  const cards=document.getElementById('cards');
  cards.replaceChildren();

  for(const x of report.cards){
   const box=element('div','card '+x.state,'');
   box.append(
    element('div','title',x.title),
    element('div','body',x.text)
   );
   cards.append(box);
  }

  const es=document.getElementById('events');
  es.replaceChildren();

  for(const x of [...report.events].reverse()){
   es.append(element(
    'div','event '+x.kind,x.time+'  '+x.text
   ));
  }
 }catch(e){
  const connection=document.getElementById('connection');
  connection.textContent=
   '面板连接中断：显示的旧数据不能代表当前状态';
  connection.style.color='#fbbf24';
 }finally{
  setTimeout(refresh,1000);
 }
}

async function mark(action){
 if(!confirm(
  '仅记录你实际观察到的现象，不会向飞机发送命令。确认记录？'
 ))return;

 try{
  const r=await fetch('/mark',{
   method:'POST',
   headers:{'Content-Type':'application/json'},
   body:JSON.stringify({action})
  });
  if(!r.ok)throw Error(r.status);
 }catch(e){
  alert('记录失败，请检查面板连接');
 }
}

function downloadReport(){
 if(!report)return;
 const blob=new Blob(
  [JSON.stringify(report,null,2)],
  {type:'application/json'}
 );
 const a=document.createElement('a');
 a.href=URL.createObjectURL(blob);
 a.download='vla-check-'+Date.now()+'.json';
 a.click();
 setTimeout(()=>URL.revokeObjectURL(a.href),1000);
}

refresh();
</script>
</html>
"""


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def reply(self, code, body, content_type):
        data = body.encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()

        try:
            self.wfile.write(data)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def do_GET(self):
        path = urlparse(self.path).path

        if path == "/":
            self.reply(200, HTML, "text/html; charset=utf-8")
        elif path == "/api":
            self.reply(
                200,
                json.dumps(snapshot(), ensure_ascii=False),
                "application/json",
            )
        else:
            self.reply(404, "Not found", "text/plain")

    def do_POST(self):
        global motor_seen, motor_stopped

        if self.path != "/mark":
            self.reply(404, "Not found", "text/plain")
            return

        try:
            length = int(self.headers.get("Content-Length", "0"))
            if not 0 < length < 1024:
                raise ValueError()

            action = json.loads(self.rfile.read(length)).get("action")
            if action not in ("motor_seen", "motor_stopped"):
                raise ValueError()

            with lock:
                if action == "motor_seen":
                    motor_seen = True
                    motor_stopped = False
                    event("人工观察确认：电机实际转动", "warn")
                else:
                    motor_stopped = True
                    event("人工观察确认：电机已停转", "ok")

            self.reply(200, '{"ok":true}', "application/json")

        except (ValueError, TypeError, AttributeError):
            self.reply(400, "Invalid request", "text/plain")


def main():
    global active_run_id
    rospy.init_node(
        "vla_readonly_monitor",
        anonymous=True,
        disable_signals=True,
    )

    active_run_id = rospy.get_param("/vla_direct_bridge/run_id", "")
    host = rospy.get_param("~bind", "192.168.1.108")
    port = int(rospy.get_param("~port", 8088))
    server = ThreadingHTTPServer((host, port), Handler)
    rospy.on_shutdown(lambda: threading.Thread(target=server.shutdown, daemon=True).start())

    topics = [
        (
            "/uav1/odin1/image/compressed",
            CompressedImage,
            receive("image"),
        ),
        (
            "/uav1/odin1/odometry",
            Odometry,
            receive("odin"),
        ),
        (
            "/uav1/mavros/local_position/odom",
            Odometry,
            receive("odom"),
        ),
        ("/uav1/mavros/state", State, on_state),
        ("/diagnostics", DiagnosticArray, on_diagnostics),
        ("/vla/bridge_status", String, on_bridge),
        ("/vla/target_candidate", PoseStamped, on_candidate),
        ("/vla/target_preview", PoseStamped, on_preview),
        (
            "/uav1/move_command",
            uav_command,
            receive("command"),
        ),
        (
            "/uav1/mavros/setpoint_raw/local",
            PositionTarget,
            receive("setpoint"),
        ),
        ("/rosout_agg", Log, on_log),
        ("/vla/coordinate_audit", String, on_audit),
        ("/vla/bench_report", String, on_bench),
        ("/vla/model_bench_report", String, on_model_bench),
    ]

    subscribers = [
        rospy.Subscriber(
            topic,
            kind,
            callback,
            queue_size=30,
            buff_size=8 * 1024 * 1024,
        )
        for topic, kind, callback in topics
    ]

    print("打开网页：http://%s:%d" % (host, port), flush=True)
    print("只读监控已启动；Ctrl+C 仅关闭面板。", flush=True)

    try:
        server.serve_forever(poll_interval=0.5)
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        rospy.signal_shutdown("monitor stopped")


if __name__ == "__main__":
    main()
