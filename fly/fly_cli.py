#!/usr/bin/env python3
"""Interactive natural-language input, reusing one existing read-only bridge."""
import argparse
import fcntl
import json
import os
from pathlib import Path
import queue
import re
import time
import uuid

from trace_result import make_result


def main():
    parser = argparse.ArgumentParser(description='输入自然语言，返回主机推理后的目标坐标')
    parser.add_argument('--once', help='只推理这一条指令，输出后退出')
    parser.add_argument('--timeout', type=float, default=35.0)
    parser.add_argument('--node', default='/vla_direct_bridge')
    args = parser.parse_args()
    base = Path(__file__).resolve().parent
    runtime = base / 'runtime'
    runtime.mkdir(exist_ok=True)
    lock = (runtime / 'input.lock').open('a')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        raise SystemExit('已有一个 fly 输入终端，请回到那个终端继续。')

    import rosgraph
    import rospy
    from rosgraph_msgs.msg import Log
    from std_msgs.msg import String, Empty
    rospy.init_node('fly_input', anonymous=True)
    node = args.node.rstrip('/')
    master = rosgraph.Master(rospy.get_name())
    events = queue.Queue(maxsize=512)

    def put(kind, value):
        try:
            events.put_nowait((kind, value, time.monotonic()))
        except queue.Full:
            pass

    def on_log(message):
        if message.name.rstrip('/') != node:
            return
        if message.msg.startswith('COORD_TRACE '):
            try:
                put('trace', json.loads(message.msg[len('COORD_TRACE '):]))
            except ValueError:
                pass
        elif message.level >= Log.WARN and message.msg.startswith('request id='):
            put('rejected', message.msg)

    def on_validated(message):
        try:
            put('validated', json.loads(message.data))
        except ValueError:
            pass

    rospy.Subscriber('/rosout', Log, on_log, queue_size=100)
    rospy.Subscriber('/vla/validated_prediction', String, on_validated, queue_size=100)
    instruction_pub = rospy.Publisher('/vla/instruction', String, queue_size=1)
    reset_pub = rospy.Publisher('/vla/reset_origin', Empty, queue_size=1)

    def precheck():
        publishers, subscribers, _ = master.getSystemState()
        incoming = dict(subscribers).get('/vla/instruction', [])
        if incoming != [node]:
            raise RuntimeError('自然语言入口必须只有一个现有桥接订阅者，当前：%s' % incoming)
        if rospy.get_param(node + '/enable_publish', None) is not False:
            raise RuntimeError('现有桥接必须配置 enable_publish=false')
        if rospy.get_param(node + '/preview_pair_alignment', None) is not True:
            raise RuntimeError('现有桥接必须配置 preview_pair_alignment=true')
        output_nodes = dict(publishers).get('/uav1/move_command', [])
        if output_nodes:
            raise RuntimeError('检测到控制命令发布者，请先检查现有控制来源：%s' % output_nodes)
        return rospy.get_param(node + '/run_id')

    deadline = time.monotonic() + 5
    while instruction_pub.get_num_connections() < 1 or reset_pub.get_num_connections() < 1:
        if time.monotonic() > deadline:
            raise SystemExit('没有连接到现有桥接；本工具不会重新启动整套无人机程序。')
        time.sleep(.05)
    precheck()
    print('fly 已连接现有桥接。输入中文或英文指令；输入 q 退出。', flush=True)
    print('坐标来自实机图像和位姿；此终端只返回预览，不向控制器发送目标。', flush=True)

    def infer(instruction):
        run_id = precheck()
        while True:
            try:
                events.get_nowait()
            except queue.Empty:
                break
        instruction_pub.publish(String(data=instruction))
        deadline = time.monotonic() + 3
        while rospy.get_param(node + '/active_instruction', '') != instruction:
            if time.monotonic() > deadline:
                raise RuntimeError('桥接未确认收到指令')
            time.sleep(.05)
        not_before = rospy.Time.now().to_sec()
        started = time.monotonic()
        # 同一句话重复输入也取新的图像/原点；旧 epoch 结果由桥接丢弃。
        reset_pub.publish(Empty())
        print('正在等待这条指令的新图像推理结果……', flush=True)
        trace = None
        validated = {}
        rejected = {}
        trace_at = None
        deadline = started + args.timeout
        last_error = None
        while time.monotonic() < deadline and not rospy.is_shutdown():
            if rospy.get_param(node + '/run_id') != run_id:
                raise RuntimeError('桥接在请求期间重新启动；请重新输入指令')
            if rospy.get_param(node + '/active_instruction') != instruction:
                raise RuntimeError('另一入口修改了指令；本次取消以避免混用结果')
            try:
                kind, value, received_at = events.get(timeout=.1)
            except queue.Empty:
                kind = None
            if kind == 'trace':
                if (value.get('instruction') == instruction and value.get('run_id') == run_id
                        and value.get('image_stamp', -1) >= not_before and trace is None):
                    trace = value
                    trace_at = received_at
            elif kind == 'validated':
                if value.get('instruction') == instruction and value.get('run_id') == run_id:
                    validated[value.get('request_id')] = value
            elif kind == 'rejected':
                match = re.match(r'request id=(\d+): (.*)', value)
                if match:
                    rejected[int(match.group(1))] = match.group(2)
                    last_error = match.group(2)
            if trace is not None:
                request_id = trace['request_id']
                if (request_id in validated or request_id in rejected
                        or time.monotonic() - trace_at > 2):
                    precheck()
                    result = make_result(trace, validated.get(request_id), rejected.get(request_id))
                    result['client_wait_s'] = time.monotonic() - started
                    result['client_result_id'] = uuid.uuid4().hex
                    name = 'result_%s_%s.json' % (result['run_id'], result['client_result_id'])
                    path = runtime / name
                    encoded = json.dumps(result, ensure_ascii=False, allow_nan=False, indent=2)
                    path.write_text(encoded, encoding='utf-8')
                    tmp = runtime / 'latest.json.tmp'
                    tmp.write_text(encoded, encoding='utf-8')
                    os.replace(str(tmp), str(runtime / 'latest.json'))
                    target, delta = result['target'], result['relative_to_observation']
                    print('\n指令：%s' % instruction)
                    print('请求：%s / %s' % (run_id, request_id))
                    print('目标坐标 [%s]：x=%.6f m，y=%.6f m，z=%.6f m，yaw=%.3f°' % (
                        result['target_frame'], target['x'], target['y'], target['z'], target['yaw']))
                    print('相对拍照位姿：dx=%+.6f m，dy=%+.6f m，dz=%+.6f m，dyaw=%+.3f°' % (
                        delta['dx'], delta['dy'], delta['dz'], delta['dyaw']))
                    label = {'passed': '桥接预览检查通过', 'rejected': '桥接预览检查未通过',
                             'unconfirmed': '尚未取得同请求的检查结论'}[result['bridge_check']]
                    print('检查：%s%s' % (label, ('（%s）' % result['rejection']) if result['rejection'] else ''))
                    print('配对坐标为预览估算；本次没有向控制器发布目标。')
                    print('记录：%s\n' % path, flush=True)
                    return result
        raise RuntimeError('等待超时，未取得本次坐标。最近桥接原因：%s' % (last_error or '未收到坐标记录'))

    if args.once:
        instruction = args.once.strip()
        if not instruction:
            raise SystemExit('指令不能为空')
        try:
            infer(instruction)
        except Exception as exc:
            raise SystemExit('推理未完成：%s' % exc)
        return
    while not rospy.is_shutdown():
        try:
            instruction = input('请输入指令 > ').strip()
        except (EOFError, KeyboardInterrupt):
            break
        if instruction.lower() in ('q', 'quit', 'exit'):
            break
        if not instruction:
            continue
        if len(instruction) > 1000:
            print('指令过长，请限制在 1000 字符内。')
            continue
        try:
            infer(instruction)
        except Exception as exc:
            print('本次未完成：%s\n请检查日志或重新输入；现有程序继续运行。' % exc, flush=True)


if __name__ == '__main__':
    main()
