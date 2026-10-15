#!/usr/bin/env python3
"""Read-only evidence collector: pair live bridge traces with exact image stamps."""
import argparse
from collections import OrderedDict
import hashlib
import io
import json
from pathlib import Path
import queue
import threading
import time
import uuid


def model_jpeg(data):
    from PIL import Image
    rgb = Image.open(io.BytesIO(data)).convert('RGB').resize((224, 224))
    out = io.BytesIO()
    rgb.save(out, format='JPEG', quality=85)
    return out.getvalue()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--label', default='current')
    parser.add_argument('--samples', type=int, default=8)
    parser.add_argument('--timeout', type=float, default=20)
    args = parser.parse_args()
    if not 1 <= args.samples <= 50 or not 0 < args.timeout <= 120:
        raise SystemExit('invalid sample count or timeout')
    import rospy
    from sensor_msgs.msg import CompressedImage
    from rosgraph_msgs.msg import Log
    rospy.init_node('fly_vision_observer', anonymous=True)
    frames = OrderedDict()
    lock = threading.Lock()
    traces = queue.Queue(maxsize=100)
    directory = Path(__file__).resolve().parent / 'runtime' / 'vision_checks' / uuid.uuid4().hex
    directory.mkdir(parents=True)

    def image_callback(message):
        with lock:
            frames[message.header.stamp.to_sec()] = bytes(message.data)
            while len(frames) > 40:
                frames.popitem(last=False)

    def log_callback(message):
        if message.name != '/vla_direct_bridge' or not message.msg.startswith('COORD_TRACE '):
            return
        try:
            trace = json.loads(message.msg[len('COORD_TRACE '):])
            traces.put_nowait(trace)
        except (ValueError, queue.Full):
            pass

    rospy.Subscriber('/uav1/odin1/image/compressed', CompressedImage, image_callback, queue_size=20)
    rospy.Subscriber('/rosout', Log, log_callback, queue_size=100)
    rows = []
    deadline = time.monotonic() + args.timeout
    print('只读采集：%s，最多 %s 条；不会更改指令或发送控制目标。' % (args.label, args.samples), flush=True)
    while len(rows) < args.samples and time.monotonic() < deadline and not rospy.is_shutdown():
        try:
            trace = traces.get(timeout=.1)
        except queue.Empty:
            continue
        with lock:
            original = frames.get(trace['image_stamp'])
        if original is None:
            continue
        resized = model_jpeg(original)
        row = {
            'label': args.label, 'run_id': trace['run_id'], 'request_id': trace['request_id'],
            'instruction': trace['instruction'], 'image_stamp': trace['image_stamp'],
            'camera_jpeg_sha256': hashlib.sha256(original).hexdigest(),
            'model_jpeg_sha256': hashlib.sha256(resized).hexdigest(),
            'model_image_note': 'reconstructed using the unchanged bridge JPEG preprocessing',
            'input_proprio_cm_deg': trace['input_proprio_cm_deg'],
            'server_action_ori_cm_rad': trace['server_action_ori_cm_rad'], 'trace': trace,
        }
        basename = '%s_%s' % (len(rows), trace['request_id'])
        (directory / (basename + '_camera.jpg')).write_bytes(original)
        (directory / (basename + '_model.jpg')).write_bytes(resized)
        row['camera_file'] = str(directory / (basename + '_camera.jpg'))
        row['model_file'] = str(directory / (basename + '_model.jpg'))
        (directory / (basename + '.json')).write_text(json.dumps(row, ensure_ascii=False, indent=2), encoding='utf-8')
        rows.append(row)
        print('request=%s image=%s model_hash=%s action=%s' % (
            row['request_id'], row['image_stamp'], row['model_jpeg_sha256'][:12],
            row['server_action_ori_cm_rad']), flush=True)
    summary = {
        'label': args.label, 'sample_count': len(rows),
        'unique_camera_images': len({r['camera_jpeg_sha256'] for r in rows}),
        'unique_model_images': len({r['model_jpeg_sha256'] for r in rows}),
        'unique_full_actions': len({tuple(r['server_action_ori_cm_rad']) for r in rows}),
        'directory': str(directory), 'rows': rows,
    }
    (directory / 'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps({k: v for k, v in summary.items() if k != 'rows'}, ensure_ascii=False), flush=True)
    if not rows:
        raise SystemExit('没有取得与图像配对的坐标记录，请检查桥接是否正在推理。')


if __name__ == '__main__':
    main()
