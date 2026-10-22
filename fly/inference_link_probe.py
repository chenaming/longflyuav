#!/usr/bin/env python3
"""One HTTP inference probe. ROS subscribers only; no control commands or services."""
import argparse
import base64
import hashlib
import io
import json
import math
import os
from pathlib import Path
import socket
import subprocess
import sys
import time
import urllib.error
import urllib.parse
import urllib.request
import uuid
import xmlrpc.client

MAX_BYTES = 8 * 1024 * 1024


class ProbeError(Exception):
    def __init__(self, kind, message):
        self.kind = kind
        super().__init__(message)


def save_json(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')


def finite4(value, label):
    if not isinstance(value, (list, tuple)) or len(value) != 4:
        raise ProbeError('invalid_action', label + ' must have four numbers')
    if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in value):
        raise ProbeError('invalid_action', label + ' contains invalid/non-finite numbers')
    return list(value)


def validate_result(result, proprio, request_id):
    if not isinstance(result, dict) or result.get('status') != 'success':
        raise ProbeError('inference_error', 'Server did not return status=success: ' + str(result)[:1600])
    if 'request_id' in result and result['request_id'] != request_id:
        raise ProbeError('response_mismatch', 'Server request_id does not match this request')
    action = finite4(result.get('action'), 'action')
    delta = finite4(result.get('action_ori'), 'action_ori')
    yaw = math.radians(proprio[3])
    expected = [proprio[0] + math.cos(yaw)*delta[0] - math.sin(yaw)*delta[1],
                proprio[1] + math.sin(yaw)*delta[0] + math.cos(yaw)*delta[1],
                proprio[2] + delta[2], yaw + delta[3]]
    errors = [abs(action[i] - expected[i]) for i in range(3)]
    errors.append(abs((action[3] - expected[3] + math.pi) % (2*math.pi) - math.pi))
    if max(errors) > 1e-3:
        raise ProbeError('coordinate_mismatch', 'action/action_ori coordinate contract mismatch: ' + str(errors))
    return {'action_cm_rad': action, 'action_ori_cm_rad': delta,
            'coordinate_max_error': max(errors)}


class TimeoutTransport(xmlrpc.client.Transport):
    def make_connection(self, host):
        conn = super().make_connection(host)
        conn.timeout = 2.0
        return conn


def capture_live(settings, directory):
    uri = settings['master_uri']
    try:
        with xmlrpc.client.ServerProxy(uri, transport=TimeoutTransport()) as master:
            answer = master.getSystemState('/inference_link_probe_check')
        if answer[0] != 1:
            raise RuntimeError(str(answer))
    except Exception as exc:
        raise ProbeError('ros_unavailable', 'ROS master unavailable at %s: %s. Start the sensor ROS stack or use --image FILE.' % (uri, exc))
    publishers = dict(answer[2][0])
    if not publishers.get(settings['image_topic']):
        raise ProbeError('no_image_publisher', 'No publisher for ' + settings['image_topic'] + '; check Odin USB ownership. No existing driver is stopped.')
    try:
        import rospy
        from sensor_msgs.msg import CompressedImage
        from nav_msgs.msg import Odometry
    except ImportError as exc:
        raise ProbeError('ros_environment', str(exc) + '; use bash ./fly/test_inference_link.sh')
    os.environ['ROS_MASTER_URI'] = uri
    # This initialization runs in a child process with an absolute parent deadline.
    rospy.init_node('inference_link_probe', anonymous=True, disable_signals=True)
    latest = {}
    def image_cb(msg):
        latest['image'] = (msg, time.monotonic())
    def odom_cb(msg):
        latest['odom'] = msg
    subscriptions = [rospy.Subscriber(settings['image_topic'], CompressedImage, image_cb,
                                      queue_size=1, buff_size=MAX_BYTES),
                     rospy.Subscriber(settings['odom_topic'], Odometry, odom_cb, queue_size=1)]
    deadline = time.monotonic() + settings['capture_timeout']
    last_age = None
    try:
        while time.monotonic() < deadline and not rospy.is_shutdown():
            sample = latest.get('image')
            if sample:
                msg, received = sample
                stamp = msg.header.stamp.to_sec()
                last_age = rospy.Time.now().to_sec() - stamp
                if stamp > 0 and 0 <= last_age <= settings['max_image_age'] and time.monotonic()-received <= settings['max_image_age']:
                    data = bytes(msg.data)
                    if not 0 < len(data) <= MAX_BYTES:
                        raise ProbeError('invalid_image', 'Compressed image is empty or exceeds 8 MiB')
                    (directory / 'source_image.bin').write_bytes(data)
                    metadata = {'source': 'live_ros', 'topic': settings['image_topic'],
                                'image_stamp': stamp, 'image_age_at_capture_s': last_age,
                                'frame_id': msg.header.frame_id, 'format': msg.format,
                                'captured_unix': time.time(), 'paired_odin': None}
                    odom = latest.get('odom')
                    if odom is not None and abs(odom.header.stamp.to_sec()-stamp) <= .05:
                        p, q = odom.pose.pose.position, odom.pose.pose.orientation
                        values = [p.x,p.y,p.z,q.x,q.y,q.z,q.w]
                        if all(math.isfinite(v) for v in values):
                            metadata['paired_odin'] = {'stamp': odom.header.stamp.to_sec(),
                                'frame_id': odom.header.frame_id, 'xyz_m': values[:3], 'quaternion_xyzw': values[3:]}
                    return metadata
            time.sleep(.02)
        raise ProbeError('image_timeout', 'No fresh image within %.1fs; last image age=%r. Check Odin/USB and ROS timestamps.' % (settings['capture_timeout'], last_age))
    finally:
        for sub in subscriptions:
            sub.unregister()
        rospy.signal_shutdown('inference-only image capture complete')


class NoRedirect(urllib.request.HTTPRedirectHandler):
    def redirect_request(self, req, fp, code, msg, headers, newurl):
        return None


def post_request(settings, directory):
    body = (directory / 'request.json').read_bytes()
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}), NoRedirect())
    request = urllib.request.Request(settings['url'], data=body,
                                     headers={'Content-Type': 'application/json'}, method='POST')
    started = time.monotonic()
    try:
        response = opener.open(request, timeout=settings['http_timeout'])
    except urllib.error.HTTPError as exc:
        response = exc
    except (urllib.error.URLError, OSError) as exc:
        raise ProbeError('http_connection_error', str(exc))
    with response:
        status = response.code
        raw = response.read(MAX_BYTES + 1)
    (directory / 'response_body.txt').write_bytes(raw[:MAX_BYTES])
    metadata = {'http_status': status, 'elapsed_s': time.monotonic()-started,
                'response_bytes': len(raw), 'response_file': str(directory / 'response_body.txt')}
    save_json(directory / 'http.json', metadata)
    if len(raw) > MAX_BYTES:
        raise ProbeError('response_too_large', 'Response exceeds 8 MiB; first 8 MiB saved')
    if not 200 <= status < 300:
        raise ProbeError('http_error', 'HTTP %s: %s' % (status, raw.decode('utf-8', 'replace')[:1800]))
    try:
        result = json.loads(raw)
    except (ValueError, UnicodeError) as exc:
        raise ProbeError('invalid_json', 'HTTP succeeded but response is not JSON: ' + str(exc))
    metadata.update(validate_result(result, settings['proprio'], settings['request_id']))
    metadata['server_status'] = result.get('status')
    return metadata


def worker(role, directory):
    settings = json.loads((directory / 'settings.json').read_text(encoding='utf-8'))
    try:
        result = capture_live(settings, directory) if role == 'capture' else post_request(settings, directory)
        save_json(directory / (role + '.json'), {'ok': True, 'data': result})
        return 0
    except Exception as exc:
        save_json(directory / (role + '.json'), {'ok': False, 'kind': getattr(exc, 'kind', role + '_error'), 'error': str(exc)})
        return 1


def bounded_worker(role, directory, timeout):
    try:
        process = subprocess.run([sys.executable, str(Path(__file__).resolve()), '--worker', role,
                                  '--worker-dir', str(directory)], timeout=timeout,
                                 stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    except subprocess.TimeoutExpired:
        raise ProbeError(role + '_timeout', role + ' exceeded absolute deadline %.1fs; worker stopped' % timeout)
    (directory / (role + '_stderr.txt')).write_bytes(process.stderr)
    path = directory / (role + '.json')
    if not path.exists():
        raise ProbeError(role + '_error', 'Worker exit %s: %s' % (process.returncode, process.stderr.decode('utf-8','replace')[-1200:]))
    result = json.loads(path.read_text(encoding='utf-8'))
    if not result.get('ok'):
        raise ProbeError(result.get('kind', role + '_error'), result.get('error', 'unknown error'))
    if process.returncode:
        raise ProbeError(role + '_error', 'Worker did not exit normally')
    return result['data']


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--url', default='http://192.168.1.10:5008/predict')
    parser.add_argument('--image', help='Use this saved image instead of ROS; never labelled as live')
    parser.add_argument('--instruction', default='fly to the chair')
    parser.add_argument('--proprio', nargs=4, type=float, metavar=('X_CM','Y_CM','Z_CM','YAW_DEG'))
    parser.add_argument('--check-connection', action='store_true', help='TCP check only; does not claim inference success')
    parser.add_argument('--master-uri', default=os.environ.get('ROS_MASTER_URI','http://localhost:11311'))
    parser.add_argument('--image-topic', default='/uav1/odin1/image/compressed')
    parser.add_argument('--odom-topic', default='/uav1/odin1/odometry')
    parser.add_argument('--connect-timeout', type=float, default=3)
    parser.add_argument('--capture-timeout', type=float, default=8)
    parser.add_argument('--http-timeout', type=float, default=20)
    parser.add_argument('--max-image-age', type=float, default=.5)
    parser.add_argument('--output-dir')
    parser.add_argument('--worker', choices=('capture','post'), help=argparse.SUPPRESS)
    parser.add_argument('--worker-dir', help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.worker:
        return worker(args.worker, Path(args.worker_dir))
    for key in ('connect_timeout','capture_timeout','http_timeout','max_image_age'):
        if not 0 < getattr(args,key) <= 120:
            parser.error(key + ' must be finite and in (0,120]')
    url = urllib.parse.urlsplit(args.url)
    if url.scheme not in ('http','https') or not url.hostname or url.username or url.password:
        parser.error('--url must be an HTTP(S) URL without embedded credentials')
    if not args.instruction.strip():
        parser.error('--instruction cannot be empty')
    proprio = args.proprio or [0.,0.,0.,0.]
    if not all(math.isfinite(v) for v in proprio):
        parser.error('--proprio must have four finite values')
    base = Path(args.output_dir) if args.output_dir else Path(__file__).resolve().parent / 'runtime' / 'inference_link'
    directory = base / (time.strftime('%Y%m%d_%H%M%S') + '_' + uuid.uuid4().hex[:8])
    directory.mkdir(parents=True, exist_ok=False)
    directory = directory.resolve()
    settings = dict(vars(args), proprio=proprio, request_id=time.time_ns()//1000000)
    save_json(directory / 'settings.json', settings)
    report = {'url': args.url, 'started_unix': time.time(), 'ok': False,
              'inference_validated': False, 'control_published': False,
              'physical_flight_validated': False, 'directory': str(directory)}
    exit_code = 1
    try:
        print('[1/3] TCP ' + args.url, flush=True)
        try:
            with socket.create_connection((url.hostname, url.port or (443 if url.scheme=='https' else 80)), args.connect_timeout):
                pass
        except OSError as exc:
            raise ProbeError('connection_failed', '%s:%s %s; ping success does not establish an HTTP service.' % (url.hostname, url.port or (443 if url.scheme=='https' else 80), exc))
        report['tcp_reachable'] = True
        if args.check_connection:
            report.update(ok=True, status='tcp_only', message='TCP reachable; no image sent and inference not tested')
        else:
            print('[2/3] Read image: ' + (args.image or args.image_topic), flush=True)
            if args.image:
                path = Path(args.image).expanduser().resolve()
                if not 0 < path.stat().st_size <= MAX_BYTES:
                    raise ProbeError('invalid_image', 'Image must be between 1 byte and 8 MiB')
                data = path.read_bytes()
                (directory/'source_image.bin').write_bytes(data)
                capture = {'source': 'saved_file', 'path': str(path), 'file_mtime': path.stat().st_mtime,
                           'live_image_verified': False}
            else:
                capture = bounded_worker('capture', directory, args.capture_timeout + 4)
                data = (directory/'source_image.bin').read_bytes()
            report['image'] = capture
            from PIL import Image
            rgb = Image.open(io.BytesIO(data)).convert('RGB').resize((224,224))
            buffer = io.BytesIO()
            rgb.save(buffer, format='JPEG', quality=85)
            jpeg = buffer.getvalue()
            (directory/'model_input.jpg').write_bytes(jpeg)
            report.update(proprio_cm_deg=proprio, proprio_source='explicit_argument' if args.proprio else 'new_observation_origin',
                          model_image_sha256=hashlib.sha256(jpeg).hexdigest())
            payload = {'image':base64.b64encode(jpeg).decode('ascii'), 'instr':args.instruction,
                       'proprio':proprio, 'request_id':settings['request_id']}
            save_json(directory/'request.json', payload)
            print('[3/3] POST /predict; proprio(cm,deg)=' + str(proprio), flush=True)
            result = bounded_worker('post', directory, args.http_timeout + 1)
            report.update(result, ok=True, status='success', inference_validated=True)
            print('Inference returned: action=' + str(result['action_cm_rad']), flush=True)
        exit_code = 0
    except Exception as exc:
        report.update(status=getattr(exc,'kind','probe_error'), error=str(exc))
        print('FAILED [' + report['status'] + ']: ' + str(exc), flush=True)
    finally:
        if (directory/'http.json').exists():
            report['http'] = json.loads((directory/'http.json').read_text(encoding='utf-8'))
        report['finished_unix'] = time.time()
        save_json(directory/'report.json', report)
        print('Report: ' + str(directory/'report.json'), flush=True)
    return exit_code


if __name__ == '__main__':
    sys.exit(main())
