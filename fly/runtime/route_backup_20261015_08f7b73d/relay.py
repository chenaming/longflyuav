#!/usr/bin/env python3
"""Restore the existing loopback inference endpoint; no ROS/control access."""
import argparse
import json
import urllib.error
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

MAX_BODY = 8 * 1024 * 1024


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--upstream', default='http://10.62.25.215:5007/predict')
    parser.add_argument('--port', type=int, default=15017)
    args = parser.parse_args()
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))

    class Handler(BaseHTTPRequestHandler):
        def reply(self, code, data, content_type='application/json'):
            self.send_response(code)
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', str(len(data)))
            self.end_headers()
            try:
                self.wfile.write(data)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def error(self, code, message):
            self.reply(code, json.dumps({'status': 'error', 'message': message}).encode())

        def do_GET(self):
            if self.path != '/healthz':
                return self.error(405, 'POST /predict required')
            self.reply(200, json.dumps({'service': 'fly-inference-relay-v1',
                                       'upstream': args.upstream}).encode())

        def do_POST(self):
            if self.path != '/predict':
                return self.error(404, 'unknown path')
            try:
                size = int(self.headers.get('Content-Length', '0'))
                if not 0 < size <= MAX_BODY:
                    return self.error(413, 'invalid request size')
                self.connection.settimeout(5)
                body = self.rfile.read(size)
                if len(body) != size:
                    return self.error(400, 'incomplete request')
                request = urllib.request.Request(args.upstream, data=body,
                                                headers={'Content-Type': 'application/json'},
                                                method='POST')
                with opener.open(request, timeout=12) as response:
                    result = response.read(MAX_BODY + 1)
                    if len(result) > MAX_BODY:
                        return self.error(502, 'response too large')
                    self.reply(response.status, result, response.headers.get_content_type())
            except urllib.error.HTTPError as exc:
                self.reply(exc.code, exc.read(MAX_BODY), exc.headers.get_content_type())
            except Exception as exc:
                self.error(502, 'upstream unavailable: %s' % exc)

        def log_message(self, fmt, *values):
            print(fmt % values, flush=True)

    server = ThreadingHTTPServer(('127.0.0.1', args.port), Handler)
    server.daemon_threads = True
    print('fly relay: 127.0.0.1:%s -> %s' % (args.port, args.upstream), flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == '__main__':
    main()
