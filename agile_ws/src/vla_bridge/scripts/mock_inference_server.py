#!/usr/bin/env python3
"""Fixed-action HTTP mock for UAV-Flow /predict. Uses stdlib only."""

from __future__ import print_function

import argparse
import json

try:
    from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
except ImportError:
    from BaseHTTPServer import BaseHTTPRequestHandler, HTTPServer as ThreadingHTTPServer


REQUEST_COUNT = {"n": 0}


class MockHandler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        print("[mock] " + (fmt % args))

    def _send_json(self, code, payload):
        body = json.dumps(payload).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path.split("?", 1)[0] == "/health":
            self._send_json(200, {"status": "ok", "requests": REQUEST_COUNT["n"]})
            return
        self._send_json(404, {"status": "error", "message": "not found"})

    def do_POST(self):
        path = self.path.split("?", 1)[0]
        length = int(self.headers.get("Content-Length", "0") or 0)
        raw = self.rfile.read(length) if length > 0 else b""
        try:
            data = json.loads(raw.decode("utf-8") or "{}")
        except ValueError:
            data = {}

        if path == "/reset":
            REQUEST_COUNT["n"] = 0
            self._send_json(200, {"status": "success"})
            return

        if path != "/predict":
            self._send_json(404, {"status": "error", "message": "not found"})
            return

        REQUEST_COUNT["n"] += 1
        request_id = data.get("request_id")
        image = data.get("image")
        image_len = len(image) if isinstance(image, str) else 0
        print(
            "[mock] n=%d keys=%s request_id=%s proprio=%s instr=%r image_b64_len=%d"
            % (
                REQUEST_COUNT["n"],
                list(data.keys()),
                request_id,
                data.get("proprio"),
                data.get("instr"),
                image_len,
            )
        )
        self._send_json(
            200,
            {
                "status": "success",
                "action": [[0.3, 0.0, 1.2, 0.0]],
                "action_ori": [[0.3, 0.0, 0.0, 0.0]],
                "request_id": request_id,
                "message": "mock action",
            },
        )


def main():
    parser = argparse.ArgumentParser(description="Mock OpenVLA-UAV /predict server")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=5007)
    args = parser.parse_args()
    server = ThreadingHTTPServer((args.host, args.port), MockHandler)
    print("[mock] listening on http://%s:%d/predict" % (args.host, args.port))
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n[mock] stopped")
        server.server_close()


if __name__ == "__main__":
    main()
