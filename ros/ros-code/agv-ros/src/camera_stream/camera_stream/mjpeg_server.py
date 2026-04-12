import threading
from http.server import BaseHTTPRequestHandler, HTTPServer

# Shared state — latest JPEG frame bytes, protected by a lock
_lock = threading.Lock()
_frame: bytes = b''
_frame_event = threading.Event()


def update_frame(jpeg_bytes: bytes):
    """Called by the ROS2 node each time a new frame arrives."""
    global _frame
    with _lock:
        _frame = jpeg_bytes
    _frame_event.set()   # wake all waiting clients
    _frame_event.clear()


class _MjpegHandler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass  # silence access logs for performance

    def do_GET(self):
        if self.path == '/stream':
            self.send_response(200)
            self.send_header('Content-Type',
                             'multipart/x-mixed-replace; boundary=frame')
            self.send_header('Cache-Control', 'no-cache')
            self.send_header('Connection', 'close')
            self.end_headers()
            try:
                while True:
                    # Block until a new frame is available (max 1 s timeout)
                    _frame_event.wait(timeout=1.0)
                    with _lock:
                        data = _frame
                    if not data:
                        continue
                    self.wfile.write(
                        b'--frame\r\n'
                        b'Content-Type: image/jpeg\r\n\r\n'
                        + data
                        + b'\r\n'
                    )
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError):
                pass  # client disconnected

        elif self.path == '/':
            # Convenience: browser-viewable page
            html = (
                b'<html><body style="margin:0;background:#000">'
                b'<img src="/stream" style="max-width:100%"/>'
                b'</body></html>'
            )
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.send_header('Content-Length', str(len(html)))
            self.end_headers()
            self.wfile.write(html)
        else:
            self.send_error(404)


def start_server(host: str = '0.0.0.0', port: int = 8080):
    """Launch the HTTP server in a daemon thread."""
    server = HTTPServer((host, port), _MjpegHandler)
    t = threading.Thread(target=server.serve_forever, daemon=True)
    t.start()
    return server