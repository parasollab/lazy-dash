"""Serve the browser viewers (viewer/) over http://127.0.0.1 and open them.

  python viewer/serve.py [--port 8765] [--no-browser]

Serves the repository root, plus /api/info, /api/list and /api/save (resources/scenes/ only).
"""
from __future__ import annotations

import argparse
import http.server
import json
import socketserver
import sys
import threading
import webbrowser
from pathlib import Path
from urllib.parse import parse_qs, urlparse

ROOT = Path(__file__).resolve().parents[1]
SAVE_DIR = ROOT / "resources" / "scenes"
TYPES = {".js": "text/javascript", ".mjs": "text/javascript", ".css": "text/css", ".html": "text/html",
         ".json": "application/json", ".jsonl": "application/x-ndjson", ".svg": "image/svg+xml",
         ".urdf": "application/xml", ".srdf": "application/xml", ".dae": "model/vnd.collada+xml",
         ".obj": "text/plain", ".csv": "text/csv", ".md": "text/markdown", ".png": "image/png"}


def inside(path: Path, folder: Path) -> bool:
    try:
        path.resolve().relative_to(folder.resolve())
        return True
    except ValueError:
        return False


class Handler(http.server.SimpleHTTPRequestHandler):
    extensions_map = {**http.server.SimpleHTTPRequestHandler.extensions_map, **TYPES}

    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=str(ROOT), **kwargs)

    def log_message(self, fmt, *args):  # errors only
        if len(args) > 1 and str(args[1])[:1] in "45":
            sys.stderr.write("%s\n" % (fmt % args))

    def end_headers(self):
        self.send_header("Cache-Control", "no-store")
        super().end_headers()

    def reply(self, code: int, body: dict):
        data = json.dumps(body).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        url = urlparse(self.path)
        if url.path == "/api/info":
            return self.reply(200, {"launcher": True, "root": ROOT.name})
        if url.path == "/api/list":
            rel = parse_qs(url.query).get("dir", ["resources/scenes"])[0]
            folder = ROOT / rel
            if not inside(folder, ROOT) or not folder.is_dir():
                return self.reply(404, {"error": f"no folder {rel}"})
            files = sorted(p for p in folder.rglob("*") if p.suffix in (".json", ".jsonl") and p.is_file())
            return self.reply(200, {"files": [{"path": p.relative_to(ROOT).as_posix(), "size": p.stat().st_size}
                                              for p in files]})
        return super().do_GET()

    def do_POST(self):
        url = urlparse(self.path)
        if url.path != "/api/save":
            return self.reply(404, {"error": "unknown endpoint"})
        # A custom header makes cross-origin pages need a CORS preflight, which this server never grants.
        if self.headers.get("X-Lazydash") != "save":
            return self.reply(403, {"error": "missing X-Lazydash header"})
        rel = parse_qs(url.query).get("path", [""])[0]
        target = ROOT / rel
        if not rel.endswith(".json") or not inside(target, SAVE_DIR):
            return self.reply(400, {"error": "scenes save only to resources/scenes/*.json"})
        text = self.rfile.read(int(self.headers.get("Content-Length", 0))).decode("utf-8")
        try:
            json.loads(text)
        except ValueError as e:
            return self.reply(400, {"error": f"not JSON: {e}"})
        target.parent.mkdir(parents=True, exist_ok=True)
        with target.open("w", encoding="utf-8", newline="\n") as f:
            f.write(text if text.endswith("\n") else text + "\n")
        return self.reply(200, {"saved": target.relative_to(ROOT).as_posix()})


class Server(socketserver.ThreadingMixIn, http.server.HTTPServer):
    daemon_threads = True
    allow_reuse_address = sys.platform != "win32"  # on Windows it would share a busy port


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--no-browser", action="store_true")
    a = ap.parse_args()
    server = None
    for port in [a.port + i for i in range(10)] + [0]:
        try:
            server = Server(("127.0.0.1", port), Handler)
            break
        except OSError:
            continue
    if server is None:
        sys.exit("viewer: no free port")
    url = f"http://127.0.0.1:{server.server_address[1]}/viewer/"
    print(f"Lazy-DaSH viewer at {url} (Ctrl+C to stop)", flush=True)
    if not a.no_browser:
        threading.Timer(0.3, webbrowser.open, (url,)).start()
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
