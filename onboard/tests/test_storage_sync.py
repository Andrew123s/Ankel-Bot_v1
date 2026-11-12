import json
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer

from ankelbot.storage import Store
from ankelbot.sync import Uploader


def test_offline_keeps_data_and_online_uploads_it(tmp_path):
    store = Store(tmp_path / "a.sqlite")
    for k in range(7):
        store.event(float(k), "test", n=k)

    # no endpoint configured, and an endpoint nobody listens on: nothing is lost
    assert Uploader(store, "").try_sync() == 0
    assert Uploader(store, "http://127.0.0.1:9/ingest", timeout=0.5).try_sync() == 0
    assert len(store.unsynced("events")) == 7

    got = []

    class H(BaseHTTPRequestHandler):
        def do_POST(self):
            got.append(json.loads(self.rfile.read(int(self.headers["Content-Length"]))))
            self.send_response(200)
            self.end_headers()

        def log_message(self, *a):
            pass

    srv = HTTPServer(("127.0.0.1", 0), H)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    try:
        n = Uploader(store, f"http://127.0.0.1:{srv.server_port}/ingest").try_sync()
    finally:
        srv.shutdown()
    assert n == 7
    assert got[0]["table"] == "events" and len(got[0]["rows"]) == 7
    assert store.unsynced("events") == []
    store.close()
