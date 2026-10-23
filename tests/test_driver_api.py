import json
import socket
import threading
from collections.abc import Generator
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import pytest
from devkit_ui import driver_api


class FakeDriverHandler(BaseHTTPRequestHandler):
    def do_GET(self) -> None:
        if self.path != '/api/status':
            self.send_error(404)
            return
        body = json.dumps({'status': 'idle'}).encode()
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)


@pytest.fixture
def fake_driver(monkeypatch: pytest.MonkeyPatch) -> Generator[None, None, None]:
    server = ThreadingHTTPServer(('127.0.0.1', 0), FakeDriverHandler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    monkeypatch.setattr(driver_api, 'DRIVER_API_URL', f'http://127.0.0.1:{server.server_port}/api')
    yield
    server.shutdown()
    server.server_close()


@pytest.mark.usefixtures('fake_driver')
async def test_forwards_the_driver_answer() -> None:
    response = await driver_api.forward_to_driver('status')
    assert response.status_code == 200
    assert response.media_type == 'application/json'
    assert json.loads(bytes(response.body)) == {'status': 'idle'}


@pytest.mark.usefixtures('fake_driver')
async def test_forwards_the_driver_status_code() -> None:
    response = await driver_api.forward_to_driver('unknown')
    assert response.status_code == 404


async def test_answers_503_when_the_driver_is_unreachable(monkeypatch: pytest.MonkeyPatch) -> None:
    with socket.socket() as unused_socket:
        unused_socket.bind(('127.0.0.1', 0))
        free_port = unused_socket.getsockname()[1]
    monkeypatch.setattr(driver_api, 'DRIVER_API_URL', f'http://127.0.0.1:{free_port}/api')
    response = await driver_api.forward_to_driver('status')
    assert response.status_code == 503
