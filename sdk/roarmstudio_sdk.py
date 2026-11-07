"""RoArmStudio v1 loopback API client. Only the Python standard library is required."""
import http.client
import json
import time
import uuid
from urllib.parse import urlsplit


class SDKError(RuntimeError):
    def __init__(self, code, message, status=None):
        super().__init__(message)
        self.code, self.status = code, status


class Studio:
    def __init__(self, url, token, timeout=5):
        endpoint = urlsplit(url)
        if (endpoint.scheme != 'http' or endpoint.netloc != f'127.0.0.1:{endpoint.port}'
                or endpoint.path not in ('', '/') or endpoint.query or endpoint.fragment):
            raise ValueError('URL must be http://127.0.0.1:PORT')
        if not token:
            raise ValueError('A Studio connection token is required')
        self.port, self.token, self.timeout = endpoint.port, token, timeout
        self.lease = None

    @classmethod
    def from_connection(cls, text, timeout=5):
        data = json.loads(text)
        return cls(data['url'], data['token'], timeout)

    def _request(self, method, path, data=None, *, binary=False):
        headers = {'Authorization': f'Bearer {self.token}'}
        if self.lease:
            headers['X-Control-Lease'] = self.lease
        body = None
        if data is not None:
            headers['Content-Type'] = 'application/json'
            body = json.dumps(data, allow_nan=False).encode()
        conn = http.client.HTTPConnection('127.0.0.1', self.port, timeout=self.timeout)
        try:
            conn.request(method, path, body, headers)
            response = conn.getresponse()
            content = response.read()
            if response.status >= 400:
                error = json.loads(content)
                raise SDKError(error['error'], error['message'], response.status)
            if binary:
                return content, int(response.headers['X-Frame-Host-Ns']), response.headers
            return json.loads(content)
        finally:
            conn.close()

    def status(self):
        return self._request('GET', '/v1/status')

    def frame(self, camera):
        if camera not in ('hand', 'workspace'):
            raise ValueError('Camera must be hand or workspace')
        data, stamp, _ = self._request('GET', f'/v1/cameras/{camera}/frame.jpg', binary=True)
        return data, stamp

    def depth(self):
        data, stamp, headers = self._request('GET', '/v1/cameras/workspace/depth.png', binary=True)
        return data, stamp, float(headers['X-Depth-Scale-Mm'])

    def stream(self, camera):
        if camera not in ('hand', 'workspace'):
            raise ValueError('Camera must be hand or workspace')
        conn = http.client.HTTPConnection('127.0.0.1', self.port, timeout=self.timeout)
        try:
            conn.request('GET', f'/v1/cameras/{camera}/stream.mjpg',
                         headers={'Authorization': f'Bearer {self.token}'})
            response = conn.getresponse()
            if response.status >= 400:
                error = json.loads(response.read())
                raise SDKError(error['error'], error['message'], response.status)
            while response.readline() == b'--frame\r\n':
                headers = http.client.parse_headers(response)
                length = int(headers['Content-Length'])
                data = response.read(length)
                if len(data) != length or response.read(2) != b'\r\n':
                    raise SDKError('stream_interrupted', 'Incomplete camera frame')
                yield data, int(headers['X-Frame-Host-Ns'])
        finally:
            conn.close()

    def acquire(self):
        if self.lease:
            raise ValueError('This client already has a lease')
        self.lease = self._request('POST', '/v1/lease', {})['lease']
        return self

    def renew(self):
        return self._request('POST', '/v1/lease/renew', {})

    def release(self):
        if self.lease:
            try:
                self._request('POST', '/v1/lease/release', {})
            except SDKError as error:
                if error.code != 'invalid_lease':
                    raise
            finally:
                self.lease = None

    def cancel_pending(self):
        try:
            return self._request('POST', '/v1/cancel', {})
        finally:
            self.lease = None

    def _action(self, kind, request_id, **fields):
        return self._request('POST', '/v1/actions',
                             dict(kind=kind, request_id=str(request_id or uuid.uuid4()), **fields))

    def move_xyz(self, x, y, z, *, allow_degraded=False, request_id=None):
        return self._action('move_xyz', request_id, xyz_mm=[x, y, z], allow_degraded=allow_degraded)

    def jog_xyz(self, dx, dy, dz, *, allow_degraded=False, request_id=None):
        return self._action('jog_xyz', request_id, xyz_mm=[dx, dy, dz], allow_degraded=allow_degraded)

    def home(self, direction=None, *, request_id=None):
        fields = {} if direction is None else {'direction': direction}
        return self._action('home', request_id, **fields)

    def grip(self, closed, *, request_id=None):
        return self._action('grip', request_id, closed=closed)

    def action(self, action_id):
        return self._request('GET', '/v1/actions/' + str(uuid.UUID(str(action_id))))

    def wait(self, action_id, timeout=35):
        deadline = time.monotonic() + timeout
        renew_at = time.monotonic() + 5
        while True:
            result = self.action(action_id)
            if result['state'] not in ('queued', 'sent'):
                return result
            now = time.monotonic()
            if now >= deadline:
                raise TimeoutError('Timed out waiting for action result')
            if self.lease and now >= renew_at:
                self.renew()
                renew_at = now + 5
            time.sleep(.1)

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.release()
