#!/usr/bin/env python3
"""Web server for the frontend.

Serves the files in WebFrontend (like `python3 -m http.server`) and adds a small API:

  GET  /api/robot           is the robot program running? with which options? last output lines
  POST /api/robot/start     start run_robot.py, body: {"intersection": bool, "duck_evasion": bool}
  POST /api/robot/stop      stop the robot program
  GET  /api/settings        the last parameters set in the frontend
  POST /api/settings        save parameters, body: {"yellow": {...}, "pid": {...}, ...}
                            (only the given sections are replaced)
  POST /api/shutdown        power the Duckiebot off through the Duckietown health API
"""
import argparse
import collections
import json
import os
import shlex
import signal
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer

WORKSHOP_DIR = os.path.dirname(os.path.abspath(__file__))
RUN_ROBOT = os.path.join(WORKSHOP_DIR, 'run_robot.py')
DEFAULT_SETTINGS_FILE = '/workspace/settings/robot_settings.json'
# Duckietown health API on the robot (the container runs with --network host).
# A plain `sudo shutdown` keeps the fans running, this powers the robot off cleanly.
SHUTDOWN_URL = os.environ.get('SHUTDOWN_URL', 'http://localhost:8085/trigger/shutdown')

# sourced before the robot program starts, so rospy and the duckietown messages can be imported
ROS_SETUP_FILES = [
    os.environ.get('ROS_SETUP', '/opt/ros/noetic/setup.bash'),
    os.environ.get('MSG_SETUP', os.path.join(WORKSHOP_DIR, 'Duckietown_msg', 'devel', 'setup.bash')),
]


class RobotProcess:
    """Starts and stops run_robot.py and keeps its last output lines."""

    def __init__(self):
        self.lock = threading.Lock()
        self.process = None
        self.options = {}
        self.output = collections.deque(maxlen=40)

    def is_running(self):
        return self.process is not None and self.process.poll() is None

    def status(self):
        with self.lock:
            running = self.is_running()
            return {
                'running': running,
                'options': self.options if running else {},
                'exit_code': None if running or self.process is None else self.process.returncode,
                'output': list(self.output),
            }

    def start(self, intersection, duck_evasion, settings_file):
        with self.lock:
            if self.is_running():
                return False

            args = ['python3', '-u', RUN_ROBOT, '--settings', settings_file]
            if intersection:
                args.append('--intersection')
            if duck_evasion:
                args.append('--duck-evasion')

            sources = ' '.join(f'source {shlex.quote(path)};' for path in ROS_SETUP_FILES if os.path.exists(path))
            command = f'{sources} exec {" ".join(shlex.quote(arg) for arg in args)}'

            self.output.clear()
            self.output.append('$ run_robot.py ' + ' '.join(arg for arg in args[5:]))
            self.process = subprocess.Popen(
                ['bash', '-c', command],
                cwd=WORKSHOP_DIR,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                # own process group, so stop() reaches the program and nothing else
                start_new_session=True,
            )
            self.options = {'intersection': intersection, 'duck_evasion': duck_evasion}
            threading.Thread(target=self._read_output, args=(self.process,), daemon=True).start()
            return True

    def _read_output(self, process):
        for line in process.stdout:
            self.output.append(line.rstrip())
        process.wait()
        self.output.append(f'robot program ended (exit code {process.returncode})')

    def stop(self):
        with self.lock:
            if not self.is_running():
                return False
            process = self.process
            # Ctrl+C: run_robot.py stops the wheels and shuts down
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
            return True


class Settings:
    """The last parameters from the frontend, stored as JSON so they survive page changes and restarts."""

    def __init__(self, path):
        self.path = path
        self.lock = threading.Lock()

    def load(self):
        try:
            with open(self.path) as file:
                data = json.load(file)
            return data if isinstance(data, dict) else {}
        except (OSError, ValueError):
            return {}

    def update(self, sections):
        with self.lock:
            data = self.load()
            data.update(sections)
            os.makedirs(os.path.dirname(self.path) or '.', exist_ok=True)
            temporary = self.path + '.tmp'
            with open(temporary, 'w') as file:
                json.dump(data, file, indent=2)
            os.replace(temporary, self.path)
            return data


# talk to the health API directly, never through an HTTP proxy from the environment
_no_proxy = urllib.request.build_opener(urllib.request.ProxyHandler({}))


def _health_api(url):
    with _no_proxy.open(url, timeout=5) as response:
        return json.loads(response.read().decode(errors='replace'))


def shutdown_robot(attempts=3):
    """Ask the Duckietown health API to power the robot off. Returns (ok, message).

    The API answers the first call with a token ("needs-confirmation") and only shuts down
    when it is called again with that token. It then tells the battery to cut the power
    and shuts down the Jetson.
    """
    last_error = ''
    for attempt in range(1, attempts + 1):
        try:
            answer = _health_api(SHUTDOWN_URL)
            print(f'[shutdown] attempt {attempt}: {answer}', flush=True)
            if answer.get('status') == 'needs-confirmation':
                url = f"{SHUTDOWN_URL}?{urllib.parse.urlencode({'token': answer['token']})}"
                answer = _health_api(url)
                print(f'[shutdown] attempt {attempt} confirmed: {answer}', flush=True)
            if answer.get('status') == 'ok':
                return True, 'shutdown accepted by the health API'
            last_error = f"health API answered: {answer}"
        except (urllib.error.URLError, OSError, ValueError, KeyError) as exc:
            last_error = f'{SHUTDOWN_URL}: {exc}'
            print(f'[shutdown] attempt {attempt} failed: {last_error}', flush=True)
        time.sleep(1)
    return False, last_error


class Handler(SimpleHTTPRequestHandler):
    robot = None
    settings = None

    def end_headers(self):
        # always load the newest frontend files
        self.send_header('Cache-Control', 'no-store')
        super().end_headers()

    def send_json(self, data, status=200):
        body = json.dumps(data).encode()
        self.send_response(status)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def read_json(self):
        length = int(self.headers.get('Content-Length') or 0)
        if length == 0:
            return {}
        data = json.loads(self.rfile.read(length))
        if not isinstance(data, dict):
            raise ValueError('expected a JSON object')
        return data

    def do_GET(self):
        path = self.path.split('?')[0]
        if path == '/api/robot':
            self.send_json(self.robot.status())
        elif path == '/api/settings':
            self.send_json(self.settings.load())
        else:
            super().do_GET()

    def do_POST(self):
        path = self.path.split('?')[0]
        try:
            body = self.read_json()
        except ValueError as exc:
            self.send_json({'error': str(exc)}, 400)
            return

        if path == '/api/robot/start':
            intersection = bool(body.get('intersection', False))
            duck_evasion = bool(body.get('duck_evasion', False))
            # remember the options for the checkboxes, and keep the duck page in line with them
            duck = self.settings.load().get('duck')
            sections = {'start': {'intersection': intersection, 'duck_evasion': duck_evasion}}
            if isinstance(duck, dict):
                sections['duck'] = {**duck, 'evade': duck_evasion}
            self.settings.update(sections)
            started = self.robot.start(intersection, duck_evasion, self.settings.path)
            self.send_json({'started': started, **self.robot.status()})
        elif path == '/api/robot/stop':
            stopped = self.robot.stop()
            self.send_json({'stopped': stopped, **self.robot.status()})
        elif path == '/api/settings':
            self.send_json(self.settings.update(body))
        elif path == '/api/shutdown':
            self.robot.stop()
            ok, message = shutdown_robot()
            self.send_json({'ok': ok, 'message': message}, 200 if ok else 502)
        else:
            self.send_json({'error': 'not found'}, 404)

    def log_message(self, format, *args):
        # the frontend asks for the robot status every few seconds, don't log that
        if '/api/robot ' not in self.requestline:
            super().log_message(format, *args)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--port', type=int, default=int(os.environ.get('FRONTEND_PORT', 8765)))
    parser.add_argument('--directory', default=os.environ.get('FRONTEND_DIR', os.path.join(WORKSHOP_DIR, 'WebFrontend')))
    parser.add_argument('--settings', default=os.environ.get('ROBOT_SETTINGS_FILE', DEFAULT_SETTINGS_FILE))
    args = parser.parse_args()

    Handler.robot = RobotProcess()
    Handler.settings = Settings(args.settings)

    server = ThreadingHTTPServer(('0.0.0.0', args.port), partial(Handler, directory=args.directory))
    server.daemon_threads = True

    def shutdown(signum, frame):
        Handler.robot.stop()
        sys.exit(0)

    signal.signal(signal.SIGTERM, shutdown)
    signal.signal(signal.SIGINT, shutdown)

    print(f'frontend on port {args.port}, files from {args.directory}, settings in {args.settings}', flush=True)
    server.serve_forever()


if __name__ == '__main__':
    main()
