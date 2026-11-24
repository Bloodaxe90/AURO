"""Helpers for week_8's launch files."""

import atexit
import os
import signal
import tempfile
import time


def temporary_file(prefix: str, suffix: str, content: str) -> str:
    """Write content to a new file, removed when the launch exits, and return its path."""
    fd, path = tempfile.mkstemp(prefix=f'week_8_{prefix}_', suffix=suffix)
    with os.fdopen(fd, 'w') as f:
        f.write(content)
    atexit.register(os.remove, path)
    return path


# running_gazebo and stop_gazebo_server are copied from the assessment package
# (assessment/tasks.py), as in week_7, so that week_8 does not depend on it.

def running_gazebo(proc='/proc'):
    """
    Return (pid, command line) of every Gazebo (gz sim) server or GUI running on this machine.

    Found by their command lines: the gz-sim-* executables, or the gz tool running 'gz sim'.
    """
    found = []
    for entry in os.listdir(proc):
        if not entry.isdigit():
            continue
        try:
            with open(os.path.join(proc, entry, 'cmdline'), 'rb') as f:
                argv = [a.decode(errors='replace') for a in f.read().split(b'\0') if a]
        except OSError:
            continue  # gone, or not ours to read
        names = [os.path.basename(a) for a in argv]
        if (any(name.startswith('gz-sim-') for name in names)
                or any(a == 'gz' and b == 'sim' for a, b in zip(names, names[1:]))):
            found.append((int(entry), ' '.join(argv)))
    return sorted(found)


def stop_gazebo_server(world_file, timeout=10.0, proc='/proc'):
    """
    Stop any Gazebo server still running on world_file; return the pids it stopped.

    ros2 launch stops the Gazebo server through the gz tool's wrapper process. When the launch
    shuts itself down, e.g. because an RViz window was closed, only the wrapper is signalled, and
    the server is left behind, with no launch, until something kills it. The launch file calls
    this when it exits: world_file is a unique temporary copy of the world, so only its own server
    matches. Each is interrupted, then killed if still running after timeout seconds.
    """
    def running(pid):
        try:
            with open(f'{proc}/{pid}/stat') as f:
                return f.read().rsplit(')', 1)[1].split()[0] != 'Z'  # a zombie has stopped
        except (OSError, IndexError):
            return False

    pids = [pid for pid, command in running_gazebo(proc) if world_file in command.split()]
    for signal_number in (signal.SIGINT, signal.SIGKILL):
        for pid in [pid for pid in pids if running(pid)]:  # not one that has gone, and been reused
            try:
                os.kill(pid, signal_number)
            except ProcessLookupError:
                pass
        end = time.monotonic() + timeout
        while time.monotonic() < end and any(running(pid) for pid in pids):
            time.sleep(0.1)
    return pids
