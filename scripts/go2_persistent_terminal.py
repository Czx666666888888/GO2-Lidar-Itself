#!/usr/bin/env python3
"""Keep an NX terminal alive across SSH disconnects and permit reattachment."""
import argparse
import errno
import fcntl
import json
import os
from pathlib import Path
import pty
import select
import signal
import socket
import subprocess
import sys
import termios
import time
import tty


def serve(directory, command):
    directory.mkdir(parents=True, exist_ok=True, mode=0o700)
    with (directory / "owner.lock").open("a") as owner:
        fcntl.flock(owner, fcntl.LOCK_EX | fcntl.LOCK_NB)
        path = directory / "terminal.sock"
        path.unlink(missing_ok=True)
        listener = socket.socket(socket.AF_UNIX)
        listener.bind(str(path))
        os.chmod(path, 0o600)
        listener.listen(1)
        listener.setblocking(False)
        pid, master = pty.fork()
        if pid == 0:
            listener.close()
            owner.close()
            os.execvp(command[0], command)
        os.set_blocking(master, False)
        client = None
        outgoing = bytearray()
        incoming = bytearray()
        stopping = False

        def stop(*_):
            nonlocal stopping
            if not stopping:
                stopping = True
                os.killpg(pid, signal.SIGINT)

        signal.signal(signal.SIGTERM, stop)
        signal.signal(signal.SIGINT, stop)
        signal.signal(signal.SIGHUP, signal.SIG_IGN)
        with (directory / "output.log").open("wb", buffering=0) as log:
            try:
                while True:
                    readers = [listener, master] + ([client] if client else [])
                    writers = ([client] if client and outgoing else []) + ([master] if incoming else [])
                    readable, writable, _ = select.select(readers, writers, [], .2)
                    if listener in readable:
                        connection, _ = listener.accept()
                        connection.setblocking(False)
                        if client:
                            connection.close()
                        else:
                            client = connection
                            outgoing = bytearray((directory / "output.log").read_bytes()[-65536:])
                    if master in readable:
                        try:
                            data = os.read(master, 65536)
                        except OSError as exc:
                            if exc.errno != errno.EIO:
                                raise
                            data = b""
                        if not data:
                            break
                        log.write(data)
                        if client:
                            outgoing.extend(data)
                            if len(outgoing) > 262144:
                                del outgoing[:-262144]
                    if client and client in readable:
                        try:
                            data = client.recv(4096)
                        except ConnectionError:
                            data = b""
                        if data:
                            incoming.extend(data)
                        else:
                            client.close()
                            client = None
                            outgoing.clear()
                    if master in writable and incoming:
                        try:
                            count = os.write(master, incoming)
                            del incoming[:count]
                        except BlockingIOError:
                            pass
                    if client and client in writable:
                        try:
                            count = client.send(outgoing)
                            del outgoing[:count]
                        except BlockingIOError:
                            pass
                        except ConnectionError:
                            client.close()
                            client = None
                            outgoing.clear()
                _, status = os.waitpid(pid, 0)
                (directory / "exit.json").write_text(json.dumps(dict(
                    wait_status=status, finished_ns=time.time_ns(), command=command)) + "\n")
                if client:
                    client.setblocking(True)
                    client.settimeout(2)
                    try:
                        client.sendall(outgoing)
                    except (OSError, socket.timeout):
                        pass
            finally:
                path.unlink(missing_ok=True)
                listener.close()
                if client:
                    client.close()
                os.close(master)
                # If the broker fails, stop its terminal process and let the
                # workflow perform its ordinary cleanup.
                try:
                    os.killpg(pid, signal.SIGINT)
                except ProcessLookupError:
                    pass


def attach(directory, command, attach_only=False):
    if not sys.stdin.isatty():
        raise RuntimeError("请用 ssh -t 连接此交互式入口")
    directory.mkdir(parents=True, exist_ok=True, mode=0o700)
    with (directory / "start.lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        with (directory / "owner.lock").open("a") as owner:
            try:
                fcntl.flock(owner, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                print("重新连接 NX 已有任务会话。", flush=True)
            else:
                if attach_only:
                    raise RuntimeError("当前没有运行中的持久任务")
                print("在 NX 创建持久任务会话。", flush=True)
                # Release the ownership lock before the detached broker takes it.
                fcntl.flock(owner, fcntl.LOCK_UN)
                with (directory / "broker.log").open("ab") as log:
                    subprocess.Popen(
                        [sys.executable, str(Path(__file__).resolve()), "--serve",
                         "--state-dir", str(directory), "--", *command],
                        stdin=subprocess.DEVNULL, stdout=log, stderr=log,
                        start_new_session=True, close_fds=True)
        peer = socket.socket(socket.AF_UNIX)
        deadline = time.monotonic() + 8
        while True:
            try:
                peer.connect(str(directory / "terminal.sock"))
                break
            except (FileNotFoundError, ConnectionRefusedError):
                if time.monotonic() >= deadline:
                    raise RuntimeError("持久终端未创建，查看 " + str(directory / "broker.log"))
                time.sleep(.05)
    print("已连接 NX 持久终端。SSH 断线后任务继续，Ctrl+C 请求停止。\r", flush=True)
    fd = sys.stdin.fileno()
    original = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        while True:
            readable, _, _ = select.select([fd, peer], [], [])
            if peer in readable:
                data = peer.recv(65536)
                if not data:
                    return
                sys.stdout.buffer.write(data)
                sys.stdout.buffer.flush()
            if fd in readable:
                data = os.read(fd, 4096)
                if not data:
                    return
                peer.sendall(data)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, original)
        peer.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--serve", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--attach-only", action="store_true",
                        help="只重连已有任务")
    parser.add_argument("--state-dir", type=Path,
                        default=Path.home() / "go2_full_nx" / "persistent_terminal")
    parser.add_argument("command", nargs=argparse.REMAINDER)
    args = parser.parse_args()
    command = args.command
    if command[:1] == ["--"]:
        command = command[1:]
    if not command:
        command = ["bash", str(Path.home() / "go2_full_nx" / "go2_nx.sh"),
                   "--execute", "--mode", "full", "--1"]
    directory = args.state_dir.resolve()
    if args.serve:
        serve(directory, command)
    else:
        attach(directory, command, args.attach_only)


if __name__ == "__main__":
    try:
        main()
    except (OSError, RuntimeError) as exc:
        print("持久终端退出 " + str(exc), file=sys.stderr)
        raise SystemExit(1)
