"""Read-only verification of the already reviewed GO2 Sport writer topology."""
import argparse
from collections import Counter
import os
from pathlib import Path
import re
import tempfile
import time
import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8
from unitree_api.msg import Request

from .node import API_QOS, LEASE_REQUEST, SPORT_MONITOR_QOS, SPORT_REQUEST, STOP_TOPIC, load_verified_writers

GO2_ADDRESS = "192.168.123.161"
BARE_DDS = "_CREATED_BY_BARE_DDS_APP_"
AUDIT_SECONDS = 7.0
DISCOVERY_SECONDS = 2.0
PARTICIPANT = re.compile(r"SPDP ST0 ([0-9a-f]+:[0-9a-f]+:[0-9a-f]+):1c1 .*? NEW .*? \(data (.*?) meta ")
SPORT_WRITER = re.compile(
    r"SEDP ST0 ([0-9a-f:]+) reliable volatile writer .*?\.rt/api/sport/request/"
    r"unitree_api::msg::dds_::Request_ .*? NEW \(as (.*?) ssm="
)


def entity_ids(gids):
    return Counter(gid[24:32] for gid in gids)


def advertises_go2(locators):
    hosts = {host for host in re.findall(r"udp/([0-9.]+):", locators) if not host.startswith("239.")}
    return hosts == {GO2_ADDRESS}


def verified_writers(trace):
    participants = {}
    writers = {}
    for line in trace.splitlines():
        if match := PARTICIPANT.search(line):
            participants[match[1]] = advertises_go2(match[2]) and " Unitree/" in line
        if match := SPORT_WRITER.search(line):
            parts = match[1].split(":")
            if len(parts) == 4:
                gid = "".join(f"{int(part, 16):08x}" for part in parts) + "0" * 16
                writers[gid] = (":".join(parts[:3]), advertises_go2(match[2]))
    return {gid for gid, (participant, from_go2) in writers.items()
            if from_go2 and participants.get(participant)}


def verify_snapshot(baseline, observed, trace, requests, lease_writers):
    if requests or lease_writers:
        raise ValueError(f"发现控制活动，Sport 请求={requests} 外部 lease 发布者={lease_writers}")
    if entity_ids(observed) != entity_ids(baseline):
        raise ValueError("当前 Sport 端点拓扑与已复核的机载名单不一致")
    if observed - verified_writers(trace):
        raise ValueError(f"有 Sport 端点无法核实为 GO2 本体 {GO2_ADDRESS} 的 Unitree 参与者")


def collect():
    rclpy.init(args=[])
    node = Node("go2_avoid_startup_writer_audit")
    snapshots = set()
    requests = 0
    lease_writers = 0
    non_native = False
    stop_requested = False

    def on_request(_msg):
        nonlocal requests
        requests += 1

    def on_stop(msg):
        nonlocal stop_requested
        stop_requested |= bool(msg.data)

    try:
        node.create_subscription(Request, SPORT_REQUEST, on_request, SPORT_MONITOR_QOS)
        node.create_subscription(Int8, STOP_TOPIC, on_stop, API_QOS)
        node.create_subscription(Int8, "/stop", on_stop, API_QOS)
        start = time.monotonic()
        while time.monotonic() - start < AUDIT_SECONDS:
            rclpy.spin_once(node, timeout_sec=.10)
            if stop_requested:
                raise ValueError("操作员停止启动")
            lease_writers = max(lease_writers, node.count_publishers(LEASE_REQUEST))
            if time.monotonic() - start >= DISCOVERY_SECONDS:
                endpoints = node.get_publishers_info_by_topic(SPORT_REQUEST)
                snapshots.add(frozenset(bytes(p.endpoint_gid).hex() for p in endpoints))
                non_native |= any(p.node_name != BARE_DDS or p.node_namespace != BARE_DDS
                                  for p in endpoints)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    if non_native or len(snapshots) != 1:
        raise ValueError("发现非机载 Sport 发布端点，或发现期间发布端点不稳定")
    return snapshots.pop(), requests, lease_writers


def refresh(baseline_path, trace_path):
    baseline = load_verified_writers(baseline_path)
    if not baseline:
        raise ValueError("缺少初次人工核实的机载白名单")

    config = ET.fromstring(os.environ["CYCLONEDDS_URI"])
    domain = config.find("Domain")
    if domain is None:
        raise ValueError("CycloneDDS 配置缺少 Domain")
    tracing = ET.SubElement(domain, "Tracing")
    ET.SubElement(tracing, "Verbosity").text = "fine"
    ET.SubElement(tracing, "OutputFile").text = str(trace_path)
    os.environ["CYCLONEDDS_URI"] = ET.tostring(config, encoding="unicode")
    observed, requests, lease_writers = collect()
    verify_snapshot(baseline, observed, trace_path.read_text(errors="replace"), requests, lease_writers)

    if observed == baseline:
        print(f"机载 Sport 白名单已核实 {len(observed)} 个 GID", flush=True)
        return
    trace_path.with_name("writers.previous").write_text(baseline_path.read_text())
    with tempfile.NamedTemporaryFile(mode="w", dir=baseline_path.parent, delete=False) as stream:
        stream.write(f"# GO2 {GO2_ADDRESS}，只读核实记录 {trace_path}\n")
        stream.write("\n".join(sorted(observed)) + "\n")
        pending = Path(stream.name)
    try:
        pending.replace(baseline_path)
    finally:
        pending.unlink(missing_ok=True)
    print(f"已更新本机白名单，核实 GO2 机载端点 {len(observed)} 个", flush=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("baseline", type=Path)
    parser.add_argument("trace", type=Path)
    args = parser.parse_args()
    try:
        refresh(args.baseline, args.trace)
    except (OSError, RuntimeError, ValueError, KeyError, ET.ParseError) as error:
        parser.exit(1, f"Sport 白名单复核失败，终止启动：{error}\n")


if __name__ == "__main__":
    main()
