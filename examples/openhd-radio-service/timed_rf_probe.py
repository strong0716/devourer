#!/usr/bin/env python3
"""Lab-only IPC sender and independent monitor receiver for timed RF slots.

Run rx on a second Pi with a kernel monitor interface already tuned to 5200
MHz. Run tx against a service built with OPENHD_TIMED_LAB_MODES=1. This tool
does not alter radio ownership or interface configuration.
"""

import argparse
import csv
import json
import select
import socket
import struct
import time


MAGIC = 0x4F484452
VERSION = 2
MARKER = b"OHTD"
PLAN_PERIOD_US = 1_800_000_000
SLOT_US = 20_000


def now_us():
    return time.monotonic_ns() // 1000


def message(kind, sequence, payload=b""):
    return struct.pack(">IHHII", MAGIC, VERSION, kind, sequence, len(payload)) + payload


def receive_ipc(sock):
    data = sock.recv(16384)
    if len(data) < 16:
        raise RuntimeError("short IPC response")
    magic, version, kind, sequence, size = struct.unpack(">IHHII", data[:16])
    if (magic, version, size) != (MAGIC, VERSION, len(data) - 16):
        raise RuntimeError("invalid IPC response")
    return kind, sequence, data[16:]


def request(sock, kind, sequence, payload=b""):
    sock.send(message(kind, sequence, payload))
    while True:
        response_kind, response_sequence, data = receive_ipc(sock)
        if response_kind == 8:
            raise RuntimeError(data.decode(errors="replace"))
        if response_kind != 7 or response_sequence != sequence:
            continue
        request_kind, code, error, verified, _, detail_len = struct.unpack(
            ">HHiBBH", data[:12]
        )
        detail = data[12:12 + detail_len].decode(errors="replace")
        if request_kind != kind or code:
            raise RuntimeError(f"request {kind} failed: {code} {error} {detail}")
        return data[12 + detail_len:]


def synchronize(sock, target, rounds=100):
    samples = []
    for sequence in range(rounds):
        sent = now_us()
        sock.sendto(struct.pack(">Q", sent), target)
        data, _ = sock.recvfrom(32)
        received = now_us()
        if len(data) != 24:
            continue
        t1, t2, t3 = struct.unpack(">QQQ", data)
        if t1 != sent:
            continue
        rtt = (received - sent) - (t3 - t2)
        remote_minus_local = ((t2 - sent) + (t3 - received)) / 2
        samples.append((rtt, remote_minus_local))
        time.sleep(0.001)
    if not samples:
        raise RuntimeError("no clock exchange samples")
    return min(samples)


def transmit(args):
    sync = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sync.settimeout(1)
    rtt_us, remote_minus_local_us = synchronize(
        sync, (args.receiver, args.sync_port)
    )
    sock = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
    sock.settimeout(5)
    sock.connect(args.socket)
    sock.send(message(1, 1))
    kind, sequence, ready = receive_ipc(sock)
    if kind != 2 or sequence != 1:
        raise RuntimeError("service did not return Ready")
    if len(ready) < 2 or not (struct.unpack(">H", ready[-2:])[0] & 0x0004):
        raise RuntimeError("lab service lacks fixed TDMA capability")
    request(sock, 3, 2, struct.pack(">IIB", 5200, 20, 0))
    request(sock, 4, 6, struct.pack(">iB", 10, 0))
    plan = (bytes([1]) + struct.pack(">IIIIQI", PLAN_PERIOD_US, 100_000,
            8_000, 3_000, 20260927, 1) + struct.pack(">II", 5200, 10_000_000)
            + bytes([1, 5]))
    generation = 1
    request(sock, 11, 3, plan + struct.pack(">I", generation))
    anchor = now_us()
    request(sock, 11, 4, bytes([2]) + struct.pack(">QI", anchor, 0))
    request(sock, 11, 5, bytes([4]) + struct.pack(">QQQQ", 0b110, 0, 0, 0))
    rtap = bytes.fromhex("00000d00008008000800373003")
    mac = bytes.fromhex("18d6c718b699")
    header = struct.pack("<HH", 0x0108, 0) + b"\xff" * 6 + mac + b"\xff" * 6
    header += struct.pack("<H", 0)
    deadline = time.monotonic() + args.seconds
    sent_data = sent_beacon = 0
    sequence = 100
    next_members = time.monotonic() + 0.5
    members_revision = 0
    while time.monotonic() < deadline:
        if time.monotonic() >= next_members:
            request(sock, 11, 500000 + members_revision,
                    bytes([4]) + struct.pack(">QQQQ", 0b110, 0, 0, 0))
            members_revision += 1
            next_members += 0.5
        stamp = now_us()
        beacon = sequence % 20 == 0
        marker = MARKER + bytes([1 if beacon else 0]) + struct.pack(">IQ", sequence, stamp)
        frame = rtap + header + marker + bytes(args.payload_bytes)
        payload = struct.pack(">II", sequence, generation) + frame
        sock.send(message(12 if beacon else 13, sequence, payload))
        if beacon:
            sent_beacon += 1
        else:
            sent_data += 1
        sequence += 1
        time.sleep(args.interval_ms / 1000)
    status = request(sock, 11, sequence + 1, bytes([7]))
    if args.generation_probe:
        request(sock, 11, 700001, plan + struct.pack(">I", 2))
        second_anchor = now_us()
        request(sock, 11, 700002, bytes([2]) + struct.pack(">QI", second_anchor, 0))
        request(sock, 11, 700003, bytes([4]) + struct.pack(">QQQQ", 0b110, 0, 0, 0))
        for i in range(100):
            kind = 2 if i % 2 == 0 else 3
            stamp = now_us()
            marker = MARKER + bytes([kind]) + struct.pack(">IQ", sequence, stamp)
            frame = rtap + header + marker + bytes(args.payload_bytes)
            payload = struct.pack(">II", sequence, 1 if kind == 2 else 2) + frame
            sock.send(message(13, sequence, payload))
            sequence += 1
            time.sleep(0.001)
        request(sock, 11, 700004, bytes([6]))
        for _ in range(10):
            stamp = now_us()
            marker = MARKER + bytes([4]) + struct.pack(">IQ", sequence, stamp)
            frame = rtap + header + marker + bytes(args.payload_bytes)
            sock.send(message(13, sequence,
                              struct.pack(">II", sequence, 2) + frame))
            sequence += 1
    else:
        request(sock, 11, sequence + 2, bytes([6]))
    end_rtt_us, end_offset_us = synchronize(
        sync, (args.receiver, args.sync_port)
    )
    sync.close()
    print(json.dumps({"anchor_us": anchor, "remote_minus_local_us": remote_minus_local_us,
                      "sync_min_rtt_us": rtt_us, "data_offered": sent_data,
                      "beacon_offered": sent_beacon,
                      "end_remote_minus_local_us": end_offset_us,
                      "end_sync_min_rtt_us": end_rtt_us,
                      "generation_probe": args.generation_probe,
                      "status": list(struct.unpack(">IIIIII", status))}))
    sock.close()


def receive(args):
    if args.socket:
        radio = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        radio.settimeout(5)
        radio.connect(args.socket)
        radio.send(message(1, 1))
        kind, sequence, _ = receive_ipc(radio)
        if kind != 2 or sequence != 1:
            raise RuntimeError("receiver service did not return Ready")
        request(radio, 3, 2, struct.pack(">IIB", 5200, 20, 0))
        radio.settimeout(None)
    else:
        radio = socket.socket(socket.AF_PACKET, socket.SOCK_RAW, socket.htons(3))
        radio.bind((args.interface, 0))
    sync = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sync.bind(("0.0.0.0", args.sync_port))
    with open(args.output, "w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["rx_local_us", "kind", "sequence", "tx_local_us"])
        deadline = time.monotonic() + args.seconds
        count = 0
        all_packets = 0
        first_frames = []
        while time.monotonic() < deadline:
            ready, _, _ = select.select([radio, sync], [], [], 0.1)
            for source in ready:
                if source is sync:
                    data, address = sync.recvfrom(32)
                    t2 = now_us()
                    if len(data) == 8:
                        sync.sendto(data + struct.pack(">QQ", t2, now_us()), address)
                else:
                    if args.socket:
                        kind, _, payload = receive_ipc(radio)
                        if kind != 6 or len(payload) < 2:
                            continue
                        frame = payload[1:]
                    else:
                        frame = radio.recv(8192)
                    stamp = now_us()
                    all_packets += 1
                    if len(first_frames) < 5:
                        first_frames.append(frame[:96].hex())
                    at = frame.find(MARKER)
                    if at < 0 or len(frame) < at + 17:
                        continue
                    kind, sequence, tx_stamp = struct.unpack(">BIQ", frame[at + 4:at + 17])
                    writer.writerow([stamp, kind, sequence, tx_stamp])
                    count += 1
    print(json.dumps({"captured": count, "all_packets": all_packets,
                      "first_frames": first_frames, "output": args.output}))


def main():
    parser = argparse.ArgumentParser()
    sub = parser.add_subparsers(dest="mode", required=True)
    rx = sub.add_parser("rx")
    rx.add_argument("--interface", default="wlan1")
    rx.add_argument("--socket")
    rx.add_argument("--seconds", type=float, default=8)
    rx.add_argument("--sync-port", type=int, default=19927)
    rx.add_argument("--output", required=True)
    tx = sub.add_parser("tx")
    tx.add_argument("--socket", required=True)
    tx.add_argument("--receiver", required=True)
    tx.add_argument("--sync-port", type=int, default=19927)
    tx.add_argument("--seconds", type=float, default=4)
    tx.add_argument("--interval-ms", type=float, default=1)
    tx.add_argument("--payload-bytes", type=int, default=128)
    tx.add_argument("--generation-probe", action="store_true")
    args = parser.parse_args()
    if args.mode == "tx" and not 0 <= args.payload_bytes <= 2000:
        parser.error("--payload-bytes must be 0..2000")
    receive(args) if args.mode == "rx" else transmit(args)


if __name__ == "__main__":
    main()
