import socket, json, time

SERVER = ("127.0.0.1", 5005)
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.settimeout(1.0)

# fixed set of targets
targets = [
    [5, 0, 5], [10, 3, 0], [0, 8, 8],
    [3, 12, 3], [15, 1, 2], [8, 6, 8],
]

for algo in ["CCD", "FABRIK", "JACOBIAN"]:
    # reset arm
    sock.sendto(json.dumps({"target_pos": [5,0,5], "algo": algo, "mode": "ROBOT", "reset": True}).encode(), SERVER)
    sock.recvfrom(1024)

    samples = []
    for t in targets:
        for _ in range(200):
            msg = {"target_pos": t, "algo": algo, "mode": "ROBOT"}
            t0 = time.perf_counter()
            sock.sendto(json.dumps(msg).encode(), SERVER)
            sock.recvfrom(1024)
            samples.append((time.perf_counter() - t0) * 1000.0)

    samples.sort()
    n = len(samples)
    print(f"{algo}: avg {sum(samples)/n:.3f} ms  "
          f"median {samples[n//2]:.3f} ms  "
          f"p95 {samples[int(n*0.95)]:.3f} ms  max {samples[-1]:.3f} ms")