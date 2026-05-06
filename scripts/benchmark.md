Benchmark: ONNX model=/home/booster/spqr/arena/models/t1-velocity-flat/t1-velocity-flat_ppo_latest.onnx
           warmup=100 iters=1000

── ONNX Runtime ──
  iterations : 1000
  total      : 24.2824 ms
  mean       : 0.0242824 ms
  min        : 0.016449 ms
  max        : 0.200866 ms
  p50        : 0.0184 ms
  p95        : 0.050848 ms
  p99        : 0.113537 ms
  throughput : 41182 inf/s

Benchmark: TRT  engine=/home/booster/spqr/arena/models/t1-velocity-flat/t1-velocity-flat_ppo_latest.engine

── TensorRT ──
  iterations : 1000
  total      : 117.144 ms
  mean       : 0.117144 ms
  min        : 0.072736 ms
  max        : 2.42137 ms
  p50        : 0.079649 ms
  p95        : 0.186049 ms
  p99        : 1.18254 ms
  throughput : 8536.5 inf/s

  TRT speedup: 0.207287x
