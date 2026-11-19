# Time an exported policy on the car's CPU the way the policy node will run it: ONNX Runtime, 1 thread, one observation
# per call. Usage (in the MentorPi container):  python3 onnx_timing.py model.onnx
import sys, time
import numpy as np
import onnxruntime as ort

o = ort.SessionOptions(); o.intra_op_num_threads = 1; o.inter_op_num_threads = 1
s = ort.InferenceSession(sys.argv[1], o, providers=['CPUExecutionProvider'])
inp = s.get_inputs()[0]
x = np.random.default_rng(0).standard_normal([1] + list(inp.shape[1:])).astype(np.float32)
for _ in range(50): s.run(None, {inp.name: x})
t = []
for _ in range(2000):
  t0 = time.perf_counter(); s.run(None, {inp.name: x}); t.append(time.perf_counter() - t0)
t = np.array(t) * 1000
print(f'{sys.argv[1]}: input {inp.shape}, median {np.median(t):.3f} ms, p99 {np.percentile(t, 99):.3f} ms, '
      f'max {t.max():.3f} ms (budget at 20 Hz: 50 ms)')
