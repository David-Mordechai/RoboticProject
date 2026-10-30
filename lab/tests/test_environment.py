"""Smoke tests for the lab image: torch works, and the ONNX export path (policy -> Pi 5) is intact."""
import numpy as np
import onnxruntime as ort
import pytest
import torch


def test_torch_cpu():
    a = torch.arange(6.0).reshape(2, 3)
    assert torch.allclose(a @ a.T, torch.tensor([[5.0, 14.0], [14.0, 50.0]]))


@pytest.mark.skipif(not torch.cuda.is_available(), reason="no GPU")
def test_torch_cuda_matches_cpu():
    x = torch.randn(256, 256, generator=torch.Generator().manual_seed(0))
    assert torch.allclose((x @ x).cpu(), (x.cuda() @ x.cuda()).cpu(), atol=1e-3)


def test_onnx_export_parity(tmp_path):
    torch.manual_seed(0)
    policy = torch.nn.Sequential(torch.nn.Linear(48, 128), torch.nn.Tanh(), torch.nn.Linear(128, 2)).eval()
    obs = torch.randn(1, 48)
    path = tmp_path / "policy.onnx"
    torch.onnx.export(policy, (obs,), str(path), input_names=["obs"], output_names=["action"], dynamo=True)
    sess = ort.InferenceSession(str(path), providers=["CPUExecutionProvider"])
    out = sess.run(None, {"obs": obs.numpy()})[0]
    np.testing.assert_allclose(out, policy(obs).detach().numpy(), atol=1e-5)
