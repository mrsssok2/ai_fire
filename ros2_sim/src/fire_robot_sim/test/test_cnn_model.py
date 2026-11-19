import os

import numpy as np
import pytest

ort = pytest.importorskip("onnxruntime")
MODEL = os.path.join(os.path.dirname(__file__), "..", "..", "..", "models", "fire_cnn.onnx")


@pytest.mark.skipif(not os.path.exists(MODEL), reason="models/fire_cnn.onnx not present")
def test_onnx_model_has_the_keras_interface():
    sess = ort.InferenceSession(MODEL, providers=["CPUExecutionProvider"])
    assert sess.get_inputs()[0].shape[1:] == [224, 224, 3]                    # NHWC like the Keras model
    out = sess.run(None, {"input": np.random.default_rng(0).random((2, 224, 224, 3), dtype=np.float32)})[0]
    assert out.shape == (2, 1) and np.all((0 <= out) & (out <= 1))             # one sigmoid output
