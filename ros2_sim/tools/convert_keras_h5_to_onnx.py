#!/usr/bin/env python3
"""Convert the user's Keras .h5 (Sequential CNN: Conv2D+ReLU / MaxPool x4, Flatten, Dropout, Dense+ReLU, Dense+Sigmoid)
to ONNX without TensorFlow (old Keras 2.x HDF5 files do not load in new Keras/TF).

    python3 convert_keras_h5_to_onnx.py keras_model.h5 ../models/fire_cnn.onnx

Needs h5py + onnx + numpy. Input is NHWC float32 [N,224,224,3] exactly like the Keras model (no normalisation inside),
output is the sigmoid [N,1]. Only the layer types of this model are supported (the script refuses anything else).
"""
import json
import sys

import h5py
import numpy as np
import onnx
from onnx import TensorProto, helper, numpy_helper


def convert(h5_path, onnx_path):
    f = h5py.File(h5_path, "r")
    cfg = json.loads(f.attrs["model_config"])
    assert cfg["class_name"] == "Sequential", cfg["class_name"]
    mw = f["model_weights"]
    nodes, inits = [], []
    x = "input"
    nodes.append(helper.make_node("Transpose", [x], ["x_nchw"], perm=[0, 3, 1, 2]))
    x = "x_nchw"
    layout = "NCHW"
    shape = (224, 224, 3)

    def weights(name):
        return np.asarray(mw[name][name]["kernel:0"]), np.asarray(mw[name][name]["bias:0"])

    for layer in cfg["config"]["layers"]:
        kind, c = layer["class_name"], layer["config"]
        name = c["name"]
        if kind == "InputLayer":
            shape = tuple(c["batch_input_shape"][1:])
        elif kind == "Conv2D":
            assert c["padding"] == "valid" and tuple(c["strides"]) == (1, 1) and c["activation"] == "relu"
            k, b = weights(name)                                   # HWIO -> OIHW
            inits += [numpy_helper.from_array(k.transpose(3, 2, 0, 1).astype(np.float32), name + "_w"),
                      numpy_helper.from_array(b.astype(np.float32), name + "_b")]
            nodes.append(helper.make_node("Conv", [x, name + "_w", name + "_b"], [name + "_c"], kernel_shape=list(c["kernel_size"])))
            nodes.append(helper.make_node("Relu", [name + "_c"], [name + "_o"]))
            x = name + "_o"
            shape = (shape[0] - c["kernel_size"][0] + 1, shape[1] - c["kernel_size"][1] + 1, c["filters"])
        elif kind == "MaxPooling2D":
            assert c["padding"] == "valid"
            nodes.append(helper.make_node("MaxPool", [x], [name + "_o"], kernel_shape=list(c["pool_size"]), strides=list(c["strides"])))
            x = name + "_o"
            shape = (shape[0] // c["strides"][0], shape[1] // c["strides"][1], shape[2])
        elif kind == "Flatten":                                    # Keras flattens NHWC: go back before flattening
            nodes.append(helper.make_node("Transpose", [x], [name + "_t"], perm=[0, 2, 3, 1]))
            nodes.append(helper.make_node("Flatten", [name + "_t"], [name + "_o"], axis=1))
            x = name + "_o"
        elif kind == "Dropout":
            pass                                                   # inference: identity
        elif kind == "Dense":
            k, b = weights(name)
            inits += [numpy_helper.from_array(k.astype(np.float32), name + "_w"), numpy_helper.from_array(b.astype(np.float32), name + "_b")]
            nodes.append(helper.make_node("Gemm", [x, name + "_w", name + "_b"], [name + "_g"]))
            act = {"relu": "Relu", "sigmoid": "Sigmoid", "softmax": "Softmax"}[c["activation"]]
            nodes.append(helper.make_node(act, [name + "_g"], [name + "_o"]))
            x = name + "_o"
        else:
            raise SystemExit(f"unsupported layer {kind}")
    out_dim = inits[-1].dims[0]
    graph = helper.make_graph(
        nodes, "fire_cnn",
        [helper.make_tensor_value_info("input", TensorProto.FLOAT, ["N", *map(int, cfg["config"]["layers"][0]["config"]["batch_input_shape"][1:])])],
        [helper.make_tensor_value_info(x, TensorProto.FLOAT, ["N", int(out_dim)])], inits)
    model = helper.make_model(graph, opset_imports=[helper.make_opsetid("", 13)])
    model.ir_version = 8
    onnx.checker.check_model(model)
    # give the output a stable name
    for n in model.graph.node:
        n.output[:] = ["output" if o == x else o for o in n.output]
    model.graph.output[0].name = "output"
    onnx.save(model, onnx_path)
    print("saved", onnx_path)


if __name__ == "__main__":
    convert(sys.argv[1], sys.argv[2])
