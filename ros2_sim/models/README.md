`fire_cnn.onnx` is converted from the project's `keras_model.h5` (see ../docs/cnn_model.md). The original `.h5`
(116 MB) is git-ignored: it exceeds GitHub's 100 MB limit. Re-create the ONNX with
`python3 ../tools/convert_keras_h5_to_onnx.py keras_model.h5 fire_cnn.onnx` (needs h5py + onnx).
