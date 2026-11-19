# The CNN (`models/`)

`/mnt/c/Users/.../keras_model.h5` from the project is **not** a Teachable-Machine / MobileNet model. It is a small
Sequential Keras 2.15 CNN (116 MB with optimizer state):

```
Input 224x224x3 -> Conv2D 32 -> MaxPool -> Conv2D 64 -> MaxPool -> Conv2D 128 -> MaxPool -> Conv2D 128 -> MaxPool
                -> Flatten -> Dropout 0.5 -> Dense 512 (ReLU) -> Dense 1 (sigmoid)       loss: binary_crossentropy
```

So there is **one** output and no `labels.txt` to recover; the question is which side of 0.5 means "fire".

## Conversion

`tools/convert_keras_h5_to_onnx.py` reads the HDF5 weights with h5py and writes the same network as ONNX (no TensorFlow:
old Keras-2 files do not load in new Keras/TF). It was checked against an independent NumPy forward pass of the same
weights (identical outputs on 8 images, e.g. 0.5269 vs 0.5269) and against the layer list above.

* `models/fire_cnn.onnx` (39 MB) is committed and used by `cnn_detector` (onnxruntime, 2 threads).
* `models/keras_model.h5` (116 MB, above GitHub's 100 MB file limit) is kept locally and **git-ignored**.

## Which output means fire? (determined empirically)

Test set: 19 real fire photos (Wikimedia Commons, public domain/CC0: campfires, bonfires, wildfires, burning buildings)
and 35 non-fire photos (forest, city, interiors, kitchen, beach, sunsets, orange flowers, a red car, desk lamps, snowy
mountains, the robot photos, hand photos). Inputs resized to 224x224 (squash, as Keras' `flow_from_directory` does)

| input scaling | fire photos: mean output | non-fire: mean output | verdict |
|---|---|---|---|
| `/255` (RGB) | **0.04** (16 of 19 below 0.03) | 0.19 | fire = output near 0 |
| `/127.5 - 1` (Teachable-Machine style) | 0.95 | 0.86 | saturated, useless |
| raw 0..255 | 0.42 | 0.74 | useless |
| `/255`, BGR | 0.72 | 0.51 | weak |

So the model was trained on 0..1 images and its output is the probability of the **non-fire** class (Keras assigns class
index 0 to `fire_images` and 1 to `non_fire_images` alphabetically in the Kaggle dataset layout). The node therefore uses
`fire confidence = 1 - output` (parameter `invert`). The original `fire.py` normalises with `/127 - 1` (Teachable
Machine style), which does not match this model, another hint that the file in Downloads is a different model than the
one `fire.py` expects.

## How good is it? (honest)

With that convention on the same photos: it flags 18 of 19 real fires (recall 95%) but also 27 of 35 non-fire photos
(false alarms 77%). Many of my non-fire photos are paintings or indoor scenes, which the Kaggle non-fire class (mostly
outdoor nature) does not contain, so this is a harsher test than the training distribution, but it means the CNN alone is
not a reliable fire detector. That is why the plan has a fusion step (flame sensors trigger, the CNN only confirms).
The measured behaviour on the simulator camera is in the README / `scripts/eval_cnn_sim.py`.
