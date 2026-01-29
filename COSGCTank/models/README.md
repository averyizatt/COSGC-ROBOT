# Models (TensorRT / ONNX)

This folder holds the ONNX model and the generated TensorRT engine used by `main.py` and the verifier script.

- Expected files:
  - `mobilenet_ssd.onnx` — the default SSD ONNX model
  - `mobilenet_ssd.engine` — the TensorRT engine built from the ONNX

## Fetch a default ONNX

Use the helper script to fetch a default SSD MobileNet ONNX model:

```bash
bash COSGC-ROBOT/COSGCTank/tools/fetch_default_onnx.sh
```

If the automatic download fails, place an SSD MobileNet ONNX at:

```
COSGC-ROBOT/COSGCTank/models/mobilenet_ssd.onnx
```

You can find ONNX models on the Hugging Face ONNX model zoo:

- https://huggingface.co/onnxmodelzoo

## Build the TensorRT engine

Convert the ONNX into a TensorRT engine using `trtexec` (Jetson Nano, FP16):

```bash
bash COSGC-ROBOT/COSGCTank/tools/trtexec_convert.sh COSGC-ROBOT/COSGCTank/models/mobilenet_ssd.onnx COSGC-ROBOT/COSGCTank/models/mobilenet_ssd.engine
```

`main.py` will auto-detect the engine at `COSGC-ROBOT/COSGCTank/models/mobilenet_ssd.engine` and enable TensorRT.
