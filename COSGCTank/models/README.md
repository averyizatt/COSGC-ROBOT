# Models (TFLite / ONNX)

This folder holds model assets used by the detector. On Raspberry Pi, the default path uses a TFLite model.

- Expected files:
  - `mobilenet_ssd.tflite` — the default SSD TFLite model

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

## Notes

- If you prefer ONNX for experimentation on a desktop, you can still place ONNX models here, but the Pi pipeline defaults to TFLite.
