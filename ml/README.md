# ML pipeline

```
pq_datagen  --clips 1200 --seed 1   --out data/train.csv     # firmware DSP on simulated clips
pq_datagen  --clips 300  --seed 777 --out data/test.csv      # independent test set
python ml/train_model.py --train data/train.csv --test data/test.csv
```

`train_model.py` trains the MLP, quantises it to full int8, evaluates the float model, the int8 model and the DSP-rules baseline on the test set, and writes:

- `build/ml/pq_model.tflite` and `build/ml/pq_model_meta.json`: normalisation, confusion matrix, accuracies;
- `firmware/src/tinyml/model_data.{h,cpp}`, via `export_model.py`.

After changing the feature list (`firmware/src/features/feature_spec.h`), regenerate the dataset and retrain. The firmware refuses a model whose feature-spec hash does not match its own.

To train on **real recordings**, produce a CSV with the same header: `clip,frame,label,dsp_class,<features...>`. The `features` message and the `FEATURES` command emit exactly this vector. Label the rows, then run `train_model.py` on it.

Details and results: [docs/tinyml.md](../docs/tinyml.md).
