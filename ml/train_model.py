"""Train the TinyML power-quality event classifier and export it to the firmware.

Pipeline (see docs/tinyml.md):
    pq_datagen (firmware DSP on simulated waveforms)  ->  train.csv / test.csv
    -> standardise features -> small MLP (Keras) -> full-integer int8 TFLite
    -> evaluate float AND int8 models on an independent test set
    -> export firmware/src/tinyml/model_data.{h,cpp}

Usage:
    python ml/train_model.py --train data/train.csv --test data/test.csv
Options:
    --hidden 32 16    hidden layer sizes
    --epochs 60
    --out-dir build/ml
"""
from __future__ import annotations

import argparse
import json
import os
import pathlib
import sys

import numpy as np

os.environ.setdefault("TF_CPP_MIN_LOG_LEVEL", "2")

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from export_model import CLASS_NAMES, export, fnv1a32, tflite_int8_probs  # noqa: E402


def load_csv(path: pathlib.Path):
    meta = {}
    with open(path) as f:
        first = f.readline()
        if first.startswith("#"):
            for tok in first[1:].split():
                k, _, v = tok.partition("=")
                meta[k] = v
            header = f.readline().strip().split(",")
        else:
            header = first.strip().split(",")
    data = np.loadtxt(path, delimiter=",", comments="#", skiprows=2 if meta else 1)
    cols = {name: i for i, name in enumerate(header)}
    feats = header[4:]
    X = data[:, 4:].astype(np.float32)
    y = data[:, cols["label"]].astype(np.int64)
    dsp = data[:, cols["dsp_class"]].astype(np.int64)
    clip = data[:, cols["clip"]].astype(np.int64)
    return X, y, dsp, clip, feats, meta


def confusion(y, p, n=8):
    m = np.zeros((n, n), dtype=np.int64)
    for t, q in zip(y, p):
        m[t, min(q, n - 1)] += 1
    return m


def report(title, y, p):
    m = confusion(y, p)
    acc = (y == p).mean()
    print(f"\n{title}: accuracy {acc * 100:.2f} %  ({len(y)} frames)")
    print(" " * 22 + " ".join(f"{c[:6]:>6}" for c in CLASS_NAMES) + "  recall")
    for i, c in enumerate(CLASS_NAMES):
        r = m[i, i] / max(1, m[i].sum())
        print(f"{c:>21} " + " ".join(f"{v:6d}" for v in m[i]) + f"  {r * 100:5.1f}%")
    return acc, m


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--train", required=True, type=pathlib.Path)
    ap.add_argument("--test", required=True, type=pathlib.Path)
    ap.add_argument("--hidden", type=int, nargs="+", default=[32, 16])
    ap.add_argument("--epochs", type=int, default=60)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--out-dir", type=pathlib.Path, default=pathlib.Path("build/ml"))
    ap.add_argument("--no-export", action="store_true")
    a = ap.parse_args()

    import tensorflow as tf
    tf.keras.utils.set_random_seed(a.seed)

    X, y, dsp_tr, clip, feats, meta = load_csv(a.train)
    Xt, yt, dsp_t, _, feats_t, meta_t = load_csv(a.test)
    if feats != feats_t:
        raise SystemExit("train/test feature columns differ")
    spec_hash = fnv1a32(",".join(feats))
    if meta.get("spec_hash") and int(meta["spec_hash"], 16) != spec_hash:
        raise SystemExit("CSV spec hash does not match its header")
    print(f"features ({len(feats)}): {', '.join(feats)}\nspec hash {spec_hash:#010x}")
    print(f"train {len(y)} frames, test {len(yt)} frames")

    # Validation split BY CLIP (frames of one clip are correlated).
    rng = np.random.default_rng(a.seed)
    clips = np.unique(clip)
    val_clips = set(rng.choice(clips, size=len(clips) // 8, replace=False).tolist())
    is_val = np.array([c in val_clips for c in clip])

    mean = X[~is_val].mean(axis=0)
    std = X[~is_val].std(axis=0)
    std[std < 1e-6] = 1.0
    Z, Zt = (X - mean) / std, (Xt - mean) / std

    counts = np.bincount(y[~is_val], minlength=8).astype(np.float64)
    cw = {i: float(counts.sum() / (8 * max(c, 1))) for i, c in enumerate(counts)}

    layers = [tf.keras.layers.Input(shape=(len(feats),))]
    for h in a.hidden:
        layers.append(tf.keras.layers.Dense(h, activation="relu"))
    layers.append(tf.keras.layers.Dense(8, activation="softmax"))
    model = tf.keras.Sequential(layers)
    model.compile(optimizer=tf.keras.optimizers.Adam(2e-3), loss="sparse_categorical_crossentropy",
                  metrics=["accuracy"])
    model.summary(print_fn=lambda s: print("  " + s))
    model.fit(Z[~is_val], y[~is_val], validation_data=(Z[is_val], y[is_val]), epochs=a.epochs,
              batch_size=256, class_weight=cw, verbose=2,
              callbacks=[tf.keras.callbacks.EarlyStopping(patience=10, restore_best_weights=True),
                         tf.keras.callbacks.ReduceLROnPlateau(patience=4, factor=0.5)])

    p_float = model.predict(Zt, verbose=0).argmax(axis=1)
    acc_f, _ = report("Float model (test set)", yt, p_float)

    # Full-integer quantisation with a representative dataset.
    rep = Z[rng.choice(len(Z), size=min(2000, len(Z)), replace=False)].astype(np.float32)

    def rep_gen():
        for r in rep:
            yield [r[None, :]]

    conv = tf.lite.TFLiteConverter.from_keras_model(model)
    conv.optimizations = [tf.lite.Optimize.DEFAULT]
    conv.representative_dataset = rep_gen
    conv.target_spec.supported_ops = [tf.lite.OpsSet.TFLITE_BUILTINS_INT8]
    conv.inference_input_type = tf.int8
    conv.inference_output_type = tf.int8
    tfl = conv.convert()

    probs_q = tflite_int8_probs(tfl, Zt.astype(np.float32))
    p_q = probs_q.argmax(axis=1)
    acc_q, m_q = report("int8 TFLite model (test set) — what runs on the MCU", yt, p_q)
    acc_dsp, _ = report("DSP threshold rules only (test set, same frames)", yt, np.clip(dsp_t, 0, 7))
    agree = (p_q == p_float).mean()
    print(f"\nint8 vs float agreement: {agree * 100:.2f} %")

    a.out_dir.mkdir(parents=True, exist_ok=True)
    tfl_path = a.out_dir / "pq_model.tflite"
    tfl_path.write_bytes(tfl)
    params = int(sum(np.prod(w.shape) for w in model.get_weights()))
    # Reference vectors: two test frames per class
    ref_idx = []
    for c in range(8):
        idx = np.where(yt == c)[0]
        ref_idx += list(rng.choice(idx, size=min(2, len(idx)), replace=False))
    meta_out = {
        "features": feats,
        "spec_hash": f"{spec_hash:#010x}",
        "mean": mean.tolist(),
        "std": std.tolist(),
        "hidden": a.hidden,
        "params": params,
        "tflite_bytes": len(tfl),
        "test_accuracy": float(acc_q),
        "test_accuracy_float": float(acc_f),
        "test_accuracy_dsp_rules": float(acc_dsp),
        "int8_float_agreement": float(agree),
        "confusion_int8": m_q.tolist(),
        "train_frames": int(len(y)),
        "test_frames": int(len(yt)),
        "ref_inputs": Xt[ref_idx].tolist(),
    }
    (a.out_dir / "pq_model_meta.json").write_text(json.dumps(meta_out, indent=1))
    print(f"\nmodel: {params} parameters, {len(tfl)} bytes int8 flatbuffer -> {tfl_path}")
    if not a.no_export:
        export(tfl_path, meta_out, Xt[ref_idx])


if __name__ == "__main__":
    main()
