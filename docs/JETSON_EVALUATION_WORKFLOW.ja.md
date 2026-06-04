# Jetson Evaluation Workflow

この文書では、MuJoCo を PC 側で動かし、Python controller を Jetson 側で動かす場合の評価フローを整理します。

## 目的

Jetson を使った評価では、次の 2 つを分けて扱います。

- `低 RTF を許容する時刻忠実評価`: `realtime_factor` が 1 未満でも、packet age や stale-command の意味づけが壁時計ベースで妥当かを確認する
- `RTF≈1 を狙う計算資源評価`: simulator をほぼ実時間で回しつつ、Jetson 側 controller が必要な計算 budget を満たせるかを確認する

この 2 つは別の実験目的であり、同じ HIL 実行でも報告を分けるべきです。

## 共通セットアップ

推奨トポロジ:

- PC 側で `simulate` を動かす
- Jetson 側で `hover-controller` を動かす
- 両側で同じ repository revision と `vehicle_params.json` を使う
- simulator は `0.0.0.0` に bind し、Jetson IP へ state packet を送る
- Jetson controller は `0.0.0.0` に bind し、PC IP へ command packet を返す

PC 側:

```powershell
uv sync
uv run mujoco-wheeled-uav-simulator simulate --fidelity-mode hil --bind-ip 0.0.0.0 --state-target-ip 192.168.0.42
```

Jetson 側:

```bash
uv sync
uv run mujoco-wheeled-uav-simulator hover-controller --fidelity-mode hil --bind-ip 0.0.0.0 --target-ip 192.168.0.10
```

`192.168.0.42` は Jetson IP、`192.168.0.10` は PC IP に読み替えてください。

## 1. 低 RTF を許容する時刻忠実評価

目的は、`realtime_factor` が 1 を下回っても、PC simulator と Jetson controller の間で壁時計ベースの timing 解釈が崩れていないことを確認することです。

推奨設定:

- `--fidelity-mode hil` を使う
- `network_fidelity` を有効にする
- state 送信遅延、command 受信遅延、jitter、stale threshold を実験条件に合わせて設定する
- packet metadata と MATLAB logging を有効にする

確認項目:

- `state_age_ms` と command packet age の推移
- `sequence_gap` と stale-command count
- 安全側 policy が意図どおり適用されたか
- `realtime_factor` が 1 未満でも runtime instrumentation の整合性が取れているか

単一 UAV の review 入口:

```matlab
jetson_timing_review('logs/hover_20260410_120000.mat')
contact_log_review('network', 'logs/hover_20260410_120000.mat')
```

複数 UAV の review 入口:

```matlab
formation_log_review('network')
```

## 2. RTF≈1 を狙う計算資源評価

目的は、Jetson 側 controller が必要な閉ループ周期を維持できるかを、ほぼ実時間の条件で確認することです。

推奨設定:

- 同じ PC simulator + Jetson controller 構成を使う
- まずは scene 複雑度と logging 負荷を抑え、そこから段階的に重くする
- 対象シナリオで `realtime_factor` が 1 に近いことを確認する
- remote link timing を含めたいなら `hil`、controller 計算時間の基準を先に見たいなら `baseline` から始める

確認項目:

- realtime factor
- controller compute time が control period budget 内か
- sustained load 下での packet age
- `baseline` に対する tracking degradation

単一 UAV の review 入口:

```matlab
jetson_compute_budget_review('logs/hover_20260410_120000.mat')
contact_log_review('network', 'logs/hover_20260410_120000.mat')
```

複数 UAV の review 入口:

```matlab
formation_log_review('rtf')
formation_log_review('network')
```

## 最低限報告する指標

時刻忠実評価では:

- state packet age
- command packet age
- sequence gap rate
- stale-command rate
- timeout count

計算資源評価では:

- realtime factor
- controller compute time
- sustained load 下での packet age
- `baseline` 比の tracking degradation

## 現時点の境界

この repository には、packet metadata、runtime age tracking、stale-command handling、HIL 向け network 擾乱注入、基本的な actuator dynamics、加法型 sensor noise と truth logging が入っています。残っている主な課題は、experiment batch の自動化と論文向け figure regeneration であり、Jetson 向け timing instrumentation の土台そのものではありません。