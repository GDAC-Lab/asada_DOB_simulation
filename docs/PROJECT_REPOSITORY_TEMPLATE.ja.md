# Project Repository Template

この文書では、この simulator を依存先として使う研究用または案件用 project repo の推奨構成をまとめます。

狙いは次です。

- この repo は reusable な simulator 機能に集中させる
- project 固有の controller、実験、評価、出版資産は外へ出す
- Git submodule で simulator の版を固定し、再現性を高める

## 推奨ディレクトリ構成

```text
project-repo/
├─ .gitmodules
├─ external/
│  └─ mujoco_wheeled_uav_simulator/
├─ controllers/
│  ├─ matlab/
│  └─ python/
├─ experiments/
│  ├─ hover_baseline/
│  ├─ contact_trials/
│  └─ formation_trials/
├─ configs/
│  ├─ vehicle/
│  ├─ scenarios/
│  └─ logging/
├─ analysis/
│  ├─ matlab/
│  └─ python/
├─ scripts/
│  ├─ run_simulator.ps1
│  ├─ run_controller.ps1
│  ├─ run_experiment.ps1
│  └─ update_simulator.ps1
├─ logs/
├─ results/
│  ├─ figures/
│  └─ tables/
├─ docs/
│  ├─ reproducibility.md
│  └─ experiment_notes.md
└─ README.md
```

## 各ディレクトリの役割

### `external/mujoco_wheeled_uav_simulator/`

この simulator repo を Git submodule として配置します。

- upstream dependency として扱う
- project 固有ロジックはここへ置かない
- 実験に使った commit を明示的に固定して管理する

### `controllers/`

project 固有の制御ロジックを置きます。

- 論文固有、案件固有の controller はここに置く
- MATLAB controller は `controllers/matlab/`
- Python controller は `controllers/python/`
- simulator 側 sample controller とは明確に分ける

### `experiments/`

project 側で定義する実験カタログを置きます。

- 実験ファミリごとにディレクトリを切ると管理しやすい
- 実験入口スクリプト、scenario 定義、補足メモをここに置く
- その場しのぎの scratch 名より、実験名を明示する

### `configs/`

project 側が所有する設定レイヤを置きます。

- simulator 既定値から派生した vehicle variant
- scenario 設定
- logging 設定
- parameter sweep 設定

重要なのは、既定値は simulator 側にあっても、実験条件そのものの所有権は project 側に置くことです。

### `analysis/`

project 側の後処理と評価を置きます。

- プロットスクリプト
- 指標計算
- 論文や報告用の figure export
- 複数 run の比較スクリプト

### `scripts/`

薄い orchestration 用の入口を置きます。

- simulator 起動ラッパー
- controller 起動ラッパー
- 実験起動スクリプト
- submodule 更新補助

ここには simulator のロジックを再実装せず、あくまで simulator と project controller を呼び出す薄いスクリプトだけを置きます。

### `docs/`

project 単位の運用メモを置きます。

- 公表結果の再現手順
- 使用した submodule commit
- 図表ごとの config ファイル対応
- simulator 既定値からの差分

## 最小の Git submodule 設定

例:

```powershell
git submodule add <simulator-repository-url> external/mujoco_wheeled_uav_simulator
git submodule update --init --recursive
```

project README には、clone 時に submodule 初期化が必要であることを明記してください。

## 再現性のために残すべき情報

重要な実験ごとに、少なくとも次を記録します。

- project repo の commit
- `external/mujoco_wheeled_uav_simulator` の submodule commit
- 使用した config ファイル
- 使用した controller 入口
- ログ出力先
- 実行日

保存先は `docs/reproducibility.md`、experiment manifest、あるいは run 時の metadata のどれでも構いません。

## 日常開発の原則

日々の作業の主たる入口は project repo 側に置きます。

- 実験起動は project repo から行う
- controller と評価コードは project repo に置く
- simulator submodule を触るのは、その変更が reusable なときだけにする

## 最初の外部 project は小さく始める

最初に作る project repo は、あえて小さく保つのが安全です。

- controller は 1 つ
- 実験ファミリは 1 つか 2 つ
- 再現メモは 1 本
- simulator 起動スクリプトと controller 起動スクリプトを 1 本ずつ

これで simulator の責務境界が妥当かどうかを十分に試せます。最初から自動化を盛り込みすぎる必要はありません。

## この repo 内の starter template

具体的な雛形は [templates/project_repo](../templates/project_repo) に追加しています。

ここには次を入れています。

- project 側 MATLAB controller の最小例
- PowerShell の起動ラッパー
- Ubuntu/Linux 用 shell wrapper
- project 側 config override の置き場所
- ログと generated XML を project 側で持つための流れ

この template で想定している外側 project 風の起動フローは、この workspace 上で Windows 環境にて一度検証済みです。

starter template には、`experiments/matlab/` と `analysis/matlab/` 配下の project 側 MATLAB 実験入口・review 入口も含めています。contact trial、formation run、事後 review は simulator repo を直接編集せず、こちらを project 側の所有物として育てる想定です。