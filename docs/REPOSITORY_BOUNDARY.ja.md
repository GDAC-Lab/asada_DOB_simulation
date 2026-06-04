# リポジトリ責務境界

このリポジトリは、最終的な研究コード置き場ではなく、共通で使い回す simulator 基盤として扱う前提にします。

長期的な運用モデルは次です。

- このリポジトリは reusable な simulator と sample controller の基盤として保つ
- 各研究プロジェクトや案件は別リポジトリで管理する
- その project 側から、このリポジトリを通常は Git submodule として参照する

## このリポジトリに置くもの

以下は共通 simulator 基盤の責務なので、このリポジトリに残します。

- MuJoCo モデル生成と simulator runtime
- UDP 通信と、共有する state/control packet の約束
- `vehicle_params.json` を中心とした共通パラメータ読込
- 複数プロジェクトで再利用できる地形、接触、ログ、解析の基本機能
- `simulate`、`check-model` などの CLI
- 外部 controller の接続方法を示す最小限の sample controller
- 参照用の基本実験フローとサンプル解析スクリプト
- 公開向け README、LICENSE、CITATION などの文書

## project 側に置くもの

以下は通常、このリポジトリではなく project 側リポジトリに置きます。

- 特定論文、特定案件、特定要件のための controller
- 1 つの研究課題に強く結びついた実験 orchestration
- project 固有の config、シナリオ、parameter sweep
- 1 つの project でしか使わない評価スクリプトや図表作成コード
- 論文用の図、表、集計結果、出版資産
- ある仮説だけを検証するための一時的な試験コード

## Core と Sample の区別

このリポジトリには reusable な core と、使い方を示す sample の両方が含まれます。ここは明確に区別して扱います。

### Core Components

core は、再利用する simulator 契約そのものを構成する部分です。

- `qav_wheel/`
- `vehicle_params.json`
- `qav_wheel.template.xml`
- `drone_sim.py` などの起動入口
- `matlab/shared/` 以下の共有補助コード

core への変更は、将来の複数 project にも効く改善だけを原則とします。

### Sample Components

sample は、接続例や基準実装を示すためのものです。

- `hovering_controller.m`
- `contact_test_controller.m`
- `multi_uav_formation_controller.m`
- `contact_log_review.m`
- `formation_log_review.m`
- `matlab/controllers/`、`matlab/experiments/`、`matlab/analysis/` 以下の実装

これらは実用的ではありますが、基本的には example と reference workflow です。project 固有 controller の常設場所にはしません。

## 変更をこの repo に入れてよいかの判断基準

新しい変更をこのリポジトリに入れる前に、次を確認します。

以下のどれかに yes なら、この repo に入れる価値があります。

- 将来の複数 project でも必要になるか
- simulator 契約そのものを改善するか
- project repo 間の重複を減らせるか
- sample workflow を明快にするが、project 固有前提は埋め込まないか

逆に、どれにも当てはまらないなら、基本的には外側の project repo に置くべきです。

## 将来の project repo の想定構成

この simulator を利用する project は、概ね次のような構成を想定します。

```text
project-repo/
├─ external/
│  └─ mujoco_wheeled_uav_simulator/
├─ controllers/
├─ experiments/
├─ configs/
├─ analysis/
├─ results/
└─ docs/
```

この形にしておくと、共通 simulator を隔離したまま、controller、実験条件、評価パイプラインは project 側で独立管理できます。

## 日常運用の原則

日々の開発では次を原則にします。

- 実験の起点は project 側リポジトリに置く
- この repo は upstream dependency として扱う
- この repo を直接触るのは、本当に reusable な変更だけに限定する
- reusable な変更は project 内で抱え込まず、この repo に戻す

## 現在の再編段階

現段階では、責務境界を文書だけでなく実装にも反映し始めています。

- Python runtime と MATLAB shared helper は reusable な core API として整理を進めている
- top-level MATLAB ファイルは、可能な範囲で thin wrapper と legacy sample 入口へ寄せている
- project 固有の experiment/review orchestration は template 側 `experiments/matlab/` と `analysis/matlab/` に受け皿を作り始めている

一方で、`matlab/experiments/` と `matlab/analysis/` の本格的な外出しや削減はまだ進行中です。

## 次に参照する文書

次段階の具体方針は、以下の文書にまとめています。

- [Project Repository Template](PROJECT_REPOSITORY_TEMPLATE.ja.md)
- [Project Integration Workflow](PROJECT_INTEGRATION_WORKFLOW.ja.md)