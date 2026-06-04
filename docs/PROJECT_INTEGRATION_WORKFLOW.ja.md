# Project Integration Workflow

この文書では、この simulator を Git submodule として参照しながら、外側の project repo から実験を回すための現実的な運用フローをまとめます。

ここで想定する環境は次です。

- Windows または Ubuntu/Linux
- `uv`
- MATLAB

`.ps1` の wrapper は Windows 向けの例です。一方で、MATLAB 側の `auto_launch` は Windows と Ubuntu/Linux の両方で使える前提に寄せています。

## 基本原則

日常運用の入口は project repo 側に置きます。

- simulator の起動は project repo から行う
- project 固有 controller の起動も project repo から行う
- simulator repo は `external/mujoco_wheeled_uav_simulator/` 以下の固定済み dependency として扱う

## 実行フロー例

### 1. project を clone する

```powershell
git clone <project-repository-url>
cd <project-repository>
git submodule update --init --recursive
```

### 2. simulator 側の Python 環境を準備する

```powershell
uv sync --project external/mujoco_wheeled_uav_simulator
```

依存関係は simulator repo が定義しているので、環境構築は simulator 側 `pyproject.toml` に従わせるのが自然です。

### 3. simulator を起動する

通常例:

```powershell
uv run --project external/mujoco_wheeled_uav_simulator mujoco-wheeled-uav-simulator simulate
```

編隊制御例:

```powershell
uv run --project external/mujoco_wheeled_uav_simulator mujoco-wheeled-uav-simulator simulate --num-uavs 3
```

独立 instance 例:

```powershell
uv run --project external/mujoco_wheeled_uav_simulator mujoco-wheeled-uav-simulator simulate --instance-id 1
```

project 側で parameter override file を持つ場合は、明示的に渡します。

```powershell
uv run --project external/mujoco_wheeled_uav_simulator mujoco-wheeled-uav-simulator simulate --params-file configs/vehicle/vehicle_params.project.json --generated-xml-dir build/generated_xml
```

## 4. project 側 controller を起動する

controller 本体は simulator submodule の中ではなく、外側の project repo に置きます。

MATLAB の典型例:

```matlab
addpath('controllers/matlab');
addpath('external/mujoco_wheeled_uav_simulator');
addpath('external/mujoco_wheeled_uav_simulator/matlab');

my_project_controller
```

simulator 側の共有補助コードを使うのは構いませんが、sample controller を直接編集して流用する運用は避ける方が安全です。

## 4.5. project 側の experiment と review を起動する

experiment orchestration と review 入口も、project repo 側の所有物として置くのが原則です。

典型例:

- `experiments/matlab/project_contact_trials.m`
- `experiments/matlab/project_formation_trials.m`
- `analysis/matlab/project_contact_review.m`
- `analysis/matlab/project_formation_review.m`

これらの入口は `external/mujoco_wheeled_uav_simulator` の shared helper や sample 実装を明示的に呼んでも構いませんが、orchestration 自体は project 側に残してください。

## 5. ログは project repo 側へ保存する

長期運用では、run で生成したログは simulator submodule ではなく project repo 側に保存するのが望ましいです。

- 推奨: `project-repo/logs/...`
- 長期運用では避けたい: `project-repo/external/mujoco_wheeled_uav_simulator/logs/...`

これにより、「どの project が生成したログか」が自然に整理されます。

## 推奨 PowerShell ラッパー

毎回生コマンドを打つより、`scripts/` に薄いラッパーを置く方が運用しやすくなります。

ただし、ここで示す `.ps1` は Windows 向けです。Ubuntu/Linux では同じ責務の `.sh` を置くか、CLI を直接呼ぶ運用にしてください。starter template には両方を入れています。

`scripts/run_simulator.ps1` の例:

```powershell
param(
    [int]$InstanceId = 0,
    [int]$NumUavs = 1
)

$simRoot = "external/mujoco_wheeled_uav_simulator"

if ($NumUavs -gt 1) {
    uv run --project $simRoot mujoco-wheeled-uav-simulator simulate --num-uavs $NumUavs
} else {
    uv run --project $simRoot mujoco-wheeled-uav-simulator simulate --instance-id $InstanceId
}
```

`scripts/run_controller.ps1` の例:

```powershell
matlab -batch "addpath('controllers/matlab'); addpath('external/mujoco_wheeled_uav_simulator'); addpath('external/mujoco_wheeled_uav_simulator/matlab'); my_project_controller"
```

これらは薄い orchestration に留め、simulator のロジックを二重実装しないでください。

## 再現性メモの推奨内容

実験ファミリごとに、最低限次を残すと後で追いやすくなります。

- project commit hash
- simulator submodule commit hash
- 実行コマンド
- controller 入口
- 使用 config
- ログ出力先

実装方法は `docs/` 配下の Markdown でも、run 時に自動生成する metadata JSON や MAT でも構いません。

## simulator submodule の更新手順

simulator 側の更新は、必要なときに意図して行います。

典型的には次の流れです。

```powershell
cd external/mujoco_wheeled_uav_simulator
git fetch
git checkout <desired-tag-or-commit>
cd ../..
git add external/mujoco_wheeled_uav_simulator
git commit -m "Update simulator submodule"
```

もし必要な更新が reusable な機能なら、まず simulator repo 側へ変更を入れ、その commit へ project 側 submodule を更新する流れにします。

## 最初の試験運用は小さく始める

最初の外部 project では、次だけあれば十分です。

- controller 入口 1 本
- 基本 hover 実験 1 本
- contact または terrain 実験 1 本
- 再現メモ 1 本

これで責務境界が妥当かどうかを検証できます。最初から大きな自動化基盤を作る必要はありません。

## starter template

この repo には、実際に動かし始めるための雛形を [templates/project_repo](../templates/project_repo) として同梱しています。

最新版の template には、project-owned MATLAB controller に加えて、`experiments/matlab/` と `analysis/matlab/` 配下の project-owned experiment/review 入口も含まれます。