# mujoco-wheeled-uav-simulator

MuJoCo 上の車輪付きクアッドロータを、Python シミュレータと MATLAB コントローラの UDP 通信で動かすサンプルです。今後の研究で使い回せるシミュレーション基盤として使うことと、論文中の制御則を他者が再現するときの参照実装にすることを主な想定用途にしています。

今後実装したい機能や検討事項は [BACKLOG.md](BACKLOG.md) で管理します。

`baseline` / `hil` の使い分けと、論文向けの fidelity 指標整理は [docs/FIDELITY_MODES.ja.md](docs/FIDELITY_MODES.ja.md) にまとめています。

## 特徴

- 車輪付きクアッドロータの MuJoCo シミュレーション
- MATLAB からのホバリング制御、接触試験、編隊制御ワークフロー
- Python と MATLAB で共有する `vehicle_params.json` ベースのパラメータ管理
- 単体機、独立 multi-instance、single-world multi-UAV に対応
- `.mat` ベースの接触ログ保存と解析
- 平面、傾斜面、関数ベース曲面地形に対応

## 必要なもの

- Python 3.12 以上
- `uv`
- MuJoCo の GUI を表示できるローカル実行環境
- MATLAB

## クイックスタート

依存関係を入れます。

```powershell
uv sync
```

通常の単体機シミュレーション:

```powershell
uv run mujoco-wheeled-uav-simulator simulate
```

リモート controller 試験では、simulator の bind 先 IP と state 送信先 IP を分けて指定できます。

```powershell
uv run mujoco-wheeled-uav-simulator simulate --bind-ip 0.0.0.0 --state-target-ip 192.168.0.42
```

```matlab
hovering_controller
```

従来のラッパー起動を使いたい場合:

```powershell
uv run python drone_sim.py
```

## よく使う実行例

### 単体機ホバリング

```powershell
uv run mujoco-wheeled-uav-simulator simulate
```

```matlab
hovering_controller
```

MATLAB の代わりに Python controller で hover 試験を回すこともできます。

```powershell
uv run mujoco-wheeled-uav-simulator hover-controller
uv run mujoco-wheeled-uav-simulator hover-controller --bind-ip 0.0.0.0 --target-ip 192.168.0.10
```

### PC simulator + Jetson Python controller

この repository では、Jetson を使う評価を「低 RTF を許容する時刻忠実評価」と「RTF≈1 を狙う計算資源評価」の 2 系統に分けて扱います。具体的な運用フローは [docs/JETSON_EVALUATION_WORKFLOW.ja.md](docs/JETSON_EVALUATION_WORKFLOW.ja.md) にまとめています。

最初のリモート構成としては、次の形をおすすめします。

- MuJoCo と `simulate` は GUI が使える Windows または Linux PC 側で動かす
- `hover-controller` は Jetson 側で動かす
- `vehicle_params.json` と packet 挙動を揃えるため、まずは両方のマシンに同じリポジトリを clone する
- Jetson 側でも使えるなら `uv` をそのまま使う

実行例:

MuJoCo を動かす PC 側:

```powershell
uv sync
uv run mujoco-wheeled-uav-simulator simulate --bind-ip 0.0.0.0 --state-target-ip 192.168.0.42
```

Python controller を動かす Jetson 側:

```bash
uv sync
uv run mujoco-wheeled-uav-simulator hover-controller --bind-ip 0.0.0.0 --target-ip 192.168.0.10
```

この例では、`192.168.0.42` は Jetson の IP、`192.168.0.10` は PC の IP に読み替えてください。

現時点の推奨は、Jetson 側にも同じリポジトリを clone して運用を始めることです。リモート実行フローが固まるまでは、その方が controller ロジック、packet 形式、共有パラメータの不一致を避けやすいためです。もし将来的に Jetson 側の導入をもっと軽くしたくなったら、その次の段階で controller 専用 dependency profile か、小さな standalone controller package へ切り出すのが自然です。

MATLAB と Python の hover controller が共通で前提にしている単一 UAV packet 仕様は次のとおりです。

- simulator からの state packet は JSON object で、少なくとも `time`, `position`, `velocity`, `angular_velocity_body`, `rotation_matrix` を含みます。
- `position`, `velocity`, `angular_velocity_body` は 3 要素ベクトルです。
- `rotation_matrix` は row-major の 3x3 回転行列を 1 次元化した 9 要素です。
- controller から simulator への command packet は JSON object で、`rotor_thrusts` または `rotor_omega` のどちらか一方を 4 要素ベクトルで持ちます。
- `hover-controller` は単一 UAV packet 専用で、`uavs` を含む複数 UAV packet を受けるとエラー終了します。
- 現在の Python `hover-controller` は通常の上向きロータだけを前提にしています。固定傾斜ロータ例は controller 側の配分一般化が入るまでは model 側サンプルとして扱ってください。

`baseline` / `hil` モードの使い分け:

- `--fidelity-mode baseline` は理想化された基準経路です。network 遅延注入、packet loss 注入、追加の sensor / actuator 劣化は入りません。
- `--fidelity-mode hil` は `vehicle_params.json` の `network_fidelity` を有効にし、state 送信遅延、command 受信遅延、jitter、packet loss、stale-command handling を反映します。
- Python と MATLAB のログには `sequence`、`source_state_sequence`、`wall_time_send_ns`、`state_age_ms`、controller 計算時間などの metadata が残るので、remote 実行でも同じ形式で評価できます。

HIL を意識した実行例:

```powershell
uv run mujoco-wheeled-uav-simulator simulate --fidelity-mode hil --bind-ip 0.0.0.0 --state-target-ip 192.168.0.42
uv run mujoco-wheeled-uav-simulator hover-controller --fidelity-mode hil --bind-ip 0.0.0.0 --target-ip 192.168.0.10
```

単一 UAV の Jetson review 入口:

```matlab
jetson_timing_review('logs/hover_20260410_120000.mat')
jetson_compute_budget_review('logs/hover_20260410_120000.mat')
```

### 1つの MuJoCo world での編隊制御

```powershell
uv run mujoco-wheeled-uav-simulator simulate --num-uavs 3
```

```matlab
multi_uav_formation_controller('num_uavs', 3)
multi_uav_formation_controller('num_uavs', 3, 'formation_radius', 2.0, 'spawn_radius', 2.0, 'base_height', 1.8)
```

### 独立した複数 simulator instance

このモードは、単体機の切り分け実験や比較用に向いています。

```powershell
uv run mujoco-wheeled-uav-simulator simulate --instance-id 0
uv run mujoco-wheeled-uav-simulator simulate --instance-id 1
uv run mujoco-wheeled-uav-simulator simulate --instance-id 0 --bind-ip 0.0.0.0 --state-target-ip 192.168.0.42
```

```matlab
hovering_controller('instance_id', 0)
hovering_controller('instance_id', 1)
contact_test_controller('landing', 'instance_id', 2)
```

### 接触試験とログ確認

```matlab
contact_test_controller('hover')
contact_test_controller('landing')
contact_test_controller('hard_landing')
contact_test_controller('ground_load')
contact_test_controller('wall')
contact_test_controller('wall_load')
```

```matlab
contact_log_review
contact_log_review('forces', 'logs/生成されたログ.mat')
contact_log_review('instantaneous', 'logs/生成されたログ.mat')
contact_log_review('network', 'logs/生成されたログ.mat')
```

### モデル生成確認のみ

```powershell
uv run mujoco-wheeled-uav-simulator check-model
uv run mujoco-wheeled-uav-simulator check-model --instance-id 1
uv run mujoco-wheeled-uav-simulator check-model --num-uavs 3
```

## Citation

このシミュレータを論文や学会発表などの academic work で利用する場合は、対応する論文、プレプリント、または project page を引用してください。引用メタデータは [CITATION.cff](CITATION.cff) にも置いてあります。出版情報が固まったら、そちらの仮の citation entry を具体的な書誌情報に差し替えてください。

## ライセンス

このプロジェクトは [MIT License](LICENSE) で公開しています。

## リポジトリ責務境界

このリポジトリは、共通 simulator 基盤と sample controller を置く場所として維持し、project 固有の controller や評価コードは外側の project repo に置く方針にします。将来の submodule 運用を前提にした具体方針は [docs/REPOSITORY_BOUNDARY.ja.md](docs/REPOSITORY_BOUNDARY.ja.md) にまとめています。

## 詳細リファレンス

<details>
<summary>リポジトリ構成</summary>

### トップレベルの主なファイル

| ファイル | 役割 |
|---------|------|
| `qav_wheel/` | Python シミュレータの本体パッケージです。地形生成、XML 構築、UDP 通信、シミュレーション実行、CLI を分割して保持します。 |
| `drone_sim.py` | 旧来の起動互換性を保つための薄い Python ラッパーです。内部では `qav_wheel` パッケージを呼び出します。 |
| `hovering_controller.m` | MATLAB 側ホバリング制御のルート入口です。内部実装は `matlab/controllers/hovering_controller_impl.m` にあります。 |
| `contact_test_controller.m` | 接触試験シナリオのルート入口です。内部実装は `matlab/experiments/contact_test_controller_impl.m` にあります。 |
| `multi_uav_formation_controller.m` | 1 つの MuJoCo world に複数 UAV を生成し、単一の MATLAB controller でフォーメーション制御するルート入口です。 |
| `contact_log_review.m` | 接触ログ解析のルート入口です。内部実装は `matlab/analysis/contact_log_review_impl.m` にあります。 |
| `formation_log_review.m` | 編隊制御ログをまとめて確認するルート入口です。内部実装は `matlab/analysis/formation_log_review_impl.m` にあります。 |
| `matlab/shared/controller_shared.m` | MATLAB controller 群で共有する通信・制御・起動補助です。 |
| `matlab/shared/simulation_logger.m` | MATLAB 側のログ保存クラスです。`logs/` 以下に `.mat` を出力します。 |
| `vehicle_params.json` | 機体・アクチュエータ・環境の共有パラメータです。Python と MATLAB の両方から参照します。 |
| `qav_wheel.template.xml` | MuJoCo モデルのテンプレートです。実行時に `vehicle_params.json` から `build/generated_xml/` 配下へ XML を生成します。 |
| `drone_body.stl` | 機体メッシュです。 |
| `pyproject.toml` | Python 依存関係の宣言です。 |
| `uv.lock` | `uv` 用のロックファイルです。 |

### Python パッケージ内訳

| モジュール | 役割 |
|---------|------|
| `qav_wheel/cli.py` | CLI 入口です。`simulate` と `check-model` を振り分けます。 |
| `qav_wheel/config.py` | `vehicle_params.json` の読込を担当します。 |
| `qav_wheel/contact.py` | MuJoCo 接触情報の集計と contact report 構築を担当します。 |
| `qav_wheel/model_builder.py` | XML テンプレートへの埋め込み値生成と `build/generated_xml/` 配下への生成 XML 出力を担当します。 |
| `qav_wheel/network.py` | UDP 通信と MATLAB からの制御入力の解釈を担当します。 |
| `qav_wheel/paths.py` | リポジトリ直下の主要ファイルパスと共通定数をまとめています。 |
| `qav_wheel/simulation.py` | MuJoCo モデル読込、viewer 設定、状態送信、`check-model` 実行を担当します。 |
| `qav_wheel/surface.py` | 平面・傾斜・曲面の設定解釈、法線計算、plane/hfield 生成を担当します。 |
| `qav_wheel/types.py` | Python 側で共有する dataclass 型を定義します。 |
| `qav_wheel/__init__.py` | パッケージの公開入口として `main` を再公開します。 |

### MATLAB 構成内訳

| パス | 役割 |
|---------|------|
| `matlab/controllers/hovering_controller_impl.m` | 通常のホバリング制御実装です。ルートの `hovering_controller.m` から呼ばれます。 |
| `matlab/experiments/contact_test_controller_impl.m` | 接触試験シナリオ実行用の controller 実装です。ルートの `contact_test_controller.m` から呼ばれます。 |
| `matlab/experiments/multi_uav_formation_controller_impl.m` | 1 つの simulator から複数 UAV 状態を受け取り、円形フォーメーションを組む実験 controller です。 |
| `matlab/analysis/contact_log_review_impl.m` | 保存済み接触ログの可視化と評価を行う分析実装です。 |
| `matlab/analysis/formation_log_review_impl.m` | 複数 UAV の編隊ログをまとめて読み込み、重心誤差、slot 誤差、RTF、接触傾向を確認します。 |
| `matlab/shared/controller_shared.m` | controller 間で共有する状態受信、制御計算、コマンド送信、起動補助をまとめています。 |
| `matlab/shared/simulation_logger.m` | 状態、制御入力、接触サマリを `.mat` に保存するロガークラスです。 |

</details>

<details>
<summary>通信ポートと実行モード</summary>

単一インスタンスでは、Python 側が `127.0.0.1:5001` へ状態を送信し、MATLAB 側が `127.0.0.1:5000` へ各ロータ推力または各ロータ角速度を返します。

複数インスタンスでは `instance_id = i` に対して次の規則でポートをずらします。

- simulator receive port: `5000 + 2*i`
- simulator state send port: `5001 + 2*i`

たとえば `instance_id = 1` なら、Python は `5002` で制御入力を受け、`5003` へ状態を送信します。MATLAB 側は `hovering_controller('instance_id', 1)` や `contact_test_controller('landing', 'instance_id', 1)` のように同じ `instance_id` を指定してください。

MATLAB controller のローカル UDP ポートが既に使用中だと表示された場合、典型的には別の MATLAB セッションや以前の controller process が同じポートを保持しています。shared controller runtime は、最近の更新で、期待していたポート番号と、Windows では取得できる場合は所有 process 情報も含めて早めに停止するようにしています。通常運用では特別な対応は不要ですが、テストを繰り返す場合は、先に古い controller セッションを完全に閉じるか、別の `instance_id` を使うのが安全です。

`multi_uav_formation_controller` は独立インスタンス方式とは別で、編隊制御の推奨経路です。`simulate --num-uavs N` で 1 つの MuJoCo world に `N` 台の UAV を生成し、状態 packet も制御 packet も配列でまとめて送受信します。

</details>

<details>
<summary>パラメータ管理</summary>

機体やアクチュエータの主要パラメータは `vehicle_params.json` に集約しています。現時点では少なくとも以下が共有化されています。

- 重力とシミュレーション刻み幅
- アーム長
- 反トルク係数
- 最大ロータ推力
- ロータ推力換算係数 `thrust_coefficient`
- 機体初期位置
- 機体ボディと車輪の主要寸法・質量
- 床、壁、機体、車輪の接触設定
- `environment.surface` による平面または関数ベース曲面の指定
- MuJoCo センサ名とセンサ対象ボディ
- `fidelity_mode`, `network_fidelity`, `actuator_dynamics`, `sensor_fidelity`, `logging_config` による baseline / HIL 実験設定

論文向けの運用では、`baseline` と `hil` は単なるオプション差ではなく別モードとして扱うのを推奨します。現在の意味づけ、推奨指標、現実装の境界は [docs/FIDELITY_MODES.ja.md](docs/FIDELITY_MODES.ja.md) を参照してください。

Python 側は `vehicle_params.json` と `qav_wheel.template.xml` から MuJoCo 用 XML を生成して読み込みます。既定では出力先は `build/generated_xml/` で、`instance_id = 0` では `qav_wheel.generated.xml`、それ以外では `qav_wheel.generated.instance_N.xml` を使います。

MATLAB 側は同じ `vehicle_params.json` から、制御に必要な質量、重力、アーム長、反トルク係数、最大推力、推力換算係数に加え、hover/contact 系 controller の既定ゲインも読み込みます。

controller の既定値は `vehicle_params.json` の `controller` に集約しています。現在は少なくとも以下をここから読めます。

- `desired_heading`
- `position_gain`, `velocity_gain`
- `attitude_gain`, `angular_velocity_gain`

編隊制御の既定値は `vehicle_params.json` の `formation` へ集約しています。現在は少なくとも以下をここから読めます。

- `num_uavs`
- `spawn_radius`
- `base_height`
- `centroid_target_xy`
- `formation_radius`
- `centroid_gain`
- `formation_gain`
- `duration_seconds`
- `idle_sleep_seconds`
- `status_display_interval`

編隊ログ review は次のように使えます。

```matlab
formation_log_review
formation_log_review('tracking')
formation_log_review('rtf')
formation_log_review('contacts')
formation_log_review('network')
formation_log_review('overview', 'logs/formation_bundle_20260410_220000.mat')
formation_log_review('overview', 'logs/formation_uav_1_20260410_220000.mat', 'logs/formation_uav_2_20260410_220000.mat', 'logs/formation_uav_3_20260410_220000.mat')
```

編隊実行では、既定で 1 ファイルの `formation_bundle*.mat` だけを残すようになっています。既定の `formation_log_review` は bundle があればそちらを優先して読みます。
bundle と UAV ごとの `.mat` を両方残したい場合は、`multi_uav_formation_controller('formation_log_mode', 'bundle_and_individual')` を使ってください。
bundle の中では、順序付きの `formation_log.logs` に加えて、`formation_log.uavs.uav_1`, `formation_log.uavs.uav_2` のような名前付きフィールドでも各 UAV のログへアクセスできます。

</details>

<details>
<summary>曲面環境</summary>

`vehicle_params.json` の `environment.surface` で、平面または `z = h(x, y)` 型の曲面を指定できます。

日常的な切替は `environment.surface.mode` を変えるのが一番簡単です。

- `"mode": "plane"` または `"mode": "floor"` で床
- `"mode": "slope"` で傾斜面
- `"mode": "paraboloid"`, `"mode": "sinusoidal"`, `"mode": "gaussian"` も同様に切替可能

たとえば床と傾斜面の切替はこの 1 行だけで済みます。

```json
"surface": {
	"mode": "plane",
	...
}
```

または

```json
"surface": {
	"mode": "slope",
	...
}
```

`mode` は簡易トグル用で、詳細形状は従来どおり `type`, `plane`, `height_function`, `parameters` の設定が使われます。

既定では `follow_surface_for_initial_position = true` なので、機体の `initial_position.z` はその地点の地表高さに対する相対高さとして扱われます。曲面や盛り上がった地形に切り替えたときに、初期状態で機体が地面へ埋まるのを防ぐためです。必要なら `false` にして従来どおり絶対座標として扱えます。

接地初期化では、左右車輪の接地条件からロール角を決め、さらに地形の `dh/dx` から初期ピッチ角も入れます。車輪と地表の初期クリアランスは `environment.surface.initial_wheel_contact_clearance` で調整できます。既定値は `0.0001` m です。

`type = "plane"`:

- 従来どおり MuJoCo の plane geom を使います
- 既存の接触試験と互換です

`type = "height_function"`:

- Python 側が `height_function` の設定から地形を生成します
- `flat` と `slope` のように平面で表せる場合は MuJoCo の plane geom に自動変換します
- `paraboloid` や `sinusoidal` のような非平面形状は MuJoCo の hfield として埋め込みます
- 現在対応している関数名は `flat`, `slope`, `paraboloid`, `sinusoidal`, `gaussian` です

代表例:

```json
"surface": {
	"type": "height_function",
	"material": "floor_mat",
	"solref": [0.002, 1.0],
	"contact": {
		"contype": 1,
		"conaffinity": 1
	},
	"height_function": {
		"x_range": [-3.0, 3.0],
		"y_range": [-3.0, 3.0],
		"grid_resolution": [121, 121],
		"name": "slope",
		"parameters": {
			"z_offset": 0.0,
			"slope_x": 0.08,
			"slope_y": 0.0
		}
	}
}
```

将来的に数式文字列そのものを評価する方式ではなく、当面は named function とパラメータ指定にしています。これは安全性と保守性を優先したためです。

`gaussian` はガウス分布状の盛り上がりやくぼみを作るための関数です。代表的なパラメータは次です。

- `amplitude`: 山の高さ。負にするとくぼみになります
- `center_x`, `center_y`: 山の中心位置
- `sigma_x`, `sigma_y`: 山の広がり

</details>

<details>
<summary>固定傾斜ロータと入力モード切替</summary>

モデル側だけで固定傾斜ロータを表現したい場合は、`vehicle_params.json` の `actuation.rotors` を使います。各ロータは body frame での位置と推力軸を持ちます。推力軸は正規化されていない値を書いても読み込み時に正規化されます。

```json
"actuation": {
	"command_mode": "omega",
	"max_rotor_thrust": 20.0,
	"yaw_moment_ratio": 0.02,
	"thrust_coefficient": 2.0e-5,
	"rotors": [
		{
			"name": "fr",
			"position_body": [0.075025, -0.100264, 0.0125],
			"thrust_axis_body": [-0.14834, 0.197905, 0.968912],
			"yaw_moment_ratio": 0.02,
			"spin_sign": 1
		}
	]
}
```

`spin_sign` は反トルクの向きを表します。`1` と `-1` を使ってください。

現在の既定 `vehicle_params.json` では、4 つのロータ推力軸はすべて通常の上向き (`[0, 0, 1]`) に戻しています。

固定傾斜ロータの実装例は [vehicle_params.tilted_rotor_example.json](vehicle_params.tilted_rotor_example.json) に置いてあります。これは前後左右へ対称に約 14.3 度だけ外向きへ傾けた 4 ロータ例です。試すときは、この `actuation.rotors` セクションを `vehicle_params.json` にコピーして `uv run mujoco-wheeled-uav-simulator check-model` で生成結果を確認してください。現時点では controller 側の配分行列は一般化していないため、この例はまず MuJoCo モデル実装と可視化のためのサンプルとして扱ってください。

現在の既定値は `vehicle_params.json` の `actuation.command_mode = "omega"` です。ここを切り替えると、通常制御と接触試験の両方で MATLAB 側の送信形式を変えられます。

```json
"actuation": {
	"command_mode": "omega"
}
```

`command_mode = "thrust"`:

- MATLAB は `rotor_thrusts` を直接送信します
- 既存の動作と互換です

`command_mode = "omega"`:

- MATLAB はコントローラ内部で計算したロータ推力を `rotor_omega = sqrt(T / k_f)` に変換して送信します
- Python は `vehicle_params.json` の `actuation.thrust_coefficient` を使って再び推力へ変換し、MuJoCo へ適用します
- 現段階ではモータ一次遅れや PWM 変換は含めていません。必要になった段階で追加する前提です

</details>

<details>
<summary>MATLAB からの自動起動</summary>

`hovering_controller.m` には、MATLAB から MuJoCo シミュレータを起動するオプションがあります。ただし、実運用では責務分離とデバッグ性の観点から、既定値は無効にしています。

自動起動を有効にする場合は、`matlab/shared/controller_shared.m` の `build_simulator_options` 内で以下を変更してください。

```matlab
'auto_launch', true, ...
```

有効時は以下の順で動作します。

1. MATLAB 側が UDP 受信ポートを確保する
2. `.venv\Scripts\python.exe` があればそれで `drone_sim.py` を起動する
3. `.venv` が無ければ `uv run python drone_sim.py` を試す

</details>

<details>
<summary>ログ保存と解析</summary>

MATLAB 側の `simulation_logger` が、以下を `.mat` に保存します。

- `meta`: 保存時刻や保存理由などのメタデータ
- `config`: 制御ゲイン、配分行列、目標値などの設定
- `state`: 時刻、位置、速度、角速度、姿勢行列
- `control`: 各ロータ推力と、必要に応じて各ロータ角速度
- `reference`: 目標位置
- `contact`: 接触数、接触力サマリ、各時刻の接触詳細

保存先は `logs/` です。`logs/` は生成物なので `.gitignore` に含めています。

保存モードは `hovering_controller.m` の `build_logging_options` で変更できます。

- `finalize`: 終了時に 1 回保存
- `periodic`: 指定秒数ごとに上書き保存
- `periodic_and_finalize`: 定期保存しつつ終了時にも保存

`contact` には全接触のサマリに加えて、`left_wheel`、`right_wheel`、`surface` の接触力サマリも入ります。`contact.details` には各時刻の接触ごとの相手 geom 名、接触位置、貫入距離、接触座標系での力・トルク、法線力が入ります。曲面地形との接触では `surface_contact`、`surface_height`、`surface_normal` も保存されます。現時点では MuJoCo が各ステップで計算した接触力をそのまま保存しており、衝撃インパルスの後処理までは行っていません。

保存した接触ログの確認には `contact_log_review.m` を使えます。

```matlab
contact_log_review
contact_log_review('noncontact')
contact_log_review('landing', 'logs/hover_20260410_120000.mat')
contact_log_review('wall', 'logs/hover_20260410_121000.mat')
contact_log_review('instantaneous', 'logs/hover_20260410_121000.mat')
contact_log_review('impact_compare', 'logs/soft.mat', 'logs/hard.mat')
```

主なモード:

- `overview`: 最新ログの基本確認
- `noncontact`: 非接触基準の確認
- `landing`: 着地時刻や床接触の確認
- `wall`: 壁接触の確認
- `forces`: 合計接触力と左右輪接触力の確認
- `instantaneous`: 瞬時接触力の生時系列とズーム表示
- `impact_compare`: 2 本のログのピーク比較

</details>

<details>
<summary>トラブルシュート</summary>

- `uv` が見つからない場合は、`uv` のインストール後にシェルを開き直してください。
- MuJoCo ウィンドウが表示されない場合は、GUI を利用できるローカル環境で実行しているか確認してください。
- MATLAB 起動時にポート競合が出る場合は、同じ MATLAB セッション内に古い `udpport` が残っていないか確認してください。
- 自動起動を使う場合は、`.venv` または `uv` のどちらかで Python 実行経路が通っている必要があります。

</details>

## 補足

- `vehicle_params.json`、`qav_wheel.template.xml`、`drone_body.stl` はリポジトリ直下に置く前提です。Python パッケージと MATLAB 実装はそこを基準に参照します。
- UDP 通信先はローカルホスト固定です。
- MATLAB 側は起動時に古い `udpport` 残骸を解放するようにしています。