# BACKLOG

このファイルでは、README とは分けて、今後実装したい機能や検討事項を管理します。

- `Completed`: いったん実装済みで、現状の目的は満たしている項目
- `Next`: 近いうちに実装したい項目
- `Investigate`: 実装前に設計や検証が必要な項目
- `Later`: 優先度は高くないが残しておきたい拡張
- `Decisions`: 現時点の運用方針

## Completed

- XML と MATLAB / Python の物理パラメータを `vehicle_params.json` ベースで共有できるようにした
- 接触力ログを `.mat` に保存できるようにした
- 合計接触力に加えて左右車輪ごとの接触力も保存できるようにした
- 各ロータ推力入力とロータ角速度入力をオプションで切り替えられるようにした
- ロータ角速度入力モードで通常制御が問題なく動くことを確認した
- `environment.surface` で平面と関数ベース曲面を切り替えられる土台を入れた
- 関数ベース曲面を MuJoCo の hfield で扱えるようにし、その接触力もログへ載せられるようにした
- 床荷重確認用の `ground_load` シナリオを追加した
- 壁押し付け確認用の `wall_load` シナリオを追加した
- 接触ログ確認用の `contact_log_review` を追加した
- 接触テスト実装を `experiments/`、解析実装を `analysis/` に整理した
- realtime factor を Python で計算し、MATLAB 表示とログ保存へ載せるようにした
- 1 つの MuJoCo world に複数 UAV を生成できるようにした
- 複数 UAV の状態・制御 packet をバッチ化し、単一の MATLAB controller で編隊制御できるようにした
- 編隊制御用の `multi_uav_formation_controller` と編隊ログ確認用の `formation_log_review` を追加した
- 編隊制御の既定値を `vehicle_params.json` の `formation` セクションへ外出しした
- 曲面上の初期姿勢を車輪接地条件と地形勾配から決めるようにし、初期接地クリアランスも設定化した
- hfield の `elevation` シリアライズ時に極端な指数表記を丸め、ガウス地形の XML 読み込み失敗を防ぐようにした
- localhost 前提の UDP endpoint 設定を整理し、シミュレータ bind IP と state 送信先 IP を外部指定できるようにした
- MATLAB なしでも hover の往復試験ができる Python controller CLI を追加した
- packet v2 metadata を state / command packet に追加し、sequence・wall-clock age・source state binding を追跡できるようにした
- simulator 側に stale-command handling と HIL 向け network delay / jitter / packet loss 注入を追加した
- MATLAB `.mat` ログに packet age・sequence・controller compute time などの network metadata を保存できるようにした
- `baseline` / `hil` fidelity mode の運用方針を docs 化した
- simulator 本体に actuator dynamics の基本モデルと sensor fidelity の基本モデルを入れ、ログへ actuator / sensor truth を保存できるようにした
- Jetson 向けに「低 RTF を許容する時刻忠実評価」と「RTF≈1 を狙う計算資源評価」を分けた workflow と review 入口を追加した
- 編隊実行時に、1 つの `formation_bundle*.mat` へまとめたログを自動生成できるようにした
- 編隊ログで `bundle_only` 保存モードと `formation_log.uavs.uav_N` 形式の名前付きアクセスを追加した

## Next

- 曲面用の接触試験シナリオと可視化モードを追加する
- ポート番号、自動起動、ログ保存モードなどの実行設定を外部から切り替えやすくする
- 論文向けに使いやすい図を出力する MATLAB スクリプトを追加する
- 接触ログから図や CSV をまとめて書き出す処理を追加する
- fidelity mode ごとの比較バッチと図表再生成フローを整える
- Jetson の時刻忠実評価と計算資源評価それぞれについて、判定基準を固定した比較テンプレートを整える

## Investigate

- リモート UDP 実行時に、bind 先 IP、送信先 IP、NAT や firewall の前提、packet loss 時の安全側挙動をどこまで標準対応に含めるか整理する
- ロータを傾けた UAV に対応するために、現在の配分行列と MuJoCo アクチュエータ設定をどう一般化するか検討する
- ロータ角速度入力をベースに、必要になった時点でモータ一次遅れや PWM ベース入力をどこまで含めるか整理する
- 曲面上の接地初期化を、現在の車輪 2 点接地ベースから、より一般の接地点最適化へ広げるべきか検討する

## Later

- 曲面上での離着陸や接地を含むシナリオを追加する
- 傾斜ロータ機や特殊配置ロータに対しても同じ制御コードを再利用できる構成へ整理する
- Jetson 配備を軽くするため、controller 専用 dependency profile か standalone controller package を検討する
- シミュレータ起動設定や機体パラメータを設定ファイル化する

## Decisions

- README には現在の使い方と現状の仕様だけを書く
- 今後の機能追加や検討事項はこの `BACKLOG.md` で管理する
- MuJoCo の自動起動は便利機能として残すが、既定値は無効にする
- 生成されるログファイルは `logs/` に保存し、Git では追跡しない
- 曲面の接触形状は mesh ではなく hfield を主系として扱う
- 複数 UAV の主系は単一 world / 単一 GUI / バッチ state-control とする
- リモート controller 対応を進める場合も、まずは UDP を維持し、MATLAB 依存を増やすのではなく Python controller を第一候補にする