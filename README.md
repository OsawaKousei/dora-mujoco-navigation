# dora + MuJoCo navigation

dora-rs でロボティクスを実装するときの構成や開発体験を、作者が確かめるための PoC です。MuJoCo 上の差動二輪ロボットを対象とし、実機での使用は想定していません。

自律走行フローでは、シミュレーターに加えて LiDAR・オドメトリを独立した dora ノードとして定義しています。推定、経路計画、制御、軌跡描画も各ノードで処理します。テレオペ用のフローもあります。

## 実行

Linux x86_64、Rust、`dora` CLI が必要です。初回起動時は MuJoCo 3.12.0 のライブラリを取得し、SHA-256 を検証して `target/` に置きます。既にインストール済みなら `MUJOCO_312_LIB_DIR` でライブラリのディレクトリを指定できます。

```sh
./scripts/run-autonomous.sh
./scripts/run-teleop.sh
```

画面なしで自律走行を試す場合:

```sh
MUJOCO_HEADLESS=1 ./scripts/run-autonomous.sh --stop-after 45s
```

開始位置、目標、障害物、速度は [configs/autonomous.toml](configs/autonomous.toml) で変更できます。自律走行中の計画経路と実走行軌跡は `out/autonomous-path.png` に描画されます。保存先は `TRAJECTORY_PLOT_PATH` で変更できます。テレオペは既定で `gnome-terminal` にキー入力画面を開きます。別の端末を使う場合は `TELEOP_TERMINAL` に `-e <command>` を受け付ける実行ファイルを指定してください。

## テスト

```sh
./scripts/test-all.sh
```

単体テスト、MuJoCo の物理テスト、テレオペと自律走行の E2E テストを実行します。

## ライセンス

[Apache License 2.0](LICENSE)。依存ライブラリにはそれぞれのライセンスが適用されます。
