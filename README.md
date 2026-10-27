# Dora + MuJoCo ナビゲーション

## テレオペ

次のコマンドで、必要な実行ファイルをビルドして dora のフローを起動します。

```sh
./scripts/run-teleop.sh
```

`simulator` が MuJoCo の画面を開き、`keyboard` がキー入力用の端末を別に開きます。
キー入力用端末にフォーカスを置き、MuJoCo の画面を見ながら操作してください。
十字キーで前進・後退・左旋回・右旋回を選択し、Space で停止、Q で入力用端末を閉じます。
フロー全体を終了するときは `dora run` 側の端末で Ctrl+C を押します。

`mujoco-rs` 6.1.0 は MuJoCo 3.12.0 を要求します。起動スクリプトは初回実行時に
Linux x86_64 用の対応ライブラリを `target/` にダウンロードし、SHA-256 を検証します。
既に MuJoCo 3.12 がある場合は、その `lib` ディレクトリを `MUJOCO_312_LIB_DIR` に指定できます。

この環境では `keyboard` が既定で `gnome-terminal` を起動します。ほかの端末を使う場合は、
`-e <command>` を受け付ける実行ファイルを `TELEOP_TERMINAL` に指定してください。
端末内の `keyboard_tty` は Unix ソケットで `keyboard` にキーを渡す補助プロセスで、
dora ノードではありません。

| 出力 | Arrow データ | 内容 |
| --- | --- | --- |
| `keyboard/key_event` | `u8` × 1 | 1: 前進、2: 後退、3: 左、4: 右、5: 停止、6: 終了 |
| `teleop/cmd_vel` | `f64` × 2 | `[並進速度 m/s, 角速度 rad/s]` |
| `simulator/robot_pose` | `f64` × 3 | `[ワールド座標 x m, y m, yaw rad]` |

MuJoCo の物理刻みは 5 ms で、100 Hz の dora タイマー１回につき２ステップ進めます。
描画は２回のタイマーイベントにつき１回です。300 ms を超えて `cmd_vel` が届かなければ
車輪を停止します。キー入力端末が切断された場合も停止イベントを送ります。
速度アクチュエータが安定するよう、モデルには `implicitfast` 積分を設定しています。

`dataflow.yml` は、独立した `lidar`・`odometry` ノードを含む自律走行向けの設計です。
ノードの責務とメッセージ契約は [`docs/autonomous-dataflow.md`](docs/autonomous-dataflow.md) に記載しています。
自律走行は `./scripts/run-autonomous.sh` で起動します。画面を使わない場合は
`MUJOCO_HEADLESS=1 ./scripts/run-autonomous.sh --stop-after 45s` とします。
既定のマップは上下から交互に張り出す２つの障害物を回避する構成です。
開始位置・目標・障害物・速度は `configs/autonomous.toml` で設定できます。
テレオペには `dataflow.teleop.yml` を使用します。

## テスト

Linux で `dora` CLI が利用できる場合、次のコマンドで単体テスト・
MuJoCo の物理テスト、PTY による `keyboard_tty` の結合テスト、Dora の E2E テストを
この順に実行します。

```sh
./scripts/test-all.sh
```

テレオペの E2E テストは `keyboard → teleop → simulator` を実際の Dora フローで起動し、
仮想端末から前進と停止を入力して `robot_pose` の移動と停止を検証します。
自律走行の E2E テストは `simulator → odometry/lidar → localization → controller` と
`planner` を起動し、衝突せずに目標へ到達して停止することを検証します。
シミュレーターには `MUJOCO_HEADLESS=1` を設定するため、画面は不要です。
E2E テストは外部コマンドとローカル IPC を使うため、通常の `cargo test` では
無視され、上記スクリプトで最後に明示的に実行されます。
