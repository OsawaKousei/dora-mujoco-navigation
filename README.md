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
描画は２回のタイマーイベントにつき１回です。300 ms 以上 `cmd_vel` が届かなければ
車輪を停止します。キー入力端末が切断された場合も停止イベントを送ります。
速度アクチュエータが安定するよう、モデルには `implicitfast` 積分を設定しています。

既存の `dataflow.yml` は、未実装の `controller`・`odometry`・`lidar` を含む
自律走行向けの構成案です。現時点のテレオペには `dataflow.teleop.yml` を使用します。
