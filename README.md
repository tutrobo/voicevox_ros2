# voicevox_ros2

## ビルド・実行

```bash
git clone https://github.com/tutrobo/voicevox_ros2.git
cd voicevox_ros2
rosdep install -yi --from-paths .
colcon build --symlink-install
ros2 run voicevox_ros2 voicevox_ros2
```

適当なワークスペースに入れて、rosdepからのcolcon buildで動きます。

## パラメータ設定

こんなかんじにいじれます。

```yaml
voicevox_ros2_node:
  ros__parameters:
    cpu_num_threads: 1
    load_models: [3, 8]
```
