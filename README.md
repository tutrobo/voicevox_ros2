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
