# choreonoid_tactile_sensor_plugin

TactileSensorPlugin

## TactileSensorItem
接触センサのシミュレーション. 値を共有メモリに書き込む.

###parameters
- `shmKey` (int. default 6555): センサ値を書き込む共有メモリのキー.
- `configFileName` (String): センサの配置が書かれたyamlファイル名
```yaml
tactile_sensor:
  -
    name: tactile_sensor0 # センサの名前. 重複禁止
    link: RLEG_ANKLE_R # 親リンク名. (VRMLのjoint名ではなく、URDFのリンク名)
    translation: [ -0.065, -0.065, -0.06 ] # 親リンク相対
    rotation: [ 1, 0, 0, 0 ] # 親リンク相対. Z軸正がfrom another object to this sensorの方向. [axis-x, axis-y, axis-z, angle(rad)]
```
