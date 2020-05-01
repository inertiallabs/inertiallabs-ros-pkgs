# inertiallabs_msgs

  ins_data

  ```
  std_msgs/Header           header
  geometry_msgs/Vector3     YPR
  float32[4]                quat_data

  ```

  gps_data

  ```
  std_msgs/Header           header
  float64                   Latitude
  float64                   Longitude
  float64                   Altitude
  float32                   East_Speed
  float32                   North_Speed
  float32                   Vertical_Speed

  ```
  sensor_data

  ```
  std_msgs/Header           header
  geometry_msgs/Vector3     Mag
  geometry_msgs/Vector3     Accel
  geometry_msgs/Vector3     Gyro
  float32                   Temp
  float32                   Vinp
  float32                   Pressure
  float32                   Barometric_Height

  ```
  imu_data 

  ```
  std_msgs/Header           header
  geometry_msgs/Vector3     YPR
  geometry_msgs/Vector3     Mag
  geometry_msgs/Vector3     Accel
  geometry_msgs/Vector3     Gyro
  float32                   Temp
  float32                   Vinp
  float32                   Pressure

  ```
