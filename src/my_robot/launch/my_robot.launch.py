controller_manager:
  ros__parameters:
    update_rate: 100  # Hz

    # Параметры для контроллера дифференциального привода
    diff_drive_controller:
      type: diff_drive_controller/DiffDriveController

      left_wheel_names: ["l_w_wheel_joint"]
      right_wheel_names: ["r_w_wheel_joint"]

      wheel_separation: 0.2  # Расстояние между колесами (примерное)
      wheel_radius: 0.05     # Радиус колеса (примерное)

      wheel_separation_multiplier: 1.0
      left_wheel_radius_multiplier: 1.0
      right_wheel_radius_multiplier: 1.0

      odom_frame_id: odom
      base_frame_id: base_link
      pose_covariance_diagonal: [0.001, 0.001, 0.001, 0.001, 0.001, 0.03]
      twist_covariance_diagonal: [0.001, 0.001, 0.001, 0.001, 0.001, 0.03]

      open_loop: true
      enable_odom_tf: true

      cmd_vel_timeout: 0.5
      publish_limited_velocity: true
      use_stamped_vel: false

      velocity_rolling_window_size: 10

      # Ограничения
      linear:
        x:
          has_velocity_limits: true
          max_velocity: 1.0  # м/с
          has_acceleration_limits: true
          max_acceleration: 0.5  # м/с²
      angular:
        z:
          has_velocity_limits: true
          max_velocity: 1.0  # рад/с
          has_acceleration_limits: true
          max_acceleration: 0.5  # рад/с²

    # Контроллер для рулевых механизмов
    steering_controller:
      type: position_controllers/JointGroupPositionController
      joints:
        - f_r_w_steering_joint
        - f_l_w_steering_joint

    # Контроллер состояния суставов
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster