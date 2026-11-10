# ESP32 dual motor firmware — C8 + A1

Firmware robot vi sai hai bánh, đóng gói trong thư mục **`esp32_dual_motor_firmware`**. Giữ thuật toán C8 và bốn bộ thông số A1 được chủ đồ án chấp nhận ngày 10/11/2026, đưa cấu hình đã chốt thành mặc định khi khởi động.

**HIGH cố định chế độ 2: giới hạn gia tốc + bù hướng IMU. Quay tại chỗ dùng phản hồi IMU và pha kết thúc C8 mặc định.** LOW đi thẳng vẫn dùng PI + encoder, không thêm vòng giữ hướng IMU cho LOW.

Build ID: `DUAL-C8-A1-PACKAGED-20261110`. Bản xuất đã build/kiểm thử offline, chưa nạp hoặc chạy lại trên xe trong đợt đóng gói. Xem [phạm vi kiểm chứng](docs/VALIDATION.md).

## Cấu trúc

```text
esp32_dual_motor_firmware/
├── esp32_dual_motor_firmware.ino      # PWM, ngắt encoder, serial, loop
├── firmware_config.h                 # Bốn bộ A1 và giới hạn mặc định
├── dual_v2_core.h                    # Quỹ đạo, đồng bộ, HIGH, quay, FSM
├── pi_aw_controller.h                # PI + FF + tracking anti-windup
├── decel_tail_shaper.h               # Tạo dạng đuôi giảm tốc
├── imu_observer.h                    # Bias gyro, quaternion/yaw
├── mpu6050_task.h                    # Đọc IMU I²C 100 Hz
├── platformio.ini
├── config/accepted_a1.json           # Snapshot cấu hình đã chấp nhận
├── docs/VALIDATION.md
├── tests/                           # Kiểm thử C++ offline
└── HUONG_DAN_DIEU_KHIEN_VA_TINH_TOAN.md
```

Root repository chính là nội dung thư mục trên. Không kèm service, web UI, Python, môi trường ảo hay binary build.

## Thông số mặc định

| Bank / motor | Kp | Ki | Kaw | FF slope (PWM/RPM) | FF offset (PWM) | Ramp (RPM/s) | Jerk (RPM/s²) |
|---|---:|---:|---:|---:|---:|---:|---:|
| LOW M1 | 3.2400 | 1.2000 | 0.5000 | 3.1300 | 45.9000 | 320 | 1500 |
| LOW M2 | 3.3300 | 0.4000 | 0.5000 | 3.2473 | 58.1475 | 320 | 1500 |
| HIGH M1 | 4.3500 | 1.2000 | 0.8000 | 3.0300 | 46.0000 | 320 | 1500 |
| HIGH M2 | 4.4400 | 0.4000 | 0.8000 | 3.2473 | 58.1475 | 320 | 1500 |

| Cấu hình | Giá trị |
|---|---|
| Trần RPM / PWM | 114.592 / 470 (PWM 10 bit) |
| Slew PWM | 27 count mỗi chu kỳ 20 ms |
| Ksync / trim tối đa / divergence | 720 RPM/m / 18 RPM / 0.040 m |
| Đường kính bánh / khoảng cách bánh | 0.100 m / 0.400 m |
| Encoder | 998.4 count/vòng bánh theo bộ giải mã hiện tại |
| LOW / nội suy / HIGH | ≤0.30 / 0.30–0.40 / ≥0.40 m/s theo tốc độ **yêu cầu** |
| Chuyển bank khi cập nhật tốc độ | 150 ms cho trọng số từ 0 đến 1 |
| HIGH: a_max, k_yaw, k_rate | 0.50 m/s²; 1.00 s⁻¹; 0.15 |
| HIGH: max_omega, slew_omega | 0.15 rad/s; 0.75 rad/s² |
| Điều khiển / serial / PWM | 50 Hz / 230400 baud / 20 kHz |

Thông số nằm trong [`firmware_config.h`](firmware_config.h). TUNE/CONFIG vẫn thay đổi được RAM lúc dừng; reset MCU nạp lại mặc định A1, không tự lưu NVS.

## Build

Với PlatformIO Core đã cài:

```powershell
git clone https://github.com/levantohe123-ctrl/firmwave_esp32.git esp32_dual_motor_firmware
cd esp32_dual_motor_firmware
pio run -e esp32dev
```

Ghim `espressif32@6.12.0` (Arduino ESP32 2.0.17 / ESP-IDF 4.4.7). Build không tự upload. Arduino IDE cần mở sketch trong thư mục tên `esp32_dual_motor_firmware` với core tương thích. Arduino ESP32 3.x có API LEDC khác, chưa được kiểm chứng ở đây.

## Khởi động

1. Sau reset: PWM=0, driver disable, IDLE. Có cấu hình sẵn không làm xe tự chạy.
2. Host mở serial230400, đọc boot ID, STATUS và các GET để xác minh cấu hình. Chỉ một host sở hữu serial.
3. Để xe đứng yên, đợi IMU warm-up ít nhất1 s rồi gửi `IMU_CALIBRATE`. Chờ calibrated/valid và mẫu mới. Hiệu chỉnh cần≥300 mẫu và≥2.9 s, có thể thất bại khi rung/di chuyển.
4. Host gửi HEARTBEAT khoảng100 ms/lần trong lúc chạy, đồng thời đọc ACK/telemetry. Watchdog480 ms; không dùng thao tác gõ tay làm nguồn heartbeat.
5. Gửi lệnh có run ID, chờ hoàn tất và xác nhận đứng yên trước chặng tiếp theo. Chọn đường trống và thời hạn phù hợp với tăng/giảm tốc.

Lệnh chuyển động: `MOVE_DIST`, `SPEED_UPDATE`, `TURN_TIME`. `HIGH_CTRL_SET` chỉ chấp nhận mode2; `TURN_CTRL_SET` chỉ chấp nhận1. Bỏ lệnh serial thử nghiệm `MOVE`, `MOVE_TIME`, `SPEED`, `TURN` để không đi vòng qua đường điều khiển đã chốt. Không giả định service cũ tương thích nếu còn gửi mode0/1 hoặc tắt turn feedback; service không nằm trong gói này.

`STOP` hoặc ký tự `!` ngắt đầu ra theo phần mềm: disable/coast, không phải cam kết phanh chủ động. HIGH và quay cần IMU hợp lệ; firmware kiểm tra điều kiện khi bắt đầu và xử lý dừng khi mất IMU.

## Tài liệu

- [Đấu nối, thuật toán, đơn vị, ví dụ và serial](HUONG_DAN_DIEU_KHIEN_VA_TINH_TOAN.md).
- [Build, test và giới hạn bằng chứng](docs/VALIDATION.md).
- Giao thức hiện tại là serial riêng, chưa có micro-ROS, node ROS2, `cmd_vel` tổng quát hoặc Nav2 bridge.
- Chấp nhận giai đoạn dual motor không có nghĩa mọi lượt đạt mọi KPI RPM. Đoạn1.2 m có thể không đủ tăng/giảm tốc để đạt0.6 m/s.
