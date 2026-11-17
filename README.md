# ESP32 dual motor firmware — C8 + A1 / Dashboard R4

Dashboard **R4 STATUS** tự hiển thị pin và trạng thái khi mở trang trên Wi-Fi robot. Nút **Kết nối robot / Ngắt kết nối** chỉ quản lý quyền điều khiển; ngắt điều khiển vẫn xem được trạng thái. Thanh pin chỉ giữ phần trăm và điện áp.

Remote đa chạm hỗ trợ vừa tiến/lùi vừa rẽ, hai bố cục cụm giữa/hai bên, WebSocket liên tục và tự kết nối lại. Phần Đặt lệnh có quãng đường, tốc độ, góc quay và hiệu chỉnh IMU 300 mẫu. Remote không yêu cầu IMU; hành trình dùng chính sách IMU của C8. Không có bảng chỉnh gain hoặc lịch sử phiên trên dashboard.

Điện thoại kết nối **DUAL-ROBOT**, mở **http://192.168.4.1**. ESP32 tự phát Wi-Fi khi được cấp nguồn phù hợp; không cần host service hoặc Internet. Xem [hướng dẫn sử dụng](docs/WIFI_DASHBOARD.md), [thay đổi R4](docs/WIFI_R4_STATUS.md) và [lộ trình ROS2](docs/ROS2_ROADMAP.md).

Build Wi-Fi: `DUAL-C8-A1-WIFI-20261117-R4-STATUS`. `pio run` mặc định build `esp32_wifi`; `pio run -e esp32dev` build bản USB. Gói R4 có 57 kiểm tra trình duyệt đạt; chi tiết và loại bằng chứng ở [hồ sơ R4](docs/wifi-validation/r4-status-verification.json).

Firmware robot vi sai hai bánh, đóng gói trong thư mục **`esp32_dual_motor_firmware`**. Giữ thuật toán C8 và bốn bộ thông số A1 được chủ đồ án chấp nhận ngày 10/11/2026, đưa cấu hình đã chốt thành mặc định khi khởi động.

**HIGH cố định chế độ 2: giới hạn gia tốc + bù hướng IMU. Quay tại chỗ dùng phản hồi IMU và pha kết thúc C8 mặc định.** LOW đi thẳng vẫn dùng PI + encoder, không thêm vòng giữ hướng IMU cho LOW.

Build ID lõi/USB: `DUAL-C8-A1-PACKAGED-20261110`. Xem [phạm vi kiểm chứng C8+A1](docs/VALIDATION.md).

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
├── manual_drive.h                    # Trộn ga/rẽ và vòng tốc độ remote
├── wifi_control.h                    # Quyền điều khiển, lệnh và watchdog
├── wifi_dashboard.h                  # HTTP/WebSocket trên ESP32
├── wifi_config.h                     # AP và build ID Wi-Fi
├── battery_estimator.h               # Bảng OCV, lọc và giữ % dưới tải
├── battery_task.h                    # Đo GPIO32 / ADC1
├── dashboard/                        # HTML/CSS/JS, logo và font cục bộ
├── tools/embed_dashboard.py          # Đóng gói assets vào flash khi build
├── platformio.ini
├── config/accepted_a1.json           # Snapshot cấu hình đã chấp nhận
├── docs/VALIDATION.md
├── tests/                           # Kiểm thử C++ offline
└── HUONG_DAN_DIEU_KHIEN_VA_TINH_TOAN.md
```

Root repository chính là nội dung thư mục trên. Không kèm host service, môi trường ảo, log thiết bị hoặc binary sao lưu. Công cụ Python đóng gói assets khi build; không cần Python/máy tính khi vận hành.

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
pio run -e esp32_wifi
# Tùy chọn: bản serial/USB
pio run -e esp32dev
```

Ghim `espressif32@6.12.0` (Arduino ESP32 2.0.17 / ESP-IDF 4.4.7). Build không tự upload. Arduino IDE cần mở sketch trong thư mục tên `esp32_dual_motor_firmware` với core tương thích. Arduino ESP32 3.x có API LEDC khác, chưa được kiểm chứng ở đây.

## Khởi động qua Wi-Fi

1. Cấp nguồn đúng cho ESP32 qua bộ hạ áp; không cấp pin 24 V trực tiếp vào chân nguồn 3,3 V/5 V.
2. Kết nối điện thoại vào `DUAL-ROBOT`, mật khẩu mặc định `DualRobot-2026`, mở `http://192.168.4.1`. Đổi cấu hình AP trong `wifi_config.h` trước khi dùng ngoài môi trường đồ án.
3. Trạng thái và pin tự cập nhật, không chiếm quyền điều khiển. Phần trăm xuất hiện khi đủ mẫu điện áp nghỉ.
4. Bấm Kết nối robot để dùng remote hoặc hiệu chỉnh IMU và Đặt lệnh. Ngắt điều khiển gửi STOP nhưng vẫn theo dõi trạng thái.
5. ESP32 tự đo pin khi dashboard đóng; điện thoại hiển thị trực tiếp khi còn kết nối Wi-Fi và trang đang hoạt động.

## Giao tiếp serial của bản USB

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
- [Dashboard Wi-Fi R4](docs/WIFI_DASHBOARD.md).
- [Đo pin và tài nguyên giao diện](docs/BATTERY_AND_BRANDING.md).
- [Các bước bổ sung khi chuyển sang ROS2](docs/ROS2_ROADMAP.md).
- Hiện có serial riêng và dashboard HTTP/WebSocket; chưa có node ROS2, odometry liên tục `(x,y,phi)`, EKF hoặc cầu nối `cmd_vel` tổng quát. Lộ trình mô tả các phần cần bổ sung.
- Chấp nhận giai đoạn dual motor không có nghĩa mọi lượt đạt mọi KPI RPM. Đoạn1.2 m có thể không đủ tăng/giảm tốc để đạt0.6 m/s.
