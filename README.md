# ESP32 Dual Motor Differential Drive Controller (ROS 2 / Pi 5 Ready)

Firmware điều khiển động cơ vi sai hai bánh cho robot di động tự hành (AGV / AMR) sử dụng vi điều khiển ESP32, tích hợp thuật toán điều khiển PI Tracking Anti-Windup, bù ma sát Feedforward, đồng bộ tức thời hai bánh (Instantaneous Cross-Coupling Sync), và chuyển dải thông số động (Dual-Bank Gain Scheduling).

Sẵn sàng giao tiếp chuẩn với **Raspberry Pi 5** qua giao thức Serial (230400 baud) phục vụ cho hệ sinh thái **ROS 2 (Humble / Jazzy) / Nav2**.

---

## 📂 Danh mục tệp nguồn

- [`esp32_dual_motor_firmware.ino`](./esp32_dual_motor_firmware.ino): Mã nguồn chính khởi tạo PWM, ngắt Encoder, IMU Task và vòng lặp điều khiển 50Hz.
- [`dual_v2_core.h`](./dual_v2_core.h): Máy trạng thái điều khiển kép FSM, Gain Scheduling, Watchdog an toàn 480ms.
- [`pi_aw_controller.h`](./pi_aw_controller.h): Bộ điều khiển PI Tracking Anti-Windup chống tích lũy sai số khi bão hòa công suất.
- [`decel_tail_shaper.h`](./decel_tail_shaper.h): Bộ định hình đuôi giảm tốc hạn chế giật (Jerk-limited).
- [`imu_observer.h`](./imu_observer.h): Bộ lọc tư thế thời gian thực ước lượng góc Yaw và Quaternion.
- [`mpu6050_task.h`](./mpu6050_task.h): Task FreeRTOS 100Hz đọc dữ liệu cảm biến MPU6050 qua I2C.
- [`platformio.ini`](./platformio.ini): File cấu hình biên dịch nhanh bằng PlatformIO.
- [`HUONG_DAN_DIEU_KHIEN_VA_TINH_TOAN.md`](./HUONG_DAN_DIEU_KHIEN_VA_TINH_TOAN.md): **Tài liệu hướng dẫn kỹ thuật toàn diện** (Công thức toán động học, ý nghĩa vật lý thông số, sơ đồ nối dây phần cứng, cú pháp giao thức Serial và cấu trúc node ROS 2 trên Pi 5).

---

## 📖 Hướng dẫn chi tiết

Xem chi tiết toàn bộ công thức toán học, giải thích thông số và giao thức tại:
👉 **[HUONG_DAN_DIEU_KHIEN_VA_TINH_TOAN.md](./HUONG_DAN_DIEU_KHIEN_VA_TINH_TOAN.md)**
