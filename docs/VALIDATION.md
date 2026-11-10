# Kiểm chứng C8 + A1

## Phạm vi

Chủ đồ án chấp nhận C8+A1 ngày10/11/2026. Bản xuất `DUAL-C8-A1-PACKAGED-20261110` đưa A1 thành boot defaults, cố định HIGH mode2, bật phản hồi IMU quay và bỏ parser các lệnh chuyển động thử nghiệm cũ. Không tinh chỉnh lại phép tính PI/FF, quỹ đạo, đồng bộ, terminal, HIGH hoặc quay C8. Runtime tuning hợp lệ giữ qua các lượt; chỉ reset MCU nạp lại mặc định.

## Kiểm tra offline

| Hạng mục | Kết quả |
|---|---|
| ESP32 build: espressif32@6.12.0 / esp32dev / Arduino | PASS |
| Admission, heartbeat, stale IMU, run/sequence, fixed policy | PASS |
| C8 gốc nạp A1 so với core xuất | 51,200 tick bằng nhau ở các trường điều khiển được kiểm tra |
| Ma trận so sánh | 28 trường hợp thẳng:7 tốc độ×2 chiều×hằng số/đổi tốc;8 quay:5/90/180/360°×2 chiều |
| Test core gồm so C8 | `CONTROL_CORE_PASS cases=77 paired_ticks=51200` |
| Guard pha cuối quay C8 | `C8_TURN_GUARDS_PASS cases=23` |

Encoder/IMU native là dữ liệu tổng hợp để so hành vi mã, không phải log xe hoặc mô hình động lực học đã hiệu chuẩn. So bitwise áp dụng trường liệt kê trong test, không chứng minh mọi trạng thái có thể xảy ra. Reference C8 gốc giữ trong hồ sơ đóng băng riêng, không nhân đôi vào gói.

Chạy test public với GCC/Clang C++17, không kết nối phần cứng:

```sh
g++ -std=c++17 -O2 -UNDEBUG tests/test_control_core.cpp -o test_control_core
./test_control_core
g++ -std=c++17 -O2 -UNDEBUG tests/test_turn_finish.cpp -o test_turn_finish
./test_turn_finish
```

Windows dùng `.exe`. Test core độc lập báo `cases=41 paired_ticks=0`; so reference chỉ bật khi cấp `C8_REFERENCE_HEADER` trỏ đến C8 gốc đã flatten include.

## Bằng chứng vận hành trước xuất bản

C8 với A1 đã thử nhiều chuỗi0.3/0.4 m/s, hai đợt14 chặng0.1–0.6 và một đợt10 chặng0.2–0.5 m/s. Người vận hành xác nhận gần trùng home/hướng và không vướng ở các đợt xác nhận tương ứng. Đây là chấp nhận vận hành, không tuyên bố mọi KPI đều đạt.

Đợt10 chặng gần nhất: sai số quãng đường trung bình encoder lớn nhất khoảng0.665 mm, biến thiên IMU lớn nhất đã ghi khoảng0.724°, yaw cuối so đầu−0.158°. Đây là ước lượng encoder/IMU, không phải phép đo đường/góc độc lập bằng thước hoặc định vị ngoài. Bám RPM còn biến thiên; các lượt đủ cửa sổ đánh giá có RMSE từng bánh khoảng0.322–2.877 RPM; một số lượt nhanh thiếu đoạn ổn định.

Lệnh0.6 m/s trên1.2 m chỉ đạt đỉnh khoảng0.50–0.52 trong các đợt gần đây. Khả năng gần0.6 từng quan sát ở phiên bản/lượt dài khác không được gán thành bằng chứng A1 đạt0.6 ổn định trên đoạn ngắn.

**Bản xuất với boot defaults/chính sách serial mới chưa nạp hoặc chạy xe lại trong đợt này.** Offline không thay thế thử phần cứng sau lần nạp tương lai. Firmware/service đang vận hành, gain RAM và baseline gốc được giữ nguyên khi đóng gói.
