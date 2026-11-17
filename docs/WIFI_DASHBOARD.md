# Dashboard Wi-Fi R4

## Kết nối và theo dõi

ESP32 phát AP `DUAL-ROBOT` khi khởi động. Mật khẩu mặc định `DualRobot-2026`, dashboard `http://192.168.4.1`. Chọn giữ kết nối nếu điện thoại báo mạng không có Internet. Giao diện nằm trong flash, không dùng CDN.

Trang tự nhận trạng thái và pin bằng `GET /api/state`, không cần bấm Kết nối robot. Nút Kết nối robot mở WebSocket `/ws` và xin quyền điều khiển; chỉ một socket điều khiển được hoạt động. Khi socket đang mở, trang dùng trạng thái đẩy mỗi 100 ms thay cho GET. Ngắt kết nối gửi STOP rồi đóng socket; trang tiếp tục theo dõi bằng GET. Một trang chỉ theo dõi không chiếm quyền của thiết bị đang điều khiển.

GET cách lần hoàn thành trước 500 ms, chỉ một yêu cầu cùng lúc, timeout 1200 ms. Dữ liệu quá 1500 ms không được hiển thị như trực tiếp. Trang tự thử lại khi mạng hoặc nguồn robot phục hồi, không phát lại lệnh chuyển động cũ. Khi trang bị ẩn/điện thoại khóa màn hình, trình duyệt có thể hạn chế cập nhật; mở lại trang sẽ tiếp tục đọc. Tác vụ đo pin trên ESP32 độc lập với trang.

## Remote

- Giữ tiến/lùi và thêm trái/phải để rẽ. Thả nút rẽ để tiếp tục theo ga còn giữ; thả hết để ngắt đầu ra.
- Chỉ nhấn trái/phải để quay tại chỗ. Remote không cần hiệu chỉnh IMU.
- Hai bố cục: bốn nút chung một cụm hoặc tiến/lùi bên trái, trái/phải bên phải. Lựa chọn được lưu trên thiết bị.
- Hỗ trợ đa chạm, trượt ngón giữa nút và tổ hợp phím mũi tên. Phím trong ô nhập không điều khiển motor.
- Thanh tốc độ thay đổi được khi đang giữ nút; tốc độ bánh giới hạn 0,6 m/s. Đảo chiều được xử lý qua giảm tốc về 0 và chờ bánh đủ chậm trước đổi dấu.
- Nút DỪNG NGAY riêng và phím Esc; không có nút dừng giữa cụm remote.

Remote dùng bộ trộn tốc độ bánh và PI/FF/ramp/anti-windup với gain A1. Không dùng vòng đồng bộ vị trí hai bánh chống lại thao tác rẽ. Remote không phải bộ bám quỹ đạo hoặc odometry toàn cục.

## Đặt lệnh và IMU

Đi thẳng: tiến/lùi, quãng đường 0,05–5 m, tốc độ 0,10–0,60 m/s. Quay tại chỗ: trái/phải, góc 5–360°, thời gian 2–60 s. C8 có thể từ chối yêu cầu không khả thi. Đoạn ngắn có thể không đủ tăng tốc để đạt tốc độ đặt trước khi giảm tốc.

Để xe đứng yên, bấm Hiệu chỉnh IMU và chờ đủ 300 mẫu cùng xác nhận hoàn tất. Quy trình có warm-up, kiểm tra tuổi mẫu và ổn định cảm biến. HIGH giữ tăng tốc + bù hướng, quay giữ phản hồi IMU và pha cuối C8. LOW đi thẳng giữ PI + encoder.

## Pin

Thanh pin chỉ hiện biểu tượng, phần trăm và điện áp. Giá trị không hợp lệ/mất dữ liệu hiện `—%`, `— V`. Cảnh báo pin thấp riêng vẫn hoạt động. Phần trăm dùng bảng điện áp Li-ion 6S; không tự coi 24,25 V là 100%, chưa có cảm biến dòng hoặc phát hiện cắm sạc. Xem [tài liệu pin](BATTERY_AND_BRANDING.md).

## Liên lạc và dừng

Quyền điều khiển không hết hạn chỉ vì đứng yên. Trong lúc chạy, thiếu nhịp điều khiển mới hơn 400 ms sẽ ngắt đầu ra; không xóa kết nối chỉ vì ngắt đầu ra. Watchdog C8 480 ms, timing, stall, polarity và giới hạn PWM vẫn áp dụng. Trình duyệt xóa nút đang giữ khi mất phản hồi/socket gián đoạn. Sau kết nối lại cần nhấn lại, không tự chạy lệnh cũ.

Driver dừng theo disable/coast, không bảo đảm phanh tức thì. Chọn không gian chạy và dùng ngắt nguồn vật lý khi cần. Trạng thái hiển thị không thay quan sát trực tiếp xe.

## Build và kiểm tra

```sh
pio run -e esp32_wifi
```

PlatformIO gọi `tools/embed_dashboard.py` nén HTML/CSS/JS, logo và font vào `dashboard_assets.h`; không upload filesystem riêng. `dashboard/preview.html` tự chứa, không liên lạc phần cứng; không dùng preview làm bằng chứng xe chạy.

Test trình duyệt cần Node.js, Playwright và Chromium tương thích. Chạy `node tests/dashboard.spec.cjs`; có thể đặt `PLAYWRIGHT_MODULE`, `PLAYWRIGHT_UTILS`, `BROWSER_EXE` để dùng gói/trình duyệt đã cài. Test tự dựng mock HTTP/WebSocket trên loopback rồi đóng sau khi hoàn thành.

[Thay đổi R4](WIFI_R4_STATUS.md) · [Lộ trình ROS2](ROS2_ROADMAP.md).
