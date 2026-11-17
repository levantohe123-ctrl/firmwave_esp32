# R4 — trạng thái tự cập nhật, kết nối chỉ dành cho điều khiển

Build `DUAL-C8-A1-WIFI-20261117-R4-STATUS`: tự theo dõi trạng thái độc lập với kết nối điều khiển; 57 kiểm tra trình duyệt đạt.

## Hành vi

- Điện thoại vào Wi-Fi DUAL-ROBOT và mở dashboard: trang tự đọc `GET /api/state`, không cần bấm Kết nối robot.
- Pin, điện áp, tốc độ, quãng đường và hướng nhận dữ liệu độc lập với quyền điều khiển. Phần trăm vẫn tuân theo thời gian chờ điện áp nghỉ của firmware; không bịa giá trị ngay khi vừa bật nguồn.
- Bấm Kết nối robot mới mở WebSocket điều khiển. Khi socket điều khiển hoạt động, dùng dữ liệu đẩy sẵn có và dừng luồng GET để tránh tải thừa.
- Bấm Ngắt kết nối gửi STOP, đóng socket điều khiển, vô hiệu hóa nút chạy/hiệu chỉnh và tiếp tục đọc trạng thái tự động.
- Trang quan sát trên thiết bị khác không chiếm quyền điều khiển.
- Luồng GET chỉ có một yêu cầu tại một thời điểm, chờ 500 ms sau mỗi lần hoàn thành, timeout 1200 ms, không dùng cache. Phản hồi muộn bị bỏ sau chuyển luồng hoặc ẩn trang. Chỉ số cũ quá 1500 ms không được coi là trực tiếp.
- Trang tự phục hồi khi mạng hoặc nguồn robot trở lại. Luồng đọc trạng thái không tự mở quyền điều khiển và không phát lại lệnh.
- Trình duyệt có thể hạn chế tác vụ khi trang bị ẩn/điện thoại khóa màn hình. Trang tạm dừng GET lúc ẩn và đọc ngay khi hiển thị lại. ESP32 vẫn tự đo pin trong nền.

## Giao diện pin

Bỏ `batteryState` và `batteryNote`: thanh pin chỉ giữ biểu tượng, phần trăm và điện áp. Cảnh báo pin thấp riêng vẫn được giữ. Giá trị không hợp lệ/mất dữ liệu hiện `—%` và `— V`.

## Phạm vi giữ nguyên

Không đổi C8/A1, gain, remote mixing, IMU, cơ chế cấp quyền điều khiển, đo ADC, hiệu chuẩn CAL1 hoặc bảng quy đổi phần trăm. 24,25 V chưa được đổi thành 100%.

## Bằng chứng và hoàn tác

- `wifi-validation/browser-r4-tests.json`: 57 kiểm tra trình duyệt trên mock HTTP/WebSocket cục bộ, không phần cứng.
- `wifi-validation/r4-status-verification.json`: hash binary và đối chiếu 11 nguồn bảo toàn.
- Binary, log build và bản sao R3 được giữ cục bộ; không đưa log thiết bị hoặc backup lên repository.

Build mới: `DUAL-C8-A1-WIFI-20261117-R4-STATUS`. Binary `.pio/build/esp32_wifi/firmware.bin`. Chỉ nạp khi người vận hành yêu cầu và xác nhận 24 V tắt, USB cắm.
