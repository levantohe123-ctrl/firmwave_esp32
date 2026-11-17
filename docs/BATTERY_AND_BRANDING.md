# Đo pin Li-ion 6S4P và tài nguyên giao diện

## GPIO32 và điện áp

GPIO32 là ADC1_CHANNEL_4, dùng được khi bật Wi-Fi. Phần cứng hiện tại: cầu chia 100 kΩ/10 kΩ, điện trở nối tiếp 1 kΩ và tụ 100 nF theo mạch đã đối chiếu trên xe. Hệ số chia áp 11; đầu vào ADC phải nằm trong giới hạn ESP32. Không đưa điện áp pin trực tiếp vào GPIO.

Firmware đọc ADC 12 bit, dùng `esp_adc_cal_characterize`, ưu tiên thông số eFuse khi có:

```text
V_pack = ADC_mV × 0,001 × 11 × 0,9855 + 0
```

Gain CAL1 = 0,9855, offset = 0 là hiệu chỉnh một điểm gần số đo đồng hồ 24,1 V và GPIO khoảng 2,19 V. Đợt đối chiếu CAL1 có 30 mẫu điện áp nghỉ khoảng 24,093–24,111 V. Không suy rộng độ chính xác đó ra toàn dải xả hoặc dưới tải motor.

Tác vụ đo 20 Hz, median 5 mẫu và EMA khoảng 1 s. Mất áp hoặc bão hòa được đánh dấu invalid, không biến thành số pin giả. Serial read-only có `BATTERY_STATUS`, ví dụ `1000 BATTERY_STATUS`.

## Phần trăm

`battery_estimator.h` chia điện áp bộ pin cho 6 và nội suy từng đoạn theo bảng tham khảo:

| Điện áp/cell | 3,20 | 3,40 | 3,50 | 3,60 | 3,70 | 3,75 | 3,80 | 3,85 | 3,95 | 4,05 | 4,20 |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| Phần trăm | 0 | 5 | 10 | 20 | 30 | 40 | 50 | 60 | 75 | 90 | 100 |

Pin 24,25 V cho mục tiêu khoảng 88,75%; 24,1 V khoảng 85%. Đề xuất quy ước 24,25 V = 100% sử dụng chưa áp dụng. Bảng không phải đường xả đo riêng cho bộ pin và không đo điện lượng bằng cảm biến dòng. Mốc 0% phần mềm không phải ngưỡng bảo vệ BMS hoặc lệnh ngắt motor.

Giữ phần trăm khi motor/chuyển động hoạt động. Sau dừng cần nghỉ ít nhất 10 s và điện áp ổn định trong dải 0,12 V ít nhất 5 s; hai điều kiện có thể diễn ra đồng thời. Sau đó phần trăm thay đổi tối đa 0,2 điểm/giây. Giới hạn này làm mượt, không tăng độ chính xác đo dung lượng.

Cảnh báo vàng khi mục tiêu ≤20%, đỏ khi ≤10%, xác nhận 5 s; hysteresis 23%/13%. Đây là thông báo, không thay BMS và không tự ngắt motor theo phần trăm. Không có tín hiệu cắm sạc; điện áp tăng chưa đủ kết luận đang sạc.

Muốn cập nhật dung lượng liên tục khi chạy, bổ sung đo dòng hai chiều và đếm điện lượng, hiệu chỉnh bằng điện áp nghỉ. Nguồn sạc cần giới hạn dòng/điện áp phù hợp cell và BMS; BMS không thay bộ điều khiển sạc.

## Logo và font

Logo nguyên bản: https://iuh.edu.vn/assets/images/icons/logo.svg. Nhận diện tham khảo: https://iuh.edu.vn/vi/nhan-dien-thuong-hieu.html. Logo thuộc IUH, lưu trong `dashboard/iuh-logo.svg` để nhận diện đồ án theo yêu cầu chủ đồ án; không tải tài nguyên ngoài khi vận hành.

Font lưu cục bộ trong `dashboard/`; giấy phép tại `dashboard/FONT-LICENSE.txt`. Không vẽ lại hoặc đổi tỷ lệ/màu logo.
