# Instruction cho agent code: Robot ROSMASTER X3 Plus gắp - thả vật theo màu

## 1. Vai trò của agent code

Bạn là agent code hỗ trợ xây dựng, mô phỏng và lập trình hệ thống robot gắp - thả vật sử dụng ROSMASTER X3 Plus.

Mục tiêu chính là tạo code đơn giản, dễ hiểu, dễ kiểm thử cho sinh viên làm project robot/AI. Khi viết code, hãy ưu tiên cấu trúc rõ ràng, có chú thích, tách module theo node ROS và điều khiển bằng FSM.

## 2. Bối cảnh bài toán

Robot sử dụng là ROSMASTER X3 Plus, hoạt động trong môi trường trong nhà:

- Mặt phẳng ổn định.
- Không có vật cản.
- Các vị trí làm việc là cố định.
- Robot có camera indoor hoặc cảm biến màu.
- Robot có cơ cấu gắp/thả vật.
- Robot có thể di chuyển đến waypoint cố định.
- Robot chọn vật cần gắp dựa trên màu sắc được chỉ định.

Không cần xử lý tránh vật cản trong bài toán này.

## 3. Các vị trí làm việc cố định

Hệ thống có 5 vị trí chính:

1. `home`
   - Vị trí bắt đầu.
   - Vị trí robot quay về sau khi hoàn thành chu trình.

2. `pick_1`
   - Vị trí gắp vật 1.
   - Robot dừng tại đây để nhận diện vật 1 theo màu được chỉ định.
   - Sau khi nhận diện đúng màu, robot gắp vật 1.

3. `drop_1`
   - Vị trí thả vật 1.
   - Robot di chuyển từ `pick_1` đến đây.
   - Robot mở gripper để thả vật 1.

4. `pick_2`
   - Vị trí gắp vật 2.
   - Robot dừng tại đây để nhận diện vật 2 theo màu được chỉ định.
   - Sau khi nhận diện đúng màu, robot gắp vật 2.

5. `drop_2`
   - Vị trí thả vật 2.
   - Robot di chuyển từ `pick_2` đến đây.
   - Robot mở gripper để thả vật 2.

## 4. Lộ trình hoạt động tổng quát

Chu trình robot cần thực hiện:

1. Robot bắt đầu tại `home`.
2. Robot di chuyển từ `home` đến `pick_1`.
3. Tại `pick_1`, robot nhận diện vật 1 theo màu được chỉ định.
4. Robot gắp vật 1.
5. Robot di chuyển đến `drop_1`.
6. Tại `drop_1`, robot thả vật 1.
7. Robot di chuyển đến `pick_2`.
8. Tại `pick_2`, robot nhận diện vật 2 theo màu được chỉ định.
9. Robot gắp vật 2.
10. Robot di chuyển đến `drop_2`.
11. Tại `drop_2`, robot thả vật 2.
12. Robot quay trở lại `home`.
13. Kết thúc chu trình.

Luồng rút gọn:

```text
home -> pick_1 -> drop_1 -> pick_2 -> drop_2 -> home
```

## 5. Kiến trúc ROS đề xuất

Ưu tiên tổ chức code thành các node sau:

1. `vision_color_node`
   - Nhận ảnh từ camera.
   - Chuyển ảnh RGB/BGR sang HSV.
   - Threshold màu theo màu mục tiêu.
   - Tìm contour lớn nhất.
   - Trả về trạng thái có tìm thấy vật hay không.
   - Có thể trả thêm tọa độ tâm vật trong ảnh.

2. `navigation_node`
   - Nhận tên waypoint cần đến.
   - Điều khiển robot di chuyển đến waypoint cố định.
   - Trả về trạng thái `move_done`.
   - Vì môi trường không có vật cản, có thể dùng điều hướng waypoint đơn giản.

3. `gripper_node`
   - Nhận lệnh `open`, `close`, `pick`, `place`.
   - Điều khiển cơ cấu gắp/thả.
   - Trả về trạng thái `gripper_done`.

4. `mission_manager_node`
   - Node quản lý nhiệm vụ chính.
   - Chạy FSM.