# 🤖 6-DOF Robotic Arm: ArUco Pick & Place (ROS 2 + MoveIt 2) 📦

🇬🇧 **Introduction:** A complete ROS 2 Jazzy project for a 6-axis robotic arm, featuring an automated Pick & Place pipeline using OpenCV/ArUco marker detection and MoveIt 2 motion planning.

🇻🇳 **Giới thiệu:** Project ROS 2 Jazzy điều khiển tay máy 6 bậc tự do, tích hợp pipeline Pick & Place tự động hoàn toàn thông qua nhận diện camera ArUco và MoveIt 2.

---

## 📂 1. Packages Overview / Cấu trúc Project

| Package | 🇬🇧 Description | 🇻🇳 Chức năng |
|---|---|---|
| 🏗️ `my_robot_description` | URDF/xacro files and RViz visualization | Khai báo mô hình URDF/xacro và RViz |
| ⚙️ `my_robot_moveit_config` | MoveIt 2 configuration (Setup Assistant) | Cấu hình MoveIt 2 (tạo bằng Setup Assistant) |
| 🐍 `my_robot_commander_py` | Python MoveItPy nodes (10-step state machine) | Node điều khiển bằng Python (state machine 10 bước) |
| 🧩 `my_robot_commander_cpp` | C++ MoveGroupInterface nodes | Node điều khiển bằng C++ tương đương |
| 👁️ `my_robot_perception` | OpenCV ArUco detector (solvePnP) → TF/Pose | Detect ArUco, tính pose 3D và publish lên TF |
| 🔌 `my_robot_hardware` | `ros2_control` open-loop stepper serial driver | Plugin điều khiển động cơ bước qua serial (STM32) |
| ✉️ `my_robot_interfaces` | Custom ROS 2 messages (`PoseCommand`) | Chứa custom message của project |
| 🚀 `my_robot_bringup` | Main launch files for mock & real hardware | Launch file tổng cho cả phần cứng ảo và thật |

---

## 💻 2. System Requirements / Yêu cầu hệ thống

🇬🇧 **Prerequisites** / 🇻🇳 **Yêu cầu cài đặt:**

- 🐢 **ROS 2:** Jazzy Jalisco (compatible with Humble / tương thích Humble)
- 🦾 **MoveIt 2** & `ros2_control`
- 📷 **Vision:** OpenCV, `cv_bridge`, `v4l-utils`, `usb_cam`
- 🔄 **Transforms:** `tf2`, `tf2_ros`, `tf2_geometry_msgs`, `tf_transformations`

---

## 🛠️ 3. Environment Guide / Hướng dẫn Môi trường

### 🪟 WSL2 (Windows)

- 🇬🇧 **USB Passthrough:** You must use [usbipd-win](https://github.com/dorssel/usbipd-win) to attach your webcam and STM32 serial port (`/dev/ttyUSB0` or `/dev/ttyACM0`) from Windows to WSL2.
- 🇻🇳 **Kết nối USB:** Bắt buộc dùng `usbipd-win` để đẩy cổng USB của webcam và STM32 từ Windows vào WSL2.
- 🇬🇧 **RViz Performance:** WSLg supports GUI out-of-the-box, but if RViz is laggy or crashes, force software rendering:
  ```bash
  export LIBGL_ALWAYS_SOFTWARE=1
  ```
- 🇻🇳 **Hiệu năng RViz:** Nếu RViz giật lag hoặc crash, hãy ép dùng CPU để render bằng lệnh trên.

### 🐧 Dual Boot Ubuntu (Recommended / Khuyên dùng)

- 🇬🇧 **USB Passthrough:** Native support. Just ensure your user has permissions for the serial port:
  ```bash
  sudo usermod -aG dialout $USER
  ```
  (requires logout/login)
- 🇻🇳 **Kết nối USB:** Chạy trực tiếp rất mượt. Chỉ cần cấp quyền đọc/ghi cổng serial (STM32) cho user bằng lệnh trên (cần logout/login lại).
- 🇬🇧 **RViz Performance:** Best performance, fully utilizes your GPU.
- 🇻🇳 **Hiệu năng RViz:** Tốt nhất, sử dụng tối đa sức mạnh GPU.

### 💿 VirtualBox / Oracle VM

- 🇬🇧 **USB Passthrough:** Install the *VirtualBox Extension Pack*. Go to VM Settings → USB → add filters for your webcam and STM32 board.
- 🇻🇳 **Kết nối USB:** Cài *Extension Pack*. Vào Settings → USB → thêm filter để truyền thẳng webcam và STM32 vào máy ảo.
- 🇬🇧 **RViz Performance:** Enable **3D Acceleration** in VM Display settings and allocate maximum video memory. It may still be laggy compared to dual boot.
- 🇻🇳 **Hiệu năng RViz:** Bật **3D Acceleration** trong phần Display và cấp tối đa VRAM. Trải nghiệm RViz vẫn có thể bị giật nhẹ.

---

## 🚀 4. Build & Run / Hướng dẫn Chạy

### 🏗️ Build Project

```bash
# 🇬🇧 Navigate to workspace & build / 🇻🇳 Vào thư mục workspace và build
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### 🎮 Run with Mock Hardware (Simulation / Chạy ảo)

🇬🇧 Run the pick & place pipeline using fake `ros2_control` components.
🇻🇳 Chạy pipeline gắp thả với phần cứng mô phỏng (không cần cắm robot).

```bash
ros2 launch my_robot_bringup pick_and_place.launch.py use_real_hardware:=false
```

### 🦾 Run with Real Hardware (Physical Robot / Chạy thật)

🇬🇧 Connect the STM32 and webcam, then run with the real hardware flag.
🇻🇳 Cắm cáp STM32 và webcam, sau đó chạy lệnh sau.

```bash
ros2 launch my_robot_bringup pick_and_place.launch.py use_real_hardware:=true
```

---

## ⚠️ 5. Important Notes / Lưu ý Quan trọng

**1. 🎯 Camera TF Calibration / Hiệu chỉnh TF Camera**

- 🇬🇧 The static TF between `base_link` and `camera_link` in `pick_and_place.launch.py` is hardcoded (`x=0.3, z=0.5, ...`). You **must** measure and update this to match your real physical setup.
- 🇻🇳 Tọa độ TF tĩnh từ `base_link` đến `camera_link` đang fix cứng trong launch file. Bạn **bắt buộc** phải đo đạc và sửa lại file launch cho khớp với vị trí đặt camera thực tế.

**2. 📷 Webcam Node**

- 🇬🇧 In `pick_and_place.launch.py`, the `usb_cam_node` may be commented out (`#usb_cam_node`). Make sure to uncomment it or run your camera driver separately when using real hardware.
- 🇻🇳 Trong file `pick_and_place.launch.py`, node `usb_cam_node` đang bị comment (`#usb_cam_node`). Nhớ bỏ comment hoặc chạy node camera riêng khi dùng robot thật.

**3. 📏 Marker Size / Kích thước ArUco**

- 🇬🇧 The default marker size is `0.04` (4 cm). Update `marker_length` in `aruco_detector_node` if your printed marker is a different size.
- 🇻🇳 Kích thước marker mặc định là 4cm (`0.04`). Hãy sửa lại tham số `marker_length` trong code perception nếu bạn in giấy to/nhỏ hơn.
