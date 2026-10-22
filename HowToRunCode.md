# How To Run Code

Các lệnh để build, chạy và sinh dataset, copy rồi dán vào terminal.
Muốn hiểu code và paper thì đọc [docs/HUONG_DAN_RRT-FConnect.md](docs/HUONG_DAN_RRT-FConnect.md).

> Môi trường: **Ubuntu 20.04 + ROS Noetic**. Trên Windows thì dùng **WSL2**.
> Mọi lệnh dưới đây chạy **trong terminal WSL**.

---

## 1. Cài môi trường (làm 1 lần)

**PowerShell (Windows):** cài Ubuntu 20.04 cho WSL. Bỏ qua nếu máy đã có.
```powershell
wsl --install -d Ubuntu-20.04
```

**WSL:** cài ROS Noetic. Bỏ qua nếu thư mục `/opt/ros/noetic` đã có.
```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install -y curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update && sudo apt install -y ros-noetic-desktop-full
```

**WSL:** cài các gói project cần.
```bash
sudo apt install -y ros-noetic-pcl-ros ros-noetic-interactive-markers libeigen3-dev nlohmann-json3-dev git
```

---

## 2. Clone và build (làm 1 lần)

> ⚠️ Clone vào **ổ của Linux** (`~`), **không** clone vào `/mnt/c/...`.
> Nếu clone vào ổ Windows, symlink `src/CMakeLists.txt` sẽ hỏng và build báo `Parse error`.

```bash
cd ~
git clone https://github.com/Bong142D/heuristic-guided-BiRRT.git
cd ~/heuristic-guided-BiRRT
chmod +x src/map_gen/map_generator/scripts/*.py

# Code đang ghi cứng đường dẫn /home/huy/rrt. Tạo sẵn thư mục này để code không crash.
sudo mkdir -p /home/huy/rrt && sudo chown $USER /home/huy/rrt
cp brrt_input.json /home/huy/rrt/brrt_input.json

source /opt/ros/noetic/setup.bash
catkin_make
```

Tạo file tham số theo **Table 1 của paper**:
```bash
cat > /home/huy/rrt/paper_table1.json <<'EOF'
{ "trial": 1, "environment": "paper_table1",
  "epsilon": 0.5, "p1": 0.8, "u_p": 1, "alpha": 1, "beta": 0.2, "gamma": 0.2,
  "n_rays": 8, "weight_grade": 1, "ray_step_size": 0.5 }
EOF
```

**Mỗi terminal mới** đều phải chạy 2 dòng này trước:
```bash
source /opt/ros/noetic/setup.bash
source ~/heuristic-guided-BiRRT/devel/setup.bash
```

Khi có code mới trên repo: chạy `cd ~/heuristic-guided-BiRRT && git pull && catkin_make`.

---

## 3. Chạy thử với bản đồ ngẫu nhiên (random forest)

Mở 3 terminal, nhớ `source` 2 dòng ở trên trước.

```bash
# Terminal 1: sinh bản đồ random forest
roslaunch path_finder map.launch
```
```bash
# Terminal 2: mở RViz
roslaunch path_finder rviz.launch
```
```bash
# Terminal 3: chạy planner
roslaunch path_finder test_planners.launch input_param:=/home/huy/rrt/paper_table1.json
```
- Ở terminal 3 sẽ có lỗi `pcd_to_pointcloud ... process has died`. **Lỗi này bình thường**, bỏ qua.
- Đợi tới khi terminal 3 in ra `glb occ set` là bản đồ đã nạp xong.

**Đặt goal:** trong RViz chọn **Goal3DTool** (phím `g`), click chuột trái rồi thả ra ngay.
Bản đồ random forest chỉ có vật cản trong vùng 100×100 ở giữa, nên chọn goal gần tâm.

Cũng có thể đặt goal bằng lệnh, chạy ở một terminal khác:
```bash
rostopic pub -1 /goal geometry_msgs/PoseStamped '{header: {frame_id: "map"}, pose: {position: {x: 30.0, y: 20.0, z: 0.0}, orientation: {w: 1.0}}}'
```

Kết quả in ra ở terminal 3. Dòng `[SOFBRRTOpitmize*] Case3 final path len: ...` là của **RRT-FConnect**.

---

## 4. Sinh dataset giống paper

Dataset gồm 4 loại bản đồ (`blobs`, `narrow`, `flappy`, `rooms`), mỗi loại 3 mức lấp đầy (10/20/40%), mỗi mức 5 bản đồ.
Mỗi bản đồ có 20 cặp start/goal. Tổng cộng 60 bản đồ, mất khoảng 15–20 phút.

```bash
# Terminal 1
roscore
```
```bash
# Terminal 2
cd ~/heuristic-guided-BiRRT
python3 generate_dataset.py
```
Kết quả nằm ở `/home/huy/rrt/dataset_maps_200x200/<loại>_<mức>/`, gồm các file `map_0.pcd … map_4.pcd` và `map_i_scenarios.json`.

---

## 5. Chạy planner trên một bản đồ của dataset

Không cần `map.launch` nữa, vì planner tự đọc file `.pcd`.

```bash
# Terminal 1
roslaunch path_finder rviz.launch
```
```bash
# Terminal 2: đổi blobs_10 / map_0 sang bản đồ muốn chạy
roslaunch path_finder test_planners.launch \
  map_file:=/home/huy/rrt/dataset_maps_200x200/blobs_10/map_0.pcd \
  input_param:=/home/huy/rrt/paper_table1.json
```
Sau đó đặt goal giống như ở mục 3.

---

## 6. Chạy thí nghiệm hàng loạt (Table 2)

⏳ **Chưa chạy được.** Cần sửa code trước: bật lại các baseline, lặp 10 lần mỗi truy vấn, bỏ các đường dẫn ghi cứng.
Chi tiết ở mục 6 của [docs/HUONG_DAN_RRT-FConnect.md](docs/HUONG_DAN_RRT-FConnect.md).

---

## Lỗi hay gặp

| Lỗi | Cách sửa |
|---|---|
| `CMake Error ... Parse error` khi chạy `catkin_make` | Đang build trong `/mnt/c`. Clone lại vào `~` như mục 2 |
| Terminal planner cứ in mãi `no map rcved yet` | Chưa chạy `map.launch` (mục 3), hoặc đường dẫn `map_file` sai (mục 5) |
| RViz không có Goal3DTool | Mở RViz khi chưa `source devel/setup.bash`. Tắt RViz, `source` lại rồi mở lại |
| `Failed to open log file: /home/huy/rrt/...` hoặc planner crash | Chưa tạo thư mục `/home/huy/rrt` (mục 2) |
| `rosrun ... Couldn't find executable dataset_map_generator.py` | Chạy lại lệnh `chmod +x` ở mục 2 |
| Không có cửa sổ RViz | Chạy `wsl --update` trong PowerShell rồi mở lại WSL |
