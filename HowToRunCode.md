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

# Dataset được sinh vào /home/huy/rrt/... (đường dẫn ghi cứng trong generate_dataset.py)
sudo mkdir -p /home/huy/rrt && sudo chown $USER /home/huy/rrt

source /opt/ros/noetic/setup.bash
catkin_make
```

Tham số theo **Table 1 của paper** có sẵn ở [src/path_finder/config/paper_table1.json](src/path_finder/config/paper_table1.json), và được dùng mặc định.

**Mỗi terminal mới** đều phải chạy 2 dòng này trước:
```bash
source /opt/ros/noetic/setup.bash
source ~/heuristic-guided-BiRRT/devel/setup.bash
```

Khi có code mới trên repo: chạy `cd ~/heuristic-guided-BiRRT && git pull && catkin_make`.

> Code thí nghiệm hàng loạt (mục 6) đang ở nhánh **`experiment-batch`**, chưa merge vào `main`. Chuyển sang nhánh đó rồi build lại:
> ```bash
> cd ~/heuristic-guided-BiRRT && git fetch && git checkout experiment-batch && catkin_make
> ```

---

## 3. Chạy thử với bản đồ ngẫu nhiên (random forest)

Mở 2 terminal, nhớ `source` 2 dòng ở trên trước.

```bash
# Terminal 1: mở RViz
roslaunch path_finder rviz.launch
```
```bash
# Terminal 2: chạy planner, tự sinh kèm bản đồ random forest
roslaunch path_finder test_planners.launch
```
Đợi tới khi terminal 2 in ra `glb occ set` là bản đồ đã nạp xong.

**Đặt goal:** trong RViz chọn **Goal3DTool** (phím `g`), click chuột trái rồi thả ra ngay.
Bản đồ random forest chỉ có vật cản trong vùng 100×100 ở giữa, nên chọn goal gần tâm.

Cũng có thể đặt goal bằng lệnh, chạy ở một terminal khác:
```bash
rostopic pub -1 /goal geometry_msgs/PoseStamped '{header: {frame_id: "map"}, pose: {position: {x: 30.0, y: 20.0, z: 0.0}, orientation: {w: 1.0}}}'
```

Kết quả in ra ở terminal 2. Dòng `[SOFBRRTOpitmize*] Case3 final path len: ...` là của **RRT-FConnect**.

**Xem từng bước** (tia, sector, APF): thêm `sof_brrt_step_mode:=true` vào lệnh ở terminal 2. Bấm khối xanh **NEXT STEP** trong RViz để chạy tiếp.

---

## 4. Sinh dataset giống paper

Paper chỉ dùng **blobs** (10/20/40%) và **narrow**. Script sinh lần lượt `blobs` → `narrow` → `flappy` → `rooms`, mỗi mức 5 bản đồ, mỗi bản đồ 20 cặp start/goal.

```bash
# Terminal 1
roscore
```
```bash
# Terminal 2
cd ~/heuristic-guided-BiRRT
python3 generate_dataset.py
```
- Có thể mất **vài giờ**, và máy phải để không sleep.
- Khi thư mục `narrow_40` đủ 5 file `.pcd` thì có thể Ctrl+C, vì flappy và rooms paper không dùng.
- Kiểm tra tiến độ: `ls /home/huy/rrt/dataset_maps_200x200/*/`

Kết quả nằm ở `/home/huy/rrt/dataset_maps_200x200/<loại>_<mức>/`, gồm các file `map_0.pcd … map_4.pcd` và `map_i_scenarios.json`.

---

## 5. Chạy planner trên một bản đồ của dataset

```bash
# Terminal 1
roslaunch path_finder rviz.launch
```
```bash
# Terminal 2: đổi blobs_10 / map_0 sang bản đồ muốn chạy
roslaunch path_finder test_planners.launch map_file:=/home/huy/rrt/dataset_maps_200x200/blobs_10/map_0.pcd
```
Sau đó đặt goal giống như ở mục 3.

---

## 6. Chạy thí nghiệm hàng loạt (Table 2)

Không cần RViz. Chỉ cần 1 terminal đã `source`.

**Chạy thử nhỏ trước** (khoảng vài giây) để chắc mọi thứ chạy được:
```bash
cd ~/heuristic-guided-BiRRT
python3 run_dataset_experiments.py --envs blobs_10 --maps 1 --max-scenarios 2 --trials 2 --out ~/rrt_results/quicktest
python3 analyze_results.py ~/rrt_results/quicktest
```

**Chạy đầy đủ như paper** (4 môi trường × 5 bản đồ × 20 cặp × 10 lần):
```bash
cd ~/heuristic-guided-BiRRT
python3 run_dataset_experiments.py --trials 10
```
- Lệnh này in ra thư mục kết quả, dạng `~/rrt_results/<ngày_giờ>`.
- Mặc định chạy `blobs_10 blobs_20 blobs_40 narrow_10`. Đổi bằng `--envs ...`.
- Chạy lại cùng `--out` thì bản đồ nào đã xong sẽ được bỏ qua.
- Bản đồ nào báo `FAILED` thì mở file log `~/rrt_results/<ngày_giờ>/<env>/map_i.log` để xem lỗi.

**Xem kết quả**, ra bảng giống Table 2, kèm số liệu của paper:
```bash
python3 analyze_results.py ~/rrt_results/<ngày_giờ>
```

---

## Lỗi hay gặp

| Lỗi | Cách sửa |
|---|---|
| `CMake Error ... Parse error` khi chạy `catkin_make` | Đang build trong `/mnt/c`. Clone lại vào `~` như mục 2 |
| Terminal planner cứ in mãi `no map rcved yet` | Đường dẫn `map_file` sai (mục 5) |
| RViz không có Goal3DTool | Mở RViz khi chưa `source devel/setup.bash`. Tắt RViz, `source` lại rồi mở lại |
| Mục 6 báo `map_i: FAILED`, log ghi `Couldn't find ...` hoặc lỗi param | Chưa `catkin_make` sau khi đổi sang nhánh `experiment-batch`, hoặc terminal chưa `source devel/setup.bash` |
| `run_dataset_experiments.py` báo `unrecognized arguments` | Đang ở nhánh `main`. Chuyển sang `experiment-batch` (cuối mục 2) |
| `rosrun ... Couldn't find executable dataset_map_generator.py` | Chạy lại lệnh `chmod +x` ở mục 2 |
| Không có cửa sổ RViz | Chạy `wsl --update` trong PowerShell rồi mở lại WSL |
