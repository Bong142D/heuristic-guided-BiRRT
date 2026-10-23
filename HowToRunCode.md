# How To Run Code

Ubuntu 20.04 + ROS Noetic (Windows: WSL2). Mọi lệnh chạy trong terminal WSL.
Giải thích code và paper: [docs/HUONG_DAN_RRT-FConnect.md](docs/HUONG_DAN_RRT-FConnect.md).

## Tình hình (23/10/2026)

Mục tiêu: reproduce RRT-FConnect, tức là chạy tham số Table 1 rồi so với Table 2 của paper.

- [x] Môi trường, build, RViz
- [x] Dataset: blobs/narrow × 10/20/40%, 30 bản đồ (file `dataset_maps_200x200.tar.gz`)
- [x] Code thí nghiệm hàng loạt, branch `experiment-batch` (chờ review PR)
- [ ] Chạy đầy đủ: đang chạy trên máy chính (blobs_10, blobs_20 xong; blobs_40 4/5)
- [ ] So với Table 2
- [ ] Hỏi GVHD: R_max (5 hay 10), chế độ exploration, class nào là EBi-RRT, mức narrow nào ứng với paper
- [ ] Multi-Robot

Người mới: mục 1 → 2 → 3 → 4 → 5 (chạy thử) → review PR.

## 1. Cài đặt

```powershell
wsl --install -d Ubuntu-20.04          # PowerShell, bỏ qua nếu đã có
```
```bash
# ROS Noetic, bỏ qua nếu đã có /opt/ros/noetic
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install -y curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update && sudo apt install -y ros-noetic-desktop-full

sudo apt install -y ros-noetic-pcl-ros ros-noetic-interactive-markers libeigen3-dev nlohmann-json3-dev git
```

## 2. Clone và build

Clone vào `~`, không clone vào `/mnt/c` (symlink hỏng, build báo `Parse error`).

```bash
cd ~ && git clone https://github.com/Bong142D/heuristic-guided-BiRRT.git
cd ~/heuristic-guided-BiRRT
git checkout experiment-batch
chmod +x src/map_gen/map_generator/scripts/*.py
sudo mkdir -p /home/huy/rrt && sudo chown $USER /home/huy/rrt
source /opt/ros/noetic/setup.bash && catkin_make
```

Mỗi terminal mới:
```bash
source /opt/ros/noetic/setup.bash && source ~/heuristic-guided-BiRRT/devel/setup.bash
```

Cập nhật code: `git pull && catkin_make`.
Tham số mặc định: [src/path_finder/config/paper_table1.json](src/path_finder/config/paper_table1.json) (Table 1).

## 3. Chạy tương tác

```bash
roslaunch path_finder rviz.launch                # terminal 1
roslaunch path_finder test_planners.launch       # terminal 2, tự sinh bản đồ random forest
```
- Đợi terminal 2 in `glb occ set`.
- Đặt goal: trong RViz chọn Goal3DTool (phím `g`), click rồi thả. Nên chọn gần tâm bản đồ.
- Hoặc đặt goal bằng lệnh:
  ```bash
  rostopic pub -1 /goal geometry_msgs/PoseStamped '{header: {frame_id: "map"}, pose: {position: {x: 30, y: 20, z: 0}, orientation: {w: 1}}}'
  ```
- Dòng kết quả của RRT-FConnect: `[SOFBRRTOpitmize*] Case3 final path len`.
- Chạy từng bước: thêm `sof_brrt_step_mode:=true`, rồi bấm NEXT STEP trong RViz.
- Dùng bản đồ dataset thay cho random forest: thêm `map_file:=/home/huy/rrt/dataset_maps_200x200/blobs_10/map_0.pcd`.

## 4. Dataset

Dùng file chung (khuyên dùng). Copy `dataset_maps_200x200.tar.gz` vào `\\wsl$\Ubuntu-20.04\home\<user>\`, rồi chạy:
```bash
tar -xzf ~/dataset_maps_200x200.tar.gz -C /home/huy/rrt
```

Tự sinh (bản đồ ngẫu nhiên nên sẽ khác bộ chung; mất vài giờ, Ctrl+C khi `narrow_40` đủ 5 bản đồ):
```bash
roscore                                          # terminal 1
cd ~/heuristic-guided-BiRRT && python3 generate_dataset.py   # terminal 2
```

Mỗi bản đồ gồm `map_i.pcd` và `map_i_scenarios.json` (20 cặp start/goal).

## 5. Thí nghiệm hàng loạt

```bash
cd ~/heuristic-guided-BiRRT

# chạy thử, vài giây
python3 run_dataset_experiments.py --envs blobs_10 --maps 1 --max-scenarios 2 --trials 2 --out ~/rrt_results/quicktest
python3 analyze_results.py ~/rrt_results/quicktest

# đầy đủ: blobs_10/20/40 + narrow_10, 5 bản đồ × 20 cặp × 10 lần; mất vài giờ
python3 run_dataset_experiments.py --trials 10 --out ~/rrt_results/full
python3 analyze_results.py ~/rrt_results/full
```
- Chạy lại cùng `--out` thì bản đồ nào đã xong sẽ được bỏ qua.
- Bản đồ báo `FAILED` thì xem log `<out>/<env>/map_i.log`.

## Lỗi thường gặp

| Lỗi | Sửa |
|---|---|
| `CMake Error ... Parse error` | Đang build trong `/mnt/c`, clone lại vào `~` |
| `no map rcved yet` mãi | `map_file` sai đường dẫn |
| Không có Goal3DTool | Mở RViz trước khi `source devel/setup.bash` |
| `unrecognized arguments` / `FAILED` | Chưa ở branch `experiment-batch` hoặc chưa `catkin_make` |
| `Couldn't find executable dataset_map_generator.py` | Chạy lại `chmod +x` (mục 2) |
| Không có cửa sổ RViz | `wsl --update` trong PowerShell |
