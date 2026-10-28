# How To Run Code

Ubuntu 20.04 + ROS Noetic (Windows: WSL2). Mọi lệnh chạy trong terminal WSL.
Giải thích code và paper: [docs/HUONG_DAN_RRT-FConnect.md](HUONG_DAN_RRT-FConnect.md).

## Tình hình (28/10/2026)

Mục tiêu: reproduce RRT-FConnect, tức là chạy tham số Table 1 rồi so với Table 2 của paper.

- [x] Môi trường, build, RViz
- [x] Dataset: 12 môi trường, 60 bản đồ (file `dataset_maps_200x200.tar.gz` trong repo)
- [x] Code thí nghiệm hàng loạt, đã gộp vào `main`. Những chỗ sửa so với code gốc: mục 6 của [HUONG_DAN_RRT-FConnect.md](HUONG_DAN_RRT-FConnect.md)
- [x] Chạy đầy đủ và so với Table 2: [docs/KET_QUA_REPRODUCE.md](KET_QUA_REPRODUCE.md). Với code gốc: độ dài đường đi và số node khớp; thời gian và số vòng lặp chưa khớp.
- [x] Quét nguyên nhân, sửa theo paper (kho 100 cặp, k-NN chính xác, R_max = 5, không bỏ mẫu nằm trong vật cản, APF luôn áp): cấu hình [paper_table1_final.json](../src/path_finder/config/paper_table1_final.json)
- [x] Số vòng lặp ở Highly/Narrow cao gấp khoảng 2 lần paper: nguyên nhân chính là code bỏ mẫu nằm trong vật cản (H9 trong KET_QUA_REPRODUCE.md)
- [x] Lỗi đo thời gian: đồng hồ của WSL bị chỉnh nhảy +1,4 s mỗi khoảng 34 s. Planner giờ đo bằng đồng hồ đơn điệu; máy chạy thí nghiệm phải sửa đồng hồ (mục 2b)
- [x] Chạy lại sạch mọi thí nghiệm có số liệu thời gian, gồm cả EBi-RRT trên flappy/rooms (lần 9 trong [KET_QUA_REPRODUCE.md](KET_QUA_REPRODUCE.md)). Với cấu hình cuối, mọi kết luận của Table 2 đều tái hiện được; cách chạy lại ở mục 6
- [ ] Multi-Robot: định hướng và bảng theo dõi trong [docs/HUONG_DAN_MULTI_ROBOT.md](HUONG_DAN_MULTI_ROBOT.md)

Người mới: mục 1 → 2 → 2b → 3 → 4 → 5 (chạy thử).

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
chmod +x src/map_gen/map_generator/scripts/*.py
sudo mkdir -p /home/huy/rrt && sudo chown $USER /home/huy/rrt
source /opt/ros/noetic/setup.bash && catkin_make
```

Mỗi terminal mới:
```bash
source /opt/ros/noetic/setup.bash && source ~/heuristic-guided-BiRRT/devel/setup.bash
```

Cập nhật code: `git pull && catkin_make`.
Tham số mặc định: [src/path_finder/config/paper_table1.json](../src/path_finder/config/paper_table1.json) (Table 1).

## 2b. Đồng hồ WSL (làm một lần, trước khi đo thời gian)

Nếu WSL bật systemd, `systemd-timesyncd` giằng co với cơ chế đồng bộ giờ của Hyper-V. Hậu quả: đồng hồ bị chỉnh chậm vài phần trăm, rồi nhảy +1,4 s mỗi khoảng 34 s (chi tiết trong [KET_QUA_REPRODUCE.md](KET_QUA_REPRODUCE.md), mục "Lỗi đồng hồ đo").

Kiểm tra (chạy 5 giây):
```bash
python3 -c "import time;m,r=time.monotonic(),time.clock_gettime(time.CLOCK_MONOTONIC_RAW);time.sleep(5);print('lech {:+.3f}%'.format(((time.monotonic()-m)/(time.clock_gettime(time.CLOCK_MONOTONIC_RAW)-r)-1)*100))"
```

Nếu lệch quá ±0.1% thì sửa như sau:
```bash
sudo systemctl mask --now systemd-timesyncd   # WSL đã được Hyper-V đồng bộ giờ
```
Sau đó chạy `wsl --shutdown` trong PowerShell rồi mở lại terminal WSL, để kernel khởi động lại với tốc độ đồng hồ mặc định. Muốn bật lại timesyncd: `sudo systemctl unmask systemd-timesyncd`.

Lưu ý: `systemctl stop` không đủ, vì WSL tự khởi động lại distro khi không còn terminal nào mở, và systemd sẽ chạy lại timesyncd.

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

Bộ bản đồ của nhóm nằm sẵn trong repo: 12 môi trường (blobs, narrow, flappy, rooms × 10/20/40%), mỗi môi trường 5 bản đồ. Bản chia sẻ ngày 23/10 chỉ có blobs/narrow (giống hệt bản này), nên ai đã dùng bản đó thì giải nén lại để có thêm flappy/rooms.
```bash
tar -xzf ~/heuristic-guided-BiRRT/dataset_maps_200x200.tar.gz -C /home/huy/rrt
```

Tự sinh thì bản đồ sẽ khác bộ của nhóm (generator không cố định seed), nên không dùng để reproduce được. Bản đồ đã có thì được bỏ qua:
```bash
roscore                                          # terminal 1
cd ~/heuristic-guided-BiRRT && python3 generate_dataset.py   # terminal 2: tất cả loại
python3 generate_dataset.py --types flappy rooms --fills 0.1 0.2 0.4   # chỉ flappy/rooms, khoảng 15 phút
```

Mỗi bản đồ gồm `map_i.pcd` và `map_i_scenarios.json` (20 cặp start/goal).

## 5. Thí nghiệm hàng loạt

```bash
cd ~/heuristic-guided-BiRRT

# chạy thử, vài giây
python3 run_dataset_experiments.py --envs blobs_10 --maps 1 --max-scenarios 2 --trials 2 --out ~/rrt_results/quicktest
python3 analyze_results.py ~/rrt_results/quicktest

# đầy đủ (code gốc): blobs_10/20/40 + narrow_10, 5 bản đồ × 20 cặp × 10 lần; khoảng nửa giờ
python3 run_dataset_experiments.py --trials 10 --out ~/rrt_results/full
python3 analyze_results.py ~/rrt_results/full
```
- Chạy lại cùng `--out` thì bản đồ nào đã xong sẽ được bỏ qua.
- Bản đồ báo `FAILED` thì xem log `<out>/<env>/map_i.log`.
- `--config src/path_finder/config/paper_table1_final.json`: RRT-FConnect gần paper nhất (bảng kết quả ở lần 9 của KET_QUA_REPRODUCE.md). `paper_table1_fixed.json` là bản sửa trước đó (lần 3). Mặc định `paper_table1.json` giữ nguyên code gốc (lần 1).
- `--envs ... flappy_10 rooms_20 ...`: chạy thêm môi trường. `--baselines brrt`: chỉ chạy RRT-Connect bên cạnh RRT-FConnect (bỏ `gb`, `ebi`).

Quét biến thể của RRT-FConnect (danh sách trong `VARIANTS` của [sweep_fconnect.py](../sweep_fconnect.py)):
```bash
python3 sweep_fconnect.py --variants orig paper_explore --trials 3 --out ~/rrt_results/sweep
python3 sweep_fconnect.py --summary-only --variants orig paper_explore --out ~/rrt_results/sweep
```

Kiểm tra thời gian bị nhiễu (máy ngủ, WSL khựng, đồng hồ nhảy) rồi chạy lại đúng các map đó. Một lần chạy bị đánh dấu khi thời gian mỗi vòng lặp của nó gấp ≥ 3 lần các lần chạy có số vòng lặp tương đương trên cùng map, thời gian mỗi node cũng gấp ≥ 3 lần các lần chạy có số node tương đương, và phần dư đủ làm lệch trung bình của map ≥ 2%. Điều kiện về node để bỏ qua các lần chạy mọc cây lớn bất thường: chúng chậm thật, không phải bị nhiễu.
```bash
python3 check_timing.py ~/rrt_results/full            # liệt kê lần chạy bị nhiễu
python3 check_timing.py ~/rrt_results/full --delete   # xoá kết quả các map đó
python3 run_dataset_experiments.py --trials 10 --out ~/rrt_results/full   # cùng lệnh cũ: chỉ chạy lại map đã xoá
```

Gộp một thuật toán chạy riêng (ví dụ EBi-RRT chạy ở lượt khác) vào cùng bảng:
```bash
python3 analyze_results.py ~/rrt_results/full --extra ~/rrt_results/full_ebi
```
Cột `vs RRT-Connect` là thời gian chia cho thời gian của RRT-Connect trong cùng lần chạy. Nên so cột này thay vì số ms, vì tốc độ máy laptop thay đổi theo thời điểm. Riêng dòng `(extra)` là thuật toán gộp từ lượt khác: cột này được quy đổi qua RRT-FConnect, thuật toán có mặt ở mọi lượt. Tỉ lệ này vẫn có sai số ngẫu nhiên: khoảng ±10–15% với 10 lần mỗi cặp, và tới ±47% với 3 lần mỗi cặp (mục "Nhiễu thời gian" của [KET_QUA_REPRODUCE.md](KET_QUA_REPRODUCE.md)).

Khi chạy dài: cắm sạc, không gập máy, và để Windows không ngủ. Trên laptop có Modern Standby, màn hình tự tắt cũng làm máy ngủ và WSL dừng chạy. Thời gian đo không bị sai (đồng hồ đơn điệu không đếm lúc máy ảo dừng), nhưng thí nghiệm bị trễ. Vì vậy nên vào Settings → System → Power & battery → Screen and sleep, đặt mục tắt màn hình khi cắm sạc thành Never.

## 6. Reproduce kết quả của nhóm

Kết quả để đối chiếu là lần 9 trong [KET_QUA_REPRODUCE.md](KET_QUA_REPRODUCE.md). Cần cùng code (`main`; đúng bản lúc chạy là tag `reproduce-lan9`), cùng bộ bản đồ (mục 4) và đồng hồ WSL đã sửa (mục 2b).

```bash
cd ~/heuristic-guided-BiRRT
F=src/path_finder/config/paper_table1_final.json
NEW="flappy_10 flappy_20 flappy_40 rooms_10 rooms_20 rooms_40"
C=~/rrt_results/clean

# cấu hình cuối: 4 môi trường của paper, rồi flappy/rooms (EBi-RRT chạy riêng vì rất chậm)
python3 run_dataset_experiments.py --config $F --out $C/full_final
python3 run_dataset_experiments.py --config $F --envs $NEW --baselines brrt gb --out $C/full_final
python3 run_dataset_experiments.py --config $F --envs $NEW --baselines ebi --out $C/ebi_new
# code gốc: trên 4 môi trường của paper, và so với cấu hình cuối trên flappy/rooms
python3 run_dataset_experiments.py --baselines brrt --out $C/full_orig
python3 sweep_fconnect.py --variants orig fx_noreject_apf --envs $NEW --trials 5 --out $C/sweep_3
# các vòng quét tìm nguyên nhân
python3 sweep_fconnect.py --trials 3 --out $C/sweep_1 --variants orig rmax5 noexplore knn_exact pool100 keep pool100_keep paper paper_keep paper_explore paper_k10 paper_k60 paper_rmax10
python3 sweep_fconnect.py --trials 5 --out $C/sweep_2 --variants orig pool100 pool100_exact pool100_rmax5 paper_explore pe_pool30 pe_pool300 pe_pool1000 pe_k10
python3 sweep_fconnect.py --trials 5 --out $C/sweep_4 --variants fx fx_noreject fx_apf fx_noreject_apf fx_expl10 fx_expl50 fx_len20 fx_len100 fx_noreject_noexpl fx_noreject_expl50 fx_noreject_len20

# in bảng
python3 analyze_results.py $C/full_final --extra $C/ebi_new
python3 analyze_results.py $C/full_orig
python3 sweep_fconnect.py --summary-only --out $C/sweep_1   # tương tự sweep_2, sweep_4; sweep_3 thêm --envs $NEW
```
- Sau mỗi lệnh chạy: `python3 check_timing.py <thư mục> --delete`, rồi chạy lại các lệnh ghi vào thư mục đó (map còn kết quả được bỏ qua). Lặp đến khi báo `0 map(s)`, tối đa 3 vòng.
- Tổng cộng khoảng 7 giờ trên laptop Intel Core Ultra 5 225H, riêng `ebi_new` khoảng 3 giờ. Chỉ cần đối chiếu Table 2 thì chạy lệnh đầu rồi `python3 analyze_results.py $C/full_final`, khoảng 20 phút.
- Không thể ra trùng từng số, vì planner lấy seed ngẫu nhiên và mỗi máy một tốc độ. So cột `vs RRT-Connect`, không so ms. Ví dụ, tỉ lệ thời gian RRT-FConnect ÷ RRT-Connect của nhóm là 1.11 / 0.74 / 0.63 / 0.92 (Simple / General / Highly / Narrow); lần chạy khác có thể lệch tới khoảng ±20%. Số vòng lặp lệch tới khoảng ±15%; độ dài đường đi và số node chỉ lệch vài phần trăm.

## Lỗi thường gặp

| Lỗi | Sửa |
|---|---|
| `CMake Error ... Parse error` | Đang build trong `/mnt/c`, clone lại vào `~` |
| `no map rcved yet` mãi | `map_file` sai đường dẫn |
| Không có Goal3DTool | Mở RViz trước khi `source devel/setup.bash` |
| `unrecognized arguments` / `FAILED` | Code cũ: `git pull` rồi `catkin_make` |
| `Couldn't find executable dataset_map_generator.py` | Chạy lại `chmod +x` (mục 2) |
| Không có cửa sổ RViz | `wsl --update` trong PowerShell |
| Nhiều lần chạy bị cộng thêm khoảng 1,4 s, thời gian dao động bất thường | Đồng hồ WSL bị chỉnh nhảy: làm mục 2b |
