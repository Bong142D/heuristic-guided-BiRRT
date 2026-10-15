# Hướng dẫn đọc code và reproduce RRT-FConnect

> **Đồ án:** Hoạch định đường đi đa robot dựa vào lấy mẫu có định hướng
> **Paper chính:** *RRT-FConnect: Fast Bidirectional Path Planning in Cluttered and Narrow-Passage Environments*
> (Truong Quynh Chi, Vu Quoc Huy, Le Hong Trang, HCMUT, PRICAI 2026)
> **Mục tiêu trước mắt:** hiểu code và reproduce phương pháp Single Robot. Chưa sửa code, chưa làm Multi-Robot.

Tài liệu này tổng hợp kết quả đọc toàn bộ repository, đối chiếu với paper. Mọi nhận xét về thuật toán đều
dựa trên source code thực tế. Chỗ nào chỉ là suy đoán thì được ghi rõ là **(chưa xác nhận)**.

---

## Mục lục

0. [Tóm tắt nhanh](#0-tóm-tắt-nhanh)
1. [Tóm tắt paper RRT-FConnect](#1-tóm-tắt-paper-rrt-fconnect)
2. [Cấu trúc repository](#2-cấu-trúc-repository)
3. [Luồng thực thi](#3-luồng-thực-thi)
4. [Thuật toán lõi trong code](#4-thuật-toán-lõi-trong-code)
5. [Đối chiếu Paper ↔ Code](#5-đối-chiếu-paper--code)
6. [Kế hoạch reproduce](#6-kế-hoạch-reproduce)
7. [Single Robot → Multi-Robot](#7-single-robot--multi-robot)
8. [Thuật ngữ](#8-thuật-ngữ)
9. [Câu hỏi cần hỏi GVHD](#9-câu-hỏi-cần-hỏi-gvhd)

---

## 0. Tóm tắt nhanh

| Câu hỏi | Trả lời |
|---|---|
| Phương pháp của paper nằm ở đâu? | Class `SOF_BRRT_Simple_Case3` trong [src/path_finder/include/path_finder/SOF_brrt_simple_case3.h](../src/path_finder/include/path_finder/SOF_brrt_simple_case3.h) |
| Vì sao tên là "SOF"? | Bước lấy mẫu theo tia/sector được phát triển từ **SOF-RRT\*** (*spatial offset sampling*, tài liệu [14] của paper) |
| Chương trình chạy từ đâu? | [src/path_finder/src/test_planner.cpp](../src/path_finder/src/test_planner.cpp), node ROS `path_finder` |
| Môi trường cần có | Ubuntu 20.04 + ROS1 Noetic (trên Windows dùng WSL2 hoặc Docker) |
| Code có khớp paper không? | Khớp ở các công thức chính (Eq. 1, 3, 4 và APF). Có **một số khác biệt** cần lưu ý, xem [mục 5.3](#53-những-điểm-code-khác-paper) |
| Chạy được ngay trên máy mình không? | **Không.** Code có nhiều đường dẫn cứng `/home/huy/...`, và một trong số đó làm chương trình crash. Xem [mục 6.2](#62-các-lỗi-chặn-khi-chạy-trên-máy-khác) |

---

## 1. Tóm tắt paper RRT-FConnect

### 1.1 Bài toán

Tìm **nhanh** một đường đi khả thi, không va chạm, từ `q_start` đến `q_goal`, trong môi trường **dày đặc vật cản**
hoặc có **lối đi hẹp**. Paper ưu tiên tốc độ có lời giải đầu tiên, không tối ưu dần như RRT\*,
và **không dùng rewiring**.

### 1.2 Nền tảng: RRT-Connect

Có hai cây: cây **S** mọc từ start và cây **T** mọc từ goal. Mỗi vòng lặp gồm:

- **EXTEND:** cây đang hoạt động tiến thêm một bước ε về phía mẫu ngẫu nhiên.
- **CONNECT:** cây kia đi tham lam (greedy), từng bước ε, về phía node vừa thêm, cho tới khi chạm nhau hoặc gặp vật cản.
- **SWAP:** đổi vai hai cây.

### 1.3 Ba đóng góp của RRT-FConnect

| # | Thành phần | Ý tưởng | Công thức |
|---|---|---|---|
| 1 | **Multi-target heuristic** (lấy cảm hứng từ EBi-RRT [1]) | Chọn một **cặp node dẫn đường** (`q_guide^S`, `q_guide^T`), mỗi node thuộc một cây, sao cho dễ nối nhất | Eq. 1, 2 |
| 1b | **Xác suất bias thích nghi ρ** | Ban đầu ưu tiên khám phá ngẫu nhiên. Khi hai cây lại gần nhau thì chuyển dần sang mở rộng có hướng | Eq. 3 |
| 2 | **LEA sampling** (Local Environment-Aware, dựa trên SOF-RRT\* [14]) | Bắn N tia quanh `q_guide^S`. Ưu tiên chọn sector thoáng và hướng về `q_guide^T` | Eq. 4 |
| 3 | **APF steering** | Mở rộng nhánh có lực hút về đích. Nếu đụng vật cản thì trượt **tiếp tuyến** dọc theo vật cản | Mục 3.3 |

### 1.4 Các công thức

**Eq. 1, heuristic của một cặp node:**

```
h(q_i^S, q_j^T) = d(q_i^S, q_j^T) + α · ( d(q_i^S, q_0^T) + d(q_0^S, q_j^T) )
```

trong đó `q_0^S` là start và `q_0^T` là goal.

**Eq. 2, chọn cặp dẫn đường:**

```
{q_guide^S, q_guide^T} = argmin h(q_i^S, q_j^T)
```

**Eq. 3, xác suất bias:**

```
ρ = ρ_max · ( 1 − exp( −μ · (h(start, goal) − h(guide pair)) / h(start, goal) ) )
```

**Eq. 4, trọng số của sector i:**

```
w_i = (r_i / R_max)^WG · cos(θ_i / 2)
W_i = w_i / Σ_j w_j
```

trong đó `r_i` là độ dài tia i và `θ_i` là góc lệch giữa tia i với hướng tới `q_guide^T`.

**APF, mục 3.3:**

```
q'_new = q_guide^S + ε · (v_r + w_t · v_t)
  w_t = sigmoid( 5 · d(q_guide^S, q_guide^T) / maxDist )            ∈ (0.5, 1)

Nếu q'_new nằm trong vật cản:
  q_new = q'_new + ε · w_o · v_o
  v_o = vector (q'_new − q_guide^S) xoay 90°, chọn chiều gần v_t hơn
  w_o = sigmoid( 5 · d(q_guide^S, q'_new) / (2ε) )                   ∈ (0.5, 1)
```

### 1.5 Algorithm 1, viết lại cho dễ đọc

```
S ← {start}, T ← {goal};  guide ← (start, goal);  ρ ← UPDATE(guide)
for k = 1..K:
    if rand() < ρ:                          # nhánh heuristic
        q_rand ← LEA_SAMPLING(guide)
        q_new  ← APF_EXTEND(S, từ q_guide^S, về q_rand)
        if thành công và CONNECT(T, từ q_guide^T, tới q_new): return PATH
    else:                                   # nhánh khám phá (RRT-Connect gốc)
        q_rand ← uniform();  q_near ← nearest(S, q_rand)
        q_new  ← EXTEND(S, q_near, q_rand)
        if thành công và CONNECT(T, từ nearest(T, q_new), tới q_new): return PATH
    SWAP(S, T);  guide ← FIND_GUIDE_PAIR(S, T);  ρ ← UPDATE(guide)
return Failure
```

### 1.6 Thiết lập thí nghiệm (Table 1)

| Nhóm | Tham số | Giá trị |
|---|---|---|
| Bản đồ | Kích thước | 200 × 200 |
| | Bước ε | 0.5 |
| Heuristic | α | 0.2 |
| | ρ_max | 0.8 |
| | μ | 1 |
| LEA | Số sector N | 8 |
| | Weight grade WG | 1 |
| | R_max | 5 |

- **Môi trường:** blobs với mật độ 10%, 20%, 40%, cộng thêm môi trường narrow passage.
- **Quy mô:** 5 bản đồ × 20 cặp start/goal × **10 lần chạy** = 1000 test case cho mỗi môi trường.
- **Baseline:** RRT-Connect, GB-RRT-Connect, EBi-RRT. Thêm Informed BiRRT\* để so độ dài đường đi theo thời gian (Fig. 8).
- **Metric:** path length (m), số node, số vòng lặp, search time (**ms**), success rate.

### 1.7 Kết quả cần đạt (Table 2, dòng RRT-FConnect)

| Môi trường | Path length | Nodes | Iterations | Time (ms) | Success |
|---|---|---|---|---|---|
| Simple (10%) | 185.1 | 483.7 | 264.7 | 1.47 | 100% |
| General (20%) | 204.2 | 836.7 | 1426.4 | 7.51 | 100% |
| Highly (40%) | 246.8 | 2624.9 | 13236.8 | 75.10 | 95% |
| Narrow passage | 280.0 | 1178.7 | 4577.7 | 16.16 | 100% |

---

## 2. Cấu trúc repository

Repository là một **workspace ROS1 (catkin)**. Thư mục `src/` chứa 6 package:

| Package | Vai trò | Mức quan trọng |
|---|---|---|
| [src/path_finder/](../src/path_finder/) | Toàn bộ thuật toán (header-only) và node chạy thử | ⭐⭐⭐ Lõi |
| [src/occ_grid/](../src/occ_grid/) | Occupancy grid: nhận point cloud, kiểm tra va chạm | ⭐⭐ Cần thiết |
| [src/map_gen/map_generator/](../src/map_gen/map_generator/) | Sinh bản đồ: `random_forest` (C++) và `dataset_map_generator.py` (blobs/narrow/flappy/rooms/…) | ⭐⭐ Tạo dataset |
| [src/map_gen/mockamap/](../src/map_gen/mockamap/) | Sinh bản đồ perlin/maze | Phụ, không dùng mặc định |
| [src/rviz_plugins/](../src/rviz_plugins/) | Tool "Goal3DTool" để click chọn goal trong RViz | Giao diện |
| [src/self_msgs_and_srvs/](../src/self_msgs_and_srvs/) | Message và service (`GlbObsRcv`) | Hạ tầng |

### 2.1 Các file ở thư mục gốc

| File | Tác dụng |
|---|---|
| [generate_dataset.py](../generate_dataset.py) | Sinh dataset: mỗi bản đồ một file `.pcd` và một file `map_i_scenarios.json` chứa 20 cặp start/goal |
| [run_dataset_experiments.py](../run_dataset_experiments.py) | Chạy batch trên dataset cho mọi file cấu hình, rồi gộp kết quả JSON |
| [experiment.ipynb](../experiment.ipynb) | Sinh file cấu hình tham số và phân tích kết quả (pandas) |
| [brrt_input.json](../brrt_input.json) | File tham số mẫu (bộ cũ, **không** phải bộ của Table 1) |
| [build.sh](../build.sh), [rviz.sh](../rviz.sh), [test.sh](../test.sh) | Build, mở RViz, chạy planner |

### 2.2 Các planner trong [include/path_finder/](../src/path_finder/include/path_finder/)

| File | Class | Ứng với trong paper |
|---|---|---|
| `SOF_brrt_simple_case3.h` | `SOF_BRRT_Simple_Case3` | ⭐ **RRT-FConnect** |
| `SOF_brrt_simple_case1.h`, `_case2.h` | `SOF_BRRT_Simple_Case1/2` | Các bản thử nghiệm trước của RRT-FConnect |
| `brrt.h` | `BRRT` | **RRT-Connect** |
| `goal_bias_brrt.h` | `GoalBiasBRRT` | **GB-RRT-Connect** (goal bias 0.3, viết cứng) |
| `brrt_simple_case1.h`, `brrt_simple_case2.h` | `BRRT_Simple_Case1/2` | Có thể là **EBi-RRT** (chưa xác nhận). Cả hai có heuristic cặp node nhưng không có ray và APF |
| `brrt_star.h` | `BRRTStar` (bật `use_informed_sampling`) | **Informed BiRRT\*** (Fig. 8) |
| `rrt.h`, `rrt_star.h`, `rrt_sharp.h` | RRT, RRT\*, RRT# | Code gốc ZJU, paper không dùng |
| `brrt_sample_gravity.h`, `brrt_nearest_gravity.h` | Cả hai đều tên `BRRT_Optimize` | Bản thử nghiệm cũ, đang bị comment |

Các file hạ tầng:

| File | Nội dung |
|---|---|
| [node.h](../src/path_finder/include/path_finder/node.h) | `TreeNode` và **`HeuristicCache`** (min-heap chứa các cặp node) |
| [sampler.h](../src/path_finder/include/path_finder/sampler.h) | `BiasSampler`: lấy mẫu đều, informed, GUILD |
| [kdtree.h](../src/path_finder/include/path_finder/kdtree.h) / [kdtree.c](../src/path_finder/src/kdtree.c) | Thư viện kd-tree (C) để tìm node gần nhất |
| [testcase.h](../src/path_finder/include/path_finder/testcase.h) | Đọc file JSON tham số, ghi file JSON kết quả |
| [custom_logger.h](../src/path_finder/include/path_finder/custom_logger.h) | Ghi log ra file |

---

## 3. Luồng thực thi

```
roslaunch path_finder test_planners.launch
 ├─ pcd_to_pointcloud: đọc map_file (.pcd) → publish /random_forest/all_map
 │    (nếu map_file rỗng: map.launch → random_forest sinh map ngẫu nhiên)
 └─ node path_finder  (test_planner.cpp)
     ├─ Constructor: tạo OccMap và các planner, đọc ROS param,
     │   đọc file JSON tham số (BRRTExperimentMultiAlgo)
     ├─ OccMap nhận point cloud → dựng occupancy grid
     └─ Timer 1 giây: executionCallback, chờ đến khi có map
          ├─ experiment = false → chế độ tương tác
          │     click goal trong RViz → goalCallback → chạy các planner có cờ bật
          │     (sau mỗi lần chạy: start ← goal cũ)
          └─ experiment = true  → experiment_test
                ├─ generate_mode: sinh 20 cặp start/goal → *_scenarios.json
                ├─ có scenarios_file: chạy lần lượt từng cặp
                └─ còn lại: 100 goal ngẫu nhiên, hoặc 1 cặp cố định
                     → run_planners_and_store → save_json
```

Vị trí trong code:

| Bước | Vị trí |
|---|---|
| `main` | [test_planner.cpp:830](../src/path_finder/src/test_planner.cpp#L830) |
| Đọc ROS param | [test_planner.cpp:278-301](../src/path_finder/src/test_planner.cpp#L278-L301) |
| Chế độ tương tác | [goalCallback, test_planner.cpp:334](../src/path_finder/src/test_planner.cpp#L334) |
| Chờ có map | [executionCallback, test_planner.cpp:534](../src/path_finder/src/test_planner.cpp#L534) |
| Chế độ thí nghiệm | [experiment_test, test_planner.cpp:743](../src/path_finder/src/test_planner.cpp#L743) |
| Chạy planner và lưu kết quả | [run_planners_and_store, test_planner.cpp:554](../src/path_finder/src/test_planner.cpp#L554) |

> ⚠️ Trong chế độ thí nghiệm, `run_planners_and_store` hiện **chỉ chạy RRT-FConnect** (`SOF_BRRT_Simple_Case3`).
> Các baseline BRRT, GoalBiasBRRT, Case1/2 đang bị comment ở
> [test_planner.cpp:610-716](../src/path_finder/src/test_planner.cpp#L610-L716).
> Muốn tái tạo Table 2 thì phải bật lại.

### 3.1 Cách tham số đi từ file JSON vào planner

```
file JSON (input_param)
  └─ testcase.h: BRRTInput {p1, u_p, alpha, beta, gamma, epsilon, p2, n_rays, weight_grade, ray_step_size}
       └─ test_planner.cpp: sof_brrt_simple_case3_ptr_->set_heuristic_param(...)
            └─ SOF_BRRT_Simple_Case3:
                 steer_length_ = epsilon
                 R_max         = 20 · epsilon      ← tính tự động, không có tham số riêng
```

Các tham số khác lấy từ launch file, ví dụ `BRRT_Optimize/max_iteration` = 100000 và
`BRRT/max_tree_node_nums` = 200000.

---

## 4. Thuật toán lõi trong code

File: [SOF_brrt_simple_case3.h](../src/path_finder/include/path_finder/SOF_brrt_simple_case3.h)

### 4.1 Các hàm chính

| Hàm | Dòng | Làm gì |
|---|---|---|
| `plan(s, g)` | [79](../src/path_finder/include/path_finder/SOF_brrt_simple_case3.h#L79) | Reset, tạo node start/goal, gọi `brrt_optimize` |
| `computeH` | [427](../src/path_finder/include/path_finder/SOF_brrt_simple_case3.h#L427) | Heuristic của cặp node (Eq. 1) |
| `update_cache_nearest_heuristic` | [447](../src/path_finder/include/path_finder/SOF_brrt_simple_case3.h#L447) | Node mới được ghép cặp với **30 node gần nhất** của cây kia, rồi đẩy vào cache |
| `computePbias` | [550](../src/path_finder/include/path_finder/SOF_brrt_simple_case3.h#L550) | Tính ρ (Eq. 3) |
| `shootRays` | [476](../src/path_finder/include/path_finder/SOF_brrt_simple_case3.h#L476) | Bắn N tia trên mặt phẳng XY, mỗi tia dừng khi gặp vật cản hoặc đạt R_max |
| `heuristicSteer` | [350](../src/path_finder/include/path_finder/SOF_brrt_simple_case3.h#L350) | APF steering (mục 3.3 của paper) |
| `vanillaSteer` | [340](../src/path_finder/include/path_finder/SOF_brrt_simple_case3.h#L340) | EXTEND thường của RRT-Connect |
| `greedySteer` | [403](../src/path_finder/include/path_finder/SOF_brrt_simple_case3.h#L403) | CONNECT tham lam |
| `brrt_optimize` | [568](../src/path_finder/include/path_finder/SOF_brrt_simple_case3.h#L568) | **Vòng lặp chính** |

### 4.2 Vòng lặp chính, từng bước

Mỗi vòng lặp rơi vào **một trong ba chế độ**:

```
┌─ Đang trong exploration (còn > 0 vòng)? ──► (1) EXPLORATION
│
└─ Không → lấy cặp có h nhỏ nhất từ cache (popMinByTree)
            ├─ Nếu 20 lần liên tiếp h không giảm → bật 50 vòng exploration, swap, sang vòng sau
            ├─ rand < ρ ──► (2) HEURISTIC
            └─ còn lại   ──► (3) UNIFORM
```

**(1) EXPLORATION**, [dòng 622-653](../src/path_finder/include/path_finder/SOF_brrt_simple_case3.h#L622-L653). *Không có trong paper.*

- Lấy mẫu đều, rồi tìm node gần nhất trong cây A.
- Gọi `heuristicSteer` với target là **chính node gần nhất**, nên v_t = 0. Kết quả là chỉ còn phần trượt tiếp tuyến khi gặp vật cản.
- Cây B nối từ node gần nhất của B.

**(2) HEURISTIC**, [dòng 676-745](../src/path_finder/include/path_finder/SOF_brrt_simple_case3.h#L676-L745). Ứng với dòng 5–11 của Algorithm 1.

1. `x_temp = q_guide^S`. Tính hướng `v = q_guide^T − q_guide^S`, đặt thành phần z bằng 0.
2. Bắn `n_rays` tia đều 360°, mỗi tia dài tối đa `R_max = 20ε`, bước dò `ray_step_size`.
3. Tính trọng số `w_i = len_i^WG · cos(Δθ_i / 2)` rồi chọn sector theo roulette wheel.
4. Lấy mẫu đều trong hình quạt: `r = √U · len_i` và θ ngẫu nhiên trong khoảng góc của sector. Đây là `q_rand`.
5. `q_new = heuristicSteer(q_guide^S, q_rand, ε, target = q_guide^T)` (APF).
6. Cây B CONNECT **từ `q_guide^T`**.

**(3) UNIFORM**, [dòng 746-779](../src/path_finder/include/path_finder/SOF_brrt_simple_case3.h#L746-L779). Ứng với dòng 12–21 của Algorithm 1: lấy mẫu đều, tìm nearest, `vanillaSteer`, rồi cây B CONNECT từ nearest.

**Phần chung sau cả ba chế độ**, [dòng 782-839](../src/path_finder/include/path_finder/SOF_brrt_simple_case3.h#L782-L839):

- Thêm `q_new` vào cây A và kd-tree A, rồi cập nhật cache cho node mới.
- `greedySteer` từ node của cây B về `q_new`. Các node trung gian được thêm vào cây B và cache.
- Nếu nối được thì ghép đường đi (`fillPath`) và **dừng ngay**, vì chỉ cần lời giải đầu tiên.
- Nếu không nối được thì swap hai cây.
- Khi một bước thất bại (mẫu nằm trong vật cản, steer bị chặn…) thì cũng swap và `continue`.

**Điều kiện dừng:** hết `max_iteration` (100000), hoặc hết node pool (200000 node).
Tham số `search_time` được đọc vào nhưng **không dùng**, nên Case3 không có giới hạn thời gian.

### 4.3 `HeuristicCache` hoạt động thế nào

File [node.h](../src/path_finder/include/path_finder/node.h).

- **Lưu trữ:** `unordered_map` lưu (cặp node, cây) → h, kèm một `priority_queue` dạng min-heap.
- **Khi thêm node mới:** ghép với 30 node gần nhất của cây kia (kd-tree), tính h rồi đưa vào heap.
- **`popMinByTree(A, B)`:** lấy cặp có h nhỏ nhất thuộc đúng cặp cây (A, B), rồi **xoá cặp đó khỏi cache**.
  Vì vậy mỗi cặp chỉ được dùng làm guide pair **một lần**.

### 4.4 Ví dụ trực quan: một vòng HEURISTIC

```
        q_guide^T ●
                 ╱
   tia dài,     ╱ θ nhỏ → trọng số lớn → hay được chọn
   hướng về    ╱
   đích       ╱   ▓▓▓ vật cản → tia ngắn → trọng số nhỏ
             ╱
 q_guide^S ●──── q_rand lấy trong hình quạt đã chọn
             ╲
              ╲ APF: q'_new = q_guide^S + ε·(v_r + w_t·v_t)
               ╲     nếu q'_new đụng vật cản: trượt thêm ε·w_o·v_o theo tiếp tuyến
```

---

## 5. Đối chiếu Paper ↔ Code

### 5.1 Bảng ánh xạ tham số

| Paper | Code / JSON | Giá trị Table 1 | Ghi chú |
|---|---|---|---|
| ε (step size) | `epsilon` → `steer_length_` | 0.5 | |
| α (Eq. 1) | `beta` **và** `gamma`, với `alpha` = 1 | α = 0.2 → `alpha=1, beta=0.2, gamma=0.2` | Code có 3 hệ số riêng. Notebook cũng đặt `alpha=1, gamma=beta` |
| ρ_max | `p1` | 0.8 | |
| μ | `u_p` | 1 | `u_p ≈ 0` thì ρ = p1 không đổi |
| N | `n_rays` | 8 | |
| WG | `weight_grade` | 1 | |
| R_max | `20 · epsilon` (tính tự động) | **5** | ⚠️ Với ε = 0.5, code cho R_max = **10** |
| — | `ray_step_size` | (paper không nêu) | Bước dò tia. Notebook đặt bằng ε |
| K (số vòng lặp tối đa) | `BRRT_Optimize/max_iteration` | (paper không nêu) | Launch đặt 100000 |
| Kích thước bản đồ | `map_size_x/y` trong launch | 200 × 200 | Độ phân giải 0.5 |

File JSON tương ứng với Table 1 (gợi ý, **chưa chạy thử**):

```json
{
  "trial": 1, "environment": "paper_table1",
  "epsilon": 0.5, "p1": 0.8, "u_p": 1,
  "alpha": 1, "beta": 0.2, "gamma": 0.2,
  "n_rays": 8, "weight_grade": 1, "ray_step_size": 0.5
}
```

### 5.2 Ánh xạ từng thành phần

| Thành phần trong paper | Code | Khớp? |
|---|---|---|
| Eq. 1, heuristic | `computeH` L427 | ✅ (khi `alpha=1, beta=gamma=α`) |
| Eq. 2, argmin trên mọi cặp | `HeuristicCache` + 30 nearest | ⚠️ Xấp xỉ, xem 5.3 |
| Eq. 3, ρ | `computePbias` L550: `p1·(1−exp(−u_p·(h0−h)/h0))` | ✅ |
| Eq. 4, trọng số sector | L687-694: `len^WG · cos(Δθ/2)` | ✅ (thiếu chia R_max, nhưng sau chuẩn hoá thì tương đương) |
| APF, w_t | `phi = ε·sigmoid(5·d/maxDist)`, với maxDist = đường chéo bản đồ | ✅ |
| APF, v_o và w_o | `T_v`, `eta = ε·sigmoid(5·d/(2ε))` | ✅ |
| Kiểm tra va chạm đoạn q_guide → q_new | L740 | ✅ |
| Alg. 1, nhánh heuristic (dòng 5–11) | Chế độ (2) | ✅ |
| Alg. 1, nhánh khám phá (dòng 12–21) | Chế độ (3) | ✅ |
| Alg. 1, SWAP và cập nhật guide/ρ (dòng 22–24) | Swap cuối vòng; pop guide đầu vòng sau | ✅ (tương đương) |
| Trả về ngay khi nối được | `break` L835 | ✅ |
| Cấu trúc lưu cặp (mục 4) | `priority_queue`, không giới hạn số cặp | ⚠️ Paper nói multiset 100 cặp |

### 5.3 Những điểm code khác paper

Cần thảo luận với GVHD hoặc tác giả, vì những điểm này ảnh hưởng đến kết quả reproduce:

1. **Chế độ exploration khi bế tắc** ([L657-674](../src/path_finder/include/path_finder/SOF_brrt_simple_case3.h#L657-L674)).
   Nếu 20 lần liên tiếp h không giảm, code chuyển sang 50 vòng exploration. Hai con số 20 và 50 viết cứng.
   **Algorithm 1 không có bước này.**
2. **Kho cặp node (candidate pool).**
   - Paper (mục 4): giữ **100 cặp tốt nhất** trong `multiset`, có pop-min và pop-max.
   - Code: heap không giới hạn kích thước; mỗi node mới chỉ ghép với **30 node gần nhất**.
   - Code **xoá** cặp sau khi dùng, nên vòng sau dùng cặp tốt nhì. Paper thì lấy argmin mỗi vòng.
3. **R_max.** Paper dùng 5, còn code tính `20·ε` = 10 khi ε = 0.5.
4. **`heuristicSteer` trả về thẳng `q_rand` khi `‖q_rand − q_guide‖ ≤ ε`** (không áp APF). Paper không đề cập trường hợp này.
5. **Cách tính chi phí cạnh** ([L783, L790](../src/path_finder/include/path_finder/SOF_brrt_simple_case3.h#L783)).
   Mỗi cạnh luôn được cộng đúng `ε`. Nhưng bước APF có thể dài khoảng 1.5–2ε, còn `vanillaSteer` có thể ngắn hơn ε.
   Vì vậy `path_length` trong file kết quả **không phải độ dài hình học thật**.
   Cần biết paper tính path length theo cách nào.
6. **Đơn vị thời gian.** Code ghi `search_time` theo **giây** (`toSec()`), còn paper báo theo **ms**.
7. **Số lần lặp lại mỗi truy vấn.** Paper chạy 10 lần cho mỗi cặp start/goal.
   [run_dataset_experiments.py](../run_dataset_experiments.py) chỉ chạy **1 lần** mỗi cặp với mỗi file cấu hình.
8. **Không gian 2D hay 3D.** Tia và vector tiếp tuyến chỉ nằm trên mặt phẳng XY. Nhưng lấy mẫu đều (`sampler_`) có lấy cả trục z.
   Bản đồ dày 1.5 m, độ phân giải 0.5, tức 3 lớp theo z.
9. **Biến thể cũ dùng công thức khác.** `BRRT_Simple_Case1/2` dùng `ρ = p1·exp(−r)`, tức chiều ngược với Eq. 3.
   Chỉ Case3 mới khớp paper.

---

## 6. Kế hoạch reproduce

### 6.1 Môi trường

- **Hệ điều hành:** Ubuntu 20.04 + ROS Noetic (giống máy tác giả trong mục 4 của paper). Trên Windows dùng **WSL2** hoặc Docker `osrf/ros:noetic-desktop-full`.
- **Gói cần cài:**
  ```bash
  sudo apt install ros-noetic-desktop-full ros-noetic-pcl-ros ros-noetic-interactive-markers \
                   libeigen3-dev nlohmann-json3-dev
  pip install -r requirements.txt   # pandas, matplotlib, itables, openpyxl (cho notebook)
  ```
  Gói `nlohmann-json3-dev` được `testcase.h` dùng nhưng **không khai báo** trong CMakeLists hay package.xml, nên phải tự cài.

### 6.2 Các lỗi chặn khi chạy trên máy khác

Danh sách dưới đây **chưa sửa**, chỉ liệt kê để biết trước.

| Mức | Vấn đề | Vị trí | Hậu quả |
|---|---|---|---|
| 🔴 | `CustomLogger("/home/huy/rrt/logger_time.txt")` | [SOF_brrt_simple_case3.h:571](../src/path_finder/include/path_finder/SOF_brrt_simple_case3.h#L571), cùng `brrt_simple_case1.h` | **Crash** (throw) nếu thư mục không tồn tại |
| 🔴 | `input_param` mặc định trỏ vào `/home/huy/...` | [test_planner.cpp:297](../src/path_finder/src/test_planner.cpp#L297), launch | Không mở được file thì mọi tham số bằng 0, ε = 0, planner đứng yên |
| 🟠 | `map_file`, `output_result` trỏ vào `/home/huy/...` | [test_planners.launch](../src/path_finder/launch/test_planners.launch) | Không có map, không ghi được kết quả |
| 🟠 | Đường dẫn cứng trong script | [generate_dataset.py:11](../generate_dataset.py#L11), [run_dataset_experiments.py](../run_dataset_experiments.py), notebook | Phải sửa trước khi chạy |
| 🟠 | Baseline bị comment trong chế độ thí nghiệm | [test_planner.cpp:610-716](../src/path_finder/src/test_planner.cpp#L610-L716) | Không có dữ liệu để so sánh |
| 🟡 | kd-tree không được giải phóng (thiếu `kd_free`) | `brrt_optimize` | Rò rỉ bộ nhớ khi chạy batch dài |
| 🟡 | Seed lấy từ `random_device` | Mọi planner | Kết quả khác nhau mỗi lần chạy, cần lấy trung bình |

### 6.3 Các bước

**Bước 1: Build**

```bash
cd ~/heuristic-guided-BiRRT
catkin_make            # hoặc: bash build.sh
source devel/setup.bash
```

**Bước 2: Chạy tương tác để hiểu thuật toán** (chưa cần dataset)

- Terminal 1: `roslaunch path_finder rviz.launch`
- Terminal 2: `roslaunch path_finder test_planners.launch map_file:=""` (dùng bản đồ random forest)
- Trong RViz chọn tool "Goal3DTool", bấm phím `g`, rồi click để đặt goal.
- Chế độ chạy từng bước: thêm `sof_brrt_step_mode:=true` (cần đổi `value` thành `default` trong launch để truyền được từ dòng lệnh).
  RViz sẽ hiện tia, sector được chọn, `q_rand` và node mới.
  Bấm khối xanh "NEXT STEP" hoặc `rostopic pub /sof_brrt/step std_msgs/Empty` để đi tiếp.

**Bước 3: Sinh dataset giống paper**

- [generate_dataset.py](../generate_dataset.py) sinh 4 loại bản đồ × 3 mức (10/20/40%) × 5 bản đồ.
  Mỗi bản đồ có 20 cặp start/goal cách nhau ít nhất 150 m (`gen_min_dist`).
- Môi trường dùng cho paper: `blobs_10`, `blobs_20`, `blobs_40`, `narrow_*`.

**Bước 4: Tạo file cấu hình theo Table 1**, xem [mục 5.1](#51-bảng-ánh-xạ-tham-số).

**Bước 5: Chạy thí nghiệm**

- Bật lại các baseline trong `run_planners_and_store`:
  BRRT ↔ RRT-Connect, GoalBiasBRRT ↔ GB-RRT-Connect, và EBi-RRT (cần xác nhận class nào).
- Lặp **10 lần** cho mỗi truy vấn.
- Chạy [run_dataset_experiments.py](../run_dataset_experiments.py).

**Bước 6: Phân tích**

- Đổi thời gian sang ms.
- Tính lại path length từ `getPath()` nếu cần.
- Tính trung bình và success rate, rồi so với [Table 2](#17-kết-quả-cần-đạt-table-2-dòng-rrt-fconnect).
- Có thể vẽ boxplot như Fig. 7.

**Bước 7 (tuỳ chọn): Fig. 8.** Chạy `BRRTStar` với `use_informed_sampling=true` và `search_time` đủ dài.
Dùng `getSolutions()` để lấy các cặp (cost, time) qua thời gian.

### 6.4 Checklist

- [ ] Build thành công
- [ ] Chạy được chế độ tương tác, thấy cây và đường đi trong RViz
- [ ] Chạy step mode, hiểu được tia, sector và APF
- [ ] Sinh dataset blobs 10/20/40 và narrow
- [ ] Tạo file cấu hình Table 1, chốt cách xử lý R_max
- [ ] Bật baseline, lặp 10 lần mỗi truy vấn
- [ ] Ra được bảng giống Table 2, ghi lại chỗ lệch và lý do

---

## 7. Single Robot → Multi-Robot

*Phần này chỉ phân tích, chưa implement.*

Code hiện tại giả định **một robot, môi trường tĩnh, đường đi hình học không có thời gian**.

### 7.1 Những thành phần có khả năng phải thay đổi

| Thành phần | Hiện tại | Vấn đề khi có nhiều robot |
|---|---|---|
| **Node** (`TreeNode`) | Chỉ có `Vector3d x` | Cần thêm thời gian hoặc chỉ số bước để biết hai robot có ở **cùng chỗ cùng lúc** không. Lập kế hoạch tập trung thì trạng thái có 3N chiều |
| **Kiểm tra va chạm** (`OccMap::isStateValid/isSegmentValid`) | Chỉ với vật cản tĩnh; robot coi là một điểm, bản đồ không inflate | Cần thêm va chạm robot–robot, phụ thuộc thời gian, có bán kính robot |
| **Class planner** | Một start/goal, 2 kd-tree, `nodes_pool_` 200k node; trạng thái lưu trong biến thành viên | N robot cần N cặp cây, tốn bộ nhớ, khó chạy song song |
| **Heuristic** (Eq. 1, 3) | Chỉ dùng S và G của một robot | Tính theo từng robot. Có thể thêm phạt khi gần đường của robot khác |
| **LEA** (`shootRays`) | Tia chỉ dừng ở vật cản tĩnh | Có thể cần coi robot hoặc đường đi đã lên kế hoạch là vật cản |
| **APF** (`heuristicSteer`) | Chỉ trượt quanh vật cản tĩnh | Có thể thêm lực đẩy hoặc lực tiếp tuyến từ robot khác |
| **Điều phối** | Không có | Phải chọn: **tập trung** (một cây trong không gian ghép), **phân rã theo ưu tiên** (robot sau coi đường của robot trước là vật cản động), hoặc **CBS** với RRT-FConnect làm planner tầng dưới |
| **Đầu ra** | `vector<Vector3d>` | N đường đi đã tham số hoá theo thời gian (vận tốc giả định) |
| **Thí nghiệm** (`testcase.h`, scenarios JSON, `generate_mode`) | Một cặp start/goal, metric cho một robot | N cặp start/goal không trùng nhau. Metric mới: tổng chi phí, makespan, số xung đột, success rate của cả nhóm |
| **RViz / test_planner** | Click một goal | Nhập và hiển thị N robot |

### 7.2 Những thành phần có thể giữ nguyên

- `occ_grid`, vì bản đồ vẫn tĩnh.
- `kdtree.c`, `sampler.h`.
- Các script sinh bản đồ.
- Phần lõi mở rộng cây: LEA, APF, greedy CONNECT, cache. Chúng có thể dùng lại nguyên vẹn nếu RRT-FConnect đóng vai trò **planner cho từng robot**.

### 7.3 Gợi ý hướng nghiên cứu

Cách tận dụng code nhiều nhất là **phân rã theo ưu tiên** (prioritized planning):

1. Lần lượt lập kế hoạch cho từng robot bằng RRT-FConnect.
2. Đường đi đã lập của các robot trước được coi là **vật cản động** trong `isStateValid`. Lúc đó hàm này cần nhận thêm tham số thời gian.

Hướng nâng cao hơn là **CBS** (Conflict-Based Search) với RRT-FConnect làm tầng dưới.

---

## 8. Thuật ngữ

| Thuật ngữ | Nghĩa |
|---|---|
| RRT | Rapidly-exploring Random Tree: cây khám phá ngẫu nhiên |
| RRT-Connect / BRRT / BiRRT | RRT hai cây, mọc từ start và từ goal |
| EXTEND | Mở rộng cây một bước ε về phía mẫu |
| CONNECT | Cây kia đi tham lam liên tục về phía node mới |
| Guide pair | Cặp node (một ở mỗi cây) có heuristic nhỏ nhất |
| LEA | Local Environment-Aware: lấy mẫu dựa trên độ thoáng cục bộ (tia và sector) |
| SOF | Spatial Offset sampling, từ SOF-RRT\* [14] |
| APF | Artificial Potential Field: trường thế nhân tạo, gồm lực hút về đích và lực đẩy hoặc tiếp tuyến từ vật cản |
| Rewiring | Nối lại cây để tối ưu chi phí (RRT\*). RRT-FConnect **không** dùng |
| Occupancy grid | Lưới ô, mỗi ô được đánh dấu là trống hoặc có vật cản |
| kd-tree | Cấu trúc dữ liệu để tìm nhanh node gần nhất |

---

## 9. Câu hỏi cần hỏi GVHD

1. Kết quả trong paper có được chạy đúng với `SOF_BRRT_Simple_Case3` ở commit này không? **Chế độ exploration khi bế tắc** (20/50) có được dùng khi chạy thí nghiệm của paper không?
2. R_max = 5 trong paper được đặt bằng cách nào, khi code đang tính `20·ε` (bằng 10 với ε = 0.5)?
3. Paper nói kho cặp gồm 100 cặp lưu trong `multiset`, còn code dùng heap không giới hạn với 30 nearest. Có bản code nào khác không?
4. Class nào là **EBi-RRT** dùng trong Table 2: `BRRT_Simple_Case1` hay `BRRT_Simple_Case2`?
5. Path length trong Table 2 được tính theo độ dài hình học thật, hay theo `cost_from_start` (ε nhân số cạnh)?
6. Số vòng lặp tối đa K, và `gen_min_dist` (150 m) có đúng như khi làm paper không?
7. Hướng Multi-Robot mong muốn là tập trung, ưu tiên hay CBS? Robot có kích thước, động học như thế nào?
