# RRT-FConnect: code và paper

Paper: *RRT-FConnect: Fast Bidirectional Path Planning in Cluttered and Narrow-Passage Environments* (Chi, Huy, Trang, HCMUT, PRICAI 2026).
Đồ án: Hoạch định đường đi đa robot dựa vào lấy mẫu có định hướng. Giai đoạn hiện tại: reproduce bản single robot.
Lệnh chạy: [HowToRunCode.md](../HowToRunCode.md).

## 1. Tóm tắt

- Phương pháp của paper là class `SOF_BRRT_Simple_Case3` trong [SOF_brrt_simple_case3.h](../src/path_finder/include/path_finder/SOF_brrt_simple_case3.h). Tiền tố "SOF" lấy từ SOF-RRT\* (tài liệu [14] của paper).
- Entry point: [test_planner.cpp](../src/path_finder/src/test_planner.cpp), node ROS `path_finder`.
- Code khớp paper ở các công thức chính (Eq. 1, 3, 4 và APF). Các điểm lệch nằm ở [mục 5](#5-điểm-lệch-code--paper).

## 2. Paper

Nền tảng là RRT-Connect: hai cây S (mọc từ start) và T (mọc từ goal). Mỗi vòng gồm EXTEND (mọc thêm một bước ε), CONNECT (cây kia đi tham lam về phía node mới) và SWAP (đổi vai hai cây). Planner dừng ngay khi có lời giải đầu tiên và không dùng rewiring.

Ba đóng góp:

**1. Cặp node dẫn đường (guide pair) và xác suất bias**
```
h(qS_i, qT_j) = d(qS_i, qT_j) + α·( d(qS_i, qT_0) + d(qS_0, qT_j) )       (1)
guide pair    = argmin h                                                (2)
ρ = ρ_max · (1 − exp(−μ·(h(start,goal) − h(guide)) / h(start,goal)))     (3)
```
Ban đầu ρ ≈ 0 nên planner khám phá ngẫu nhiên. Khi hai cây lại gần nhau, ρ tăng dần và planner chuyển sang mở rộng có định hướng.

**2. LEA sampling:** bắn N tia quanh `qS_guide`. Chọn một sector theo trọng số, rồi lấy mẫu trong sector đó.
```
w_i = (r_i / R_max)^WG · cos(θ_i / 2)                                    (4)
```
Trong đó `r_i` là độ dài tia i (độ thoáng), `θ_i` là góc lệch giữa tia i và hướng tới `qT_guide`.

**3. APF steering**
```
q'_new = qS_guide + ε·(v_r + w_t·v_t),   w_t = sigmoid(5·d(guide)/maxDist)
nếu q'_new va chạm:  q_new = q'_new + ε·w_o·v_o
  v_o = (q'_new − qS_guide) xoay 90° về phía v_t
  w_o = sigmoid(5·d(qS_guide, q'_new)/(2ε))
```

Algorithm 1: với xác suất ρ, chạy LEA → APF_EXTEND từ `qS_guide` → CONNECT từ `qT_guide`. Còn lại, chạy RRT-Connect thường. Cuối mỗi vòng: SWAP, chọn lại guide pair, cập nhật ρ.

**Table 1 (tham số):** bản đồ 200×200, ε=0.5, α=0.2, ρ_max=0.8, μ=1, N=8, WG=1, R_max=5.

**Thí nghiệm:** môi trường blobs 10/20/40% và narrow passage. Mỗi môi trường: 5 bản đồ × 20 cặp × 10 lần = 1000 lần chạy. Baseline: RRT-Connect, GB-RRT-Connect, EBi-RRT. Fig. 8 so thêm với Informed BiRRT\*.

**Table 2, dòng RRT-FConnect:**

| Môi trường | Path length | Nodes | Iterations | Time (ms) | Success |
|---|---|---|---|---|---|
| Simple 10% | 185.1 | 483.7 | 264.7 | 1.47 | 100% |
| General 20% | 204.2 | 836.7 | 1426.4 | 7.51 | 100% |
| Highly 40% | 246.8 | 2624.9 | 13236.8 | 75.10 | 95% |
| Narrow | 280.0 | 1178.7 | 4577.7 | 16.16 | 100% |

## 3. Cấu trúc repo

Workspace ROS1 catkin, các package nằm trong `src/`:

| Package | Vai trò |
|---|---|
| `path_finder` | Các planner (header-only) và node chạy thử |
| `occ_grid` | Occupancy grid, kiểm tra va chạm |
| `map_gen/map_generator` | Sinh bản đồ: `random_forest`, `dataset_map_generator.py` |
| `map_gen/mockamap` | Bản đồ perlin/maze, không dùng |
| `rviz_plugins` | Goal3DTool |
| `self_msgs_and_srvs` | Message và service |

Planner và vai trò trong paper:

| File | Class | Trong paper |
|---|---|---|
| `SOF_brrt_simple_case3.h` | `SOF_BRRT_Simple_Case3` | RRT-FConnect |
| `brrt.h` | `BRRT` | RRT-Connect |
| `goal_bias_brrt.h` | `GoalBiasBRRT` | GB-RRT-Connect (bias 0.3, viết cứng) |
| `brrt_simple_case1.h` | `BRRT_Simple_Case1` | EBi-RRT (chưa xác nhận) |
| `brrt_star.h` | `BRRTStar` + informed sampling | Informed BiRRT\* |
| `SOF_brrt_simple_case1/2.h`, `brrt_simple_case2.h`, `*_gravity.h` | | Các bản thử nghiệm cũ |
| `rrt*.h` | RRT, RRT\*, RRT# | Không dùng |

Hạ tầng: `node.h` (`TreeNode`, `HeuristicCache`), `sampler.h`, `kdtree.c`, `testcase.h` (đọc và ghi JSON).

## 4. Luồng chạy và thuật toán

```
test_planners.launch
 ├─ map_file rỗng → random_forest  |  map_file = .pcd → pcd_to_pointcloud
 └─ path_finder: OccMap nhận bản đồ → chờ có map
      ├─ experiment=false: chờ goal từ RViz (goalCallback)
      └─ experiment=true : đọc scenarios_file, mỗi cặp × trials_per_query
                           → run_planners_and_store → JSON
```

Vòng lặp chính `brrt_optimize` trong Case3 có 3 chế độ:

1. **Exploration** (L622): nếu 20 lần liên tiếp h không giảm thì bật 50 vòng exploration: lấy mẫu đều, steer có tránh vật cản nhưng không hút về đích. *Paper không có chế độ này.*
2. **Heuristic** (L676, khi `rand < ρ`): lấy guide pair từ cache → `shootRays` → chọn sector → lấy mẫu → `heuristicSteer` (APF) từ `qS_guide` → CONNECT từ `qT_guide`.
3. **Uniform** (L746): RRT-Connect thường.

Phần chung: thêm node vào cây và cập nhật cache (30 node gần nhất của cây kia) → `greedySteer` → nối được thì dừng, không thì swap. Dừng khi hết `max_iteration` (100000) hoặc hết node pool (200000). Không có giới hạn thời gian.

| Hàm | Dòng | Paper |
|---|---|---|
| `computeH` | 427 | Eq. 1 |
| `update_cache_nearest_heuristic` | 447 | Eq. 2 (xấp xỉ) |
| `computePbias` | 550 | Eq. 3 |
| `shootRays` + chọn sector | 476, 684 | Eq. 4 |
| `heuristicSteer` | 350 | APF |
| `greedySteer` | 403 | CONNECT |

## 5. Điểm lệch code ↔ paper

Ánh xạ tham số: `epsilon`=ε, `p1`=ρ_max, `u_p`=μ, `n_rays`=N, `weight_grade`=WG. Riêng α của paper ứng với `alpha=1, beta=gamma=α`.

1. **R_max:** code tính `20·ε`, ra 10 với ε=0.5. Paper dùng 5.
2. **Exploration khi bế tắc** (ngưỡng 20/50): không có trong Algorithm 1.
3. **Kho cặp node:** paper giữ 100 cặp tốt nhất trong multiset. Code dùng heap không giới hạn, ghép với 30 node gần nhất, và xoá cặp sau khi dùng.
4. **Chi phí cạnh:** code cộng ε cho mỗi cạnh. Trên branch `experiment-batch`, `path_length` đã được tính lại theo độ dài hình học thật, còn `path_cost` giữ chi phí nội bộ.
5. **Đơn vị:** `search_time` lưu theo giây. `analyze_results.py` đổi sang ms.
6. **Không gian:** tia và APF chỉ chạy trên mặt phẳng XY, còn lấy mẫu đều có cả trục z (bản đồ 3 lớp).
7. `BRRT_Simple_Case1/2` dùng `exp(−r)`, tức chiều ngược với Eq. 3.

## 6. Reproduce

Đã sửa trên branch `experiment-batch`:
- Bỏ các đường dẫn `/home/huy` và `CustomLogger` gây crash.
- Bật baseline, lặp N lần mỗi cặp, ghi độ dài thật.
- Thêm `run_dataset_experiments.py` và `analyze_results.py`.

Chưa sửa: `kd_free` (rò rỉ bộ nhớ), seed ngẫu nhiên, đường dẫn trong `generate_dataset.py`.

Các bước: build → dataset → chạy thử → chạy đầy đủ → `analyze_results.py` → so với Table 2. Lệnh xem trong [HowToRunCode.md](../HowToRunCode.md).

## 7. Single → Multi-Robot

Code hiện giả định một robot, môi trường tĩnh, đường đi không có thời gian.

Cần đổi:
- `TreeNode`: thêm thời gian.
- Kiểm tra va chạm: thêm va chạm robot–robot, phụ thuộc thời gian, có bán kính robot.
- Planner: N cặp cây, trạng thái không nằm trong biến thành viên.
- Heuristic, LEA và APF: xét cả robot khác.
- Đầu ra: N đường đi theo thời gian.
- Metric: tổng chi phí, makespan, số xung đột.

Giữ được: `occ_grid`, `kdtree`, `sampler`, các script sinh bản đồ, và phần lõi mở rộng cây.

Hướng tận dụng code nhiều nhất là prioritized planning: RRT-FConnect lập kế hoạch cho từng robot, đường của robot trước trở thành vật cản động của robot sau. Hướng nâng cao hơn là CBS với RRT-FConnect làm tầng dưới.

## 8. Câu hỏi cho GVHD

1. Thí nghiệm của paper có bật chế độ exploration (20/50) không?
2. R_max là 5 hay `20·ε`?
3. Kho 100 cặp trong multiset: có bản code khác không?
4. EBi-RRT là `BRRT_Simple_Case1` hay `Case2`?
5. Path length trong paper là độ dài thật hay ε × số cạnh?
6. Mức narrow nào ứng với môi trường narrow passage của paper? K và `gen_min_dist` dùng bao nhiêu?
7. Multi-Robot theo hướng tập trung, ưu tiên hay CBS? Robot có kích thước và động học thế nào?
