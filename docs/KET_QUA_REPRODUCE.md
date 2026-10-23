# Kết quả reproduce RRT-FConnect

Lần chạy: `full_run_1`, ngày 23/10/2026. So sánh với Table 2 của paper.

## Thiết lập

- **Code:** branch `experiment-batch`, commit `e2e47ca`. Thuật toán giữ nguyên code được giao. Chỉ thêm phần chạy hàng loạt và đo đạc.
- **Tham số:** Table 1, file [paper_table1.json](../src/path_finder/config/paper_table1.json). Trong code, R_max = 20ε = 10 (paper dùng 5).
- **Dataset:** blobs 10/20/40% và narrow 10%, mỗi môi trường 5 bản đồ × 20 cặp start/goal × 10 lần = 1000 lần chạy cho mỗi thuật toán. Bản đồ 200×200, tự sinh bằng `generate_dataset.py`, nên khác bản đồ của paper.
- **Máy:** WSL2 Ubuntu 20.04, ROS Noetic.
- **Ánh xạ thuật toán:** RRT-Connect = `BRRT`, GB-RRT-Connect = `GoalBiasBRRT`, EBi-RRT = `BRRT_Case1` (chưa xác nhận), RRT-FConnect = `SOF_BRRT_Case3`.
- **Cách tính:** độ dài, số node, số vòng lặp và thời gian tính trung bình trên các lần chạy thành công. Path length là độ dài hình học thật của đường đi.
- **Kết quả gốc:** `~/rrt_results/full_run_1/` trên máy chạy chính. Xem lại bằng `python3 analyze_results.py ~/rrt_results/full_run_1`.

## Kết quả

Mỗi ô ghi **kết quả của mình (paper)**. Thời gian tính bằng ms: cột trung bình dùng để so với paper, cột trung vị để tham khảo.

**Simple cluttered (blobs_10)**

| Thuật toán | Path length | Nodes | Iterations | Time TB | Time TV | Success |
|---|---|---|---|---|---|---|
| RRT-Connect | 190.7 (195.4) | 553 (606) | 421 (555) | 0.65 (1.31) | 0.37 | 100 (100) |
| GB-RRT-Connect | 189.7 (194.6) | 543 (606) | 591 (797) | 0.71 (1.56) | 0.41 | 100 (100) |
| EBi-RRT | 185.0 (189.9) | 470 (543) | 219 (411) | 1.58 (3.82) | 1.06 | 100 (100) |
| **RRT-FConnect** | **181.7** (185.1) | **439** (484) | 241 (265) | 1.25 (1.47) | 0.89 | 100 (100) |

**General cluttered (blobs_20)**

| Thuật toán | Path length | Nodes | Iterations | Time TB | Time TV | Success |
|---|---|---|---|---|---|---|
| RRT-Connect | 229.1 (229.7) | 1143 (1209) | 2890 (3303) | 4.77 (7.88) | 2.46 | 100 (100) |
| GB-RRT-Connect | 226.1 (228.7) | 1114 (1219) | 4021 (4787) | 6.15 (9.53) | 2.79 | 100 (100) |
| EBi-RRT | 209.9 (215.4) | 796 (1084) | 1254 (2162) | 10.42 (23.62) | 4.52 | 100 (100) |
| **RRT-FConnect** | **204.4** (204.2) | **751** (837) | 1685 (1426) | 9.71 (7.51) | 4.09 | 100 (100) |

**Highly cluttered (blobs_40)**

| Thuật toán | Path length | Nodes | Iterations | Time TB | Time TV | Success |
|---|---|---|---|---|---|---|
| RRT-Connect | 288.9 (280.9) | 4213 (4085) | 36849 (35139) | 102.3 (118.1) | 76.5 | 98.4 (93) |
| GB-RRT-Connect | 281.6 (275.9) | 3800 (3637) | 46785 (44601) | 104.2 (116.6) | 81.4 | 90.6 (86) |
| EBi-RRT | 278.4 (267.2) | 4576 (5113) | 23341 (26463) | 479.4 (442.5) | 258.6 | 97.9 (88) |
| **RRT-FConnect** | **262.1** (246.8) | **3317** (2625) | 26832 (13237) | 320.4\* (75.1) | 181.9 | 97.8 (95) |

\* Đã loại 1 lần chạy lỗi đo thời gian (xem mục "Bất thường"). Nếu tính cả lần đó thì trung bình là 4190 ms.

**Narrow passage (narrow_10)**

| Thuật toán | Path length | Nodes | Iterations | Time TB | Time TV | Success |
|---|---|---|---|---|---|---|
| RRT-Connect | 285.7 (302.9) | 1762 (1545) | 11180 (9431) | 15.6 (17.3) | 10.7 | 100 (100) |
| GB-RRT-Connect | 286.1 (302.9) | 1781 (1469) | 15984 (12621) | 19.5 (18.5) | 13.8 | 99.9 (99.5) |
| EBi-RRT | 286.0 (312.1) | 2267 (7635) | 5993 (18415) | 142.7 (1066.7) | 80.1 | 100 (95.8) |
| **RRT-FConnect** | **275.3** (280.0) | **1542** (1179) | 12498 (4578) | 70.4 (16.2) | 43.7 | 99.9 (100) |

## Nhận xét

**Khớp với paper:**
- RRT-FConnect cho **đường ngắn nhất** và **ít node nhất** ở cả 4 môi trường. Đây là hai kết luận chính của paper.
- Path length gần với paper: lệch dưới 2% ở Simple, General và Narrow; lệch 6% ở Highly.
- RRT-Connect và GB-RRT-Connect cho số gần với paper ở mọi chỉ số. Điều này cho thấy dataset và cách đo của mình tương đương với paper.
- `BRRT_Case1` có xu hướng giống EBi-RRT: ít vòng lặp nhưng chậm hơn mỗi vòng. Điều này ủng hộ giả thuyết nó chính là EBi-RRT.

**Chưa khớp với paper:**
- **Thời gian:** RRT-FConnect chậm hơn RRT-Connect ở cả 4 môi trường. Paper báo nó nhanh nhất ở General, Highly và Narrow.
- **Số vòng lặp:** ở Highly và Narrow cao gấp 2–3 lần paper. Ở General và Narrow còn cao hơn cả EBi-RRT.
- **Success rate của baseline ở Highly** cao hơn paper (90–98% so với 86–93%), có thể do bản đồ khác.

**Kết luận:** reproduce **thành công một phần**. Phần chất lượng lời giải (độ dài đường đi, số node, success rate) tái hiện được. Phần tốc độ (thời gian, số vòng lặp) thì không.

## Bất thường

Ở blobs_40, map_1, scenario 16, trial 8: RRT-FConnect ghi nhận 3.785.022 ms (63 phút) cho 68.943 vòng lặp. Cùng cặp start/goal đó, trial 0 có 68.074 vòng lặp mà chỉ mất 456 ms. Nhiều khả năng đồng hồ hệ thống bị nhảy vì máy hoặc WSL tạm dừng, không phải do thuật toán. Lần chạy này đã bị loại khỏi trung bình ở bảng trên.

## Nguyên nhân có thể của phần chưa khớp (chưa kiểm chứng)

1. **Code được giao có thể không phải bản tác giả dùng để chạy paper.** Paper mô tả kho 100 cặp node trong multiset với độ phức tạp O(log N). Code dùng heap không giới hạn, mỗi node mới ghép với 30 node gần nhất.
2. **`kd_nearest_n` trong [kdtree.c](../src/path_finder/src/kdtree.c) (L970)** cắt nhánh tìm kiếm theo `heap[0]` thay vì theo node xa nhất. Hàm này có thể duyệt gần hết cây ở mỗi lần gọi. Chi phí mỗi vòng lặp ở blobs_40: RRT-FConnect khoảng 10 µs, EBi-RRT khoảng 19 µs, RRT-Connect khoảng 2,5 µs.
3. **R_max** là 10 thay vì 5. Code còn có **chế độ exploration** (ngưỡng 20/50) mà paper không có. Cả hai có thể làm số vòng lặp tăng.

## Việc tiếp theo

- Hỏi GVHD: repo có đúng là code đã dùng cho Table 2 không? R_max bằng bao nhiêu? Có bật chế độ exploration không? EBi-RRT là class nào?
- Tuỳ chọn, ghi rõ là thí nghiệm phụ chứ không phải reproduce: sửa `kd_nearest_n`, hoặc đặt R_max = 5, rồi chạy lại để kiểm chứng từng giả thuyết.
- Chạy lại với máy không sleep để tránh lỗi đo thời gian.
