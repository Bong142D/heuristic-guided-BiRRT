# Kết quả reproduce RRT-FConnect

So sánh với Table 2 của paper. Các lần làm:

| Lần | Ngày | Nội dung | Trạng thái |
|---|---|---|---|
| 1. `full_run_1` | 23/10/2026 | Chạy code gốc, tham số Table 1 | Xong. Thời gian đo bằng đồng hồ lỗi (lần 7) |
| 2. `sweep_1`, `sweep_2` | 27/10/2026 | Quét từng nguyên nhân bằng cờ trong file JSON tham số | Xong. Như trên; đo lại ở lần 9 |
| 3. `full_run_2` | 27/10/2026 | Chạy lại 4 môi trường của paper với cấu hình đã sửa | Xong. Như trên; flappy/rooms dừng giữa chừng (xem dưới) |
| 4. `sweep_4` | 28/10/2026 | Tìm nguyên nhân số vòng lặp còn cao gấp 2 lần paper (H9–H11), chọn cấu hình cuối | Xong. Đo lại ở lần 9 |
| 5. `full_run_3` | 28/10/2026 | Chạy đầy đủ 10 môi trường bằng cấu hình cuối | Xong, nhưng thời gian đo bằng đồng hồ lỗi. Thay bằng lần 9 |
| 6. `sweep_3` | 28/10/2026 | Code gốc so với cấu hình cuối trên flappy/rooms | Như lần 5 |
| 7. Kiểm tra thời gian | 28/10/2026 | Chạy lại các map bị nhiễu thời gian. Phát hiện **đồng hồ của WSL bị chỉnh nhảy +1,4 s mỗi khoảng 34 s** | Xong: đã tìm ra nguyên nhân và sửa |
| 8. EBi-RRT trên flappy/rooms | 28/10/2026 | Chạy riêng EBi-RRT (thuật toán chậm nhất) trên bản đồ mới | Dừng sau 15 phút để sửa đồng hồ; chuyển vào lần 9 |
| 9. Chạy lại sạch (`clean/`) | 28/10/2026 | Đồng hồ đã sửa, planner đo bằng đồng hồ đơn điệu. Chạy lại lần 5, 6, `sweep_4`, `sweep_2`, EBi-RRT trên flappy/rooms, `sweep_1` | Xong 15:52 |

**Vì sao đổi thứ tự (28/10, 03:00).** Lúc đó flappy/rooms của `full_run_2` mới xong 6/30 map. Mỗi map flappy_20 mất 13–17 phút, phần lớn là do EBi-RRT (2,7 s mỗi lần chạy, so với khoảng 0,2 s của các thuật toán khác). Nếu `sweep_4` tìm ra cấu hình gần paper hơn thì toàn bộ flappy/rooms sẽ phải chạy lại. Vì vậy mình dừng `full_run_2`, chạy `sweep_4` trước, chọn cấu hình cuối, rồi chạy đầy đủ một lần bằng cấu hình đó. EBi-RRT trên bản đồ mới được tách ra chạy sau cùng.

Lần 4–8 và lần 9 đều do script tự chạy nối tiếp. Log nằm ở `~/rrt_results/pipeline.log` trong WSL.

## Tóm tắt

- **Kết quả cuối (lần 9, đồng hồ đã sửa):** với cấu hình cuối, ở 4 môi trường của paper, RRT-FConnect tái hiện mọi kết luận của Table 2:
  - đường ngắn nhất, ít node nhất, ít vòng lặp nhất;
  - nhanh nhất ở General, Highly và Narrow. Tỉ lệ thời gian so với RRT-Connect là 0.74 / 0.63 / 0.92 (paper 0.95 / 0.64 / 0.93);
  - chậm hơn RRT-Connect một chút ở Simple: 1.11 (paper 1.12);
  - success cao nhất ở Highly;
  - sai số của các tỉ lệ thời gian khoảng ±10–15% (khoảng tin cậy 95%). Simple (1.00–1.22) và Narrow (0.82–1.01) sát mức ngang, giống paper.
- **Ngoài paper (flappy/rooms, lần 9):**
  - RRT-FConnect vẫn dùng ít vòng lặp nhất và giải được nhiều bài nhất ở mật độ 40%.
  - Nhưng nó chậm hơn RRT-Connect, tới 1.9 lần; riêng rooms_20 thì ngang.
  - EBi-RRT chậm nhất, chậm hơn RRT-Connect 9–20 lần.
- **Máy chỉ ảnh hưởng số ms tuyệt đối.** Nhanh hay chậm so với RRT-Connect là do thuật toán và bản đồ. Chi phí mỗi vòng so với RRT-Connect gần như không đổi giữa các lần đo trên máy này, và giữa máy này với máy của paper (mục "Máy có quyết định nhanh hay chậm không").
- **Code gốc (lần 1):** độ dài đường đi và số node khớp paper. Thời gian thì không khớp: RRT-FConnect chậm hơn RRT-Connect 2–3 lần.
- **Nguyên nhân chính về thời gian (lần 2):** kho cặp node không giới hạn (heap + hash map, mỗi node mới thêm 30 cặp). Kho này làm mỗi vòng lặp chậm 2–3 lần. Thay bằng **kho 100 cặp tốt nhất** như paper mô tả thì thời gian khớp paper. Hai nguyên nhân phụ: k-NN trả về hàng xóm sai, và R_max.
- **Code gốc có chế độ exploration mà paper không nhắc tới.** Chế độ này phải giữ lại. Tắt đi thì cấu hình cuối chậm hơn RRT-Connect 1.8–2.7 lần ở General, Highly và Narrow (lần 4). Với code còn bỏ mẫu nằm trong vật cản, success ở blobs_40 còn tụt từ khoảng 99% xuống 80–84% (lần 2).
- **Nguyên nhân chính về số vòng lặp (lần 4):** code **bỏ mẫu ngẫu nhiên nằm trong vật cản nhưng vẫn tính là một vòng lặp**. Trong khi đó, RRT-Connect và Algorithm 1 của paper vẫn steer về phía mẫu đó. Bỏ hành vi này thì số vòng lặp ở Highly từ 26k còn 15.2k (paper 13.2k), ở Narrow từ 12k còn 5.8k (paper 4.6k).
- **Cấu hình cuối:** [paper_table1_final.json](../src/path_finder/config/paper_table1_final.json) = Table 1 + R_max 5 + kho 100 cặp + k-NN chính xác + không bỏ mẫu nằm trong vật cản + APF luôn áp + giữ exploration.
- **Lỗi đo thời gian (lần 7):** đồng hồ thực của WSL bị chỉnh nhảy +1,4 s mỗi khoảng 34 s, do `systemd-timesyncd` giằng co với cơ chế đồng bộ giờ của Hyper-V, mà mọi planner lại đo bằng đồng hồ thực. Thời gian ở lần 1–6 vì vậy bị nhiễu. Đã sửa cả môi trường lẫn code (đo bằng đồng hồ đơn điệu), và chạy lại sạch ở lần 9.

## Thiết lập chung

- **Code:** branch `experiment-batch`. Lần 1 dùng commit `e2e47ca`, chưa sửa thuật toán. Lần 2 và 3 thêm cờ thử nghiệm. Mặc định của các cờ giữ nguyên hành vi gốc, nên lần 1 vẫn chạy lại được. Lần 9 đổi thêm cách đo thời gian của 4 planner (đồng hồ đơn điệu).
- **Tham số:** Table 1. Lần 1 dùng [paper_table1.json](../src/path_finder/config/paper_table1.json), lần 3 dùng [paper_table1_fixed.json](../src/path_finder/config/paper_table1_fixed.json), lần 5 và 9 dùng [paper_table1_final.json](../src/path_finder/config/paper_table1_final.json).
- **Dataset:** bản đồ 200×200, tự sinh bằng `generate_dataset.py`, nên khác bản đồ của paper. Bộ bản đồ đã dùng là file `dataset_maps_200x200.tar.gz` ở gốc repo. Mỗi môi trường có 5 bản đồ × 20 cặp start/goal. Lần chạy đầy đủ lặp 10 lần mỗi cặp, tức 1000 lần chạy cho mỗi thuật toán. Cách chạy lại mọi kết quả: mục 6 của [HowToRunCode.md](HowToRunCode.md).
- **Máy:** WSL2 Ubuntu 20.04, ROS Noetic, Intel Core Ultra 5 225H (laptop).
- **Ánh xạ thuật toán:** RRT-Connect = `BRRT`, GB-RRT-Connect = `GoalBiasBRRT`, EBi-RRT = `BRRT_Case1` (chưa xác nhận), RRT-FConnect = `SOF_BRRT_Case3`.
- **Cách tính:** độ dài, số node, số vòng lặp và thời gian tính trung bình trên các lần chạy thành công; success tính trên mọi lần chạy. Path length là độ dài hình học thật. Mỗi lần chạy dừng khi nối được hai cây hoặc hết 100.000 vòng lặp (không giới hạn thời gian).
- **Đo thời gian:** từ lần 9, planner đo bằng đồng hồ đơn điệu (`ros::SteadyTime`) trên máy đã sửa đồng hồ, và mọi lần chạy đều được tính. Lần 1–6 đo bằng đồng hồ thực bị chỉnh nhảy, và lần chạy nào lớn hơn 100 lần trung vị thì bị loại khỏi trung bình. Bộ lọc này đã bỏ, vì nó loại nhầm cả lần chạy dài thật (mục "Lỗi đồng hồ đo").
- **Kết quả gốc:** thư mục `~/rrt_results/<tên lần chạy>/` trên máy chạy chính.

---

## Lần 1: code gốc (`full_run_1`)

Mỗi ô ghi **kết quả của mình (paper)**. Thời gian tính bằng ms; TB là trung bình, TV là trung vị. Cột thời gian đo bằng đồng hồ lỗi (mục "Lỗi đồng hồ đo"). Số liệu sạch của code gốc nằm ở lần 9.

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

\* Đã loại 1 lần chạy lỗi đồng hồ: blobs_40, map_1, scenario 16, trial 8 ghi nhận 3.785.022 ms cho 68.943 vòng lặp, trong khi trial 0 của cùng cặp có 68.074 vòng lặp mà chỉ mất 456 ms.

**Narrow passage (narrow_10)**

| Thuật toán | Path length | Nodes | Iterations | Time TB | Time TV | Success |
|---|---|---|---|---|---|---|
| RRT-Connect | 285.7 (302.9) | 1762 (1545) | 11180 (9431) | 15.6 (17.3) | 10.7 | 100 (100) |
| GB-RRT-Connect | 286.1 (302.9) | 1781 (1469) | 15984 (12621) | 19.5 (18.5) | 13.8 | 99.9 (99.5) |
| EBi-RRT | 286.0 (312.1) | 2267 (7635) | 5993 (18415) | 142.7 (1066.7) | 80.1 | 100 (95.8) |
| **RRT-FConnect** | **275.3** (280.0) | **1542** (1179) | 12498 (4578) | 70.4 (16.2) | 43.7 | 99.9 (100) |

**Khớp với paper:**
- RRT-FConnect cho đường ngắn nhất và ít node nhất ở cả 4 môi trường.
- RRT-Connect và GB-RRT-Connect gần với paper ở mọi chỉ số, nên dataset và cách đo tương đương với paper.

**Chưa khớp:**
- RRT-FConnect chậm hơn RRT-Connect ở cả 4 môi trường. Paper báo nó nhanh nhất ở General, Highly và Narrow.
- Số vòng lặp ở Highly và Narrow cao gấp 2–3 lần paper.

---

## Lần 2: tìm nguyên nhân

### Kế hoạch

1. So code với paper và liệt kê mọi điểm khác nhau có thể ảnh hưởng tới thời gian hoặc số vòng lặp (bảng giả thuyết bên dưới).
2. Với mỗi điểm khác nhau, thêm một cờ trong file JSON tham số, mặc định giữ hành vi gốc. Cờ được đọc trong [testcase.h](../src/path_finder/include/path_finder/testcase.h) và truyền vào `SOF_BRRT_Simple_Case3::set_variant`.
3. **Vòng quét 1:** đổi **từng cờ một** so với code gốc, cộng thêm vài tổ hợp "đúng như paper mô tả". Chỉ chạy RRT-FConnect, 3 lần mỗi cặp (300 lần chạy mỗi môi trường). Bản đo lại ở lần 9 có thêm RRT-Connect chạy xen kẽ.
4. **Vòng quét 2:** dựa vào kết quả vòng 1, giữ exploration và thử các tổ hợp còn lại. Lần này chạy RRT-Connect xen kẽ trong cùng lần chạy để so **tỉ lệ thời gian** (xem "Nhiễu thời gian" bên dưới). 5 lần mỗi cặp.
5. Chọn cấu hình tốt nhất mà vẫn đúng mô tả của paper, rồi chạy lại đầy đủ (lần 3).

Lệnh: `python3 sweep_fconnect.py --variants <tên> --trials N --out <thư mục>`. Danh sách biến thể nằm trong `VARIANTS` của [sweep_fconnect.py](../sweep_fconnect.py).

### Giả thuyết và cờ tương ứng

| # | Điểm khác code ↔ paper | Cờ (gốc → paper) | Kết luận |
|---|---|---|---|
| H1 | R_max = 20ε = 10, paper dùng 5 | `r_max`: 10 → 5 | Ảnh hưởng nhỏ, nằm trong nhiễu: chi phí mỗi vòng lệch không quá 6%, số vòng lặp lệch trong mức sai số |
| H2 | Chế độ exploration (20 lần không cải thiện → 50 vòng lấy mẫu đều), paper không có | `exploration`: true → false | **Phải giữ.** Tắt đi thì ở General, Highly và Narrow số vòng lặp tăng 1.4–2.1 lần và chậm hơn 2.3–5 lần; success ở blobs_40 giảm còn 80–84% (giữ: 98.7%) |
| H3 | `kd_nearest_n` cắt nhánh theo `heap[0]` thay vì phần tử xa nhất | `knn_exact`: false → true | Bản gốc trả về 6–14% hàng xóm sai. Sửa đi thì nhanh hơn 16–40% khi đã có kho 100 cặp (đo lại ở lần 9) |
| H4 | Kho cặp node: heap + hash map không giới hạn, còn paper dùng multiset giữ 100 cặp tốt nhất | `pool_size`: 0 → 100 | **Nguyên nhân chính.** Mỗi vòng lặp nhanh hơn 2–3 lần (Simple: khoảng 1.6 lần), số vòng lặp không đổi |
| H5 | Cặp dẫn đường bị xoá sau khi dùng (pop), hay giữ lại đến khi có cặp tốt hơn (Eq. 2 là argmin) | `pop_guide`: true → false | Giữ lại thì tệ hơn: số vòng lặp tăng 6–72%, đường dài hơn tới 4%. Success ở blobs_40 thấp hơn trung bình khoảng 5 điểm (thấp hơn ở 5/6 phép so, gồm 3 lượt thử thêm ở lần 9). Paper cũng nói "pop-min", nên giữ pop |
| H6 | Số hàng xóm ghép cặp k = 30 (paper không nói) | `knn_k`: 10 / 30 / 60 | k càng nhỏ thì mỗi vòng càng rẻ. So với k = 30, chi phí mỗi vòng (so với RRT-Connect cùng lượt) của k = 10 thấp hơn 10–26%, của k = 60 cao hơn 2–16%; số vòng lặp chênh trong mức sai số. Vì vậy k = 10 nhanh hơn k = 30 ở cả 8 phép so (`paper_k10` ở vòng quét 1, `pe_k10` ở vòng quét 2, mỗi vòng 4 môi trường). Mức chênh đo được là 15–40%, nhưng một phần trong đó là sai số ngẫu nhiên. Paper không nói k, nên cấu hình cuối vẫn giữ 30 của code |
| H7 | Kích thước kho (paper: 100) | `pool_size`: 30 / 100 / 300 / 1000 | 30–300 tương đương; 1000 chậm hơn một chút |
| H8 | Lỗi đồng hồ do máy tạm dừng (lần 1) | Lọc lần chạy > 100 lần trung vị | Lúc đầu thêm bộ lọc vào `analyze_results.py` và `sweep_fconnect.py`. Sau đó bỏ: nguyên nhân thật là đồng hồ WSL bị chỉnh nhảy (lần 7), đã sửa tận gốc |

Sau lần 3, số vòng lặp vẫn cao gấp 2–2.7 lần paper ở Highly và Narrow, nên mình thêm 3 giả thuyết, thử ở lần 4 (`sweep_4`). Gốc so sánh là cấu hình đã sửa `fx` (= `paper_explore`):

| # | Điểm khác code ↔ paper | Cờ (gốc → thử) | Kết luận |
|---|---|---|---|
| H9 | Nhánh uniform, nhánh heuristic và nhánh exploration đều **bỏ mẫu nằm trong vật cản** và vẫn tính là một vòng lặp. RRT-Connect và Algorithm 1 không bỏ mà vẫn steer về phía mẫu. Khoảng 40% mẫu nằm trong vật cản ở blobs_40, khoảng 2/3 ở narrow_10 | `reject_invalid_sample`: true → false | **Đúng, là nguyên nhân chính của số vòng lặp.** Highly 26.394 → 15.100 (paper 13.237), Narrow 12.333 → 5.728 (paper 4.578); success ở Highly 98.0% → 100% |
| H10 | Ngưỡng exploration 20/50 (paper không có chế độ này, nên không biết giá trị đúng) | `explore_trigger` 10 / 20 / 50, `explore_len` 20 / 50 / 100 | 20/50 của code là hợp lý: các giá trị khác không giảm được số vòng lặp đáng kể; 50 hoặc 20 làm chậm hơn |
| H11 | `heuristicSteer` trả thẳng mẫu khi mẫu nằm trong vòng ε, không áp APF. Paper luôn áp APF | `apf_always`: false → true | Ảnh hưởng nhỏ. Được giữ trong cấu hình cuối vì đúng mô tả của paper |

Chưa thử (không đặt cờ): lấy mẫu đều có cả trục z (bản đồ 3 lớp), trong khi tia và APF chỉ chạy trên XY. Baseline cũng lấy mẫu như vậy mà vẫn khớp paper, nên đây có thể là thiết lập chung.

### Kiểm tra k-NN (H3)

Test C độc lập so với tìm kiếm vét cạn, k = 30, 2000 truy vấn ngẫu nhiên trên bản đồ 200×200:

| Số node trong cây | `kd_nearest_n` gốc | Hàng xóm sai | `kd_nearest_n_exact` | Hàng xóm sai |
|---|---|---|---|---|
| 500 | 4.3 µs | 6.3% | 2.4 µs | 0% |
| 2000 | 5.3 µs | 8.7% | 2.8 µs | 0% |
| 5000 | 6.4 µs | 9.7% | 8.7 µs | 0% |
| 20000 | 11.4 µs | 14.1% | 6.8 µs | 0% |

Giả thuyết cũ ("duyệt gần hết cây") là sai: hàm gốc không chậm hơn bao nhiêu, nhưng trả về kết quả sai.

### Vòng quét 1 (`sweep_1`, 3 lần mỗi cặp)

Số liệu đo lại ở lần 9 (`clean/sweep_1`, đồng hồ đã sửa). Bản đo đầu tiên (27/10) chỉ chạy RRT-FConnect nên chỉ có số ms, mà số ms thì phụ thuộc tốc độ máy lúc đo (mục "Nhiễu thời gian" bên dưới). Bản đo lại chạy RRT-Connect xen kẽ như vòng quét 2, nên có thêm cột ÷ RRT-C: thời gian RRT-FConnect chia thời gian RRT-Connect trong cùng lượt.

`paper` = `r_max` 5 + tắt exploration + `knn_exact` + kho 100 cặp, tức đúng những gì paper mô tả. `paper_explore` = `paper` nhưng giữ exploration. `keep` = giữ cặp dẫn đường thay vì xoá sau khi dùng (H5). `k10`, `k60` = số hàng xóm ghép cặp (H6), `rmax10` = R_max 10 thay vì 5.

| Biến thể | General (blobs_20): iter / µs mỗi iter / ÷ RRT-C | Highly (blobs_40): iter / µs mỗi iter / ÷ RRT-C / success | Narrow (narrow_10): iter / µs mỗi iter / ÷ RRT-C |
|---|---|---|---|
| orig (code gốc) | 1946 / 6.94 / 2.65 | 28977 / 9.70 / 2.69 / 98.7% | 12578 / 3.96 / 3.35 |
| rmax5 | 1926 / 6.32 / 2.43 | 27349 / 9.51 / 2.62 / 97.3% | 12054 / 3.99 / 2.92 |
| noexplore | 2861 / 12.45 / 6.24 | 39377 / 31.13 / 10.48 / **84.3%** | 20849 / 11.62 / 15.94 |
| knn_exact | 1717 / 5.15 / 2.00 | 30025 / 9.20 / 2.76 / 98.3% | 11801 / 3.38 / 2.66 |
| **pool100** | 1759 / **3.03** / 1.12 | 26585 / **3.48** / 0.98 / 99.7% | 11760 / **1.57** / 1.14 |
| keep | 2520 / 5.37 / 2.70 | 32697 / 8.33 / 2.73 / 96.7% | 13348 / 3.22 / 3.01 |
| pool100_keep | 2395 / 2.16 / 1.16 | 32887 / 2.91 / 1.02 / 96.0% | 14066 / 1.27 / 1.36 |
| paper | 3589 / 3.83 / 3.40 | 40889 / 5.41 / 2.47 / 79.7% | 22068 / 2.66 / 4.01 |
| paper_keep | 3960 / 2.58 / 2.06 | 50444 / 3.60 / 1.97 / 70.7% | 24861 / 1.64 / 2.66 |
| **paper_explore** | 1677 / 2.04 / 0.77 | 30116 / 2.45 / 0.76 / 98.7% | 12261 / 1.07 / 0.79 |
| paper_k10 | 3067 / 2.86 / 2.00 | 37237 / 4.98 / 2.01 / 86.7% | 17792 / 2.24 / 2.72 |
| paper_k60 | 2699 / 4.27 / 2.04 | 38678 / 6.69 / 2.71 / 80.0% | 21207 / 2.99 / 4.43 |
| paper_rmax10 | 3082 / 3.68 / 2.16 | 42205 / 5.86 / 2.47 / 81.3% | 20508 / 2.38 / 3.36 |
| *Paper Table 2* | *1426 / – / 0.95* | *13237 / – / 0.64 / 95%* | *4578 / – / 0.93* |

- Kết luận rút ra giống bản đo đầu tiên (cột Kết luận của bảng giả thuyết đã cập nhật theo số đo lại), trừ H6: chi phí mỗi vòng của k = 10 thấp hơn k = 30 rõ rệt, nên k = 10 nhanh hơn, chứ không phải "không khác đáng kể" như bản đo đầu tiên.
- Ở bản đo đầu tiên, `paper_rmax10` nhanh hơn `paper` dù phải bắn tia dài gấp đôi, và lúc đó mình coi đây là dấu hiệu của nhiễu. Đo lại thì chi phí mỗi vòng của hai biến thể (so với RRT-Connect cùng lượt) chênh nhau không quá 10%, tức tia dài hơn không làm vòng lặp đắt hơn. Còn chênh lệch tổng thời gian (1.47 / 2.16 / 3.36 so với 1.74 / 3.40 / 4.01 ở Simple, General, Narrow) nằm trong sai số của một lượt (mục "Nhiễu thời gian" bên dưới).
- Độ dài đường đi của mọi biến thể lệch code gốc từ −6% đến +3%. Các biến thể tắt exploration có đường ngắn hơn ở blobs_40 chỉ vì chúng thất bại ở những bài khó, mà độ dài chỉ tính trên các lần chạy thành công.
- Bảng đầy đủ (gồm blobs_10): `python3 sweep_fconnect.py --summary-only --out ~/rrt_results/clean/sweep_1`.

### Nhiễu thời gian

Trong bản đo đầu tiên của vòng quét 2, cùng biến thể `pool100` có lúc đo được 38 µs/iter, có lúc 4 µs/iter. Máy là laptop chạy WSL2, nên tốc độ CPU thay đổi theo điện năng và nhiệt. Ghim tiến trình vào một core không giúp được, vì trong WSL mọi core đều đo như nhau. Từ vòng quét 2, RRT-Connect được chạy xen kẽ từng truy vấn với RRT-FConnect, và cột **T/T_RRTC** (thời gian RRT-FConnect chia thời gian RRT-Connect trong cùng lần chạy) được dùng để so sánh.

Ngoài tốc độ CPU còn một nguồn nhiễu thứ hai, mãi đến lần 7 mới tìm ra: đồng hồ của WSL bị chỉnh nhảy (mục "Lỗi đồng hồ đo"). Bảng của cả hai vòng quét trong mục này là số liệu đo lại ở lần 9, sau khi sửa.

**Sai số ngẫu nhiên (tính ở lần 9).** Kể cả khi máy và đồng hồ đều ổn, tỉ lệ của một lượt vẫn dao động, vì số vòng lặp có đuôi dài: vài bài khó chiếm phần lớn tổng thời gian. Ví dụ, RRT-Connect giống hệt nhau ở cả 13 lượt của `clean/sweep_1`, vậy mà thời gian trung bình của nó lệch nhau 6–10% (độ lệch chuẩn tương đối). Phần lớn mức lệch này đến từ số vòng lặp (3–9%); tốc độ máy (µs mỗi vòng) chỉ lệch 3–5%. Bảng dưới là sai số của tỉ lệ RRT-FConnect ÷ RRT-Connect, tức nửa độ rộng của khoảng tin cậy 95%. Khoảng tin cậy tính bằng bootstrap, lấy mẫu lại theo từng cặp start/goal, vì các lần lặp của cùng một cặp không độc lập với nhau.

| Tỉ lệ RRT-FConnect ÷ RRT-Connect | 300 lần chạy mỗi môi trường (`sweep_1`) | 500 (`sweep_2`) | 1000 (`full_final`) |
|---|---|---|---|
| Thời gian | ±12–47% | ±9–29% | ±8–15% |
| Số vòng lặp | ±9–36% | ±7–20% | ±6–12% |
| Chi phí mỗi vòng | ±6–25% | ±4–13% | ±4–6% |

Cột `full_final` không tính flappy_40, nơi rất ít lần chạy thành công (RRT-FConnect 18.5%, RRT-Connect 5.5%). Ở đó sai số của thời gian là −18% / +24%, của chi phí mỗi vòng là −11% / +15%.

- Sai số lớn nhất ở Simple và General, nơi thời gian dồn vào vài bài khó.
- Chi phí mỗi vòng ổn định hơn nhiều, vì không chịu đuôi dài của số vòng lặp. Khi so hai biến thể, chênh lệch chi phí mỗi vòng đáng tin hơn chênh lệch thời gian.
- Trong một vòng quét, hai biến thể chênh nhau dưới khoảng 30% về thời gian thì chưa đủ để kết luận. Chạy lại `paper` 3 lượt (giai đoạn F của lần 9) cho tỉ lệ ở narrow_10 từ 2.84 đến 4.01.
- Ở bảng chính (`full_final`), sai số đủ nhỏ cho mọi kết luận, trừ những chỗ sát mức ngang (mục "Kết quả: 4 môi trường của paper" của lần 9).

### Vòng quét 2 (`sweep_2`, 5 lần mỗi cặp, RRT-Connect chạy xen kẽ)

Số liệu đo lại ở lần 9 (`clean/sweep_2`, đồng hồ đã sửa); trong ngoặc là bản đo đầu tiên (27/10, đồng hồ lỗi). Tỉ lệ T/T_RRTC dưới 1 nghĩa là RRT-FConnect nhanh hơn RRT-Connect.

| Biến thể | Simple | General | Highly | Narrow |
|---|---|---|---|---|
| orig (code gốc) | 2.25 (1.98) | 2.18 (2.09) | 3.00 (2.44) | 3.23 (3.00) |
| pool100 | 1.25 (1.15) | 1.00 (1.12) | 1.14 (0.91) | 1.56 (1.27) |
| pool100_exact | 1.05 (0.98) | 0.82 (0.75) | 0.79 (0.61) | 0.93 (1.00) |
| pool100_rmax5 | 1.48 (1.14) | 1.08 (1.13) | 1.04 (1.12) | 1.28 (1.41) |
| **paper_explore** (kho 100 + exact + R_max 5 + exploration) | 1.02 (1.05) | 0.83 (0.63) | 0.72 (0.75) | 0.85 (0.84) |
| pe_pool30 | 0.88 (1.01) | 0.70 (0.68) | 0.59 (0.57) | 0.85 (0.73) |
| pe_pool300 | 1.17 (1.10) | 0.75 (0.73) | 0.78 (0.80) | 0.95 (0.98) |
| pe_pool1000 | 1.26 (1.18) | 0.78 (0.77) | 0.67 (0.73) | 0.92 (1.07) |
| pe_k10 | 0.87 (0.90) | 0.55 (0.60) | 0.57 (0.61) | 0.64 (0.70) |
| *Paper Table 2* | *1.12* | *0.95* | *0.64* | *0.93* |

Số vòng lặp của mọi biến thể ở đây gần bằng code gốc (Highly: 24k–29k, Narrow: 11.3k–12.5k).

Đo lại cho cùng kết luận:
- Kho cặp không giới hạn là nguyên nhân chính. Code gốc chậm hơn RRT-Connect 2.2–3.2 lần; chỉ thay bằng kho 100 cặp là còn 1.0–1.6 lần.
- k-NN chính xác giúp nhanh thêm 16–40% (`pool100` → `pool100_exact`).
- R_max = 5 cho tác động không nhất quán khi đứng một mình (`pool100_rmax5` so với `pool100`): chậm hơn ở Simple và General, nhanh hơn ở Highly và Narrow. Khi kết hợp với k-NN chính xác thì ngang hoặc nhanh hơn một chút (`paper_explore` so với `pool100_exact`).

**Chọn cấu hình cho lần 3:** `paper_explore`, file [paper_table1_fixed.json](../src/path_finder/config/paper_table1_fixed.json). Lý do: đây là đúng những gì paper mô tả (R_max = 5, kho 100 cặp, k-NN đúng), chỉ giữ thêm chế độ exploration vì không có nó thì planner thất bại nhiều. `pe_pool30` và `pe_k10` nhanh hơn một chút nhưng dùng tham số paper không nói tới.

---

## Lần 3: chạy lại 4 môi trường của paper với cấu hình đã sửa (`full_run_2`)

Cả 4 thuật toán, 10 lần mỗi cặp. RRT-FConnect dùng `paper_table1_fixed.json`, baseline không đổi. Lần chạy này ban đầu gồm cả 6 môi trường flappy/rooms, nhưng phần đó đã dừng ở 6/30 map để chuyển sang lần 4 (xem bảng trạng thái ở đầu tài liệu). Flappy/rooms được chạy lại đầy đủ ở lần 5 bằng cấu hình cuối.

Mỗi ô ghi **kết quả của mình (paper)**. Thời gian tính bằng ms, đo bằng đồng hồ lỗi; cấu hình này (`fx`) được đo lại sạch trong `sweep_4` của lần 9.

**Simple cluttered (blobs_10)**

| Thuật toán | Path length | Nodes | Iterations | Time TB | Time TV | Success |
|---|---|---|---|---|---|---|
| RRT-Connect | 190.3 (195.4) | 552 (606) | 418 (555) | 0.67 (1.31) | 0.39 | 100 (100) |
| GB-RRT-Connect | 189.0 (194.6) | 537 (606) | 570 (797) | 0.71 (1.56) | 0.44 | 100 (100) |
| EBi-RRT | 184.8 (189.9) | 469 (543) | 213 (411) | 1.53 (3.82) | 1.11 | 100 (100) |
| **RRT-FConnect** | **181.6** (185.1) | **441** (484) | 248 (265) | 0.68 (1.47) | 0.55 | 100 (100) |

**General cluttered (blobs_20)**

| Thuật toán | Path length | Nodes | Iterations | Time TB | Time TV | Success |
|---|---|---|---|---|---|---|
| RRT-Connect | 228.6 (229.7) | 1132 (1209) | 2955 (3303) | 4.79 (7.88) | 2.46 | 100 (100) |
| GB-RRT-Connect | 226.6 (228.7) | 1091 (1219) | 3970 (4787) | 5.52 (9.53) | 2.92 | 100 (100) |
| EBi-RRT | 211.3 (215.4) | 852 (1084) | 1359 (2162) | 12.92 (23.62) | 4.92 | 100 (100) |
| **RRT-FConnect** | **204.6** (204.2) | **759** (837) | 1700 (1426) | **3.81** (7.51) | 1.70 | 100 (100) |

**Highly cluttered (blobs_40)**

| Thuật toán | Path length | Nodes | Iterations | Time TB | Time TV | Success |
|---|---|---|---|---|---|---|
| RRT-Connect | 290.4 (280.9) | 4420 (4085) | 38001 (35139) | 122.0 (118.1) | 91.1 | 98.5 (93) |
| GB-RRT-Connect | 285.0 (275.9) | 3866 (3637) | 47383 (44601) | 119.1 (116.6) | 92.7 | 90.8 (86) |
| EBi-RRT | 276.1 (267.2) | 4617 (5113) | 23266 (26463) | 581.8 (442.5) | 318.2 | 98.7 (88) |
| **RRT-FConnect** | **260.8** (246.8) | **3244** (2625) | 26081 (13237) | **75.6** (75.1) | 44.1 | 97.3 (95) |

**Narrow passage (narrow_10)**

| Thuật toán | Path length | Nodes | Iterations | Time TB | Time TV | Success |
|---|---|---|---|---|---|---|
| RRT-Connect | 284.8 (302.9) | 1762 (1545) | 11040 (9431) | 18.6 (17.3) | 12.0 | 100 (100) |
| GB-RRT-Connect | 285.5 (302.9) | 1738 (1469) | 15720 (12621) | 21.9 (18.5) | 15.4 | 100 (99.5) |
| EBi-RRT | 286.9 (312.1) | 2343 (7635) | 6230 (18415) | 183.8 (1066.7) | 107.1 | 100 (95.8) |
| **RRT-FConnect** | **275.2** (280.0) | **1543** (1179) | 12257 (4578) | **16.1** (16.2) | 11.5 | 100 (100) |

So với lần 1, chỉ cột thời gian của RRT-FConnect thay đổi đáng kể. Độ dài đường đi, số node, số vòng lặp và success gần như giữ nguyên.

---

## Lần 4: tìm nguyên nhân số vòng lặp (`sweep_4`)

5 lần mỗi cặp, 4 môi trường của paper, RRT-Connect chạy xen kẽ. Gốc là `fx` (cấu hình của lần 3). Bảng dưới là **số liệu đo lại ở lần 9** (`clean/sweep_4`, đồng hồ đã sửa). Bản đo lần đầu (28/10, 03:06–03:43, đồng hồ lỗi) cho cùng nhóm biến thể dẫn đầu.

Mỗi ô ghi **số vòng lặp / tỉ lệ thời gian so với RRT-Connect / success (%)**. Xếp theo điểm gần paper (cột Điểm, xem dưới). "Hợp lệ" nghĩa là giữ được kết luận về thời gian của paper và success không giảm quá 2 điểm so với `fx`.

| Biến thể | Điểm | Hợp lệ | Simple | General | Highly | Narrow |
|---|---|---|---|---|---|---|
| *Paper Table 2* | – | – | *265 / 1.12 / 100* | *1426 / 0.95 / 100* | *13237 / 0.64 / 95* | *4578 / 0.93 / 100* |
| **fx_noreject_apf** (chọn) | 0.392 | có | 192 / 1.24 / 100 | 1048 / 0.69 / 100 | 15005 / 0.66 / 100 | 5760 / 0.86 / 100 |
| fx_noreject | 0.443 | có | 196 / 1.10 / 100 | 1028 / 0.65 / 100 | 16401 / 0.78 / 99.8 | 5680 / 0.96 / 100 |
| fx_noreject_expl50 | 0.502 | không | 189 / 1.14 / 100 | 1215 / 0.89 / 100 | 16813 / 0.94 / 99.8 | 6550 / 1.22 / 100 |
| fx_noreject_len20 | 0.524 | có | 166 / 1.03 / 100 | 1105 / 0.72 / 100 | 16164 / 0.88 / 99.8 | 6165 / 0.99 / 100 |
| fx_len100 | 0.603 | có | 259 / 0.99 / 100 | 1634 / 0.71 / 100 | 26606 / 0.57 / 98.8 | 11919 / 0.69 / 100 |
| fx_expl10 | 0.635 | có | 266 / 1.01 / 100 | 1754 / 0.72 / 100 | 27440 / 0.57 / 98.8 | 12199 / 0.72 / 100 |
| fx_apf | 0.645 | có | 253 / 1.06 / 100 | 1764 / 0.76 / 100 | 26006 / 0.59 / 97.8 | 12165 / 0.85 / 100 |
| fx (lần 3) | 0.703 | có | 244 / 1.10 / 100 | 1602 / 0.63 / 100 | 27382 / 0.68 / 97.8 | 12585 / 0.90 / 100 |
| fx_len20 | 0.837 | không | 246 / 1.14 / 100 | 1882 / 0.93 / 100 | 29525 / 0.98 / 96.4 | 12958 / 1.19 / 100 |
| fx_expl50 | 0.841 | không | 260 / 1.16 / 100 | 1929 / 0.89 / 100 | 30022 / 1.02 / 96.2 | 12704 / 1.16 / 100 |
| fx_noreject_noexpl | 1.137 | không | 227 / 1.28 / 100 | 2007 / 1.83 / 100 | 30604 / 2.52 / 96.2 | 10799 / 2.70 / 100 |

**Nhận xét:**
- **Không bỏ mẫu nằm trong vật cản (H9) giải thích phần lớn chênh lệch số vòng lặp.**
  - So `fx_noreject_apf` với `fx`, số vòng lặp giảm 21% (Simple), 35% (General), 45% (Highly) và 54% (Narrow). Highly và Narrow về gần paper: lệch 13–26%, thay vì gấp 2–2.7 lần.
  - Success ở Highly lên 100%.
  - Mỗi vòng lặp giờ làm việc thật (steer về phía mẫu) thay vì bỏ ngay, nên µs mỗi vòng lặp tăng (Highly: 2.3 lên 4.0 µs). Tổng thời gian vì vậy gần như không đổi.
- **Exploration vẫn cần thiết ngay cả khi không bỏ mẫu:** tắt nó (`fx_noreject_noexpl`) thì chậm hơn RRT-Connect 1.8–2.7 lần ở General, Highly và Narrow.
- **APF luôn áp (H11)** không đổi nhiều số vòng lặp. Khi đã không bỏ mẫu, bật APF làm nhanh hơn một chút ở Highly (0.66 so với 0.78) và Narrow (0.86 so với 0.96).
- `fx_len100` và `fx_expl10` nhanh nhất ở Highly (0.57), nhưng vẫn bỏ mẫu nằm trong vật cản, nên số vòng lặp gấp đôi paper.

**Cách chọn cấu hình cuối.**
- Script chấm điểm mỗi biến thể bằng trung bình, trên 4 môi trường, của |ln(của mình ÷ paper)| cho độ dài, số node và số vòng lặp. Thời gian không được chấm vì phụ thuộc máy. Thay vào đó, biến thể phải giữ kết luận của paper là RRT-FConnect nhanh hơn RRT-Connect ở General, Highly và Narrow, và không làm success giảm quá 2 điểm so với `fx`.
- Ở lần đo đầu (đồng hồ lỗi), script chọn `fx_noreject` (0.411), sát nút `fx_noreject_apf` (0.413). Khi đó mình chọn `fx_noreject_apf` vì hai lý do: nó đúng mô tả của paper hơn (paper luôn áp APF), và nó giữ đủ kết luận định tính về thời gian ở cả 4 môi trường.
- Trên số liệu sạch, **script tự chọn `fx_noreject_apf`** (0.392, đứng đầu). Biến thể này chậm hơn RRT-Connect một chút ở Simple (1.24) và nhanh hơn ở General, Highly và Narrow (0.69, 0.66, 0.86).

**Cấu hình cuối:** [paper_table1_final.json](../src/path_finder/config/paper_table1_final.json) = Table 1 + R_max 5 + kho 100 cặp + k-NN chính xác + không bỏ mẫu nằm trong vật cản + APF luôn áp + giữ exploration (20/50).

---

## Lần 5: chạy đầy đủ với cấu hình cuối (`full_run_3`)

Xong lúc 06:01 ngày 28/10: 4 môi trường của paper với cả 4 thuật toán, 6 môi trường flappy/rooms với RRT-Connect, GB-RRT-Connect và RRT-FConnect, 10 lần mỗi cặp. Thời gian của lần này đo bằng đồng hồ lỗi nên **không dùng**. Bảng chính thức ở lần 9, chạy lại y hệt sau khi sửa đồng hồ.

Bản đồ mới: flappy_10/20/40 (4 bức tường ngang, mỗi bức có một khe rộng 20 m, cộng chướng ngại hình hộp) và rooms_10/20/40 (lưới 3×4 phòng, cửa rộng 20 m, cộng chướng ngại hình hộp), sinh ngày 27/10 bằng `generate_dataset.py --types flappy rooms --fills 0.1 0.2 0.4`.

---

## Lần 6: code gốc so với cấu hình cuối trên flappy và rooms (`sweep_3`)

Xong lúc 05:13 ngày 28/10. Như lần 5, thời gian đo lại ở lần 9.

---

## Lỗi đồng hồ đo (lần 7)

### Từ "máy khựng" đến lỗi đồng hồ

Ban đầu mình nghĩ thời gian bị nhiễu do máy ngủ hoặc WSL khựng, nên chọn cách **chạy lại cả map** thay vì lọc hay ước lượng. Bản đầu của [check_timing.py](../check_timing.py) đánh dấu một lần chạy khi thời gian mỗi vòng lặp lớn hơn 20 lần trung vị của cùng thuật toán trên cùng map. Sau đó xoá map và chạy lại, tối đa 3 vòng. Kết quả không hội tụ: lần 5–6 có 29 map bị đánh dấu, chạy lại còn 8, rồi 2, rồi vẫn 2. Mỗi lần chạy lại lại phát sinh lần nhiễu mới.

So mỗi lần chạy với **các lần chạy có số vòng lặp tương đương** thì thấy rõ hơn. Lý do: thời gian mỗi vòng lặp tăng theo kích thước cây, nên trung vị của cả map che mất các lần chạy dài. Kết quả: hàng trăm lần chạy có cùng một dấu hiệu, là **thời gian bị cộng thêm một khoảng gần như cố định**. Khoảng này là 1,3–1,9 s (riêng ở `sweep_2` ngày 27/10 là 0,4–0,9 s), xuất hiện ở mọi thuật toán và mọi môi trường. Ví dụ: RRT-Connect, flappy_20/map_4, có ba lần chạy bị cộng thêm 1,6–1,9 s.

### Nguyên nhân: đồng hồ thực của WSL bị chỉnh nhảy

| Kiểm tra (28/10, 06:05–06:20) | Kết quả |
|---|---|
| Chương trình ngủ 5 ms rồi đo khoảng trống trên đồng hồ đơn điệu, chạy 8 phút | Không có lần dừng nào trên 100 ms. **Máy ảo không hề bị dừng** |
| So đồng hồ thực (`CLOCK_REALTIME`) với đồng hồ đơn điệu mỗi 5 ms, chạy 10 phút | 14 lần đồng hồ thực **nhảy tới +1,28 đến +1,54 s**, đều đặn khoảng 34 s một lần |
| Đo độ lệch giờ WSL − Windows mỗi 2 s | Hình răng cưa: WSL chậm dần khoảng 4,3% (tụt 1,3–1,4 s sau 30 s), rồi nhảy tới |
| Đo tốc độ đồng hồ WSL so với Windows trong 25 s | `MONOTONIC` chậm 4,40%; `MONOTONIC_RAW` (TSC thô) đúng 0,00% |
| Đọc trạng thái `adjtimex` của kernel | `tick` thay đổi liên tục trong khoảng 9165–9735 µs, thay vì 10000. Tức đồng hồ bị chỉnh chậm 2,6–8,3% |

Distro WSL trên máy này bật systemd. Vì vậy có hai cơ chế đồng bộ giờ chạy song song:
- `systemd-timesyncd`, đồng bộ qua NTP và hỏi server mỗi 32 s;
- cơ chế đồng bộ giờ của Hyper-V (kernel WSL chạy với `hv_utils.timesync_implicit=1`).

Hai cơ chế giằng co nhau: timesyncd chỉnh `tick` làm đồng hồ chạy chậm, đồng hồ tụt dần sau giờ thật, rồi bị kéo nhảy tới.

Mọi planner đo thời gian bằng `ros::Time::now()`, tức đồng hồ thực, nên:
- **lần chạy vắt qua một bước nhảy bị cộng thêm khoảng 1,4 s**. Xác suất này tỉ lệ với độ dài lần chạy: một lần chạy 100 ms có khoảng 0,3% khả năng;
- **các lần chạy còn lại bị đo ngắn đi 3–8%**.

Trung bình trên rất nhiều lần chạy thì hai sai số gần như bù nhau, vì về lâu dài đồng hồ thực vẫn bám giờ thật. Nhưng nhiễu rất lớn. Một bước nhảy 1,4 s rơi vào 1000 lần chạy làm trung bình tăng 1,4 ms, gấp đôi cả trung bình của blobs_10. Ngược lại, ở các môi trường ngắn hiếm khi có bước nhảy nào rơi vào, nên thời gian bị đo thấp hơn thật vài phần trăm.

### Cách sửa

1. **Môi trường WSL:** `systemctl mask systemd-timesyncd` (Hyper-V đã đồng bộ giờ cho WSL) và đặt lại `tick` = 10000 µs.
   - `stop` thôi không đủ. Khi không còn phiên nào mở, WSL tự tắt distro; lần mở sau, systemd lại chạy timesyncd. Nhật ký cho thấy timesyncd tự khởi động lại 3 lần trong 06:24–06:28.
   - Sau khi sửa, `MONOTONIC`, `MONOTONIC_RAW` và `REALTIME` đều đúng 0,00% so với Windows, và độ lệch WSL − Windows đứng yên trong khoảng ±2 ms.
   - Hướng dẫn cho máy khác nằm ở mục 2b của [HowToRunCode.md](HowToRunCode.md).
2. **Code:** RRT-Connect, GB-RRT-Connect, EBi-RRT và RRT-FConnect giờ đo thời gian bằng `ros::SteadyTime` (đồng hồ đơn điệu). Đồng hồ thực có bị chỉnh nhảy thì kết quả cũng không bị ảnh hưởng. Bốn planner này dừng theo số vòng lặp chứ không theo thời gian, nên thuật toán không thay đổi.
3. **Kiểm tra trước mỗi giai đoạn:** pipeline đo trong 5 s và yêu cầu `MONOTONIC` chạy cùng tốc độ với `MONOTONIC_RAW` (lệch dưới 0,1%); nếu không thì dừng. Kiểm tra này đã chặn một lần khởi chạy khi timesyncd tự chạy lại (lệch −7,4%), trước khi có số liệu nào được ghi.
4. **[check_timing.py](../check_timing.py) mới:**
   - So mỗi lần chạy với các lần chạy cùng thuật toán, cùng map, có số vòng lặp chênh không quá 2 lần.
   - Đánh dấu khi thời gian mỗi vòng lặp gấp ≥ 3 lần, và phần dư đủ làm lệch trung bình của map ≥ 2% (đồng thời ≥ 20 ms).
   - Trên dữ liệu cũ, bản mới đánh dấu 9–26 map ở mỗi thư mục (tổng cộng 163 lần chạy). Ở 146/163 lần chạy, phần dư từ 0,3 s trở lên (đa số 1–2 s), đúng độ lớn của một hoặc hai bước nhảy.
   - Bổ sung sau lần 9 (giai đoạn F): thời gian mỗi node cũng phải gấp ≥ 3 lần các lần chạy có số node tương đương. Lý do: có những lần chạy mọc cây lớn gấp 2–3 lần bình thường so với số vòng lặp của nó, nên chậm thật chứ không phải bị nhiễu. Lần 9 được làm sạch bằng tiêu chí cũ, chặt hơn; kiểm tra lại mọi thư mục `clean/` bằng tiêu chí mới thì không map nào bị đánh dấu.
5. **Bỏ bộ lọc "lớn hơn 100 lần trung vị"** trong [analyze_results.py](../analyze_results.py) và [sweep_fconnect.py](../sweep_fconnect.py). Bộ lọc này loại nhầm cả lần chạy dài thật, ví dụ một lần GB-RRT-Connect ở blobs_20 (lần 1) chạy 61.567 vòng lặp trong 794 ms. Giờ mọi lần chạy đều được tính; map nào bị nhiễu thì chạy lại.

### Ảnh hưởng tới số liệu cũ

- **Độ dài đường đi, số node, số vòng lặp, success:** không bị ảnh hưởng, vì không phụ thuộc đồng hồ.
- **Thời gian ở lần 1–6:** nhiễu như trên. Các kết luận dựa trên chênh lệch lớn vẫn đúng, chẳng hạn code gốc chậm hơn RRT-Connect 2–3 lần, hay kho 100 cặp làm mỗi vòng lặp nhanh hơn 2–3 lần. Mọi số liệu thời gian dùng cho kết luận cuối lấy từ lần 9.

---

## Lần 9: chạy lại sạch (`~/rrt_results/clean/`)

Dataset, tham số và cách chạy giống hệt các lần trước. Chỉ khác ở chỗ đồng hồ đã sửa và planner đo bằng đồng hồ đơn điệu. Cấu hình cuối là [paper_table1_final.json](../src/path_finder/config/paper_table1_final.json); code gốc là [paper_table1.json](../src/path_finder/config/paper_table1.json), với mọi cờ để mặc định. Bảng đo lại của các vòng quét nằm ngay tại mục của chúng, thay cho bản đo cũ: `sweep_1` và `sweep_2` ở lần 2, `sweep_4` ở lần 4.

| Giai đoạn | Nội dung | Trạng thái |
|---|---|---|
| A | `full_final`: cấu hình cuối, 4 môi trường của paper, 4 thuật toán, 10 lần mỗi cặp | Xong 06:52. Kiểm tra thời gian: 1 map bị đánh dấu (một lần GB-RRT-Connect ở blobs_40 dư 636 ms), chạy lại thì sạch |
| B | `full_final` trên flappy/rooms (RRT-Connect, GB-RRT-Connect, RRT-FConnect); `full_orig`: code gốc và RRT-Connect trên 4 môi trường của paper | Xong 07:39. Không map nào bị đánh dấu |
| C | `sweep_4` (11 biến thể) và `sweep_3` (code gốc so với cấu hình cuối trên flappy/rooms), 5 lần mỗi cặp | Xong 09:01. Kiểm tra thời gian: vòng 1 đánh dấu 2 map. Riêng `fx_noreject_noexpl/blobs_40/map_0` bị đánh dấu 3 vòng liền, mỗi lần một bài khác nhau, cùng tỉ lệ khoảng 3.4 lần; nhiều khả năng đây là lần chạy chậm thật của biến thể không có exploration, không phải nhiễu máy (giai đoạn F đã xác nhận hiện tượng này). Từ 08:13 máy chạy bằng pin; tỉ lệ T/T_RRTC không bị ảnh hưởng vì RRT-Connect chạy xen kẽ |
| D | `sweep_2` (9 biến thể), 5 lần mỗi cặp | Xong 09:31. Vòng 1 đánh dấu 2 map, chạy lại thì sạch |
| E | `ebi_new`: EBi-RRT trên flappy/rooms, 10 lần mỗi cặp | Xong 14:47. Từ 11:18 đến 13:31 màn hình tự tắt nên laptop vào Modern Standby, WSL đứng 2 giờ 12 phút khi đang chạy `flappy_40/map_4`. Số đo không bị ảnh hưởng: không lần chạy nào bị cộng thêm khoảng thời gian dừng (đồng hồ đơn điệu không đếm lúc máy ảo đứng), và thời gian mỗi vòng ở map_4 khớp 4 map còn lại. Kiểm tra thời gian: 1 map bị đánh dấu (một lần RRT-FConnect ở `flappy_40/map_0` dư 376 ms), chạy lại thì sạch |
| F | `sweep_1` (13 biến thể), 3 lần mỗi cặp | Xong 15:52. Kiểm tra thời gian: vòng 1 đánh dấu 19 map, vòng 2 còn 7, vòng 3 còn 3; sau 3 vòng chạy lại còn 2 map, đều của `paper_keep`. Phép thử thêm (15:57–16:14, `~/rrt_results/keep_test/`): chạy lại `paper_keep` và `paper` trên blobs_40 và narrow_10 thêm 3 lượt. `paper_keep` bị đánh dấu 11–16 lần chạy mỗi lượt, `paper` 1–3 lần, RRT-Connect (chạy xen kẽ với cả hai) tổng cộng 1 lần. Nhiễu máy thì phải rơi đều vào cả ba, nên đây là đặc tính của thuật toán. 46/47 lần chạy bị đánh dấu có cây lớn gấp 1.5–3.7 lần các lần chạy cùng số vòng lặp, trong khi thời gian mỗi node vẫn bình thường (dưới 3 lần). Vì vậy giữ nguyên số liệu: tỉ lệ thời gian của `paper_keep` trong bảng (1.97 ở blobs_40, 2.66 ở narrow_10) nằm trong khoảng của 3 lượt thử (1.88–1.98 và 1.89–2.66). `check_timing.py` được bổ sung điều kiện theo node (lần 7, mục "Cách sửa") |

### Kết quả: 4 môi trường của paper (`clean/full_final`)

Cấu hình cuối, 1000 lần chạy cho mỗi thuật toán ở mỗi môi trường. Mỗi ô ghi **của mình (paper)**. Cột "÷ RRT-Connect" là thời gian trung bình chia cho thời gian của RRT-Connect trong cùng lần chạy. Dòng "code gốc" lấy từ `clean/full_orig`, một lần chạy riêng của code gốc xen kẽ với RRT-Connect, nên tỉ lệ thời gian của nó tính theo RRT-Connect của lần chạy đó.

| Môi trường | Iterations RRT-FConnect | Time RRT-FConnect ÷ RRT-Connect | Code gốc ÷ RRT-Connect | Success RRT-FConnect / RRT-Connect (%) |
|---|---|---|---|---|
| Simple cluttered (blobs_10) | 182 (265) | 1.11 (1.12) | 2.27 | 100.0 / 100.0 (100 / 100) |
| General cluttered (blobs_20) | 1057 (1426) | 0.74 (0.95) | 2.42 | 100.0 / 100.0 (100 / 100) |
| Highly cluttered (blobs_40) | 15213 (13237) | 0.63 (0.64) | 2.94 | 99.7 / 98.0 (95 / 93) |
| Narrow passage (narrow_10) | 5766 (4578) | 0.92 (0.93) | 3.56 | 100.0 / 100.0 (100 / 100) |

So với code gốc, cấu hình cuối nhanh hơn 2.0 lần (Simple), 3.3 lần (General), 4.7 lần (Highly) và 3.9 lần (Narrow), tính theo tỉ lệ với RRT-Connect.

Khoảng tin cậy 95% của tỉ lệ thời gian RRT-FConnect ÷ RRT-Connect (bootstrap theo cặp start/goal, mục "Nhiễu thời gian" của lần 2): Simple 1.00–1.22, General 0.64–0.84, Highly 0.57–0.70, Narrow 0.82–1.01. Tỉ lệ của paper nằm trong khoảng này ở Simple, Highly và Narrow; ở General thì không. Simple và Narrow sát mức ngang: RRT-FConnect chậm hơn (Simple) hoặc nhanh hơn (Narrow) một chút, giống paper, nhưng chiều chênh lệch chưa chắc chắn.

**Simple cluttered (blobs_10)**

| Thuật toán | Path length | Nodes | Iterations | Time TB | Time TV | ÷ RRT-Connect | Success |
|---|---|---|---|---|---|---|---|
| RRT-Connect | 190.2 (195.4) | 547 (606) | 409 (555) | 0.63 (1.31) | 0.37 | 1.00 | 100.0 (100) |
| GB-RRT-Connect | 190.0 (194.6) | 541 (606) | 566 (797) | 0.74 (1.56) | 0.43 | 1.17 (1.19) | 100.0 (100) |
| EBi-RRT | 185.1 (189.9) | 477 (543) | 227 (411) | 1.68 (3.82) | 1.14 | 2.66 (2.92) | 100.0 (100) |
| **RRT-FConnect** | **182.4** (185.1) | **447** (484) | 182 (265) | 0.70 (1.47) | 0.54 | 1.11 (1.12) | 100.0 (100) |
| RRT-FConnect (code gốc) | 181.8 | 443 | 255 | 1.37 | 0.91 | 2.27 | 100.0 |

**General cluttered (blobs_20)**

| Thuật toán | Path length | Nodes | Iterations | Time TB | Time TV | ÷ RRT-Connect | Success |
|---|---|---|---|---|---|---|---|
| RRT-Connect | 229.3 (229.7) | 1112 (1209) | 2809 (3303) | 4.53 (7.88) | 2.44 | 1.00 | 100.0 (100) |
| GB-RRT-Connect | 226.5 (228.7) | 1055 (1219) | 3698 (4787) | 4.94 (9.53) | 2.57 | 1.09 (1.21) | 100.0 (100) |
| EBi-RRT | 210.9 (215.4) | 842 (1084) | 1353 (2162) | 12.74 (23.62) | 4.73 | 2.81 (3.00) | 100.0 (100) |
| **RRT-FConnect** | **205.0** (204.2) | **731** (837) | 1057 (1426) | 3.34 (7.51) | 1.62 | 0.74 (0.95) | 100.0 (100) |
| RRT-FConnect (code gốc) | 205.7 | 785 | 1860 | 11.65 | 4.32 | 2.42 | 100.0 |

**Highly cluttered (blobs_40)**

| Thuật toán | Path length | Nodes | Iterations | Time TB | Time TV | ÷ RRT-Connect | Success |
|---|---|---|---|---|---|---|---|
| RRT-Connect | 288.0 (280.9) | 4364 (4085) | 37738 (35139) | 108.1 (118.1) | 82.40 | 1.00 | 98.0 (93) |
| GB-RRT-Connect | 283.5 (275.9) | 3772 (3637) | 47097 (44601) | 103.2 (116.5) | 82.79 | 0.95 (0.99) | 91.8 (86) |
| EBi-RRT | 277.6 (267.2) | 4506 (5113) | 22970 (26463) | 455.1 (442.5) | 250.0 | 4.21 (3.75) | 97.5 (88) |
| **RRT-FConnect** | **265.9** (246.8) | **2950** (2625) | 15213 (13237) | 68.08 (75.10) | 37.48 | 0.63 (0.64) | 99.7 (95) |
| RRT-FConnect (code gốc) | 262.2 | 3410 | 27565 | 316.3 | 175.9 | 2.94 | 97.7 |

**Narrow passage (narrow_10)**

| Thuật toán | Path length | Nodes | Iterations | Time TB | Time TV | ÷ RRT-Connect | Success |
|---|---|---|---|---|---|---|---|
| RRT-Connect | 285.6 (302.9) | 1733 (1545) | 10727 (9431) | 15.01 (17.30) | 10.11 | 1.00 | 100.0 (100) |
| GB-RRT-Connect | 285.8 (302.9) | 1738 (1469) | 15329 (12621) | 18.19 (18.52) | 13.04 | 1.21 (1.07) | 100.0 (99.5) |
| EBi-RRT | 287.3 (312.1) | 2287 (7635) | 6071 (18414) | 128.2 (1066.7) | 78.16 | 8.54 (61.66) | 100.0 (95.8) |
| **RRT-FConnect** | **277.7** (280.0) | **1395** (1179) | 5766 (4578) | 13.76 (16.16) | 9.83 | 0.92 (0.93) | 100.0 (100) |
| RRT-FConnect (code gốc) | 274.6 | 1514 | 12254 | 52.45 | 35.61 | 3.56 | 100.0 |

**Đối chiếu với các khẳng định của Table 2:**

| Khẳng định của paper | Của mình |
|---|---|
| RRT-FConnect cho đường ngắn nhất ở cả 4 môi trường | Đúng |
| RRT-FConnect dùng ít node nhất ở cả 4 môi trường | Đúng |
| RRT-FConnect dùng ít vòng lặp nhất ở cả 4 môi trường | Đúng (ở Narrow sát EBi-RRT: 5766 so với 6071) |
| RRT-FConnect nhanh nhất ở General, Highly, Narrow; ở Simple chậm hơn RRT-Connect một chút | Đúng. Tỉ lệ so với RRT-Connect lệch paper không quá 0.01 ở Simple, Highly, Narrow; ở General thì nhanh hơn paper (0.74 so với 0.95) |
| Success ở Highly: RRT-FConnect > RRT-Connect > EBi-RRT > GB-RRT-Connect | Đúng thứ tự (99.7 > 98.0 > 97.5 > 91.8; paper 95 > 93 > 88 > 86). Bản đồ của mình dễ hơn một chút, nên mọi thuật toán đều cao hơn paper |

**Còn lệch:**
- Số vòng lặp của RRT-FConnect: −31% (Simple), −26% (General), +15% (Highly), +26% (Narrow) so với paper. Baseline cũng lệch theo cùng hướng ở Simple/General (RRT-Connect −26%, −15%), nên một phần là do bản đồ khác.
- Thời gian tuyệt đối ở Simple và General nhanh hơn paper khoảng 2 lần (máy khác); ở Highly và Narrow gần bằng paper.
- Ở Highly và Narrow, RRT-FConnect vẫn cho đường ngắn nhất nhưng cách biệt nhỏ hơn paper: ngắn hơn RRT-Connect 7.7% và 2.8%, so với 12.1% và 7.6% trong paper.
- EBi-RRT ở Narrow khác paper rất xa (2287 node, 6071 vòng lặp, 128 ms so với 7635, 18414, 1067 ms). Ánh xạ `BRRT_Case1` = EBi-RRT chưa được xác nhận.

**Thành công theo ngân sách thời gian (Highly cluttered).** Trung bình trên các lần chạy thành công không cho biết trong cùng một khoảng thời gian thì thuật toán nào giải được nhiều bài hơn. Bảng dưới tính trên mọi lần chạy: tỉ lệ lần chạy tìm được đường trong không quá T ms. Ngân sách lớn nhất là 200 ms, vì thuật toán nhanh nhất đã tiêu hết 100.000 vòng lặp sau khoảng 230 ms.

| Thuật toán | ≤ 5 ms | ≤ 10 ms | ≤ 20 ms | ≤ 50 ms | ≤ 100 ms | ≤ 200 ms | Success (100k vòng) |
|---|---|---|---|---|---|---|---|
| RRT-Connect | 1.1% | 3.8% | 10.5% | 30.4% | 57.2% | 84.3% | 98.0% |
| GB-RRT-Connect | 0.9% | 2.5% | 7.9% | 25.9% | 53.2% | 81.4% | 91.8% |
| EBi-RRT | 0.1% | 1.0% | 3.8% | 13.6% | 27.6% | 42.0% | 97.5% |
| **RRT-FConnect** | **10.2%** | **20.4%** | **31.2%** | **58.3%** | **76.8%** | **93.1%** | **99.7%** |

RRT-FConnect dẫn ở mọi ngân sách: trong 50 ms nó giải được 58% số lần chạy, gần gấp đôi RRT-Connect (30%).

### Kết quả: flappy và rooms (`clean/full_final`)

Các môi trường này không có trong paper. Mỗi thuật toán có 1000 lần chạy trên mỗi môi trường. Chữ đậm là tốt nhất trong cột, tính trung bình trên các lần chạy thành công.

Ba thuật toán RRT-Connect, GB-RRT-Connect và RRT-FConnect chạy xen kẽ trong `full_final` (giai đoạn B). EBi-RRT chạy ở lượt riêng `clean/ebi_new` (giai đoạn E), xen kẽ với RRT-FConnect.

Theo RRT-FConnect, thuật toán có mặt ở cả hai lượt, thì lượt EBi chạy chậm hơn 4–19%. Vì vậy tỉ lệ thời gian của EBi-RRT được quy đổi như sau: (EBi-RRT ÷ RRT-FConnect cùng lượt) × (RRT-FConnect ÷ RRT-Connect của `full_final`). Riêng các cột ms của EBi-RRT giữ nguyên số đo.

| Môi trường | Iterations RRT-FConnect / RRT-Connect | Time RRT-FConnect ÷ RRT-Connect | Time EBi-RRT ÷ RRT-Connect | Success RRT-FConnect / RRT-Connect / GB-RRT-Connect / EBi-RRT (%) |
|---|---|---|---|---|
| flappy_10 | 6632 / 9031 | 1.38 | 20.2 | 100 / 100 / 100 / 100 |
| flappy_20 | 25825 / 33842 | 1.40 | 15.9 | 99.8 / 96.4 / 85.4 / 88.0 |
| flappy_40 | 55261 / 72305 | 1.88 | 8.9 | 18.5 / 5.5 / 2.5 / 7.4 |
| rooms_10 | 2537 / 3571 | 1.42 | 14.4 | 100 / 100 / 100 / 100 |
| rooms_20 | 7337 / 13276 | 1.02 | 13.8 | 100 / 99.9 / 99.7 / 100 |
| rooms_40 | 63825 / 78294 | 1.68 | 10.1 | 55.5 / 8.8 / 3.1 / 13.1 |

**flappy_10**

| Thuật toán | Path length | Nodes | Iterations | Time TB | Time TV | ÷ RRT-Connect | Success |
|---|---|---|---|---|---|---|---|
| RRT-Connect | 368.9 | 3552 | 9031 | 29.39 | 16.50 | 1.00 | 100.0 |
| GB-RRT-Connect | 368.8 | 3610 | 13033 | 35.01 | 20.61 | 1.19 | 100.0 |
| EBi-RRT | 377.1 | 5412 | 12464 | 633.6 | 399.6 | 20.20 | 100.0 |
| **RRT-FConnect** | **362.8** | **2955** | 6632 | 40.42 | 24.91 | 1.38 | 100.0 |

**flappy_20**

| Thuật toán | Path length | Nodes | Iterations | Time TB | Time TV | ÷ RRT-Connect | Success |
|---|---|---|---|---|---|---|---|
| RRT-Connect | 417.1 | 7694 | 33842 | 137.2 | 90.47 | 1.00 | 96.4 |
| GB-RRT-Connect | **388.9** | **6448** | 40769 | 121.8 | 90.27 | 0.89 | 85.4 |
| EBi-RRT | 398.7 | 11813 | 37807 | 2319.5 | 1555.5 | 15.91 | 88.0 |
| **RRT-FConnect** | 422.2 | 7155 | 25825 | 192.5 | 107.4 | 1.40 | 99.8 |

**flappy_40**

| Thuật toán | Path length | Nodes | Iterations | Time TB | Time TV | ÷ RRT-Connect | Success |
|---|---|---|---|---|---|---|---|
| RRT-Connect | 266.2 | 3521 | 72305 | 116.9 | 99.11 | 1.00 | 5.5 |
| GB-RRT-Connect | **254.6** | **3046** | 79303 | 119.0 | 111.9 | 1.02 | 2.5 |
| EBi-RRT | 271.8 | 8087 | 49823 | 1282.5 | 961.9 | 8.85 | 7.4 |
| **RRT-FConnect** | 287.6 | 6028 | 55261 | 219.9 | 182.5 | 1.88 | 18.5 |

**rooms_10**

| Thuật toán | Path length | Nodes | Iterations | Time TB | Time TV | ÷ RRT-Connect | Success |
|---|---|---|---|---|---|---|---|
| RRT-Connect | 245.0 | 1497 | 3571 | 7.17 | 4.38 | 1.00 | 100.0 |
| GB-RRT-Connect | 242.2 | 1496 | 5151 | 8.62 | 5.32 | 1.20 | 100.0 |
| EBi-RRT | 240.2 | 2200 | 5448 | 107.5 | 55.09 | 14.40 | 100.0 |
| **RRT-FConnect** | **238.2** | **1341** | 2537 | 10.17 | 5.59 | 1.42 | 100.0 |

**rooms_20**

| Thuật toán | Path length | Nodes | Iterations | Time TB | Time TV | ÷ RRT-Connect | Success |
|---|---|---|---|---|---|---|---|
| RRT-Connect | 275.6 | 2757 | 13276 | 29.11 | 20.39 | 1.00 | 99.9 |
| GB-RRT-Connect | 273.0 | 2717 | 18514 | 33.77 | 24.16 | 1.16 | 99.7 |
| EBi-RRT | 267.2 | 4378 | 16197 | 429.5 | 290.3 | 13.82 | 100.0 |
| **RRT-FConnect** | **262.4** | **2230** | 7337 | 29.78 | 20.05 | 1.02 | 100.0 |

**rooms_40**

| Thuật toán | Path length | Nodes | Iterations | Time TB | Time TV | ÷ RRT-Connect | Success |
|---|---|---|---|---|---|---|---|
| RRT-Connect | 273.9 | 4737 | 78294 | 176.8 | 171.0 | 1.00 | 8.8 |
| GB-RRT-Connect | **265.0** | **3275** | 74579 | 115.8 | 103.3 | 0.66 | 3.1 |
| EBi-RRT | 294.8 | 10678 | 68046 | 2013.3 | 1923.4 | 10.08 | 13.1 |
| **RRT-FConnect** | 308.1 | 7169 | 63825 | 297.0 | 275.7 | 1.68 | 55.5 |

**Nhận xét:**

1. **RRT-FConnect dùng ít vòng lặp nhất ở cả 6 môi trường, nhưng chậm hơn RRT-Connect 1.4–1.9 lần** (riêng rooms_20 là 1.02).
   - Số vòng lặp chỉ giảm 18–45% so với RRT-Connect, trong khi ở blobs_40 giảm 60%.
   - Mỗi vòng lặp của RRT-FConnect lại tốn khoảng gấp đôi: 6.1 so với 3.3 µs ở flappy_10, 4.0 so với 2.0 µs ở rooms_10.
   - Giả thuyết: heuristic chọn cặp dùng khoảng cách đường thẳng và không biết tới tường. Ở flappy/rooms, cặp node gần nhau nhất thường nằm hai bên một bức tường, nên hướng dẫn sai, và thuật toán phải dựa vào chế độ exploration.
2. **Độ dài và số node: RRT-FConnect tốt nhất ở flappy_10, rooms_10 và rooms_20.**
   - Ở flappy_20/40 và rooms_40, bảng trên cho thấy GB-RRT-Connect cho đường ngắn hơn. Đó là sai lệch do chọn mẫu: trung bình chỉ tính trên lần chạy thành công, mà GB-RRT-Connect bỏ cuộc ở các bài khó.
   - So ghép cặp trên 847 lần chạy (map, bài, lần thử) mà cả 3 thuật toán đều giải được ở flappy_20, RRT-FConnect tốt nhất ở cả độ dài đường đi, số node lẫn số vòng lặp, nhưng vẫn chậm hơn RRT-Connect:

     | flappy_20, 847 lần chạy chung | Path length | Nodes | Iterations | Time TB (ms) |
     |---|---|---|---|---|
     | RRT-Connect | 387.6 | 6686 | 29556 | 111.5 |
     | GB-RRT-Connect | 386.8 | 6414 | 40524 | 121.0 |
     | RRT-FConnect | **381.3** | **5736** | **20494** | 136.5 |

   - flappy_40 và rooms_40 chỉ có 8 và 4 lần chạy chung, quá ít để so ghép cặp.
3. **Mật độ 40%: RRT-FConnect thành công nhiều hơn hẳn trong giới hạn 100.000 vòng lặp**: 18.5% so với 5.5% ở flappy_40, và 55.5% so với 8.8% ở rooms_40.
   - Tỉ lệ thấp là do thuật toán gặp khó, không phải do bài vô nghiệm. Kiểm tra bằng BFS trên chính lưới chiếm chỗ của planner (voxel 0,5 m, không phình vật cản): flappy_40 có 99/100 bài có lời giải, rooms_40 có 98/100. Kiểm tra chéo: không có bài nào planner giải được mà BFS báo vô nghiệm.
   - Ở flappy_40, 61 bài có lời giải nhưng không thuật toán nào giải được trong 30 lần thử (10 lần × 3 thuật toán).
   - Với cùng ngân sách thời gian thì khác biệt rất nhỏ (bảng dưới). Ở hai môi trường này, ngân sách lớn nhất so được là 50 ms, vì RRT-Connect hết 100.000 vòng lặp sau khoảng 83–96 ms. Để so công bằng ở đây cần chạy lại với giới hạn thời gian thay vì giới hạn vòng lặp (mục Việc tiếp theo).
4. **EBi-RRT chậm nhất, chậm hơn RRT-Connect 8.9–20 lần.**
   - Mỗi vòng lặp của nó tốn gấp 9–16 lần RRT-Connect, và nó dùng nhiều node nhất (gấp 1.5–2.3 lần RRT-Connect).
   - Ở mật độ 40%, success của EBi-RRT (7.4% ở flappy_40, 13.1% ở rooms_40) cao hơn RRT-Connect nhưng thấp hơn nhiều so với RRT-FConnect. Ở flappy_20, nó đạt 88.0%, thấp hơn RRT-Connect (96.4%).
   - Nếu chia thẳng thời gian của EBi-RRT cho RRT-Connect của `full_final`, không quy đổi, thì ra 11–22 lần. Kết luận vẫn vậy.
   - EBi-RRT không có trong bảng so ghép cặp và bảng ngân sách thời gian, vì nó chạy ở lượt khác.

**Thành công theo ngân sách thời gian** (tỉ lệ trên mọi lần chạy; RRT-FConnect / RRT-Connect):

| Môi trường | ≤ 20 ms | ≤ 50 ms | ≤ 200 ms | Success (100k vòng) |
|---|---|---|---|---|
| blobs_40 | 31.2% / 10.5% | 58.3% / 30.4% | 93.1% / 84.3% | 99.7% / 98.0% |
| flappy_20 | 19.5% / 21.4% | 31.1% / 34.3% | 65.0% / 71.3% | 99.8% / 96.4% |
| rooms_20 | 49.9% / 49.3% | 81.4% / 83.6% | 99.7% / 99.6% | 100% / 99.9% |
| flappy_40 | 0.8% / 0.1% | 2.7% / 0.8% | – | 18.5% / 5.5% |
| rooms_40 | 0.8% / 0.0% | 2.5% / 0.5% | – | 55.5% / 8.8% |

Ở flappy_20 và rooms_20, với cùng ngân sách thời gian, RRT-Connect giải được bằng hoặc nhiều hơn RRT-FConnect. RRT-FConnect chỉ hơn khi được chạy đủ lâu: success trong 100.000 vòng lặp cao hơn.

### Code gốc so với cấu hình cuối trên flappy và rooms (`clean/sweep_3`)

5 lần mỗi cặp (500 lần chạy), RRT-Connect chạy xen kẽ. Mỗi ô ghi **code gốc → cấu hình cuối**.

| Môi trường | Iterations | Time ÷ RRT-Connect | Success (%) |
|---|---|---|---|
| flappy_10 | 10494 → 6691 | 7.07 → 1.31 | 100 → 100 |
| flappy_20 | 36097 → 27056 | 5.51 → 1.51 | 89.8 → 99.8 |
| flappy_40 | 60967 → 59588 | 3.58 → 1.95 | 6.0 → 20.6 |
| rooms_10 | 4088 → 2402 | 4.79 → 1.33 | 100 → 100 |
| rooms_20 | 12426 → 7061 | 4.08 → 0.98 | 100 → 100 |
| rooms_40 | 63546 → 61631 | 3.29 → 1.67 | 16.4 → 56.8 |

Các sửa đổi vẫn có ích trên những bản đồ mà paper không dùng:
- Tính theo tỉ lệ với RRT-Connect, cấu hình cuối nhanh hơn code gốc 1.8–5.4 lần.
- Success ở mật độ 40% tăng 3.4–3.5 lần.
- Tỉ lệ thời gian của cấu hình cuối ở đây khớp với `full_final` ở trên, lệch không quá 0.11.

### Máy có quyết định nhanh hay chậm không

Thời gian của một lần chạy bằng số vòng lặp nhân với thời gian mỗi vòng. Vì vậy tỉ lệ thời gian RRT-FConnect ÷ RRT-Connect (F÷R) là tích của hai tỉ lệ:
- **Tỉ lệ số vòng lặp:** chỉ phụ thuộc thuật toán, bản đồ và bài toán, không phụ thuộc máy.
- **Tỉ lệ chi phí mỗi vòng:** phụ thuộc lượng việc trong mỗi vòng (kho cặp, k-NN, APF), và có thể phụ thuộc máy (CPU, cache).

Bảng dưới tách hai phần này từ `clean/full_final`. Số liệu tính trên các lần chạy thành công, giống các bảng ở trên. Số trong ngoặc là của paper, suy ra từ Table 2: thời gian chia cho số vòng lặp.

| Môi trường | Time F÷R | Số vòng F÷R | Chi phí mỗi vòng F÷R | µs mỗi vòng của RRT-Connect |
|---|---|---|---|---|
| Simple (blobs_10) | 1.11 (1.12) | 0.44 (0.48) | 2.49 (2.35) | 1.54 (2.36) |
| General (blobs_20) | 0.74 (0.95) | 0.38 (0.43) | 1.96 (2.21) | 1.61 (2.39) |
| Highly (blobs_40) | 0.63 (0.64) | 0.40 (0.38) | 1.56 (1.69) | 2.87 (3.36) |
| Narrow (narrow_10) | 0.92 (0.93) | 0.54 (0.49) | 1.71 (1.92) | 1.40 (1.83) |
| flappy_10 | 1.38 | 0.73 | 1.87 | 3.25 |
| flappy_20 | 1.40 | 0.76 | 1.84 | 4.05 |
| flappy_40 | 1.88 | 0.76 | 2.46 | 1.62 |
| rooms_10 | 1.42 | 0.71 | 2.00 | 2.01 |
| rooms_20 | 1.02 | 0.55 | 1.85 | 2.19 |
| rooms_40 | 1.68 | 0.82 | 2.06 | 2.26 |

Sai số (khoảng tin cậy 95%) của cột Time F÷R khoảng ±8–15%, của số vòng ±6–12%, của chi phí mỗi vòng ±4–6%. Ở flappy_40 và rooms_40, nơi ít lần chạy thành công, sai số lớn hơn: tới ±24% (thời gian) và ±15% (chi phí mỗi vòng).

Nhận xét:
- **Số ms tuyệt đối phụ thuộc máy.** Mỗi vòng RRT-Connect trên máy này nhanh hơn trên máy của paper 1.2–1.5 lần (một phần do bản đồ khác). Vì vậy chỉ nên so tỉ lệ, không so ms.
- **Chi phí mỗi vòng F÷R gần như không đổi khi đổi máy.**
  - Cấu hình cuối được đo hai lần trên máy này, cách nhau vài giờ: `full_final` lúc 06:30–07:30, `sweep_4` và `sweep_3` lúc 07:40–09:00. Ở cả 10 môi trường, chi phí mỗi vòng F÷R của hai lần lệch nhau không quá 6%.
  - Trong cùng hai lần đo đó, thời gian mỗi vòng của chính RRT-Connect có chỗ lệch tới 19% (blobs_40: 2.87 so với 2.42 µs).
  - So với máy của paper, chi phí mỗi vòng F÷R lệch từ −11% đến +6%.
- **Nhanh hay chậm là do thuật toán và bản đồ.**
  - Mỗi vòng RRT-FConnect tốn gấp 1.6–2.5 lần RRT-Connect. Vì vậy nó chỉ nhanh hơn khi số vòng lặp giảm xuống dưới 1 ÷ (chi phí mỗi vòng F÷R), tức khoảng 0.4–0.65 lần.
  - Ở General, Highly và Narrow, số vòng giảm còn 0.38–0.54 lần, đủ để nhanh hơn. Ở Simple, số vòng là 0.44 lần, nhỉnh hơn mức hòa 0.40 nên chậm hơn một chút, giống paper.
  - Ở flappy/rooms, số vòng chỉ giảm còn 0.55–0.82 lần nên chậm hơn. Riêng rooms_20 (1.02) coi như ngang.
- **Đổi máy chỉ làm tỉ lệ thời gian xê dịch khoảng 10%.** Mức này có thể đảo kết quả ở chỗ đang ngang như rooms_20, nhưng không đảo được những chỗ chậm hơn 1.4–1.9 lần.
- **Phần lệch còn lại ở General (0.74 so với 0.95) đến từ hai nguồn gần bằng nhau.** Một là số vòng lặp (0.38 so với 0.43), do bản đồ khác. Hai là chi phí mỗi vòng (1.96 so với 2.21), có thể do máy hoặc do bản đồ. Không có bản đồ và máy của paper thì không tách được hai nguồn này.
- **Tốc độ tuyệt đối dao động theo thời điểm.** Máy này dùng CPU lai Intel Core Ultra 5 225H (4 nhân P, 10 nhân E), và Windows tự chọn nhân để chạy WSL. Dao động này không làm lệch tỉ lệ: các thuật toán chạy xen kẽ trên cùng bài toán, cách nhau vài giây, nên chịu cùng một mức dao động.

---

## Kết luận

Dựa trên số liệu sạch của lần 9.

1. **Reproduce thành công ở 4 môi trường của paper.**
   - Mọi kết luận định tính của Table 2 đều tái hiện được: RRT-FConnect cho đường ngắn nhất, dùng ít node nhất và ít vòng lặp nhất; nhanh nhất ở General, Highly và Narrow; chậm hơn RRT-Connect một chút ở Simple; success cao nhất ở Highly.
   - Về định lượng, tỉ lệ thời gian so với RRT-Connect lệch paper không quá 0.01 ở Simple, Highly và Narrow; ở General thì nhanh hơn paper (0.74 so với 0.95). Sai số của các tỉ lệ này khoảng ±10–15%, nên mức khớp tới 0.01 có phần may mắn. Còn chênh lệch ở General thì vượt sai số (khoảng tin cậy 0.64–0.84).
   - Độ dài đường đi lệch paper không quá 8%, số node không quá 19%, số vòng lặp từ −31% đến +26%. Baseline cũng lệch cỡ đó, vì bản đồ do mình tự sinh.
2. **Để khớp paper, code gốc cần 5 sửa đổi, và cả 5 đều đúng với mô tả của paper:**
   - kho 100 cặp tốt nhất. Đây là nguyên nhân chính về thời gian: code gốc chậm hơn RRT-Connect 2.3–3.6 lần;
   - k-NN chính xác;
   - R_max = 5;
   - không bỏ mẫu nằm trong vật cản. Đây là nguyên nhân chính về số vòng lặp;
   - APF luôn áp.

   Ngoài ra phải giữ chế độ exploration của code gốc, dù paper không nhắc tới. Tắt nó đi thì chậm hơn RRT-Connect 1.8–2.7 lần ở General, Highly và Narrow, và success ở Highly giảm.
3. **Ngoài paper (flappy/rooms):**
   - RRT-FConnect vẫn dùng ít vòng lặp nhất, và ở mật độ 40% giải được nhiều bài hơn hẳn (rooms_40: 55.5% so với 8.8% của RRT-Connect và 13.1% của EBi-RRT).
   - EBi-RRT chậm nhất, chậm hơn RRT-Connect 9–20 lần.
   - Nhưng nó chậm hơn RRT-Connect, tới 1.9 lần; riêng rooms_20 thì ngang. Lý do: số vòng lặp chỉ giảm còn 0.55–0.82 lần, không bù được chi phí mỗi vòng cao gấp 1.8–2.5 lần (mục "Máy có quyết định nhanh hay chậm không").
   - Giả thuyết cho việc số vòng giảm ít: heuristic chọn cặp dùng khoảng cách đường thẳng nên không biết tới tường.
   - Điểm này quan trọng cho hướng multi-robot, vì các bản đồ có khe hẹp (mục 8.2 của [HUONG_DAN_MULTI_ROBOT.md](HUONG_DAN_MULTI_ROBOT.md)).
4. **Đo thời gian:**
   - Đồng hồ của WSL (timesyncd giằng co với Hyper-V) đã làm nhiễu số liệu thời gian của lần 1–6. Lỗi đã được sửa, và mọi kết luận về thời gian đều lấy từ lần 9.
   - Máy chỉ ảnh hưởng tới số ms tuyệt đối. Tỉ lệ chi phí mỗi vòng so với RRT-Connect lệch không quá 6% giữa hai lần đo trên máy này, và không quá 11% so với máy của paper.
   - Mỗi tỉ lệ có sai số ngẫu nhiên, vì số vòng lặp có đuôi dài: ±8–15% với 1000 lần chạy (`full_final`), tới ±47% với 300 lần chạy của một lượt quét. Vì vậy kết luận từ các vòng quét chỉ dựa trên chênh lệch lớn, hoặc trên chi phí mỗi vòng, vốn có sai số nhỏ hơn nhiều.
   - `check_timing.py` (tìm lần chạy bị nhiễu) giờ so cả thời gian mỗi node, vì lần chạy mọc cây lớn bất thường thì chậm thật, không phải bị nhiễu (giai đoạn F của lần 9).
5. **Còn mở:**
   - EBi-RRT (`BRRT_Case1`) khác paper rất xa ở Narrow, nên ánh xạ thuật toán chưa được xác nhận.
   - Số vòng lặp ở Highly và Narrow còn cao hơn paper 15–26%.
   - flappy_40 và rooms_40 cần thí nghiệm giới hạn theo thời gian mới so được công bằng.

## Việc tiếp theo

- [x] Lần 4 (`sweep_4`): thử H9–H11, chọn cấu hình cuối `paper_table1_final.json`.
- [x] Lần 5 (`full_run_3`) và lần 6 (`sweep_3`): chạy xong, nhưng thời gian đo bằng đồng hồ lỗi.
- [x] Lần 7: tìm ra lỗi đồng hồ WSL; sửa môi trường, code đo thời gian và `check_timing.py`.
- [x] Lần 9: chạy lại sạch mọi thí nghiệm có số liệu thời gian, rồi điền bảng kết quả cuối (gồm EBi-RRT trên flappy/rooms, việc của lần 8 cũ).
- [ ] flappy_40 và rooms_40: chạy với giới hạn thời gian (ví dụ 1 s) thay vì 100.000 vòng lặp. Giới hạn vòng lặp làm thuật toán có vòng lặp rẻ dừng sớm hơn về thời gian, nên chưa so được success công bằng.
- [ ] Sinh kịch bản có kiểm tra start/goal nối được với nhau (BFS trên lưới chiếm chỗ). Hiện tại flappy_40 có 1/100 và rooms_40 có 2/100 bài vô nghiệm.
- [ ] (Tuỳ chọn) Thử heuristic dùng khoảng cách trên lưới (geodesic) cho flappy/rooms, nơi khoảng cách đường thẳng dẫn hướng sai qua tường.
