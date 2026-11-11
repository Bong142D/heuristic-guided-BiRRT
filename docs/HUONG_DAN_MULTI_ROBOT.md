# Hoạch định đường đi đa robot trên bản đồ tĩnh dựa trên RRT-FConnect

Tài liệu định hướng cho giai đoạn đa robot của đồ án *Hoạch định đường đi đa robot dựa vào lấy mẫu có định hướng*.

Bản 04/11/2026 thay cho bản 29/10. Phần đa robot được chia thành hai giai đoạn, làm lần lượt:

- **Giai đoạn 1, đang làm: path planning trên bản đồ tĩnh.** Tìm N đường đi hình học và các vùng giao nhau giữa chúng. Chưa có thời gian hay vận tốc. Các đường **được phép đi gần nhau**.
- **Giai đoạn 2, làm sau: phối hợp.** Xếp thứ tự qua vùng giao nhau, đặt thời gian chờ và vận tốc, dựa trên đầu ra của giai đoạn 1.

Tài liệu trả lời:
1. Phạm vi và bài toán là gì (mục 1).
2. Đánh giá bằng tiêu chí nào (mục 2), trên những kịch bản nào (mục 3).
3. Các hướng đã có và hướng đồ án chọn (mục 4–5).
4. Câu hỏi nghiên cứu, thiết kế và thí nghiệm (mục 6–8).
5. **Làm gì tiếp theo, theo thứ tự nào, sửa ở đâu trong code, khi nào coi là xong** (mục 9). Những điểm nhóm cần chốt nằm ở mục 10.

**Bắt đầu từ đâu:** bảng theo dõi của giai đoạn 1 ở mục 9.

**Cách đọc chứng cứ.** Mỗi khẳng định về công trình khác có số tham khảo `[n]`, danh sách ở mục 11:
- ✓: đã đối chiếu với tóm tắt hoặc nội dung bài.
- ~: chỉ xác nhận được thông tin thư mục, hoặc qua nguồn thứ cấp. Cần đọc bản gốc trước khi trích trong báo cáo.

Số liệu single robot của đồ án lấy từ lần 9 trong [KET_QUA_REPRODUCE.md](KET_QUA_REPRODUCE.md). Code single robot: [HUONG_DAN_RRT-FConnect.md](HUONG_DAN_RRT-FConnect.md). Lệnh chạy: mục 7 của [HowToRunCode.md](HowToRunCode.md).

**Mục lục**
1. [Bài toán](#1-bài-toán)
2. [Tiêu chí đánh giá](#2-tiêu-chí-đánh-giá)
3. [Kịch bản và phân loại thất bại](#3-kịch-bản-và-phân-loại-thất-bại)
4. [Các hướng đã có](#4-các-hướng-đã-có)
5. [Hướng của đồ án](#5-hướng-của-đồ-án)
6. [Câu hỏi nghiên cứu](#6-câu-hỏi-nghiên-cứu)
7. [Thiết kế kỹ thuật](#7-thiết-kế-kỹ-thuật)
8. [Thiết kế thí nghiệm](#8-thiết-kế-thí-nghiệm)
9. [Hướng dẫn làm tiếp và bảng theo dõi](#9-hướng-dẫn-làm-tiếp-và-bảng-theo-dõi)
10. [Câu hỏi cần chốt](#10-câu-hỏi-cần-chốt)
11. [Tài liệu tham khảo](#11-tài-liệu-tham-khảo)

---

## 1. Bài toán

### 1.1. Phạm vi

| | Giai đoạn 1: path planning | Giai đoạn 2: phối hợp | Ngoài phạm vi |
|---|---|---|---|
| Câu hỏi | Mỗi robot đi theo đường nào? | Ai qua trước, chờ bao lâu, đi nhanh bao nhiêu? | |
| Thời gian, vận tốc | Không có | Có | Quỹ đạo có gia tốc, động học phức tạp |
| Môi trường | Bản đồ tĩnh, biết trước | Như giai đoạn 1 | Vật cản động, vật cản mới, lối đi bị đóng |
| Robot tránh nhau | Chưa xử lý: chỉ tìm ra chỗ cần tránh | Thứ tự qua vùng giao nhau, dừng chờ | Lập kế hoạch không gian–thời gian, tránh phản ứng |
| Sự cố khi chạy | Không xét | Không xét | Robot trễ, robot hỏng, lập lại đường |
| Đầu ra | N đường và các vùng giao nhau | Kế hoạch chạy được | |
| Kiểm tra | Đo hình học | Mô phỏng robot chạy | |

Tĩnh ở đây nghĩa là môi trường tĩnh. Các kịch bản sự cố S1–S8 của bản 29/10 đã được nhóm bỏ khỏi phạm vi.

### 1.2. Phát biểu giai đoạn 1

Cho:
- bản đồ chiếm chỗ tĩnh 2D (200 × 200 m, lưới 0.5 m, như dataset hiện có);
- N robot hình đĩa bán kính r. Robot i có điểm đầu `s_i` và điểm đích `g_i`;
- lề an toàn m. Đặt `D = 2r + m`.

Cần tìm:
- với mỗi robot i, một đường gấp khúc `P_i` nối `s_i` với `g_i`, cách vật cản ít nhất `D/2`;
- tất cả **vùng giao nhau**: mỗi vùng là một đoạn liên tiếp mà hai đường cách nhau dưới `D`, ghi bằng khoảng độ dài cung `[vào, ra]` trên cả hai đường.

Hai đường được phép đi gần nhau hoặc cắt nhau. Mục tiêu là lập nhanh, đường ngắn và **ít chồng chéo**, vì mỗi vùng giao nhau là một việc giai đoạn 2 phải xử lý.

**Giai đoạn 1 chưa đảm bảo robot không va chạm.** Điều đó chỉ có sau giai đoạn 2, trừ khi các đường không có vùng giao nhau nào: khi đó tâm hai robot luôn cách nhau ít nhất `D`, với mọi vận tốc (M4, mục 8).

### 1.3. Giả định mặc định

| Giả định | Đề xuất | Lý do |
|---|---|---|
| Không gian | 2D (x, y) | Tia LEA và APF của RRT-FConnect đều làm trên mặt phẳng |
| Robot | Đĩa r = 0.25 m, m = 0.1 m, nên D = 0.6 m | r = 0.5 m làm nhiều bản đồ vỡ thành mảnh (mục 3.3) |
| Lập kế hoạch | Tập trung: một máy biết mọi robot và bản đồ | Phạm vi đồ án |
| Kiểm tra kết quả | Chương trình hình học riêng, độc lập với planner | Không tin kết quả do planner tự báo |

### 1.4. Giới hạn lý thuyết

- Dời nhiều vật thể độc lập là bài toán PSPACE-hard [2]. Dời nhiều đĩa là NP-hard mạnh [3].
- Tách bài toán thành đường rồi phối hợp là không đầy đủ: phối hợp chỉ đầy đủ so với các đường đã chọn [9]. Đường chọn ở giai đoạn 1 có thể khiến giai đoạn 2 vô nghiệm, ví dụ khi đích của hai robot cùng nằm trong một vùng giao nhau: robot nào tới trước cũng đứng đó chắn robot kia mãi. Vì vậy giai đoạn 1 đo riêng số vùng có điểm đầu, đích, và phương pháp chính tránh chúng từ đầu (giữ chỗ, mục 5.2).
- Mức nghiêm ngặt hơn, các đường tách hẳn nhau, cũng khó: tìm k đường rời nhau là NP-complete kể cả trên đồ thị phẳng [43], tìm K đường dày không cắt nhau là NP-hard khi số lỗ không cố định [41]. Cách làm lần lượt có thể không có thứ tự nào thành công [44].

Vì vậy báo cáo không hứa hẹn tính đầy đủ hay tối ưu. Thay vào đó, đo bằng thí nghiệm và phân loại các ca thất bại.

---

## 2. Tiêu chí đánh giá

### 2.1. Vấn đề và tiêu chí trong paper RRT-FConnect

**Vấn đề paper giải quyết** [6]:
- Mục tiêu: tìm **nhanh một lời giải khả thi** thay vì tối ưu dần, trong môi trường **nhiều vật cản và có lối hẹp**.
- Nhược điểm của RRT lấy mẫu đều mà paper nêu:
  - tốn công khám phá vùng không liên quan, nên hội tụ chậm;
  - ở vùng dày vật cản, đường đi vòng nhiều, chất lượng kém;
  - ở lối hẹp, xác suất lấy được mẫu trong vùng trống thấp.

**Năm tiêu chí của paper.** Dùng nguyên cho mọi lần gọi planner:

| Tiêu chí | Ý nghĩa theo paper | Ghi chú cho đồ án |
|---|---|---|
| Path length | Chất lượng đường | Độ dài hình học thật, đo trên XY |
| Number of nodes | Bộ nhớ, độ phức tạp khám phá | |
| Number of iterations | Tốc độ hội tụ | Không phụ thuộc máy |
| Search time | Hiệu quả tính toán | So bằng tỉ lệ với RRT-Connect chạy xen kẽ, vì số ms phụ thuộc máy |
| Success rate | Độ tin cậy, trong giới hạn vòng lặp | Với đa robot: dùng giới hạn thời gian thay cho giới hạn vòng lặp (mục 8) |

Paper còn đánh giá độ ổn định qua phân bố (trung vị, IQR). Đồ án báo cáo trung vị, IQR và khoảng tin cậy.

**Chỗ cần kiểm chứng.** Ở single robot, RRT-FConnect nhanh hơn RRT-Connect 8–37% ở General, Highly, Narrow, nhưng chậm hơn tới 1.9 lần trên flappy/rooms. Các số đó đo với robot dạng điểm. Robot có bán kính làm bản đồ chật hơn, nên kết quả có thể khác (RQ1).

### 2.2. Tiêu chí của tập đường (giai đoạn 1)

| Tiêu chí | Định nghĩa |
|---|---|
| Success | Mọi robot có đường trong giới hạn, kiểm tra độc lập đạt (5.3) |
| Tổng độ dài, độ dài lớn nhất | `Σ độ dài P_i` và `max độ dài P_i`. Tỉ lệ với sum of costs và makespan của MAPF [1] khi không ai phải chờ, nên là cận dưới của hai đại lượng đó |
| Thời gian lập | Tổng thời gian tìm đường của N robot; thời gian chặn ô và BFS; thời gian tìm vùng giao nhau, ghi riêng |
| Số vùng giao nhau, số cặp xung đột | Số việc giai đoạn 2 phải xử lý |
| Độ dài vùng giao nhau | Tổng độ dài các vùng, lấy trung bình trên hai đường của mỗi vùng |
| Tỉ lệ chồng chéo | Phần tổng độ dài đường nằm trong ít nhất một vùng |
| Số vùng có điểm đầu, đích | Vùng chứa điểm đầu hoặc đích của một trong hai robot. Đây là loại dễ gây bế tắc nhất ở giai đoạn 2 |
| Số vùng đối đầu, cùng chiều | Vùng dài ít nhất `5D` trên cả hai đường, tức hai đường thật sự chạy dọc nhau; ngắn hơn thì coi là cắt nhau. Đối đầu trong khe hẹp là loại khó nhất cho giai đoạn 2 |

Mọi tiêu chí được xem theo số robot N, mật độ vật cản, độ rộng khe và loại kịch bản.

### 2.3. Tiêu chí chất lượng đường

Benchmark planner cho robot di động [5] dùng thêm độ cong và khoảng cách tới vật cản. Với đồ án:
- **Độ trơn:** tổng góc quay và góc quay lớn nhất tại các đỉnh. Nếu có rút gọn đường (I3) thì báo cáo cả trước và sau.
- **Clearance:** khoảng cách nhỏ nhất từ đường tới vật cản, do chương trình kiểm tra độc lập đo.

### 2.4. Tiêu chí của giai đoạn 2 (để sau)

Sum of costs và makespan có tính thời gian chờ, tổng thời gian chờ, số deadlock phải sửa, khoảng cách nhỏ nhất giữa hai robot khi chạy. Hai đại lượng đầu không tối ưu được đồng thời [4].

---

## 3. Kịch bản và phân loại thất bại

### 3.1. Các loại kịch bản

| Loại (`kind`) | Cách sinh | Dùng để đo |
|---|---|---|
| `random_50`, `random_150` | Điểm đầu, đích ngẫu nhiên, khoảng cách s–g ít nhất 50 hoặc 150 m | Trường hợp tổng quát. 150 m là `gen_min_dist` của single robot |
| `parallel` | Điểm đầu ở dải dưới (y từ −95 tới −75 m), đích ở dải trên (y từ 75 tới 95 m), cùng thứ tự trái phải. Đường không buộc phải cắt nhau | Dòng cùng chiều, sức chứa khe |
| `crossing` | Như `parallel`, nhưng đích xếp ngược thứ tự. Đường buộc phải cắt nhau | Vùng giao nhau do cắt |
| `swap` | Như `parallel`, nhưng cứ một robot thì đi ngược chiều | Vùng đối đầu, chỗ khó nhất cho giai đoạn 2 |

Với `random_150`, phần lớn các cặp có đoạn s–g cắt nhau: ước lượng Monte Carlo (đoạn thẳng, chưa tính vật cản) cho 64% số cặp, so với 30% ở `random_50`. Vì vậy số vùng giao nhau sẽ tăng nhanh theo N.

### 3.2. Điều kiện hợp lệ của kịch bản

`generate_multi_scenarios.py` kiểm tra ngay khi sinh:
- mọi điểm đầu, đích nằm trong ô trống của bản đồ đã nới;
- điểm của hai robot khác nhau cách nhau ít nhất `D + √2·res` (khoảng 1.3 m), để ô chứa điểm này không bị đĩa giữ chỗ của robot kia chặn (5.2);
- mỗi robot nối được s với g trên bản đồ đã nới và đã chặn đĩa giữ chỗ của mọi robot khác (BFS).

Ý này lấy từ điều kiện "hạ tầng hợp lệ" của RPP [20].

**BFS phải đi 4 hướng.** Hai ô chỉ chạm nhau ở góc thì không có lối đi. Lần thử đầu dùng 8 hướng, nên trên blobs_40 đã sinh ra kịch bản thực ra vô nghiệm: RRT-Connect chạy hết 100.000 vòng trong 0.05 s mà cây không mọc được nút nào.

### 3.3. Bản đồ dùng được

Nới vật cản theo `D/2`: một ô bị chiếm nếu hình vuông của ô cách vật cản dưới `D/2`. Cách này đảm bảo mọi điểm trong ô trống đều đủ xa, nhưng trên lưới 0.5 m, D/2 = 0.6 m làm mỗi vật cản nở thêm 1 m mỗi phía, và D/2 = 0.3 m thêm 0.5 m. Kết quả đo trên cả 5 bản đồ của mỗi môi trường, lấy trung bình:

| Môi trường | Ô trống, chưa nới | r = 0.5 m (D/2 = 0.6): ô trống | vùng lớn nhất | r = 0.25 m (D/2 = 0.3): ô trống | vùng lớn nhất |
|---|---|---|---|---|---|
| blobs_10 | 91% | 75% | 100% | 83% | 100% |
| blobs_20 | 83% | 54% | 99% | 67% | 100% |
| blobs_40 | 68% | 29% | **12%** | 44% | 91% |
| narrow_10 | 33% | 21% | 89% | 27% | 100% |
| narrow_20 | 29% | 13% | **16%** | 20% | 96% |
| narrow_40 | 22% | 4% | **4%** | 10% | **7%** |
| flappy_10 | 91% | 81% | 100% | 86% | 100% |
| flappy_20 | 81% | 49% | 72% | 64% | 100% |
| flappy_40 | 61% | 15% | **2%** | 32% | **6%** |
| rooms_10 | 91% | 71% | 100% | 81% | 100% |
| rooms_20 | 81% | 43% | 88% | 61% | 100% |
| rooms_40 | 61% | 13% | **2%** | 30% | **7%** |

"Vùng lớn nhất" là tỉ lệ ô trống thuộc vùng liên thông lớn nhất. Khi con số này nhỏ, bản đồ đã vỡ thành mảnh, không còn đường đi xa.

Hệ quả:
- Đồ án dùng r = 0.25 m, m = 0.1 m.
- Dùng 9 môi trường: blobs_10/20/40, narrow_10/20, flappy_10/20, rooms_10/20. **Bỏ narrow_40, flappy_40, rooms_40**: các bản đồ này có lời giải với robot dạng điểm, nhưng không còn lối đi cho robot có kích thước trên lưới 0.5 m.
- Muốn dùng r lớn hơn thì phải có lưới mịn hơn và sinh lại bản đồ (Q5, mục 10).

### 3.4. Sức chứa khe

Số đường đi song song tối đa qua một khe rộng w, với D = 0.6 m. Vượt con số này thì bắt buộc có vùng giao nhau, và phải nhờ giai đoạn 2:

| Khe w (m) | 1 | 2 | 3 | 4 | 6 | 10 | 20 |
|---|---|---|---|---|---|---|---|
| Liên tục: `⌊w / D⌋` | 1 | 3 | 5 | 6 | 10 | 16 | 33 |
| Lưới 0.5 m | 0 | 1 | 2 | 3 | 5 | 9 | 19 |

Hàng lưới là mức tốt nhất, tính cho đường thẳng dọc khe, tường trùng biên ô. Khe 20 m của dataset hiếm khi buộc phải chồng chéo, nên cần thêm biến thể khe hẹp 2, 4, 6 m.

Biến thể khe chỉ dùng cho flappy và rooms. Bản đồ `narrow` có một hành lang dài 140 m, nên vật cản ngẫu nhiên bít mất lối: 13/15 bản đồ thử (khe 2, 4, 6 m, mật độ 10%) không còn đường từ dải dưới lên dải trên. Cùng phép thử, rooms thông 15/15 và flappy 13/15. Trên bản đồ bị bít, script chỉ sinh được kịch bản ngẫu nhiên trong vùng lớn nhất, và báo thiếu các loại còn lại.

### 3.5. Phân loại thất bại

| `fail_class` | Điều kiện | Nghĩa là |
|---|---|---|
| `planner_limit` | BFS xác nhận có đường, nhưng planner không tìm được trong giới hạn | Lỗi của planner. Đây là chỗ so RRT-FConnect với RRT-Connect |
| `no_path` | BFS không nối được s với g trên bản đồ đã nới | Kịch bản sai. Không được xảy ra vì đã kiểm tra khi sinh |
| `blocked` | Chỉ M4: đường đã lập chặn hết lối của robot sau, và đã hết số lần đổi thứ tự | Lỗi của cách tách hẳn. Chưa chắc vô nghiệm |
| `time_total` | Hết tổng thời gian cho cả N robot | |

---

## 4. Các hướng đã có

### 4.1. Tách đường và phối hợp: nền của đồ án

**Ý tưởng.** Bước 1: tìm đường hình học cho từng robot. Bước 2: chỉ phối hợp việc đi dọc các đường đó (chờ, chậm lại, thứ tự) để tránh nhau. Hai giai đoạn của đồ án đi theo đúng cách tách này.

| Công trình | Điểm chính |
|---|---|
| Kant & Zucker 1986 [7] | Đề xuất tách bài toán thành đường đi + vận tốc |
| O'Donnell & Lozano-Pérez 1989 [8] | Phối hợp hai tay máy, không va chạm và không deadlock |
| Leroy, Laumond, Siméon 1999 [9] | Coordination diagram, bao vật cản trong diagram bằng hình hộp. Đầy đủ theo độ phân giải, nhưng chỉ so với các đường đã cho. Có ví dụ với hơn 100 robot |
| Peng & Akella 2005 [10] | Phối hợp nhiều robot dọc đường cho trước, có ràng buộc động học, bằng quy hoạch nguyên hỗn hợp (MILP). Thử với robot kiểu xe, 12 robot |
| MAPF-POST, Hönig và cộng sự 2016 [11] | Hậu xử lý kế hoạch MAPF bằng mạng thời gian đơn, trong thời gian đa thức, để tôn trọng giới hạn vận tốc và khoảng cách an toàn |
| **Pecora và cộng sự 2018** [12] | **Gần nhất với hướng của đồ án.** Tìm các *vùng tới hạn* (critical section): nơi dải quét thân robot dọc đường của hai robot chồng lên nhau. Ràng buộc thứ tự điều phối việc vào và đi qua các vùng này; ngoài vùng thì không cần điều phối. Không rời rạc hoá môi trường hay đường đi. Có code: `coordination_oru` |

**Hệ quả cho đồ án.** Khung "đường đi + vùng tới hạn + thứ tự" đã có [12], và sẽ là cơ sở cho giai đoạn 2. Đóng góp của đồ án không nằm ở khung phối hợp, mà ở planner tạo đường (lấy mẫu có định hướng) và ở cách planner giảm chồng chéo giữa các robot (I1, mục 5.4).

### 4.2. Thực thi theo thứ tự (giai đoạn 2)

| Công trình | Điểm chính |
|---|---|
| MAPF-DP, Ma, Kumar, Koenig 2017 [14] | Hành động di chuyển có xác suất bị trễ. Chính sách thực thi phát lệnh GO/STOP để không va chạm khi robot bị trễ |
| k-robust, Atzmon và cộng sự 2020 [15] | Kế hoạch k-robust vẫn thực thi được khi xảy ra tối đa k lần trễ |
| **ADG**, Hönig và cộng sự 2019 [13] | Action Dependency Graph ghi quan hệ thứ tự giữa hành động của các robot; robot chỉ làm một hành động khi các hành động phải xảy ra trước đã xong. Chịu được robot chậm lại |
| SADG, Berndt và cộng sự 2020 [16] | ADG giữ thứ tự của các xe dọc đường, nên thực thi không deadlock. Bản "switchable" cho phép chọn lại thứ tự trong lúc chạy |
| Mannucci, Pallottino, Pecora 2019 [17] | Phối hợp tập trung khi liên lạc không tin cậy |

Ghi kế hoạch dưới dạng thứ tự thay vì giờ cụ thể thì kế hoạch vẫn đúng khi robot chạy nhanh chậm khác dự tính. Các công trình ADG làm trên lưới; giai đoạn 2 dùng cùng ý đó cho đường liên tục, như Pecora [12].

### 4.3. Thứ tự ưu tiên

Dùng cho I1 (robot nào lập đường trước), M4, và thứ tự qua vùng ở giai đoạn 2. Các công trình này đều lập kế hoạch trong không gian–thời gian, đồ án chỉ mượn heuristic:
- Gán ưu tiên rồi lập kế hoạch lần lượt: Erdmann & Lozano-Pérez [18].
- Một thứ tự cố định có thể thất bại dù lời giải tồn tại; tìm thứ tự bằng leo đồi ngẫu nhiên: Bennewitz và cộng sự [19].
- **RPP** [20] chỉ ra hai kiểu thất bại: robot ưu tiên cao đứng ở đích chắn đường, và robot ưu tiên cao đâm robot đang đứng ở điểm đầu. Luật của RPP: khi lập đường, tránh vùng điểm đầu của các robot ưu tiên thấp hơn. Đồ án mượn ý này cho bước giữ chỗ (5.2) và điều kiện hợp lệ của kịch bản (3.2).
- PBS [21]: tìm kiếm trên các thứ tự ưu tiên từng phần, chỉ thêm quan hệ ưu tiên khi gặp xung đột.
- Ưu tiên trước robot có ít lựa chọn đường (ít lớp đồng luân): Wu và cộng sự [22].
- Phân robot ra các lớp đường khác nhau về mặt tô-pô để giảm tắc nghẽn: Wang và cộng sự [47]. Bài này vẫn dùng không gian–thời gian; đồ án chỉ tham khảo ý chia lối cho I1.

### 4.4. Đường tách nhau và định tuyến dây: cơ sở của baseline M4

Các đường tách hẳn nhau tương đương *đường dày* không chạm nhau. Về lý thuyết: Polishchuk & Mitchell [41] giải trong O(n + K) cho K cặp điểm trên biên đa giác đơn, và chỉ ra bài toán NP-hard khi số lỗ không cố định; Tahmasbi & Mirehi [42] (~) xét K đường không cắt nhau ngắn nhất cho các đĩa đơn vị.

Về cách làm, bài toán giống định tuyến dây trên mạch in và chip:

| Trong định tuyến dây | Trong đồ án | Nguồn |
|---|---|---|
| Nối từng dây một, dây đã nối không sửa. Thường xếp theo độ dài tăng dần | M4: lập lần lượt, ngắn trước | [44] |
| Có thể không tồn tại thứ tự nào nối được hết | Giới hạn của M4 (mục 1.4) | [44] |
| Vòng đầu cho dùng chung tài nguyên, sau đó tăng dần mức phạt tới khi không còn dùng chung (PathFinder) | Gần với I1: cho đi chung nhưng phạt | [46] |
| Mỗi vòng, gỡ và nối lại mọi dây, kể cả dây không ở vùng tắc, để nhường chỗ | Có thể dùng để giảm vùng giao nhau sau khi đã có đường | [45][46] |

Khác biệt cần nhớ: router dò đường ngắn nhất theo chi phí, nên mức phạt có tác dụng ngay. RRT-FConnect chỉ tìm đường khả thi, không tối ưu chi phí, nên cơ chế phạt phải đi qua tia LEA và APF.

### 4.5. Các hướng ngoài phạm vi

| Họ | Ví dụ | Vì sao ngoài phạm vi |
|---|---|---|
| Lập lại đường khi môi trường đổi | ERRT [23], Ferguson [24], RRTX [25] | Bản đồ không đổi; đã bỏ các kịch bản sự cố |
| Không gian–thời gian, robot khác là vật cản động | SIPP [26], ST-RRT\* [27], SI-RRT [28][29] | Đưa thời gian vào planner, trái với cách tách hai giai đoạn. Bản 28/10 chọn hướng này |
| Tránh va chạm phản ứng | ORCA [30] | Cơ chế cục bộ khi chạy |
| Ghép không gian của mọi robot | dRRT [31], dRRT\* [32]; ghép khi cần [33][34] | Tia LEA và lực APF không có nghĩa trong không gian 2N chiều |
| CBS | CBS [35]; K-CBS [36] | Tầng dưới phải nhận ràng buộc theo thời gian |
| Học máy | PRIMAL [37] | Cần huấn luyện, làm trên lưới |

Tổng quan về hoạch định lấy mẫu và đa robot: [38][39].

---

## 5. Hướng của đồ án

### 5.1. Tổng quan

```
Giai đoạn 1 (path planning, không thời gian)
   A0  Nới vật cản theo D/2
   A1  Giữ chỗ: với robot k, chặn đĩa bán kính D quanh điểm đầu, đích của mọi robot khác
       BFS kiểm tra còn đường, rồi gọi RRT-FConnect(s_k, g_k)
   A2  Tìm vùng giao nhau giữa mọi cặp đường
   A3  Kiểm tra độc lập, đo tiêu chí

Giai đoạn 2 (phối hợp, làm sau)
   B1  Chọn thứ tự qua từng vùng → đồ thị phụ thuộc; kiểm tra chu trình (deadlock)
   B2  Có deadlock: đổi thứ tự, hoặc lập lại đường của một robot tránh chỗ gây deadlock
   B3  Đặt thời gian chờ và vận tốc; mô phỏng, kiểm tra va chạm
```

**RRT-FConnect giữ nguyên thuật toán.** Tia LEA (`shootRays`), APF (`heuristicSteer`) và kiểm tra va chạm (`isSegmentValid`) đều hỏi bản đồ qua `isStateValid`, nên chặn ô trong bản đồ thì cả ba tự tính tới phần bị chặn. Riêng heuristic chọn cặp (`computeH`) vẫn dùng khoảng cách đường thẳng (5.5).

### 5.2. Các bước của giai đoạn 1

**A0. Nới vật cản.** Nới theo `D/2` (mục 3.3). Bản đồ này được lưu lại làm bản đồ gốc, mỗi lần gọi planner đều khôi phục từ đây.

**A1. Lập đường.**
- **Giữ chỗ.** Với robot k, chặn đĩa bán kính `D` quanh điểm đầu và đích của mọi robot khác. Nhờ vậy không đường nào đi qua chỗ robot khác đứng, nên không có vùng giao nhau chứa điểm đầu hay đích. Đây là loại vùng dễ làm giai đoạn 2 bế tắc nhất (mục 1.4). Nếu giữ chỗ làm robot k hết đường thì bỏ giữ chỗ cho riêng robot đó và ghi lại (`reserve_dropped`).
- **Cách chặn.** Một ô bị chặn nếu khoảng cách nhỏ nhất từ hình vuông của ô tới hình cần tránh nhỏ hơn `D`. Nhờ vậy mọi điểm trong ô trống đều đủ xa, không cần cộng bù sai số lưới.
- **BFS trước khi gọi planner.** RRT không tự biết bài vô nghiệm, nên sẽ chạy tới hết giới hạn. BFS đi 4 hướng (mục 3.2) đủ rẻ để chạy mỗi lần.
- Giữ chỗ không phụ thuộc thứ tự lập đường, nên các robot lập độc lập với nhau.

**A2. Vùng giao nhau.**
- Lấy điểm trên mỗi đường cách nhau 0.1 m. Với mỗi cặp đường, điểm nào cách đường kia dưới `D` thì thuộc vùng. Sai số do dùng điểm thay cho đường thẳng khoảng 2 mm.
- Hai đoạn liền nhau được gộp thành một vùng nếu chúng cách nhau dưới `D` trên cả hai đường, để một robot đi đan theo mép đường kia chỉ tính là một vùng.
- Mỗi vùng ghi: khoảng `[vào, ra]` trên hai đường, khoảng cách nhỏ nhất, chiều (cùng chiều, đối đầu, hoặc chỉ cắt nhau), và điểm đầu, đích nào nằm trong vùng. Giai đoạn 2 lấy dữ liệu trực tiếp từ đây.

### 5.3. Kiểm tra độc lập

`check_multi_paths.py` chỉ dùng hình học của đường gấp khúc và bản đồ gốc chưa nới, không dùng chung code với planner:
- đường bắt đầu đúng ở `s_i`, kết thúc đúng ở `g_i`, độ dài đã lưu là đúng;
- đường cách vật cản gốc ít nhất `D/2`;
- một cặp đường có vùng giao nhau khi và chỉ khi chúng cách nhau dưới `D`, tính bằng khoảng cách chính xác giữa hai đường gấp khúc;
- giữ chỗ (M2, M3, M4): không đường nào cách điểm đầu, đích của robot khác dưới `D`;
- M4: mọi cặp đường cách nhau ít nhất `D`.

`check_multi_map.py` so các hàm nới, chặn, khôi phục và BFS của `OccMap` với một bản viết riêng bằng Python, theo từng ô.

Nếu kiểm tra thấy sai thì đó là lỗi cài đặt phải sửa, không được tính là một ca thất bại thông thường.

### 5.4. Đóng góp có thể của phần "lấy mẫu có định hướng"

Mọi phần thêm đặt sau cờ JSON, mặc định là RRT-FConnect gốc, như ở lần reproduce.

| # | Cải tiến | Ý tưởng | Đo bằng |
|---|---|---|---|
| I1 | **Planner biết đường của robot khác** | Lập lần lượt, coi đường đã có của các robot trước là "vật cản mềm": tia LEA đi gần đường đó coi như ngắn hơn, nên sector đó ít được chọn; APF thêm lực đẩy nhẹ. Nhờ vậy các đường tự tách ra khi có lối khác, nhưng vẫn đi chung được khi chỉ có một khe | Số vùng, tỉ lệ chồng chéo, độ dài tăng thêm, thời gian lập |
| I2 | Heuristic biết tường | Trong `computeH`, thay khoảng cách đường thẳng tới start, goal bằng khoảng cách trên lưới (BFS, tính lại cho mỗi robot) | Thời gian lập trên flappy/rooms, nơi RRT-FConnect đang chậm hơn RRT-Connect |
| I3 | Rút gọn đường | Code hiện không rút gọn đường, nên đường RRT gấp khúc, dài hơn và dễ chồng chéo hơn mức cần. Rút gọn bằng `isSegmentValid` trên bản đồ của robot đó | Tổng độ dài, độ trơn, số vùng |

I1 là cải tiến đúng trọng tâm đề tài nhất: nó dùng chính cơ chế lấy mẫu có định hướng (LEA, APF) để giảm chồng chéo, mà vẫn không có thời gian trong cây. Có I1 thì thứ tự lập đường mới có ý nghĩa ở giai đoạn 1, và cần so các cách chọn thứ tự (mục 4.3).

### 5.5. Điểm yếu đã biết

- **Giai đoạn 1 đứng một mình chưa an toàn** (mục 1.2). Phải nêu rõ trong báo cáo nếu đồ án dừng ở giai đoạn này.
- **Không có I1 thì M1 gần như N lần single robot.** Phần nghiên cứu của giai đoạn 1 nằm ở việc đo chồng chéo và giảm nó.
- **Đường RRT gấp khúc làm số vùng tăng mạnh.** Khi đi chung hành lang, hai đường chạm rồi rời nhau liên tục: trong bài thử nhỏ, flappy_10 có trung bình khoảng 12 vùng mỗi cặp xung đột, narrow_10 khoảng 8, blobs_10 chưa tới 2. Một ca narrow_10 với 8 robot có 196 vùng trong 27 cặp. Nhiều vùng ngắn như vậy làm giai đoạn 2 phức tạp không cần thiết, nên I3 có thể có ích hơn dự tính.
- **Heuristic chọn cặp không biết tường.** Heuristic dùng khoảng cách đường thẳng. Ở single robot, trên flappy/rooms số vòng lặp chỉ giảm 18–45% so với RRT-Connect, trong khi mỗi vòng tốn khoảng gấp đôi, nên RRT-FConnect chậm hơn tới 1.9 lần (riêng rooms_20 ngang). I2 nhắm vào điểm này.
- **Lưới thô tốn chỗ** (mục 3.3, 3.4). Lưới 0.5 m buộc phải dùng robot nhỏ và bỏ các bản đồ 40% trừ blobs. Lưới mịn hơn phải sinh lại bản đồ, vì `.pcd` hiện có điểm cách nhau 0.5 m.
- **Khác với số liệu single robot.** Nới vật cản làm bản đồ khác đi, nên số liệu giai đoạn 1 không so trực tiếp được với lần reproduce (robot dạng điểm).
- **Tách đường rồi phối hợp không đầy đủ** (mục 1.4). Cần đếm các ca đường chọn ở giai đoạn 1 làm giai đoạn 2 thất bại.

### 5.6. Giai đoạn 2 (phác thảo, chi tiết hoá khi tới)

**B1. Thứ tự.**
- Mặc định: robot tới vùng sớm hơn (đi liên tục với vận tốc tối đa) thì đi trước.
- Ràng buộc bắt buộc: điểm đầu của j nằm trong vùng thì j phải rời vùng trước khi i vào; điểm đích của j nằm trong vùng thì i phải qua vùng trước khi j tới đích.
- Đồ thị phụ thuộc: nút là cặp (robot, vùng). Cạnh nối các vùng theo thứ tự đi trên cùng một đường, và cạnh thứ tự giữa hai robot ở cùng một vùng. Đồ thị có chu trình nghĩa là có deadlock.

**B2. Sửa deadlock.** Thử đổi thứ tự ở các vùng trong chu trình. Không được thì lập lại đường cho một robot trong chu trình, coi vùng gây chu trình là vật cản. Giới hạn số lần thử.

**B3. Thực thi.** Robot đi dọc đường; trước một vùng, chờ ở mép vùng cho tới khi các robot phải đi trước đã ra khỏi vùng. Bộ mô phỏng 2D kiểm tra va chạm độc lập ở mỗi bước. Baseline: thực thi theo giờ cố định.

---

## 6. Câu hỏi nghiên cứu

| # | Câu hỏi | Giả thuyết kiểm chứng được | Thí nghiệm |
|---|---|---|---|
| RQ1 | Với robot có kích thước, RRT-FConnect có lập N đường tốt hơn RRT-Connect không? | Thời gian lập thấp hơn và tổng độ dài ngắn hơn trên bản đồ dày vật cản và có lối hẹp. Có thể thua trên flappy/rooms | M1 so với M0 |
| RQ2 | Giữ chỗ được và mất gì? | Số vùng có điểm đầu, đích về 0. Độ dài và thời gian tăng không đáng kể | M3 so với M1 |
| RQ3 | Cho planner biết đường của robot khác (I1) có giảm chồng chéo không? | Ít vùng giao nhau hơn, đường dài thêm không quá vài phần trăm, nhất là khi còn nhiều lối | M5 so với M3 |
| RQ4 | Chồng chéo tăng thế nào theo N và độ rộng khe? Tách hẳn đường thì mất bao nhiêu? | Số vùng tăng khoảng theo N². M4 dài hơn và success giảm nhanh khi N tăng, nên phối hợp ở giai đoạn 2 là cần thiết | M3 và M4; N = 2–16; khe 2, 4, 6, 20 m |
| RQ5 (giai đoạn 2) | Đường ít chồng chéo hơn có cho kế hoạch tốt hơn không? | Thời gian chờ và makespan giảm, ít deadlock hơn | M3 và M5 đưa qua giai đoạn 2 |

---

## 7. Thiết kế kỹ thuật

**Đã có sẵn:**
- `occ_grid`, `kdtree`, `sampler`;
- RRT-FConnect (`SOF_brrt_simple_case3.h`) với cấu hình [paper_table1_final.json](../src/path_finder/config/paper_table1_final.json), RRT-Connect (`brrt.h`). Cả hai cùng có `plan(s, g)` và `getPath()`.

**Thêm cho giai đoạn 1:**

| Thành phần | File | Ghi chú |
|---|---|---|
| Seed cố định | `sampler.h`, các planner, `testcase.h`, `test_planner.cpp` | Khoá `seed` trong JSON tham số. Mặc định −1 là ngẫu nhiên như cũ |
| Giới hạn thời gian | `brrt.h`, `SOF_brrt_simple_case3.h` | `set_time_limit`. Mặc định tắt. Trước đây `search_time_` được đọc nhưng vòng lặp không dùng |
| Giải phóng cây kd | `brrt.h`, `SOF_brrt_simple_case3.h` | `kd_free` sau khi đã đo xong thời gian. Trước đây mỗi lần `plan` đều rò bộ nhớ |
| Nới, chặn, khôi phục, BFS | `occ_map.h`, `occ_map.cpp` | Chỉ ghi vào `occupancy_buffer_`, giữ nguyên `isStateValid`, nên không thêm chi phí cho planner |
| Đường và vùng giao nhau | `multi/path_utils.h`, `multi/conflict_zones.h` | Không phụ thuộc ROS. Kiểm thử: `test/test_multi_zones.cpp` |
| Node, launch | `multi_robot_node.cpp`, `multi_robot.launch` | Chạy mọi phương pháp xen kẽ trên cùng kịch bản, cùng seed |
| Script | `multi_common.py`, `generate_multi_scenarios.py`, `run_multi_experiments.py`, `check_multi_paths.py`, `check_multi_map.py`, `analyze_multi.py`, `plot_multi_paths.py` | |

**Trục z.** Planner không giữ z cố định: mẫu ở chế độ khám phá có z ngẫu nhiên (`uniformSamplingOnce` lấy cả z), nên đỉnh của đường có thể nằm ở các lớp z khác nhau. Hai lớp trên sàn của bản đồ giống hệt nhau (đã kiểm tra trên một bản đồ mỗi loại). Vì vậy việc chặn làm trên mọi lớp z, và mọi khoảng cách, độ dài của phần đa robot đều đo trên XY.

---

## 8. Thiết kế thí nghiệm

| Hạng mục | Lựa chọn |
|---|---|
| Bản đồ | blobs_10/20/40, narrow_10/20, flappy_10/20, rooms_10/20 (có sẵn), cộng flappy_10 và rooms_10 với khe 2, 4, 6 m |
| Số robot N | 2, 4, 8, 16 |
| Kịch bản | `random_50`, `random_150`, `parallel`, `crossing`, `swap`. Mỗi (bản đồ, loại, N): 10 kịch bản × 1 lần chạy (ban đầu định 5 lần; giảm ngày 11/11, xem nhật ký quyết định) |
| Giới hạn (chốt sau thăm dò 11/11) | Mỗi lần gọi planner: 5 s. Cả N robot: 60 s. M4 chỉ chạy khi N ≤ 8 (`--separated-max-n 8`). Không dùng giới hạn 100.000 vòng của single robot: với nó chỉ 5–18% số robot tìm được đường trên blobs_40, với 5 s là 90–100% (thử 10 robot mỗi loại) |
| Báo cáo | Trung vị và IQR; tỉ lệ kèm khoảng tin cậy bootstrap theo kịch bản |

**Phương pháp so sánh:**

| Mã | Phương pháp | Tên trong code | Dùng để |
|---|---|---|---|
| M0 | RRT-Connect [40], từng robot độc lập | `connect+independent` | Baseline |
| M1 | RRT-FConnect, từng robot độc lập | `fconnect+independent` | RQ1 |
| M2 | RRT-Connect + giữ chỗ | `connect+reserve` | Baseline của M3 |
| **M3** | **RRT-FConnect + giữ chỗ** | `fconnect+reserve` | Phương pháp chính |
| M4 | Tách hẳn: RRT-FConnect lập lần lượt, ngắn trước; đường đã lập thành vật cản; bị chặn thì đưa robot đó lên đầu rồi lập lại, tối đa N lần | `fconnect+separated` | Mức nghiêm ngặt nhất, không cần giai đoạn 2. RQ4 |
| M5 | M3 + I1 | chưa làm | RQ3 |
| M6 | M3 + I2 | chưa làm | Bản đồ có tường |
| M7 | M3 + I3 | chưa làm | Độ dài, độ trơn |

**Tiêu chí:** như mục 2.
- Bảng chính: success, tổng độ dài, số vùng giao nhau, tỉ lệ chồng chéo, thời gian lập.
- Bảng phụ: số vùng có điểm đầu, đích; vùng đối đầu; các loại thất bại.

**Bài học từ lần reproduce, áp dụng ngay:**
- Mọi thay đổi đặt sau cờ, mặc định là hành vi cũ.
- **Đo thời gian bằng đồng hồ đơn điệu** (`ros::SteadyTime`, `std::chrono::steady_clock`), và kiểm tra đồng hồ WSL trước khi chạy (mục 2b của [HowToRunCode.md](HowToRunCode.md)).
- Chạy baseline xen kẽ và so tỉ lệ thời gian, vì tốc độ CPU của laptop thay đổi theo điện năng và nhiệt.
- Tỉ lệ thời gian có sai số ngẫu nhiên lớn: ±8–15% với 1000 lần chạy, tới ±47% với 300 lần. Phải chạy đủ số lần và kèm khoảng tin cậy.
- Kiểm tra kịch bản có lời giải (BFS) ngay khi sinh.
- Cố định seed và ghi seed vào JSON kết quả.

### Kết quả thăm dò (11/11/2026)

Mục đích: chốt giới hạn và ước lượng thời gian trước khi chạy đủ bảng. 15 môi trường (9 môi trường ở trên và các biến thể khe), mỗi môi trường 1 bản đồ (map_0), 20 kịch bản (5 loại × N = 2, 4, 8, 16), 1 lần chạy, M0–M4, giới hạn như bảng trên. Kết quả ở `~/rrt_results/multi/probe/` trong WSL.

**Thời gian chạy** (một tiến trình, 95 lần chạy mỗi bản đồ): bản đồ thưa 1–2 phút (blobs_10/20, flappy_10, narrow_10, rooms_10, flappy_10_gap4); 2–11 phút (flappy_10_gap6, rooms_10_gap4/gap6, rooms_20, flappy_20, rooms_10_gap2); 15–25 phút (flappy_10_gap2, blobs_40, narrow_20). Tổng 1,73 giờ. Ở bản đồ thưa, gần hết thời gian là của M4 với N = 8; ở bản đồ khó thì cả 5 phương pháp đều chậm, nặng nhất là N = 16.

**Số lần thành công** (cột M4 tính trên N = 2, 4, 8):

| Môi trường | M0 `connect+independent` | M1 `fconnect+independent` | M2 `connect+reserve` | M3 `fconnect+reserve` | M4 `fconnect+separated` |
|---|---|---|---|---|---|
| blobs_10 | 20/20 | 20/20 | 20/20 | 20/20 | 13/15 |
| blobs_20 | 20/20 | 20/20 | 20/20 | 20/20 | 10/15 |
| blobs_40 | 18/20 | 15/20 | 16/20 | 15/20 | 4/15 |
| narrow_10 | 20/20 | 20/20 | 20/20 | 20/20 | 5/15 |
| narrow_20 | 6/20 | 12/20 | 5/20 | 11/20 | 0/15 |
| flappy_10 | 20/20 | 20/20 | 20/20 | 20/20 | 3/15 |
| flappy_10_gap2 | 6/20 | 9/20 | 6/20 | 8/20 | 1/15 |
| flappy_10_gap4 | 20/20 | 20/20 | 20/20 | 20/20 | 2/15 |
| flappy_10_gap6 | 20/20 | 20/20 | 20/20 | 20/20 | 3/15 |
| flappy_20 | 20/20 | 19/20 | 20/20 | 20/20 | 0/15 |
| rooms_10 | 20/20 | 20/20 | 20/20 | 20/20 | 11/15 |
| rooms_10_gap2 | 17/20 | 17/20 | 16/20 | 18/20 | 4/15 |
| rooms_10_gap4 | 20/20 | 20/20 | 20/20 | 20/20 | 8/15 |
| rooms_10_gap6 | 20/20 | 20/20 | 20/20 | 20/20 | 8/15 |
| rooms_20 | 20/20 | 20/20 | 20/20 | 20/20 | 9/15 |
| **Tổng** | 89.0% | 90.7% | 87.7% | 90.7% | 36.0% |

Nhận xét (mỗi ô chỉ có 1 bản đồ và 1 lần chạy, nên đây là dấu hiệu, chưa phải kết luận):
- **Thất bại của M0–M3 dồn vào 4 bản đồ**: narrow_20, flappy_10_gap2, blobs_40, rooms_10_gap2. Ở 11 bản đồ còn lại gần như 100%. Mọi thất bại của M0–M3 đều là `planner_limit` (BFS xác nhận có đường), không có `no_path`.
- **RRT-FConnect thành công nhiều hơn RRT-Connect ở hai bản đồ khó nhất**: narrow_20 12/20 so với 6/20 (độc lập), 11/20 so với 5/20 (giữ chỗ); flappy_10_gap2 9/20 so với 6/20. Ở blobs_40 thì ngược lại (15 so với 18). Cần bảng đầy đủ để biết chênh lệch có thật không.
- **Giữ chỗ không làm giảm success rõ rệt** (M3 90.7% bằng M1), và đưa số vùng có điểm đầu, đích về 0 như thiết kế. Không lần nào phải bỏ giữ chỗ vì chặn lối robot khác.
- **Tách hẳn (M4) thất bại nhanh theo N**: 67% với N = 2, 33% với N = 4, 8% với N = 8. Phần lớn là `blocked` (132 lần), tức đường đã lập chặn hết lối của robot sau. Đây là lý do cần cho phép đường đi gần nhau và để giai đoạn 2 lo phần thứ tự.
- **Thời gian**: RRT-FConnect so với RRT-Connect (bảng gộp mọi môi trường của `analyze_multi.py --merge-envs`) dao động 0.4–1.35 lần, khoảng tin cậy rộng vì mỗi ô chỉ có 15 lần chạy.

**Quyết định từ thăm dò:**
- Giữ giới hạn 5 s / 60 s. Thất bại chỉ tập trung ở 4 bản đồ khó; tăng giới hạn sẽ kéo dài chủ yếu ở chính các bản đồ đó (đã chiếm phần lớn thời gian chạy).
- Giữ M4 chỉ đến N = 8: với N = 8 nó đã gần như luôn `blocked`, và là phương pháp tốn thời gian nhất.
- Thiết kế ban đầu (5 bản đồ × 10 kịch bản × 5 lần chạy) gấp 250 lần thăm dò, tức khoảng 16–31 ngày máy với một tiến trình. Giảm còn 1 lần chạy mỗi kịch bản (gấp 50 lần, khoảng 3,6 ngày). Độ biến thiên vẫn đến từ 50 kịch bản khác nhau mỗi ô (môi trường, loại, N).
- Kiểm tra độc lập (`check_multi_paths.py`) chưa chạy trên dữ liệu thăm dò: với 1425 lần chạy nó mất hơn 10 phút, và đã dừng để không tranh CPU với thí nghiệm đầy đủ. Chạy sau khi thí nghiệm đầy đủ xong.
- Khi sinh kịch bản đầy đủ, 240/15.000 kịch bản không sinh được: flappy_10_gap2/map_1 và flappy_10_gap4/map_4 bị bít (vùng lớn nhất chỉ 60% ô trống), nên chỉ có `random_50`, `random_150`. Đúng như đã biết ở mục 3.4.

---

## 9. Hướng dẫn làm tiếp và bảng theo dõi

Mục này nói **làm gì tiếp theo, theo thứ tự nào, sửa ở đâu trong code, và khi nào coi là xong**. Ước lượng tính cho một người làm toàn thời gian.

### Quy ước làm việc
- Mỗi giai đoạn làm trên một branch tách từ `main`. Giai đoạn 1: `multi-path-planning`. Xong và kiểm thử rồi mới gộp vào `main`.
- Thay đổi trong planner đặt sau cờ JSON, mặc định là hành vi cũ. Trước khi gộp, chạy kiểm thử hồi quy: cùng seed phải cho cùng kết quả.
- Đo thời gian bằng đồng hồ đơn điệu. Trước thí nghiệm dài, kiểm tra đồng hồ WSL (mục 2b của [HowToRunCode.md](HowToRunCode.md)).
- Xong một việc: đánh dấu `[x]` và ghi ngày vào bảng, ghi kết quả hoặc vấn đề vào nhật ký tiến độ. Quyết định mới thì ghi vào nhật ký quyết định.

### Chia việc cho hai người (đề xuất)
| Người | Phụ trách |
|---|---|
| A: planner | Các cải tiến I1, I2 (1.10, 1.11), thí nghiệm RQ1, RQ3 |
| B: thí nghiệm | Chạy hàng loạt và phân tích, cải tiến I3 (1.12), thí nghiệm RQ2, RQ4 |
| Cả hai | Chốt câu hỏi và đọc tài liệu, viết báo cáo |

### Chuẩn bị (song song, không chặn việc code)

**Chốt câu hỏi.** Họp với GVHD, trả lời mục 10. Câu quan trọng nhất là Q1: chia hai giai đoạn và bỏ các kịch bản sự cố.

**Đọc tài liệu.** Mỗi bài trả lời câu hỏi dưới đây, rồi ghi 3–5 dòng ngay dưới mục tham khảo đó ở mục 11:
- [12]: vùng tới hạn được tính thế nào, robot dừng chờ ở đâu, thứ tự được chọn và sửa ra sao?
- [13]: ADG được xây từ kế hoạch thế nào, khi nào một robot được đi tiếp?
- [20]: luật RPP và điều kiện "hạ tầng hợp lệ" áp dụng cho bản đồ rooms thế nào?
- [46]: cơ chế phạt của PathFinder có thể chuyển sang tia LEA và APF thế nào (cho I1)?

| # | Việc | Ước lượng | Trạng thái | Ghi chú |
|---|---|---|---|---|
| C.1 | Reproduce, có cấu hình khớp paper | – | [x] | `paper_table1_final.json`; lần 9 của KET_QUA_REPRODUCE.md |
| C.2 | Giải thích phần số vòng lặp còn lệch | 1–2 ngày | [x] | 28/10: do code bỏ mẫu nằm trong vật cản (H9) |
| C.3 | Chốt câu hỏi mục 10 với GVHD | 0.5 ngày | [ ] | |
| C.4 | Đọc [12], [13], [20], [46], ghi tóm tắt | 2–3 ngày | [ ] | Ưu tiên [12] |

### Giai đoạn 1: path planning trên bản đồ tĩnh (khoảng 3 tuần)

**Mục tiêu:** có N đường và các vùng giao nhau, đo được và tự kiểm tra được đúng sai. Trả lời RQ1–RQ4.

**Đã xong phần hạ tầng và thăm dò.** Lệnh chạy: mục 7 của [HowToRunCode.md](HowToRunCode.md). Thí nghiệm đầy đủ (1.9) đang chạy từ 11/11; xong thì chạy kiểm tra độc lập, phân tích, rồi làm I3 (1.12).

**Xong khi:**
- không nới, không chặn thì single robot chạy như trước: cùng seed cho cùng kết quả, chi phí mỗi vòng không đổi quá mức nhiễu;
- `check_multi_map.py` và `check_multi_paths.py` không báo sai;
- có bảng ở mục 8 cho M0–M4 trên đủ 9 môi trường và các biến thể khe;
- mỗi cải tiến có bảng so với M3 và một kết luận có ích hay không. Kết luận "không có ích" cũng ghi lại.

| # | Việc | Ước lượng | Trạng thái | Ghi chú |
|---|---|---|---|---|
| 1.1 | Seed cố định | 0.5 ngày | [x] 04/11 | Cùng seed: 40/40 bản ghi giống hệt trên 4 planner. Không seed: 0/40 |
| 1.2 | Bản đồ: nới, chặn, khôi phục, BFS | 1–2 ngày | [x] 04/11 | So từng ô với bản Python trên 5 bản đồ: không lệch |
| 1.3 | Giới hạn thời gian, giải phóng cây kd | 0.5 ngày | [x] 04/11 | Chi phí mỗi vòng so với build cũ: −1% tới +5.5%, trong mức nhiễu đã biết |
| 1.4 | Tìm vùng giao nhau kèm kiểm thử; node, launch, JSON kết quả | 3–4 ngày | [x] 04/11 | `test_multi_zones` đạt |
| 1.5 | Sinh kịch bản N robot | 1 ngày | [x] 04/11 | |
| 1.6 | Kiểm tra độc lập, phân tích, vẽ hình | 2 ngày | [x] 04/11 | Vẽ bằng `plot_multi_paths.py` thay cho RViz, để dùng được trong báo cáo |
| 1.7 | Bản đồ khe 2, 4, 6 m | 0.5–1 ngày | [x] 04/11 | Chỉ flappy, rooms (mục 3.4). Lưu ở `dataset_maps_gap.tar.gz` |
| 1.8 | Chạy thăm dò, chốt giới hạn | 1 ngày | [x] 11/11 | 15/15 môi trường, 1,73 giờ. Giữ 5 s / 60 s, M4 đến N = 8. Kết quả: mục 8, "Kết quả thăm dò" |
| 1.9 | Thí nghiệm RQ1, RQ2, RQ4 (M0–M4) | khoảng 3,6 ngày máy | [ ] | Bảng chính của báo cáo. Đang chạy từ 11/11 12:06: 5 bản đồ × 10 kịch bản × 1 lần chạy, kết quả ở `~/rrt_results/multi/full/` |
| 1.10 | I1: planner biết đường của robot khác, kèm so các cách chọn thứ tự | 3–4 ngày | [ ] | Đóng góp chính |
| 1.11 | I2: heuristic biết tường | 2 ngày | [ ] | |
| 1.12 | I3: rút gọn đường | 1 ngày | [ ] | Làm trước vì rẻ, và số vùng hiện rất cao do đường gấp khúc (5.5) |
| 1.13 | Thí nghiệm RQ3 | 2 ngày | [ ] | |

### Giai đoạn 2: phối hợp (khoảng 3 tuần, chi tiết hoá khi tới)

**Mục tiêu:** từ N đường và các vùng giao nhau của giai đoạn 1, ra kế hoạch chạy được không va chạm. Trả lời RQ5. Thiết kế ở mục 5.6.

| # | Việc | Ước lượng | Trạng thái | Ghi chú |
|---|---|---|---|---|
| 2.1 | Thứ tự qua vùng, đồ thị phụ thuộc, phát hiện deadlock (B1) | 3–4 ngày | [ ] | Đầu vào là `zones` trong JSON kết quả |
| 2.2 | Sửa deadlock (B2) | 2–3 ngày | [ ] | |
| 2.3 | Thời gian chờ, vận tốc, mô phỏng, kiểm tra va chạm (B3) | 4–5 ngày | [ ] | |
| 2.4 | Thí nghiệm RQ5 | 2 ngày | [ ] | |

### Báo cáo
| # | Việc | Trạng thái |
|---|---|---|
| R.1 | Chương tổng quan: mục 1–4 | [ ] |
| R.2 | Chương reproduce single robot | [ ] |
| R.3 | Chương phương pháp: mục 5, 7 | [ ] |
| R.4 | Chương thí nghiệm | [ ] |

### Nhật ký quyết định
| Ngày | Quyết định | Căn cứ | Người |
|---|---|---|---|
| 28/10/2026 | ~~Hướng chính: prioritized + RPP + SI-RRT-FConnect (lập kế hoạch không gian–thời gian)~~. Thay bằng quyết định ngày 29/10 | Bản 28/10 của tài liệu | |
| 28/10/2026 | Đo thời gian bằng đồng hồ đơn điệu; trên máy chạy WSL thì mask `systemd-timesyncd` | Lần 7 của KET_QUA_REPRODUCE.md | |
| 29/10/2026 | ~~Hướng chính: lập kế hoạch tĩnh (đường RRT-FConnect + thứ tự qua vùng giao nhau, xong trước khi chạy) và xử lý sự cố khi chạy~~. Thay bằng quyết định ngày 04/11 | Yêu cầu của nhóm | |
| 04/11/2026 | Chia hai giai đoạn: trước mắt path planning trên bản đồ tĩnh, đường được đi gần nhau; thứ tự qua vùng giao nhau, chờ và vận tốc là giai đoạn sau. Bỏ các kịch bản sự cố S1–S8. Đường tách hẳn nhau chỉ là baseline | Yêu cầu của nhóm; mục 1 | |
| 04/11/2026 | r = 0.25 m, m = 0.1 m; bỏ narrow_40, flappy_40, rooms_40 | Đo trên 60 bản đồ; mục 3.3 | |
| 11/11/2026 | Giữ giới hạn 5 s mỗi lần gọi planner, 60 s cho cả N robot; M4 chỉ chạy khi N ≤ 8 | Thăm dò; mục 8, "Kết quả thăm dò" | |
| 11/11/2026 | Thí nghiệm đầy đủ: 1 lần chạy mỗi kịch bản thay vì 5, giữ 5 bản đồ × 10 kịch bản; chạy một tiến trình, không sửa code để chạy song song | Bản 5 lần chạy cần 16–31 ngày máy; bản này khoảng 3,6 ngày | |

### Nhật ký tiến độ
| Ngày | Đã làm | Kết quả / vấn đề | Việc tiếp theo |
|---|---|---|---|
| 28/10/2026 | Reproduce: cấu hình cuối (H9: không bỏ mẫu nằm trong vật cản); tìm ra và sửa lỗi đồng hồ WSL | Số vòng lặp và tỉ lệ thời gian gần paper; số liệu thời gian cũ bị nhiễu | Chạy lại sạch (lần 9) |
| 28/10/2026 | Chạy lại sạch (lần 9): 10 môi trường, EBi-RRT trên flappy/rooms, các vòng quét | 4 môi trường của paper: tái hiện mọi kết luận của Table 2. Flappy/rooms: RRT-FConnect chậm hơn RRT-Connect tới 1.9 lần, nhưng giải được nhiều bài nhất ở mật độ 40% | Chốt hướng đa robot |
| 29/10/2026 | Soạn tài liệu theo hướng lập kế hoạch tĩnh có thứ tự và sự cố | Chờ nhóm chốt | Chốt phạm vi |
| 04/11/2026 | Chia hai giai đoạn. Làm hạ tầng giai đoạn 1: seed, giới hạn thời gian, giải phóng cây kd, nới và chặn ô, BFS, vùng giao nhau, node, kịch bản, kiểm tra độc lập, phân tích, vẽ, bản đồ khe | Bài thử nhỏ 1500 lần chạy (5 bản đồ, N = 2–8): kiểm tra độc lập 1068 lần thành công, không vi phạm; giữ chỗ đưa số vùng có điểm đầu, đích về 0. Tìm và sửa ba lỗi của chính thiết kế này: BFS 8 hướng, giới hạn 100.000 vòng, tiêu chí đối đầu quá lỏng. Thăm dò mới xong 2/15 môi trường, chưa có kết luận | Chạy nốt thăm dò (1.8) |
| 11/11/2026 | Chạy nốt thăm dò (1.8); sinh kịch bản đầy đủ; bắt đầu thí nghiệm đầy đủ (1.9) lúc 12:06 | Thăm dò: M0–M3 thành công 88–91%, thất bại dồn vào narrow_20, flappy_10_gap2, blobs_40, rooms_10_gap2; M4 36%, giảm nhanh theo N. Laptop vào Modern Standby nhiều lần trong 12:27–15:07 vì chưa giữ thức (số đo không bị ảnh hưởng, chỉ chậm); giữ thức từ 15:25. blobs_20/map_1 chạy cùng lúc với một lần kiểm tra khác (15:10–15:22), thời gian của bản đồ này có thể hơi nhiễu | Chờ 1.9 xong (khoảng 15/11); chạy `check_multi_paths.py`, `analyze_multi.py`; cân nhắc chạy lại blobs_20/map_1 |

---

## 10. Câu hỏi cần chốt

| # | Câu hỏi | Đề xuất |
|---|---|---|
| Q1 | GVHD có đồng ý chia hai giai đoạn và bỏ các kịch bản sự cố không? | Đồng ý. Nếu vẫn cần sự cố thì thêm vào sau giai đoạn 2 |
| Q2 | Kích thước robot? | r = 0.25 m, m = 0.1 m (mục 3.3) |
| Q3 | Có bỏ các bản đồ 40% trừ blobs không? | Bỏ. Chúng không còn lối đi cho robot có kích thước |
| Q4 | Khoảng cách s–g trong kịch bản? | Dùng cả 50 m và 150 m; 150 m để so được với single robot |
| Q5 | Có cần lưới mịn hơn 0.5 m không? | Giữ 0.5 m. Chỉ đổi nếu cần robot lớn hơn hoặc các bản đồ 40% |
| Q6 | Giai đoạn 2 dùng mô hình robot nào, chạy trên đâu? | Đĩa holonomic, mô phỏng 2D. Vi sai và Gazebo là mở rộng |

---

## 11. Tài liệu tham khảo

Tra cứu ngày 28/10/2026, riêng [5], [10], [12]–[17], [23]–[25] tra ngày 29/10/2026, [41]–[47] tra ngày 04/11/2026. ✓ = đã đối chiếu nội dung; ~ = chỉ xác nhận thư mục hoặc qua nguồn thứ cấp. Số thứ tự cũ được giữ nguyên.

**Bài toán, độ phức tạp, tiêu chí**
1. ✓ R. Stern et al., *Multi-Agent Pathfinding: Definitions, Variants, and Benchmarks*, SoCS 2019. [arXiv:1906.08291](https://arxiv.org/abs/1906.08291)
2. ~ J. Hopcroft, J. Schwartz, M. Sharir, *On the Complexity of Motion Planning for Multiple Independent Objects; PSPACE-Hardness of the "Warehouseman's Problem"*, 1984. [bản scan](https://usercontent.one/wp/sokoban.dk/wp-content/uploads/2016/02/Hopcroft-et-Al-On-the-compelxity-of-motion-planning.pdf)
3. ~ P. Spirakis, C. Yap, *Strong NP-hardness of moving many discs*, Information Processing Letters 19(1):55–59, 1984. [DOI](https://doi.org/10.1016/0020-0190(84)90130-3)
4. ✓ J. Yu, S. LaValle, *Structure and Intractability of Optimal Multi-Robot Path Planning on Graphs*, AAAI 2013. [AAAI](https://ojs.aaai.org/index.php/AAAI/article/view/8541)
5. ✓ E. Heiden, L. Palmieri, K. O. Arras, G. S. Sukhatme, S. Koenig, *Experimental Comparison of Global Motion Planning Algorithms for Wheeled Mobile Robots*, arXiv 2020. [arXiv:2003.03543](https://arxiv.org/abs/2003.03543). Benchmark phát triển tiếp: Bench-MR, IEEE RA-L 6(3):4536–4543, 2021. [DOI](https://doi.org/10.1109/LRA.2021.3068913)
6. ✓ T. Q. Chi, V. Q. Huy, L. H. Trang, *RRT-FConnect: Fast Bidirectional Path Planning in Cluttered and Narrow-Passage Environments*, PRICAI 2026. Paper chính của đồ án.

**Tách đường và phối hợp (giai đoạn 2)**

7. ~ K. Kant, S. Zucker, *Toward efficient trajectory planning: the path-velocity decomposition*, IJRR, 1986 (chưa đối chiếu số trang).
8. ~ P. O'Donnell, T. Lozano-Pérez, *Deadlock-free and collision-free coordination of two robot manipulators*, ICRA 1989, pp. 484–489.
9. ✓ S. Leroy, J.-P. Laumond, T. Siméon, *Multiple Path Coordination for Mobile Robots: A Geometric Algorithm*, IJCAI 1999. [mlanthology](https://mlanthology.org/ijcai/1999/leroy1999ijcai-multiple). Bản tạp chí: *Path coordination for multiple mobile robots: a resolution-complete algorithm*. [HAL](https://hal.laas.fr/hal-01987712)
10. ✓ J. Peng, S. Akella, *Coordinating Multiple Robots with Kinodynamic Constraints Along Specified Paths*, IJRR 24(4):295–310, 2005. [PDF](https://webpages.charlotte.edu/sakella/papers/PengAkellaIJRR05.pdf)
11. ✓ W. Hönig et al., *Multi-Agent Path Finding with Kinematic Constraints*, ICAPS 2016. [AAAI](https://ojs.aaai.org/index.php/ICAPS/article/view/13796)
12. ✓ F. Pecora, H. Andreasson, M. Mansouri, V. Petkov, *A Loosely-Coupled Approach for Multi-Robot Coordination, Motion Planning and Control*, ICAPS 2018, pp. 485–493. [AAAI](https://ojs.aaai.org/index.php/ICAPS/article/view/13923). Code: [coordination_oru](https://github.com/FedericoPecora/coordination_oru)

**Thực thi theo thứ tự (giai đoạn 2)**

13. ✓ W. Hönig, S. Kiesel, A. Tinka, J. Durham, N. Ayanian, *Persistent and Robust Execution of MAPF Schedules in Warehouses*, IEEE RA-L 4(2), 2019. [PDF](https://act.usc.edu/publications/Hoenig_RAL2019.pdf)
14. ✓ H. Ma, T. K. S. Kumar, S. Koenig, *Multi-Agent Path Finding with Delay Probabilities*, AAAI 2017, pp. 3605–3612. [arXiv:1612.05309](https://arxiv.org/abs/1612.05309)
15. ✓ D. Atzmon, R. Stern, A. Felner, G. Wagner, R. Barták, N.-F. Zhou, *Robust Multi-Agent Path Finding and Executing*, JAIR 67:549–579, 2020. [JAIR](https://jair.org/index.php/jair/article/view/11734)
16. ✓ A. Berndt, N. van Duijkeren, L. Palmieri, T. Keviczky, *A Feedback Scheme to Reorder a Multi-Agent Execution Schedule by Persistently Optimizing a Switchable Action Dependency Graph*, arXiv 2020. [arXiv:2010.05254](https://arxiv.org/abs/2010.05254)
17. ✓ A. Mannucci, L. Pallottino, F. Pecora, *Provably Safe Multi-Robot Coordination With Unreliable Communication*, IEEE RA-L, 2019. [PDF](https://www.centropiaggio.unipi.it/sites/default/files/provably_safe_multi_robot_coordination_with_unreliable_communication.pdf)

**Thứ tự ưu tiên**

18. ✓ M. Erdmann, T. Lozano-Pérez, *On Multiple Moving Objects*, Algorithmica 2:477–521, 1987. [CMU](https://www.ri.cmu.edu/publications/on-multiple-moving-objects)
19. ~ M. Bennewitz, W. Burgard, S. Thrun, *Finding and optimizing solvable priority schemes for decoupled path planning techniques for teams of mobile robots*, Robotics and Autonomous Systems, 2002. [PDF](https://www.hrl.uni-bonn.de/papers/bennewitz02ras.pdf)
20. ✓ M. Čáp, P. Novák, A. Kleiner, M. Selecký, *Prioritized Planning Algorithms for Trajectory Coordination of Multiple Mobile Robots*, IEEE T-ASE 12(3):835–849, 2015. [arXiv:1409.2399](https://arxiv.org/abs/1409.2399)
21. ✓ H. Ma, D. Harabor, P. Stuckey, J. Li, S. Koenig, *Searching with Consistent Prioritization for Multi-Agent Path Finding*, AAAI 2019. [AAAI](https://ojs.aaai.org/index.php/AAAI/article/view/4758)
22. ✓ W. Wu, S. Bhattacharya, A. Prorok, *Multi-Robot Path Deconfliction through Prioritization by Path Prospects*, ICRA 2020. [arXiv:1908.02361](https://arxiv.org/abs/1908.02361)

**Ngoài phạm vi: lập lại đường khi môi trường thay đổi**

23. ✓ J. Bruce, M. Veloso, *Real-Time Randomized Path Planning for Robot Navigation*, IROS 2002, pp. 2383–2388. [CMU](https://www.ri.cmu.edu/publications/real-time-randomized-path-planning-for-robot-navigation)
24. ✓ D. Ferguson, N. Kalra, A. Stentz, *Replanning with RRTs*, ICRA 2006, pp. 1243–1248. [CMU](https://www.ri.cmu.edu/publications/replanning-with-rrts)
25. ✓ M. Otte, E. Frazzoli, *RRTX: Asymptotically Optimal Single-Query Sampling-Based Motion Planning with Quick Replanning*, IJRR 35(7):797–822, 2016. [PDF](https://www.ottelab.com/html_stuff/pdf_files/Otte.Frazzoli.IJRR15.pdf)

**Ngoài phạm vi: không gian–thời gian, ghép, CBS, học máy**

26. ✓ M. Phillips, M. Likhachev, *SIPP: Safe Interval Path Planning for Dynamic Environments*, ICRA 2011. [CMU](https://www.ri.cmu.edu/publications/sipp-safe-interval-path-planning-for-dynamic-environments)
27. ✓ F. Grothe, V. Hartmann, A. Orthey, M. Toussaint, *ST-RRT\*: Asymptotically-Optimal Bidirectional Motion Planning through Space-Time*, ICRA 2022. [arXiv:2203.02176](https://arxiv.org/abs/2203.02176)
28. ✓ N. Kerimov, A. Onegin, K. Yakovlev, *Safe Interval Randomized Path Planning For Manipulators*, ICAPS 2025. [arXiv:2412.19567](https://arxiv.org/abs/2412.19567)
29. ✓ N. Kerimov, A. Onegin, K. Yakovlev, *SI-RRT and ST-RRT\* For Prioritized Multi-Manipulator Planning: Empirical Evaluation*, ICR 2025. [trang bài](https://ia.spcras.ru/index.php/icr2026/article/view/17212)
30. ✓ J. van den Berg, S. Guy, M. Lin, D. Manocha, *Reciprocal n-Body Collision Avoidance*, ISRR 2009 (xuất bản 2011). [DOI](https://doi.org/10.1007/978-3-642-19457-3_1)
31. ✓ K. Solovey, O. Salzman, D. Halperin, *Finding a needle in an exponential haystack: Discrete RRT for exploration of implicit roadmaps in multi-robot motion planning*, IJRR 35(5):501–513, 2016. [arXiv:1305.2889](https://arxiv.org/abs/1305.2889)
32. ✓ R. Shome, K. Solovey, A. Dobson, D. Halperin, K. Bekris, *dRRT\*: Scalable and Informed Asymptotically-Optimal Multi-Robot Motion Planning*, Autonomous Robots 44:443–467, 2020. [arXiv:1903.00994](https://arxiv.org/abs/1903.00994)
33. ✓ G. Wagner, M. Kang, H. Choset, *Probabilistic path planning for multiple robots with subdimensional expansion*, ICRA 2012. [CMU](https://www.ri.cmu.edu/publications/probabilistic-path-planning-for-multiple-robots-with-subdimensional-expansion)
34. ✓ K. Okumura, X. Défago, *Quick Multi-Robot Motion Planning by Combining Sampling and Search*, IJCAI 2023. [arXiv:2203.00315](https://arxiv.org/abs/2203.00315)
35. ✓ G. Sharon, R. Stern, A. Felner, N. Sturtevant, *Conflict-based search for optimal multi-agent pathfinding*, Artificial Intelligence 219:40–66, 2015. [PDF](https://people.engr.tamu.edu/guni/pistar/Papers/CBS-AIJ15.pdf)
36. ✓ J. Kottinger, S. Almagor, M. Lahijanian, *Conflict-Based Search for Multi-Robot Motion Planning with Kinodynamic Constraints*, IROS 2022. [arXiv:2207.00576](https://arxiv.org/abs/2207.00576)
37. ✓ G. Sartoretti et al., *PRIMAL: Pathfinding via Reinforcement and Imitation Multi-Agent Learning*, RA-L 4(3):2378–2385, 2019. [arXiv:1809.03531](https://arxiv.org/abs/1809.03531)

**Tổng quan và nền tảng single robot**

38. ✓ A. Orthey, C. Chamzas, L. Kavraki, *Sampling-Based Motion Planning: A Comparative Review*, Annual Review of Control, Robotics, and Autonomous Systems 7:285–310, 2024. [arXiv:2309.13119](https://arxiv.org/abs/2309.13119)
39. ✓ H.-D. Bui, *A Survey of Multi-Robot Motion Planning*, arXiv 2023 (báo cáo kỳ thi tiến sĩ). [arXiv:2310.08599](https://arxiv.org/abs/2310.08599)
40. J. Kuffner, S. LaValle, *RRT-Connect: An efficient approach to single-query path planning*, ICRA 2000.

**Đường tách nhau và định tuyến dây (baseline M4)**

41. ✓ V. Polishchuk, J. S. B. Mitchell, *Thick non-crossing paths and minimum-cost flows in polygonal domains*, SoCG 2007, pp. 56–65. [DOI](https://doi.org/10.1145/1247069.1247079)
42. ~ M. Tahmasbi, N. Mirehi, *Thick non-crossing paths in a polygonal domain*, Optimization Letters 9:743–753, 2015. [DOI](https://doi.org/10.1007/s11590-014-0776-0) (chưa đọc được nội dung)
43. ✓ S. A. Amiri, A. Golshani, S. Kreutzer, S. Siebertz, *Vertex Disjoint Paths in Upward Planar Graphs*, arXiv 2013. [arXiv:1312.1526](https://arxiv.org/abs/1312.1526). Dùng phần mở đầu để tổng hợp độ phức tạp; bài này dẫn lại từ Garey & Johnson 1979 và Robertson & Seymour 1995.
44. ✓ E. Rosenberg, *A New Iterative Supply/Demand Router with Rip-Up Capability for Printed Circuit Boards*, DAC 1987 (chưa đối chiếu số trang). [PDF](https://cdn.hackaday.io/files/1666717130852064/DAC87_721.PDF)
45. ✓ R. Nair, *A Simple Yet Effective Technique for Global Wiring*, IEEE TCAD, 1987. [DOI](https://doi.org/10.1109/TCAD.1987.1270260)
46. ✓ L. McMurchie, C. Ebeling, *PathFinder: A Negotiation-Based Performance-Driven Router for FPGAs*, FPGA 1995. [DOI](https://doi.org/10.1145/201310.201328)
47. ✓ X. Wang, A. Sahin, S. Bhattacharya, *Coordination-free Multi-robot Path Planning for Congestion Reduction Using Topological Reasoning*, arXiv 2022. [arXiv:2205.00955](https://arxiv.org/abs/2205.00955)
