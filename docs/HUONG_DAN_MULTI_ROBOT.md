# Hoạch định đa robot tĩnh dựa trên RRT-FConnect

Tài liệu định hướng cho giai đoạn đa robot của đồ án *Hoạch định đường đi đa robot dựa vào lấy mẫu có định hướng*.

Bản 29/10/2026 thay cho bản 28/10. Bản cũ lập kế hoạch trong không gian–thời gian, coi robot khác là vật cản động, tức là lập kế hoạch động. Hướng của nhóm là **lập kế hoạch tĩnh**: lập xong đường cho mọi robot trước khi chạy, và chỉ sửa đường khi gặp sự cố trong lúc chạy.

Tài liệu trả lời:
1. Bài toán là gì, "tĩnh" nghĩa là gì (mục 1).
2. Đánh giá bằng tiêu chí nào (mục 2), trước những sự cố nào (mục 3).
3. Các hướng đã có và hướng đồ án chọn (mục 4–5).
4. Câu hỏi nghiên cứu, thiết kế và thí nghiệm (mục 6–8).
5. **Làm gì tiếp theo, theo thứ tự nào, sửa ở đâu trong code, khi nào coi là xong** (mục 9). Những điểm nhóm cần chốt nằm ở mục 10.

**Bắt đầu từ đâu:** mục 9, giai đoạn 0, bước 1: chốt các câu hỏi ở mục 10 với GVHD.

**Cách đọc chứng cứ.** Mỗi khẳng định về công trình khác có số tham khảo `[n]`, danh sách ở mục 11:
- ✓: đã đối chiếu với tóm tắt hoặc nội dung bài.
- ~: chỉ xác nhận được thông tin thư mục, hoặc qua nguồn thứ cấp. Cần đọc bản gốc trước khi trích trong báo cáo.

Số liệu single robot của đồ án lấy từ lần 9 trong [KET_QUA_REPRODUCE.md](KET_QUA_REPRODUCE.md). Code single robot: [HUONG_DAN_RRT-FConnect.md](HUONG_DAN_RRT-FConnect.md).

**Mục lục**
1. [Bài toán](#1-bài-toán)
2. [Tiêu chí đánh giá](#2-tiêu-chí-đánh-giá)
3. [Kịch bản sự cố](#3-kịch-bản-sự-cố)
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

### 1.1. Lập kế hoạch tĩnh trong đồ án này

| | Lập kế hoạch tĩnh (hướng của đồ án) | Lập kế hoạch động (ngoài phạm vi) |
|---|---|---|
| Lúc lập kế hoạch | Xong hết trước khi robot nào chuyển động | Trong lúc chạy, lặp lại liên tục hoặc theo chu kỳ |
| Môi trường khi lập | Bản đồ tĩnh đã biết | Có vật cản động, biết hoặc dự đoán được theo thời gian |
| Robot khác | Đường của chúng cố định. Hai robot tránh nhau bằng **thứ tự đi qua chỗ hai đường gặp nhau**, quyết định trước khi chạy | Là vật cản động trong không gian–thời gian (ví dụ SI-RRT [28]), hoặc được né theo kiểu phản ứng (ORCA [30]) |
| Khi chạy | Bám đường và thứ tự. **Chỉ lập lại đường khi có sự cố** (mục 3) | Lập lại thường xuyên |
| Planner đơn robot | RRT-FConnect giữ nguyên, chạy trên bản đồ tĩnh | Phải thêm chiều thời gian vào cây |

Kế hoạch tĩnh của mỗi robot gồm hai phần, đều có trước khi chạy:
- **Đường đi**: đường gấp khúc do RRT-FConnect tìm trên bản đồ tĩnh.
- **Thứ tự**: ở mỗi đoạn mà đường của hai robot đi sát nhau (*vùng giao nhau*, mục 5.2), robot nào đi qua trước. Robot đi sau dừng chờ ở mép vùng cho tới khi robot đi trước ra khỏi vùng.

Thứ tự không cần biết giờ chính xác, nên kế hoạch vẫn đúng khi robot chạy nhanh hay chậm hơn dự tính (sự cố S1, mục 3).

Mức nghiêm ngặt hơn là cấm hẳn hai đường đi sát nhau, tức không có vùng giao nhau nào. Mức này không giải được khi mọi robot phải qua cùng một khe, nên chỉ dùng làm baseline (M1, mục 8).

### 1.2. Phát biểu

Cho:
- bản đồ chiếm chỗ tĩnh 2D (lưới 0.5 m, như dataset hiện có);
- N robot hình đĩa bán kính r, vận tốc tối đa `v_max`. Robot i có điểm đầu `s_i` và điểm đích `g_i`.

Trước khi chạy, cần tìm:
- với mỗi robot i, một đường `P_i` từ `s_i` đến `g_i`, cách vật cản tĩnh ít nhất r;
- một thứ tự đi qua cho mọi vùng giao nhau, sao cho khi mọi robot bám đường và tuân thứ tự thì:
  - hai robot bất kỳ luôn cách nhau ít nhất `2r + m` (m là lề an toàn);
  - không có deadlock;
  - mọi robot tới đích.

Khi chạy, nếu xảy ra sự cố (mục 3), sửa đường và thứ tự của các robot bị ảnh hưởng rồi chạy tiếp.

### 1.3. Giả định mặc định

| Giả định | Đề xuất | Lý do |
|---|---|---|
| Không gian | 2D (x, y) | Tia LEA và APF của RRT-FConnect đều làm trên mặt phẳng |
| Robot | Đĩa r = 0.5 m, holonomic, `v_max` = 1 m/s, được dừng chờ | Đủ để làm rõ bài toán phối hợp. Robot vi sai để sau |
| Sau khi tới đích | Đứng yên tại đích | Chuẩn của MAPF [1]. Tạo ra trường hợp robot đứng ở đích chắn đường robot khác |
| Lập kế hoạch | Tập trung: một máy biết mọi robot và bản đồ | Phạm vi đồ án |
| Thông tin khi chạy | Máy trung tâm biết vị trí mọi robot. Vật cản mới chỉ được phát hiện khi robot tới gần (tầm cảm biến `d_s`) | Gần thực tế, đủ để tạo sự cố |
| Kiểm tra kết quả | Bộ mô phỏng 2D riêng, kiểm tra va chạm độc lập với planner | Không tin kết quả do planner tự báo |

### 1.4. Giới hạn lý thuyết

- Dời nhiều vật thể độc lập là bài toán PSPACE-hard [2]. Dời nhiều đĩa là NP-hard mạnh [3].
- Ngay trên đồ thị, tối ưu makespan hay tổng chi phí cũng NP-hard, và các mục tiêu này không tối ưu được đồng thời [4].
- Hướng tĩnh tách rời còn yếu hơn: chỉ đầy đủ so với các đường đã chọn [9]. Ví dụ thất bại điển hình: đích của A nằm trên đường của B, và đích của B nằm trên đường của A. Robot nào tới đích trước cũng đứng đó chắn robot kia mãi. Khi đó buộc phải sửa đường (bước A4, mục 5.2).

Vì vậy báo cáo không hứa hẹn tính đầy đủ hay tối ưu. Thay vào đó, đo bằng thí nghiệm cái giá của sự đơn giản (success, makespan) và phân loại các ca thất bại.

---

## 2. Tiêu chí đánh giá

### 2.1. Vấn đề và tiêu chí trong paper RRT-FConnect

**Vấn đề paper giải quyết** [6]:
- Mục tiêu: tìm **nhanh một lời giải khả thi** thay vì tối ưu dần, trong môi trường **nhiều vật cản và có lối hẹp**.
- Nhược điểm của RRT lấy mẫu đều mà paper nêu:
  - tốn công khám phá vùng không liên quan, nên hội tụ chậm;
  - ở vùng dày vật cản, đường đi vòng nhiều, chất lượng kém;
  - ở lối hẹp, xác suất lấy được mẫu trong vùng trống thấp.
- Paper nhận xét phương pháp không cần rewiring, nên **hứa hẹn cho lập kế hoạch thời gian thực**.

**Năm tiêu chí của paper.** Dùng nguyên cho mọi lần gọi planner, cả lúc lập kế hoạch ban đầu lẫn mỗi lần sửa đường:

| Tiêu chí | Ý nghĩa theo paper | Ghi chú cho đồ án |
|---|---|---|
| Path length | Chất lượng đường | Độ dài hình học thật |
| Number of nodes | Bộ nhớ, độ phức tạp khám phá | |
| Number of iterations | Tốc độ hội tụ | Không phụ thuộc máy |
| Search time | Hiệu quả tính toán | So bằng tỉ lệ với RRT-Connect chạy xen kẽ, vì số ms phụ thuộc máy |
| Success rate | Độ tin cậy, trong giới hạn vòng lặp | Với đa robot: dùng giới hạn thời gian thay vì giới hạn vòng lặp (mục 8) |

Paper còn đánh giá độ ổn định qua phân bố (trung vị, IQR), và so chất lượng đường theo thời gian với Informed BiRRT\*. Đồ án báo cáo trung vị, IQR và khoảng tin cậy cho mọi tiêu chí.

**Ba điểm của paper quyết định vai trò của RRT-FConnect trong hướng tĩnh:**
- Lối hẹp là nơi đường của nhiều robot dồn vào nhau, tức là nơi sinh ra vùng giao nhau và deadlock.
- Tìm nhanh lời giải khả thi là điều kiện để **sửa đường khi có sự cố** trong lúc robot đang chạy.
- Nhận định này cần kiểm chứng. Ở single robot, RRT-FConnect nhanh hơn RRT-Connect 8–37% ở General, Highly, Narrow, nhưng chậm hơn tới 1.9 lần trên bản đồ có tường (flappy/rooms).

### 2.2. Tiêu chí của kế hoạch đa robot

Các tiêu chí thường dùng trong MAPF và hoạch định đa robot [1]:

| Tiêu chí | Định nghĩa |
|---|---|
| Success | Mọi robot tới đích, không va chạm (kiểm tra độc lập), trong giới hạn thời gian lập kế hoạch |
| Sum of costs (SOC) | Tổng thời gian tới đích của các robot, kể cả thời gian chờ: `Σ (độ dài P_i / v_max + thời gian chờ_i)`. Báo cáo thêm tổng độ dài đường |
| Makespan | Thời điểm robot cuối cùng tới đích |
| Tổng thời gian chờ | Tổng thời gian các robot dừng ở mép vùng giao nhau |
| Số vùng giao nhau, số deadlock phải sửa | Độ "chật" của kế hoạch. Vùng giao nhau càng nhiều thì một sự cố càng dễ lan sang nhiều robot |
| Thời gian lập kế hoạch | Tổng, và tách theo: lập đường, tìm vùng giao nhau và thứ tự, sửa deadlock |

SOC và makespan không tối ưu được đồng thời [4]. Đồ án chọn SOC làm mục tiêu chính và báo cáo cả hai.

### 2.3. Tiêu chí chất lượng đường

Benchmark planner cho robot di động [5] dùng thêm: độ cong trung bình và lớn nhất (đo độ trơn và công điều khiển), khoảng cách trung bình tới vật cản (clearance), số lần đảo chiều (cusp), thời gian tìm lời giải đầu tiên, và tỉ lệ lời giải hợp lệ. Với đồ án:
- **Độ trơn:** tổng góc quay và góc quay lớn nhất tại các đỉnh của đường gấp khúc. Đường của RRT gấp khúc nhiều, nên nếu có làm trơn thì báo cáo cả trước và sau.
- **Clearance:** khoảng cách nhỏ nhất và trung bình từ đường tới vật cản.
- **Khoảng cách nhỏ nhất giữa hai robot** khi chạy: phải ≥ 2r. Đây là lề an toàn thực tế.

### 2.4. Tiêu chí khi có sự cố

| Tiêu chí | Định nghĩa |
|---|---|
| Tỉ lệ hoàn thành | Số robot tới đích / số robot phải tới đích (không tính robot bị hỏng) |
| Suy giảm | Makespan và SOC tăng thêm so với chạy cùng kịch bản không có sự cố |
| Độ trễ phục hồi | Từ lúc phát hiện sự cố tới lúc mọi robot bị ảnh hưởng có kế hoạch mới |
| Số lần lập lại đường, số robot bị ảnh hưởng | |
| Va chạm, deadlock | Phải bằng 0. Xảy ra thì tính là thất bại |
| Thời gian dừng chờ thêm | |

Mọi tiêu chí ở mục này đều được xem theo số robot N, mật độ vật cản và độ rộng khe.

---

## 3. Kịch bản sự cố

Kế hoạch tĩnh dựa trên hai giả định: bản đồ đúng, và robot chạy theo kế hoạch. Mỗi sự cố dưới đây phá một trong hai giả định đó.
- Mỗi sự cố có tham số để tăng dần độ nặng.
- Sự cố được tiêm cùng thời điểm, cùng vị trí (cùng seed) cho mọi phương pháp, để so theo cặp.

| # | Sự cố | Cách mô phỏng (tham số) | Ảnh hưởng tới kế hoạch | Xử lý mặc định |
|---|---|---|---|---|
| S1 | Trễ: xuất phát muộn hoặc đi chậm | Robot k xuất phát trễ Δ ∈ {2, 10, 30} s, hoặc đi với 0.5·`v_max` trong T s | Giờ tới các vùng giao nhau lệch so với dự tính | Không cần sửa đường: thực thi theo thứ tự (5.3) vẫn không va chạm, chỉ chờ lâu hơn. Tùy chọn: đổi thứ tự khi chờ quá ngưỡng (I4) |
| S2 | Dừng tạm thời | Robot dừng T ∈ {5, 30} s tại một điểm, chọn ngẫu nhiên hoặc trong khe | Robot phía sau phải chờ. Dừng trong vùng giao nhau thì chặn cả vùng | Chờ tới ngưỡng `T_wait`. Quá ngưỡng thì robot bị chặn lập lại đường, coi robot đang dừng là vật cản |
| S3 | Hỏng hẳn | Robot dừng mãi tại một điểm, thành vật cản tĩnh hình đĩa | Chặn đường robot khác. Trong khe hẹp có thể chặn hoàn toàn | Các robot bị ảnh hưởng lập lại đường. Nếu BFS xác nhận không còn đường thì ghi là vô nghiệm, không tính là lỗi của planner |
| S4 | Vật cản mới, không có trong bản đồ | Hộp a × a (a ∈ {1, 3} m) đặt trên đường robot k. Robot chỉ thấy khi cách ≤ `d_s` | Đường của robot k, có thể cả robot khác, không còn hợp lệ | Lập lại đường từ vị trí hiện tại trên bản đồ đã cập nhật, rồi tính lại vùng giao nhau và thứ tự cho phần đường còn lại |
| S5 | Lối đi bị đóng | Đóng một khe (narrow, flappy) hoặc một cửa (rooms) sau t giây | Nhiều robot cùng phải đi vòng xa và dồn vào khe còn lại | Như S4, nhưng cho nhiều robot cùng lúc. Đây là ca khó nhất cho tốc độ sửa đường |
| S6 | Lệch khỏi đường khi bám | Nhiễu vị trí, lệch tối đa e ∈ {0.1, 0.3} m | Khoảng cách thực tế giữa các robot và tới vật cản nhỏ hơn dự tính | Lề an toàn m khi nới vật cản và khi tìm vùng giao nhau; đo khoảng cách nhỏ nhất thực tế |
| S7 (tùy chọn) | Đổi đích, thêm robot giữa chừng | Robot k nhận đích mới tại thời điểm t | Như lập kế hoạch cho một robot mới giữa các robot đang chạy | Lập đường mới, thêm vùng giao nhau với phần đường còn lại của các robot khác |
| S8 (tùy chọn) | Mất liên lạc | Robot không nhận được kế hoạch mới trong T s | Không áp dụng được kế hoạch đã sửa | Robot dừng an toàn ở mép vùng giao nhau tiếp theo. Tham khảo [17] |

**Phát hiện sự cố:**
- Trễ: so tiến độ thực tế với lịch danh định (5.2, A3).
- Dừng, hỏng: robot báo trạng thái, hoặc không tiến được trong một khoảng thời gian.
- Vật cản mới, lối bị đóng: cảm biến tầm `d_s` thấy ô bị chiếm trên đoạn đường phía trước.
- Deadlock lúc chạy: đồ thị "ai đang chờ ai" có chu trình.

**Vì sao sự cố quyết định việc so sánh các phương pháp:**
- **Thực thi theo giờ gãy ngay khi có trễ, thực thi theo thứ tự thì không.** Thực thi theo giờ nghĩa là mỗi robot tới vùng giao nhau đúng giờ đã tính; chỉ cần S1 là đủ gây va chạm. Thực thi theo thứ tự thì vẫn an toàn khi robot bị trễ [13][14]. Đây là lý do đồ án dùng thứ tự (5.3).
- **Đường dồn vào cùng một khe thì một sự cố lan sang nhiều robot.** Vì vậy số vùng giao nhau (2.2) là tiêu chí quan trọng, và là chỗ để cải tiến planner (I1, mục 5.4).
- **Với S3–S5, độ trễ phục hồi phụ thuộc trực tiếp vào tốc độ planner.** Đây là chỗ kiểm chứng nhận định "thời gian thực" của paper [6].
- **Vị trí tiêm quyết định độ nặng.** Cùng một robot hỏng, ở vùng trống thì gần như vô hại, ở giữa khe hẹp thì chặn cả bản đồ. Vì vậy tiêm theo hai chế độ: ngẫu nhiên trên đường, và tại điểm tới hạn (trong khe hoặc trong vùng giao nhau).

---

## 4. Các hướng đã có

### 4.1. Tách đường và phối hợp: nền của hướng tĩnh

**Ý tưởng.** Bước 1: tìm đường hình học cho từng robot. Bước 2: chỉ phối hợp việc đi dọc các đường đó (chờ, chậm lại, thứ tự) để tránh nhau.

| Công trình | Điểm chính |
|---|---|
| Kant & Zucker 1986 [7] | Đề xuất tách bài toán thành đường đi + vận tốc |
| O'Donnell & Lozano-Pérez 1989 [8] | Phối hợp hai tay máy, không va chạm và không deadlock |
| Leroy, Laumond, Siméon 1999 [9] | Coordination diagram, bao vật cản trong diagram bằng hình hộp. Đầy đủ theo độ phân giải, nhưng chỉ so với các đường đã cho. Có ví dụ với hơn 100 robot |
| Peng & Akella 2005 [10] | Phối hợp nhiều robot dọc đường cho trước, có ràng buộc động học, bằng quy hoạch nguyên hỗn hợp (MILP). Thử với robot kiểu xe, 12 robot |
| MAPF-POST, Hönig và cộng sự 2016 [11] | Hậu xử lý kế hoạch MAPF bằng mạng thời gian đơn, trong thời gian đa thức, để tôn trọng giới hạn vận tốc và khoảng cách an toàn |
| **Pecora và cộng sự 2018** [12] | **Gần nhất với hướng của đồ án.** Tìm các *vùng tới hạn* (critical section): nơi dải quét thân robot dọc đường của hai robot chồng lên nhau. Ràng buộc thứ tự điều phối việc vào và đi qua các vùng này; ngoài vùng thì không cần điều phối, và robot có thể đi nối đuôi nhau qua một vùng. Không rời rạc hoá môi trường hay đường đi. Thứ tự được sửa bằng heuristic ngay trong lúc robot chạy. Có code: `coordination_oru` |

**Hệ quả cho đồ án.** Khung "đường đi + vùng tới hạn + thứ tự" đã có [12]. Đóng góp của đồ án không nằm ở khung phối hợp, mà ở:
- planner tạo đường (lấy mẫu có định hướng);
- cách planner giảm chồng chéo giữa các robot (I1, mục 5.4);
- tốc độ và chất lượng sửa đường khi có sự cố.

`coordination_oru` có thể dùng để đối chiếu cách cài đặt.

### 4.2. Thực thi bền vững khi robot bị trễ

| Công trình | Điểm chính |
|---|---|
| MAPF-DP, Ma, Kumar, Koenig 2017 [14] | Hành động di chuyển có xác suất bị trễ. Chính sách thực thi phát lệnh GO/STOP để không va chạm khi robot bị trễ |
| k-robust, Atzmon và cộng sự 2020 [15] | Kế hoạch k-robust vẫn thực thi được khi xảy ra tối đa k lần trễ. Có điều kiện cần và đủ, thuật toán tìm kế hoạch k-robust tối ưu, và chính sách thực thi bền vững kể cả khi liên lạc bị trễ |
| **ADG**, Hönig và cộng sự 2019 [13] | Action Dependency Graph ghi quan hệ thứ tự giữa hành động của các robot trong một kế hoạch MAPF; robot chỉ làm một hành động khi các hành động phải xảy ra trước đã xong. Không phụ thuộc bộ giải MAPF. Chịu được giới hạn động lực học chưa biết, robot chậm lại và vật cản bất ngờ. Cần ít liên lạc hơn MAPF-POST, vì robot chỉ báo khi xong một hành động |
| SADG, Berndt và cộng sự 2020 [16] | ADG giữ thứ tự của các xe dọc đường, nên thực thi không deadlock. Bản "switchable" cho phép chọn lại thứ tự trong lúc chạy; mọi lựa chọn hợp lệ đều không va chạm |
| Mannucci, Pallottino, Pecora 2019 [17] | Phối hợp tập trung khi liên lạc không tin cậy: an toàn được chứng minh nếu lỗi chỉ là trễ; khi mất gói thì xác suất mất an toàn có cận trên |

**Ý nghĩa cho đồ án.** Kế hoạch tĩnh ghi dưới dạng thứ tự, thay vì giờ cụ thể, chịu được trễ (S1, S2) mà không phải lập lại. Các công trình ADG làm trên lưới (MAPF). Đồ án dùng cùng ý đó cho đường liên tục do RRT-FConnect sinh ra, như Pecora [12].

### 4.3. Thứ tự ưu tiên

Hướng tĩnh vẫn phải quyết định hai việc: ai đi trước ở mỗi vùng giao nhau, và khi sửa deadlock thì sửa đường của robot nào. Các công trình về prioritized planning cho các heuristic để quyết định, dù bản gốc của chúng lập kế hoạch trong không gian–thời gian:
- Gán ưu tiên rồi lập kế hoạch lần lượt: Erdmann & Lozano-Pérez [18].
- Một thứ tự cố định có thể thất bại dù lời giải tồn tại; tìm thứ tự bằng leo đồi ngẫu nhiên: Bennewitz và cộng sự [19].
- **RPP** [20] chỉ ra hai kiểu thất bại: robot ưu tiên cao đứng ở đích chắn đường, và robot ưu tiên cao đâm robot đang đứng ở điểm đầu.
  - Luật của RPP: khi lập đường, tránh vùng điểm đầu của các robot ưu tiên thấp hơn.
  - Điều kiện đảm bảo: trong "hạ tầng hợp lệ", tức mọi cặp điểm đầu/đích nối được bằng đường cách vật cản ≥ r và cách các điểm đầu/đích khác ≥ 2r, cách này luôn thành công, với điều kiện planner đơn robot là đầy đủ.
  - Đồ án mượn ý này ở dạng tĩnh: khi có thể, lập đường tránh vùng điểm đầu và điểm đích của các robot khác (A1, mục 5.2).
- PBS [21]: tìm kiếm trên các thứ tự ưu tiên từng phần, chỉ thêm quan hệ ưu tiên khi gặp xung đột.
- Ưu tiên trước robot có ít lựa chọn đường (ít lớp đồng luân): Wu và cộng sự [22].

### 4.4. Lập lại đường khi môi trường thay đổi (single robot)

Phần xử lý sự cố cần lập lại đường nhanh, từ vị trí hiện tại của robot:

| Công trình | Điểm chính |
|---|---|
| ERRT, Bruce & Veloso 2002 [23] | RRT xen kẽ lập kế hoạch và thực thi. Bộ nhớ waypoint của các kế hoạch trước giúp lập lại nhanh hơn RRT thường. Đã dùng trong một hệ nhiều robot thời gian thực |
| Ferguson, Kalra, Stentz 2006 [24] | Sửa cây RRT khi không gian cấu hình thay đổi: chỉ bỏ phần cây vừa trở nên không hợp lệ, giữ phần còn lại rồi mở rộng tiếp. Minh hoạ trên bài toán nhiều robot |
| RRTX, Otte & Frazzoli 2016 [25] | Giữ và sửa liên tục một đồ thị tìm kiếm khi vật cản thay đổi; tối ưu tiệm cận |

Cách mặc định là lập lại từ đầu bằng RRT-FConnect. Dùng lại cây cũ là một cải tiến (I3, mục 5.4).

### 4.5. Các hướng ngoài phạm vi

| Họ | Ví dụ | Vì sao ngoài phạm vi |
|---|---|---|
| Lập kế hoạch không gian–thời gian, robot khác là vật cản động | SIPP [26], ST-RRT\* [27], SI-RRT [28], SI-RRT trong prioritized [29] | Là lập kế hoạch động. Bản 28/10 của tài liệu chọn hướng này (SI-RRT-FConnect); nhóm đã đổi sang hướng tĩnh. Có thể dùng làm mốc so sánh nếu GVHD yêu cầu (Q6, mục 10) |
| Tránh va chạm phản ứng | ORCA [30] | Cơ chế cục bộ khi chạy, không lập kế hoạch toàn cục. Có những instance RPP giải được mà ORCA không giải được [20] |
| Ghép không gian của mọi robot | dRRT [31], dRRT\* [32]; ghép khi cần [33][34] | Đầy đủ hơn, nhưng tia LEA và lực APF không có nghĩa trong không gian 2N chiều, tức là bỏ mất phần "lấy mẫu có định hướng" |
| CBS | CBS [35]; K-CBS với planner lấy mẫu làm tầng dưới [36] | Tầng dưới phải nhận ràng buộc theo thời gian. Có thể nhắc như hướng mở rộng |
| Học máy | PRIMAL [37] | Cần huấn luyện, làm trên lưới |

Tổng quan về hoạch định lấy mẫu và đa robot: [38][39].

---

## 5. Hướng của đồ án

### 5.1. Tổng quan

Ba lớp. Chỉ lớp C chạy trong lúc robot di chuyển, và chỉ khi có sự cố.

```
A. Lập kế hoạch (trước khi chạy)
   A1  RRT-FConnect lập đường cho từng robot trên bản đồ tĩnh đã nới vật cản
   A2  Tìm vùng giao nhau giữa mọi cặp đường
   A3  Chọn thứ tự qua từng vùng → đồ thị phụ thuộc; kiểm tra chu trình (deadlock)
   A4  Có deadlock: đổi thứ tự, hoặc lập lại đường của một robot tránh chỗ gây deadlock
B. Thực thi: robot bám đường; trước mỗi vùng giao nhau, chờ các robot đi trước ra khỏi vùng
C. Xử lý sự cố (mục 3): chờ → sửa đường robot bị ảnh hưởng trên bản đồ đã cập nhật
   → cập nhật A2–A4 cho phần đường còn lại → nếu thất bại thì lập lại cho mọi robot đang chạy
```

RRT-FConnect giữ nguyên thuật toán, và chạy ở A1, A4 và C. Đây là chỗ đóng góp "lấy mẫu có định hướng" của đồ án được đo trực tiếp:
- ở A1: tốc độ lập đường và chất lượng đường;
- ở C: tốc độ sửa đường.

### 5.2. Lập kế hoạch (A1–A4)

**A1. Đường từng robot.**
- Bản đồ được nới vật cản theo `r + m`.
- Tùy chọn (luật kiểu RPP [20]): coi vùng bán kính `2r + m` quanh điểm đầu và điểm đích của các robot khác là vật cản. Nếu robot không tìm được đường thì bỏ ràng buộc này cho riêng robot đó. Luật này giảm deadlock kiểu "robot ở đích chắn đường".
- Các robot độc lập với nhau, nên lập đường song song được.

**A2. Vùng giao nhau.**
- Với mỗi cặp đường (i, j): đi dọc đường i theo bước δ (ví dụ 0.1 m), tính khoảng cách tới đường j. Đoạn nào có khoảng cách < `2r + m` là vùng giao nhau. Mỗi vùng ghi khoảng độ dài cung `[vào, ra]` trên cả hai đường.
- Điểm đầu và điểm đích cũng là chỗ bị chiếm: robot đứng ở điểm đầu trước khi xuất phát, và đứng mãi ở đích sau khi tới. Vì vậy chúng được xét như vùng.
- Trên cùng một đường, các vùng chồng nhau hoặc cách nhau dưới 2r thì gộp lại. Nhờ vậy chỗ dừng chờ luôn nằm ngoài mọi vùng, và robot đang chờ không chắn vùng khác.

**A3. Thứ tự.**
- Mặc định: robot tới vùng sớm hơn (đi liên tục với `v_max`) thì đi trước.
- Ràng buộc bắt buộc:
  - điểm đầu của j nằm trong vùng: j phải rời vùng trước khi i vào;
  - điểm đích của j nằm trong vùng: i phải qua vùng trước khi j tới đích.
- Đồ thị phụ thuộc: nút là cặp (robot, vùng). Cạnh nối các vùng theo thứ tự đi trên cùng một đường, và cạnh thứ tự giữa hai robot ở cùng một vùng. Đồ thị có chu trình nghĩa là có deadlock.
- Từ đồ thị tính được lịch danh định (giờ tới, thời gian chờ), từ đó ra SOC và makespan dự kiến.

**A4. Sửa deadlock.**
1. Thử đổi thứ tự ở các vùng nằm trong chu trình.
2. Không được (ví dụ hai đích nằm trên đường của nhau, mục 1.4) thì lập lại đường cho một robot trong chu trình, coi vùng gây chu trình là vật cản. Chọn robot theo heuristic: đường ngắn hơn, hoặc ít lựa chọn đường hơn [22].
3. Giới hạn số lần thử. Quá giới hạn thì báo thất bại.

### 5.3. Thực thi (B)

- Bộ mô phỏng 2D, bước Δt (ví dụ 0.05 s). Robot đi dọc đường với vận tốc không quá `v_max`.
- Trước khi vào một vùng, robot kiểm tra các robot phải đi trước đã ra khỏi vùng chưa; chưa thì dừng ở mép vùng.
- Đây là cách thực thi theo thứ tự của ADG [13], áp dụng cho đường liên tục như Pecora [12]. Nhờ vậy kế hoạch không gãy khi robot đi nhanh chậm khác dự tính.
- Chờ tới khi robot đi trước ra hẳn khỏi vùng là cách đơn giản và bảo thủ. Pecora [12] cho robot cùng chiều đi nối đuôi nhau trong vùng; đây là tối ưu có thể thêm sau.

**Baseline để so: thực thi theo giờ.** Robot đi đúng lịch danh định, không chờ theo phụ thuộc. Không có sự cố thì hai cách cho kết quả như nhau; có S1 hoặc S2 thì cách theo giờ có thể va chạm.

**Kiểm tra va chạm độc lập.** Mỗi bước mô phỏng, mọi cặp robot phải cách nhau ≥ 2r và mọi robot phải ngoài vật cản. Planner tự báo thành công là chưa đủ.

### 5.4. Đóng góp có thể của phần "lấy mẫu có định hướng"

Mọi phần thêm đặt sau cờ JSON, mặc định là RRT-FConnect gốc, như ở lần reproduce.

| # | Cải tiến | Ý tưởng | Đo bằng |
|---|---|---|---|
| I1 | **Planner biết đường của robot khác** | Khi lập đường cho robot k (A1 làm lần lượt, hoặc ở A4, C), coi đường đã có của các robot khác là "vật cản mềm": tia LEA coi vùng có đường robot khác là kém thoáng hơn, APF thêm lực đẩy nhẹ. Nhờ vậy các đường tự tách ra khi có lối khác, nhưng vẫn đi chung được khi chỉ có một khe | Số vùng giao nhau, thời gian chờ, SOC, độ dài đường tăng thêm |
| I2 | Heuristic biết tường | Trong heuristic chọn cặp, thay khoảng cách đường thẳng bằng khoảng cách trên lưới (BFS tính trước từ start và goal) | Thời gian lập đường trên flappy/rooms, nơi RRT-FConnect đang chậm hơn RRT-Connect |
| I3 | Dùng lại cây khi sửa đường | Giữ phần cây còn hợp lệ, như [23][24], thay vì lập lại từ đầu | Độ trễ phục hồi |
| I4 | Đổi thứ tự khi chạy | Chọn lại thứ tự ở các vùng khi một robot bị trễ quá ngưỡng, như SADG [16] hoặc heuristic của Pecora [12] | Suy giảm makespan khi có S1, S2 |

I1 là cải tiến đúng trọng tâm đề tài nhất. Nó dùng chính cơ chế lấy mẫu có định hướng (LEA, APF) để giảm chồng chéo giữa các robot, mà vẫn là lập kế hoạch tĩnh: cây không có thời gian.

### 5.5. Điểm yếu đã biết

- **Heuristic chọn cặp không biết tường.** Heuristic dùng khoảng cách đường thẳng. Ở single robot, trên flappy/rooms số vòng lặp chỉ giảm 18–45% so với RRT-Connect, trong khi mỗi vòng tốn khoảng gấp đôi, nên RRT-FConnect chậm hơn tới 1.9 lần (riêng rooms_20 ngang). Khi sửa đường lúc có sự cố, điểm này có thể làm RRT-FConnect thua RRT-Connect về độ trễ phục hồi. I2 nhắm vào điểm này.
- **Hướng tách đường không đầy đủ** (mục 1.4): có instance giải được mà khung này thất bại. Cần đếm và phân loại các ca thất bại.
- **Dataset hiện tại gần như không có xung đột.** Khe rộng 20 m (`gap_width` trong `dataset_map_generator.py`; narrow viết cứng `gap_w = 20`), trong khi robot chỉ rộng 1 m. Cần sinh thêm biến thể khe 2–4 m (mục 7).

---

## 6. Câu hỏi nghiên cứu

| # | Câu hỏi | Giả thuyết kiểm chứng được | Thí nghiệm |
|---|---|---|---|
| RQ1 | Trong khung tĩnh, RRT-FConnect có cho kế hoạch đa robot tốt hơn RRT-Connect không? | Thời gian lập kế hoạch thấp hơn và SOC nhỏ hơn (đường ngắn hơn) trên bản đồ dày vật cản và có lối hẹp. Có thể thua trên bản đồ có tường (flappy/rooms) | M2 so với M3, không sự cố, N = 2–16 |
| RQ2 | Khi có sự cố, planner và cách thực thi ảnh hưởng thế nào? | Với S1–S2, thực thi theo thứ tự không va chạm, thực thi theo giờ thì có. Với S3–S5 trên blobs/narrow, RRT-FConnect phục hồi nhanh hơn | S1–S6, hai chế độ tiêm |
| RQ3 | Cho planner biết đường của robot khác (I1) có giảm chồng chéo không? | Ít vùng giao nhau và ít thời gian chờ hơn, đường dài thêm không quá vài phần trăm, và kế hoạch ít bị ảnh hưởng hơn khi có sự cố | M3 so với M4 |
| RQ4 | Khung tĩnh chịu được tới đâu khi N tăng và khe hẹp lại? | Success giảm khi khe 2 m và N ≥ 8, chủ yếu do deadlock không sửa được | N = 2–16 (32); khe 2–4 m và 20 m |
| RQ5 (tùy chọn) | So với một phương pháp lập kế hoạch động thì được và mất gì? | Makespan của khung tĩnh dài hơn, nhưng xử lý sự cố đơn giản hơn | Chỉ làm nếu GVHD yêu cầu (Q6, mục 10) |

---

## 7. Thiết kế kỹ thuật

**Dùng lại:**
- `occ_grid`, `kdtree`, `sampler`;
- RRT-FConnect (`SOF_brrt_simple_case3.h`) với cấu hình [paper_table1_final.json](../src/path_finder/config/paper_table1_final.json);
- các script sinh bản đồ và chạy hàng loạt.

**Cần thêm:**
- **Nới vật cản** theo `r + m` trong `OccMap` (`inflate_radius`).
- **Cập nhật bản đồ khi chạy:** thêm vật cản mới, robot hỏng (hình đĩa), đóng khe vào `OccMap`, rồi gọi lại planner. Kiểm thử hồi quy: khi không cập nhật gì, kết quả reproduce phải giữ nguyên.
- **Đường và độ dài cung:** lấy đường gấp khúc từ planner (bỏ trục z), tham số hoá theo độ dài cung.
- **Tìm vùng giao nhau** (A2), kèm kiểm thử các trường hợp:
  - hai đường cắt nhau;
  - đi ngược chiều trong cùng hành lang;
  - song song cách nhau 3r;
  - điểm đích nằm trên đường của robot khác.
- **Đồ thị phụ thuộc**, phát hiện chu trình, lịch danh định (A3).
- **Bộ mô phỏng thực thi** (5.3), tiêm sự cố (mục 3), kiểm tra va chạm độc lập.
- **Sinh kịch bản N robot**, ba chế độ:
  - ngẫu nhiên;
  - đổi chỗ: hai nhóm đi ngược chiều qua nút thắt;
  - hạ tầng: điểm đầu/đích cách nhau ≥ 2r và cách vật cản ≥ r, cho luật kiểu RPP [20].

  Mỗi robot phải có đường khi đứng một mình (kiểm tra bằng BFS).
- **Bản đồ khe 2–4 m** cho narrow, flappy, rooms.
- **RViz:** phát lại N robot theo thời gian, tô vùng giao nhau và chỗ xảy ra sự cố. Đây là công cụ debug quan trọng nhất.
- **Chạy hàng loạt và phân tích:** dựa trên `run_dataset_experiments.py` và `analyze_results.py`.

---

## 8. Thiết kế thí nghiệm

| Hạng mục | Lựa chọn |
|---|---|
| Bản đồ | blobs_10/20/40, narrow_10, flappy_10/20/40, rooms_10/20/40 (có sẵn), cộng biến thể khe 2–4 m |
| Số robot N | 2, 4, 8, 16 (32 nếu kịp) |
| Kịch bản | Ngẫu nhiên, đổi chỗ, hạ tầng. Mỗi (bản đồ, N): 10 kịch bản × 5 lần chạy |
| Sự cố | Không sự cố; S1–S6, mỗi loại 2–3 mức độ nặng, hai chế độ tiêm (ngẫu nhiên, điểm tới hạn), cùng seed cho mọi phương pháp |
| Giới hạn | Lập kế hoạch cho cả N robot: 10 s (60 s khi xem giới hạn). Mỗi lần sửa đường: 1 s |
| Báo cáo | Trung vị và IQR; tỉ lệ kèm khoảng tin cậy bootstrap theo kịch bản |

**Phương pháp so sánh:**

| Mã | Phương pháp | Dùng để |
|---|---|---|
| M0 | Đường độc lập, không phối hợp | Đếm xung đột; cận dưới của độ dài đường |
| M1 | Tách hẳn: robot sau coi đường của robot trước là vật cản | Mức tĩnh nghiêm ngặt nhất |
| M2 | Khung 5.1 với RRT-Connect [40] | Baseline cùng khung |
| **M3** | **Khung 5.1 với RRT-FConnect** | Phương pháp chính |
| M4 | M3 + I1 | RQ3 |
| M5 | M3 + I2 | Bản đồ có tường |
| Khi có sự cố | Thực thi theo giờ / theo thứ tự; sửa robot bị ảnh hưởng / lập lại cho tất cả; I3, I4 nếu làm | RQ2 |

**Tiêu chí:** như mục 2.
- Bảng chính: success, SOC, makespan, thời gian lập kế hoạch.
- Khi có sự cố: tỉ lệ hoàn thành, suy giảm makespan, độ trễ phục hồi.

**Bài học từ lần reproduce, áp dụng ngay:**
- Mọi thay đổi đặt sau cờ, mặc định là hành vi cũ.
- **Đo thời gian bằng đồng hồ đơn điệu** (`ros::SteadyTime`, `std::chrono::steady_clock`), và kiểm tra đồng hồ WSL trước khi chạy (mục 2b của [HowToRunCode.md](HowToRunCode.md)).
- Chạy baseline xen kẽ và so tỉ lệ thời gian, vì tốc độ CPU của laptop thay đổi theo điện năng và nhiệt.
- Tỉ lệ thời gian có sai số ngẫu nhiên lớn: ±8–15% với 1000 lần chạy, tới ±47% với 300 lần. Phải chạy đủ số lần và kèm khoảng tin cậy.
- Kiểm tra kịch bản có lời giải (BFS) ngay khi sinh.
- Dùng giới hạn thời gian, không dùng giới hạn vòng lặp.
- Cố định seed và ghi seed vào JSON kết quả.

---

## 9. Hướng dẫn làm tiếp và bảng theo dõi

Mục này nói **làm gì tiếp theo, theo thứ tự nào, sửa ở đâu trong code, và khi nào coi là xong**. Mỗi giai đoạn gồm: mục tiêu, các bước, điều kiện xong, và bảng theo dõi. Ước lượng tính cho một người làm toàn thời gian.

**Việc ngay bây giờ:** giai đoạn 0, bước 1: chốt các câu hỏi ở mục 10 với GVHD.

### Quy ước làm việc
- Mỗi giai đoạn làm trên một branch tách từ `main` (ví dụ `multi-ha-tang`). Xong và kiểm thử rồi mới gộp vào `main`.
- Thay đổi trong planner đặt sau cờ JSON, mặc định là hành vi cũ. Trước khi gộp, chạy kiểm thử hồi quy: một bản đồ blobs_10, vài lần chạy, cùng seed, kết quả phải giống trước khi sửa.
- Đo thời gian bằng đồng hồ đơn điệu. Trước thí nghiệm dài, kiểm tra đồng hồ WSL (mục 2b của [HowToRunCode.md](HowToRunCode.md)).
- Xong một việc: đánh dấu `[x]` và ghi ngày vào bảng của giai đoạn, ghi kết quả hoặc vấn đề vào nhật ký tiến độ. Quyết định mới thì ghi vào nhật ký quyết định.

### Chia việc cho hai người (đề xuất)
| Người | Phụ trách |
|---|---|
| A: planner | Seed cố định (0.5), bản đồ (1.1), các lời gọi planner ở A1, A4 và C, các cải tiến I1–I3 (giai đoạn 4) |
| B: hạ tầng và thí nghiệm | Node đa robot, thư viện vùng giao nhau, thứ tự và mô phỏng, kịch bản, RViz, chạy hàng loạt và phân tích (1.2–1.7), tiêm sự cố (3.1) |
| Cả hai | Chốt câu hỏi và đọc tài liệu (giai đoạn 0), chạy thí nghiệm, viết báo cáo |

### Giai đoạn 0: chốt và chuẩn bị (khoảng 1 tuần)

**Mục tiêu:** thống nhất bài toán, nắm các công trình gần nhất, và làm planner chạy lặp lại được.

**Các bước:**
1. **Chốt câu hỏi.** Họp với GVHD, trả lời Q1–Q7 ở mục 10 và ghi vào nhật ký quyết định. Trong buổi đó, báo luôn rằng khung phối hợp đã có trong [12], nên đóng góp của đồ án nằm ở planner (mục 4.1).
2. **Đọc tài liệu.** Đọc [12], [13], [20], [24]. Mỗi bài trả lời câu hỏi dưới đây, rồi ghi 3–5 dòng ngay dưới mục tham khảo đó ở mục 11:
   - [12]: vùng tới hạn được tính thế nào, robot dừng chờ ở đâu, thứ tự được chọn và sửa ra sao?
   - [13]: ADG được xây từ kế hoạch thế nào, khi nào một robot được đi tiếp?
   - [20]: luật RPP và điều kiện "hạ tầng hợp lệ" áp dụng cho bản đồ rooms thế nào?
   - [24]: khi bản đồ đổi, sửa cây RRT thì bỏ phần nào, giữ phần nào?
3. **Seed cố định.**
   - Hiện mọi bộ sinh số lấy seed từ `std::random_device`: trong hàm khởi tạo của `BiasSampler` (`sampler.h`), và trong `brrt_optimize` của `SOF_brrt_simple_case3.h`.
   - Thêm khóa `seed` vào file JSON tham số (đọc ở `testcase.h`). Mặc định −1 là giữ ngẫu nhiên như cũ; seed ≥ 0 thì mọi bộ sinh số khởi tạo từ seed đó.
   - Ghi seed vào JSON kết quả.

**Xong khi:**
- mục 10 có câu trả lời;
- chạy hai lần cùng seed cho cùng đường đi và cùng số vòng lặp;
- seed −1 vẫn chạy ngẫu nhiên như cũ.

| # | Việc | Ước lượng | Trạng thái | Ghi chú |
|---|---|---|---|---|
| 0.1 | Reproduce, có cấu hình khớp paper | – | [x] | `paper_table1_final.json`; bảng đầy đủ 10 môi trường ở lần 9 của KET_QUA_REPRODUCE.md |
| 0.2 | Giải thích phần số vòng lặp còn lệch | 1–2 ngày | [x] | 28/10: do code bỏ mẫu nằm trong vật cản (H9) |
| 0.3 | Chốt câu hỏi mục 10 với GVHD | 0.5 ngày | [ ] | |
| 0.4 | Đọc [12], [13], [20], [24], ghi tóm tắt | 2–3 ngày | [ ] | Ưu tiên [12] |
| 0.5 | Seed cố định | 0.5 ngày | [ ] | |

### Giai đoạn 1: hạ tầng (khoảng 2 tuần)

**Mục tiêu:** chạy được N robot trên một bản đồ, từ lập kế hoạch tới mô phỏng, xem được trên RViz, và tự kiểm tra được đúng sai. Chưa cần đủ mọi phương pháp.

**Các bước, theo thứ tự:**
1. **Bản đồ** (`src/occ_grid/include/occ_grid/occ_map.h`, `src/occ_grid/src/occ_map.cpp`).
   - Hiện `OccMap` chỉ nạp vật cản một lần từ point cloud (`globalCloudCallback`), không nới vật cản, và `setOccupancy` là hàm private.
   - Thêm các hàm public:
     - nới vật cản theo bán kính, trên mặt phẳng XY;
     - thêm vật cản hình hộp và hình đĩa trong lúc chạy (cho S3–S5);
     - xoá các vật cản đã thêm, để chạy kịch bản tiếp theo.
   - Planner dùng chung con trỏ `env::OccMap::Ptr` (xem chỗ tạo planner trong `test_planner.cpp`), nên bản đồ đổi thì planner thấy ngay. Không phải sửa RRT-FConnect.
2. **Node đa robot.** File mới `src/path_finder/src/multi_robot_node.cpp`, thêm `add_executable` vào `src/path_finder/CMakeLists.txt`, và một launch mới dựa trên `test_planners.launch`.
   - Đầu vào: bản đồ `.pcd`, file kịch bản N robot (JSON), file tham số RRT-FConnect.
   - Tạo một `OccMap` và một `SOF_BRRT_Simple_Case3`. Lập đường cho từng robot bằng `plan(s, g)`, rồi lấy `getPath()`.
   - Đầu ra: JSON gồm đường của từng robot, vùng giao nhau, thứ tự, lịch, kết quả mô phỏng và các tiêu chí ở mục 2.
3. **Thư viện đa robot** (header mới trong `src/path_finder/include/path_finder/multi/`). Mỗi phần có một chương trình kiểm thử nhỏ:
   - `path_utils.h`: tham số hoá đường gấp khúc theo độ dài cung (bỏ trục z), lấy vị trí theo độ dài cung, tính độ trơn và clearance (mục 2.3).
   - `conflict_zones.h`: bước A2, kiểm thử bốn trường hợp ở mục 7.
   - `precedence.h`: bước A3: đồ thị phụ thuộc, phát hiện chu trình, lịch danh định.
   - `executor.h`: bước B: mô phỏng theo bước Δt, chờ theo thứ tự, kiểm tra va chạm độc lập. Phần sự cố thêm ở giai đoạn 3.
4. **Kịch bản N robot.** Script mới (ví dụ `generate_multi_scenarios.py`) sinh ba chế độ ở mục 7 trên các bản đồ `.pcd` đã có. Dùng BFS trên lưới đã nới để kiểm tra mỗi robot có đường khi đứng một mình.
5. **Bản đồ khe hẹp.**
   - `dataset_map_generator.py` đã có tham số `~gap_width` (mặc định 20 m) cho flappy và rooms; riêng narrow viết cứng `gap_w = 20.0`.
   - Cho narrow dùng `gap_width`, thêm tùy chọn `--gap-width` vào `generate_dataset.py`, và đặt tên thư mục kèm độ rộng khe (ví dụ `narrow_10_gap3`).
   - Mặc định vẫn là 20 m, nên các bản đồ cũ không đổi.
6. **RViz.** Phát lại N robot theo thời gian: đĩa robot, đường, vùng giao nhau, chỗ xảy ra sự cố. Dùng lại lớp `visualization::Visualization` mà `test_planner.cpp` đang dùng.
7. **Chạy hàng loạt.**
   - `run_multi_experiments.py`, theo mẫu `run_dataset_experiments.py`: mỗi bản đồ một lần chạy, bỏ qua bản đồ đã xong.
   - `analyze_multi.py`: in bảng theo mục 2, kèm khoảng tin cậy.

**Xong khi:**
- không nới và không thêm vật cản thì kết quả single robot (cùng seed) giống hệt trước khi sửa;
- kịch bản 2 robot đi ngược chiều qua khe 3 m chạy hết, không va chạm, một robot chờ ngoài khe, và trên RViz thấy rõ điều đó.

| # | Việc | Ước lượng | Trạng thái | Ghi chú |
|---|---|---|---|---|
| 1.1 | Bản đồ: nới vật cản, thêm/xoá vật cản khi chạy, kiểm thử hồi quy | 1–2 ngày | [ ] | |
| 1.2 | Node đa robot, launch, JSON kết quả | 1–2 ngày | [ ] | |
| 1.3 | Thư viện: độ dài cung, vùng giao nhau, đồ thị phụ thuộc, mô phỏng, kèm kiểm thử | 4–5 ngày | [ ] | |
| 1.4 | Sinh kịch bản N robot | 1 ngày | [ ] | |
| 1.5 | Bản đồ khe 2–4 m | 0.5–1 ngày | [ ] | |
| 1.6 | RViz phát lại | 1–2 ngày | [ ] | Làm song song được |
| 1.7 | Chạy hàng loạt và phân tích | 2 ngày | [ ] | Dựa trên script hiện có |

### Giai đoạn 2: khung tĩnh, chưa có sự cố (khoảng 1 tuần)

**Mục tiêu:** có kết quả cho RQ1 và RQ4 (mục 6).

**Các bước:**
1. M0: lập đường độc lập, đếm vùng giao nhau. Kết quả này cho thấy vì sao cần phối hợp.
2. M1: lập lần lượt; đường đã lập của các robot trước được thêm vào bản đồ thành vật cản (dùng hàm thêm vật cản ở bước 1.1), và xoá đi sau mỗi kịch bản.
3. M2 và M3: các bước A1–A4 của mục 5.2, với RRT-Connect (`BRRT`) và với RRT-FConnect.
4. Chạy thử nhỏ trước: N = 2 và 4 trên blobs_20 và narrow khe 3 m, xem trên RViz từng ca thất bại. Sau đó mới chạy đủ bảng ở mục 8.

**Xong khi:** có bảng success, SOC, makespan, thời gian lập kế hoạch cho N = 2–8, và danh sách các ca thất bại đã phân loại (deadlock không sửa được, hết thời gian, kịch bản vô nghiệm).

| # | Việc | Ước lượng | Trạng thái | Ghi chú |
|---|---|---|---|---|
| 2.1 | M0, M1 | 1 ngày | [ ] | |
| 2.2 | M2, M3: thứ tự + sửa deadlock (A3, A4) | 2–3 ngày | [ ] | |
| 2.3 | Thí nghiệm RQ1, RQ4 | 2 ngày | [ ] | |

### Giai đoạn 3: sự cố (khoảng 1.5 tuần)

**Mục tiêu:** có kết quả cho RQ2.

**Các bước:**
1. Mỗi sự cố là một bản ghi trong JSON kịch bản: loại (S1–S8), robot, thời điểm, vị trí, tham số. Nhờ vậy mọi phương pháp gặp đúng cùng một sự cố.
2. Trong `executor.h`: tiêm sự cố, phát hiện (mục 3), rồi xử lý theo bậc của lớp C (mục 5.1):
   1. chờ;
   2. sửa đường cho robot bị ảnh hưởng, từ vị trí hiện tại, trên bản đồ đã cập nhật;
   3. tính lại vùng giao nhau và thứ tự cho phần đường còn lại;
   4. cuối cùng mới lập lại cho tất cả.
3. Thêm baseline thực thi theo giờ (mục 5.3).
4. Đo độ trễ phục hồi bằng đồng hồ đơn điệu.

**Xong khi:** có bảng cho từng sự cố × mức độ × phương pháp (tỉ lệ hoàn thành, suy giảm makespan, độ trễ phục hồi), và thực thi theo thứ tự không có va chạm nào.

| # | Việc | Ước lượng | Trạng thái | Ghi chú |
|---|---|---|---|---|
| 3.1 | Tiêm sự cố S1–S6 | 2 ngày | [ ] | |
| 3.2 | Xử lý: chờ, sửa đường robot bị ảnh hưởng, lập lại cho tất cả | 2–3 ngày | [ ] | |
| 3.3 | Thực thi theo giờ (baseline) | 0.5 ngày | [ ] | |
| 3.4 | Thí nghiệm RQ2 | 2 ngày | [ ] | Bảng chính của báo cáo |

### Giai đoạn 4: cải tiến (khoảng 2 tuần)

**Mục tiêu:** đóng góp riêng của đồ án (mục 5.4). Làm I1 trước.

**Các bước** (mọi thay đổi trong `SOF_brrt_simple_case3.h` đều sau cờ JSON):
- **I1:**
  - thêm hàm nhận các điểm trên đường của robot khác, lưu vào một kd-tree riêng (`kdtree.c` có sẵn);
  - trong `shootRays`, tia đi gần các điểm đó thì dừng sớm hơn, nên sector đó ít được chọn hơn;
  - trong `heuristicSteer` (APF), thêm lực đẩy nhẹ từ các điểm đó.
- **I2:** tính trước khoảng cách trên lưới bằng BFS từ start và từ goal (trên lưới đã nới), rồi dùng trong `computeH` thay cho khoảng cách đường thẳng.
- **I3:** giữ cây của lần lập trước; khi sửa đường, bỏ phần cây không còn hợp lệ như [24], rồi mở rộng tiếp.
- **I4** (tùy chọn): đổi thứ tự ở một vùng khi robot đi trước bị trễ quá ngưỡng.

Mỗi cải tiến được so với M3 trên cùng kịch bản, cùng seed, cùng sự cố.

**Xong khi:** mỗi cải tiến có bảng so với M3 và một kết luận có ích hay không. Kết luận "không có ích" cũng ghi lại.

| # | Việc | Ước lượng | Trạng thái | Ghi chú |
|---|---|---|---|---|
| 4.1 | I1: planner biết đường robot khác + thí nghiệm RQ3 | 3–4 ngày | [ ] | |
| 4.2 | I2: heuristic biết tường | 2 ngày | [ ] | |
| 4.3 | I3: dùng lại cây khi sửa đường | 2–3 ngày | [ ] | |
| 4.4 | I4: đổi thứ tự khi chạy | 2 ngày | [ ] | Tùy chọn |
| 4.5 | RQ5: so với một phương pháp lập kế hoạch động | – | [ ] | Chỉ khi GVHD yêu cầu |

### Giai đoạn 5: báo cáo
| # | Việc | Trạng thái |
|---|---|---|
| 5.1 | Chương tổng quan: mục 1–4 | [ ] |
| 5.2 | Chương reproduce single robot | [ ] |
| 5.3 | Chương phương pháp: mục 5, 7 | [ ] |
| 5.4 | Chương thí nghiệm: RQ1–RQ4 (RQ5) | [ ] |

### Nhật ký quyết định
| Ngày | Quyết định | Căn cứ | Người |
|---|---|---|---|
| 28/10/2026 | ~~Hướng chính: prioritized + RPP + SI-RRT-FConnect (lập kế hoạch không gian–thời gian)~~. Thay bằng quyết định ngày 29/10 | Bản 28/10 của tài liệu | |
| 28/10/2026 | Đo thời gian bằng đồng hồ đơn điệu; trên máy chạy WSL thì mask `systemd-timesyncd` | Lần 7 của KET_QUA_REPRODUCE.md | |
| 29/10/2026 | Hướng chính: lập kế hoạch tĩnh (đường RRT-FConnect + thứ tự qua vùng giao nhau, xong trước khi chạy) và xử lý sự cố khi chạy. Lập kế hoạch không gian–thời gian chuyển ra ngoài phạm vi | Yêu cầu của nhóm; mục 1, 5 | |

### Nhật ký tiến độ
| Ngày | Đã làm | Kết quả / vấn đề | Việc tiếp theo |
|---|---|---|---|
| 28/10/2026 | Reproduce: cấu hình cuối (H9: không bỏ mẫu nằm trong vật cản); tìm ra và sửa lỗi đồng hồ WSL | Số vòng lặp và tỉ lệ thời gian gần paper; số liệu thời gian cũ bị nhiễu | Chạy lại sạch (lần 9) |
| 28/10/2026 | Chạy lại sạch (lần 9): 10 môi trường, EBi-RRT trên flappy/rooms, các vòng quét | 4 môi trường của paper: tái hiện mọi kết luận của Table 2. Flappy/rooms: RRT-FConnect chậm hơn RRT-Connect tới 1.9 lần, nhưng giải được nhiều bài nhất ở mật độ 40% | Giai đoạn 0.3–0.5 |
| 29/10/2026 | Soạn lại tài liệu theo hướng tĩnh: tiêu chí (mục 2), kịch bản sự cố (mục 3), khung đề xuất (mục 5), hướng dẫn làm tiếp từng giai đoạn (mục 9) | Chờ nhóm chốt các câu hỏi ở mục 10 | Giai đoạn 0 |

---

## 10. Câu hỏi cần chốt

| # | Câu hỏi | Đề xuất |
|---|---|---|
| Q1 | Kế hoạch tĩnh có cho robot dừng chờ ở mép vùng giao nhau không (đường được phép đi sát nhau, phân xử bằng thứ tự)? Hay bắt buộc các đường không đi sát nhau? | Cho chờ (5.1). Tách hẳn chỉ làm baseline M1 |
| Q2 | Khi có sự cố, chỉ sửa đường robot bị ảnh hưởng hay lập lại cho tất cả? | Sửa robot bị ảnh hưởng trước, thất bại mới lập lại cho tất cả. So cả hai |
| Q3 | Mô hình robot: đĩa holonomic, hay robot vi sai (giới hạn độ cong)? | Holonomic trước; vi sai là mở rộng |
| Q4 | Robot biết sự cố bằng cách nào: máy trung tâm biết ngay, hay chỉ thấy trong tầm cảm biến? | Vật cản mới: chỉ thấy trong tầm `d_s`. Robot trễ hay hỏng: máy trung tâm biết ngay |
| Q5 | Sự cố nào là bắt buộc? | S1–S5 bắt buộc; S6–S8 tùy thời gian |
| Q6 | Có cần so với một phương pháp lập kế hoạch động (RQ5)? | Tùy GVHD |
| Q7 | Chạy trên mô phỏng 2D + RViz, hay Gazebo / robot thật? | Mô phỏng 2D + RViz; Gazebo nếu kịp |

---

## 11. Tài liệu tham khảo

Tra cứu ngày 28/10/2026, riêng [5], [10], [12]–[17], [23]–[25] tra ngày 29/10/2026. ✓ = đã đối chiếu nội dung; ~ = chỉ xác nhận thư mục hoặc qua nguồn thứ cấp.

**Bài toán, độ phức tạp, tiêu chí**
1. ✓ R. Stern et al., *Multi-Agent Pathfinding: Definitions, Variants, and Benchmarks*, SoCS 2019. [arXiv:1906.08291](https://arxiv.org/abs/1906.08291)
2. ~ J. Hopcroft, J. Schwartz, M. Sharir, *On the Complexity of Motion Planning for Multiple Independent Objects; PSPACE-Hardness of the "Warehouseman's Problem"*, 1984. [bản scan](https://usercontent.one/wp/sokoban.dk/wp-content/uploads/2016/02/Hopcroft-et-Al-On-the-compelxity-of-motion-planning.pdf)
3. ~ P. Spirakis, C. Yap, *Strong NP-hardness of moving many discs*, Information Processing Letters 19(1):55–59, 1984. [DOI](https://doi.org/10.1016/0020-0190(84)90130-3)
4. ✓ J. Yu, S. LaValle, *Structure and Intractability of Optimal Multi-Robot Path Planning on Graphs*, AAAI 2013. [AAAI](https://ojs.aaai.org/index.php/AAAI/article/view/8541)
5. ✓ E. Heiden, L. Palmieri, K. O. Arras, G. S. Sukhatme, S. Koenig, *Experimental Comparison of Global Motion Planning Algorithms for Wheeled Mobile Robots*, arXiv 2020. [arXiv:2003.03543](https://arxiv.org/abs/2003.03543). Benchmark phát triển tiếp: Bench-MR, IEEE RA-L 6(3):4536–4543, 2021. [DOI](https://doi.org/10.1109/LRA.2021.3068913)
6. ✓ T. Q. Chi, V. Q. Huy, L. H. Trang, *RRT-FConnect: Fast Bidirectional Path Planning in Cluttered and Narrow-Passage Environments*, PRICAI 2026. Paper chính của đồ án.

**Tách đường và phối hợp**

7. ~ K. Kant, S. Zucker, *Toward efficient trajectory planning: the path-velocity decomposition*, IJRR, 1986 (chưa đối chiếu số trang).
8. ~ P. O'Donnell, T. Lozano-Pérez, *Deadlock-free and collision-free coordination of two robot manipulators*, ICRA 1989, pp. 484–489.
9. ✓ S. Leroy, J.-P. Laumond, T. Siméon, *Multiple Path Coordination for Mobile Robots: A Geometric Algorithm*, IJCAI 1999. [mlanthology](https://mlanthology.org/ijcai/1999/leroy1999ijcai-multiple). Bản tạp chí: *Path coordination for multiple mobile robots: a resolution-complete algorithm*. [HAL](https://hal.laas.fr/hal-01987712)
10. ✓ J. Peng, S. Akella, *Coordinating Multiple Robots with Kinodynamic Constraints Along Specified Paths*, IJRR 24(4):295–310, 2005. [PDF](https://webpages.charlotte.edu/sakella/papers/PengAkellaIJRR05.pdf)
11. ✓ W. Hönig et al., *Multi-Agent Path Finding with Kinematic Constraints*, ICAPS 2016. [AAAI](https://ojs.aaai.org/index.php/ICAPS/article/view/13796)
12. ✓ F. Pecora, H. Andreasson, M. Mansouri, V. Petkov, *A Loosely-Coupled Approach for Multi-Robot Coordination, Motion Planning and Control*, ICAPS 2018, pp. 485–493. [AAAI](https://ojs.aaai.org/index.php/ICAPS/article/view/13923). Code: [coordination_oru](https://github.com/FedericoPecora/coordination_oru)

**Thực thi bền vững**

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

**Lập lại đường khi môi trường thay đổi**

23. ✓ J. Bruce, M. Veloso, *Real-Time Randomized Path Planning for Robot Navigation*, IROS 2002, pp. 2383–2388. [CMU](https://www.ri.cmu.edu/publications/real-time-randomized-path-planning-for-robot-navigation)
24. ✓ D. Ferguson, N. Kalra, A. Stentz, *Replanning with RRTs*, ICRA 2006, pp. 1243–1248. [CMU](https://www.ri.cmu.edu/publications/replanning-with-rrts)
25. ✓ M. Otte, E. Frazzoli, *RRTX: Asymptotically Optimal Single-Query Sampling-Based Motion Planning with Quick Replanning*, IJRR 35(7):797–822, 2016. [PDF](https://www.ottelab.com/html_stuff/pdf_files/Otte.Frazzoli.IJRR15.pdf)

**Ngoài phạm vi: lập kế hoạch động, ghép, CBS, học máy**

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
