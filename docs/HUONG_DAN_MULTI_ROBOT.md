# Hoạch định đa robot dựa trên RRT-FConnect: các hướng tiếp cận, chứng cứ và lựa chọn

Tài liệu định hướng cho giai đoạn 2–3 của đồ án *Hoạch định đường đi đa robot dựa vào lấy mẫu có định hướng*. Tài liệu trả lời ba câu hỏi:
1. Hiện có những họ phương pháp nào cho bài toán đa robot, mỗi họ có chứng cứ gì (mục 1–5)?
2. Với đồ án này, nên chọn hướng nào, và vì sao (mục 6–7)?
3. Làm cụ thể ra sao, theo dõi tiến độ ở đâu (mục 8–10)?

**Cách đọc chứng cứ.** Mỗi khẳng định về một công trình có số tham khảo `[n]`, danh sách ở mục 11. Ký hiệu trong danh sách:
- ✓: đã đối chiếu với tóm tắt hoặc nội dung bài trên arXiv, nhà xuất bản hoặc trang của tác giả (ngày 28/10/2026).
- ~: chỉ xác nhận được thông tin thư mục, hoặc nội dung lấy từ nguồn thứ cấp. Cần đọc bản gốc trước khi trích trong báo cáo.

Số liệu của chính đồ án lấy từ [KET_QUA_REPRODUCE.md](KET_QUA_REPRODUCE.md). Code single robot: [HUONG_DAN_RRT-FConnect.md](HUONG_DAN_RRT-FConnect.md).

**Mục lục**
1. [Bài toán và thuật ngữ](#1-bài-toán-và-thuật-ngữ)
2. [Giới hạn lý thuyết và hệ quả](#2-giới-hạn-lý-thuyết-và-hệ-quả)
3. [Tiêu chí chọn hướng cho đồ án](#3-tiêu-chí-chọn-hướng-cho-đồ-án)
4. [Các họ phương pháp và chứng cứ](#4-các-họ-phương-pháp-và-chứng-cứ)
5. [Bảng so sánh tổng hợp](#5-bảng-so-sánh-tổng-hợp)
6. [Lập luận chọn hướng](#6-lập-luận-chọn-hướng)
7. [Câu hỏi nghiên cứu và giả thuyết](#7-câu-hỏi-nghiên-cứu-và-giả-thuyết)
8. [Thiết kế kỹ thuật của hướng được chọn](#8-thiết-kế-kỹ-thuật-của-hướng-được-chọn)
9. [Thiết kế thí nghiệm](#9-thiết-kế-thí-nghiệm)
10. [Lộ trình và bảng theo dõi](#10-lộ-trình-và-bảng-theo-dõi)
11. [Tài liệu tham khảo](#11-tài-liệu-tham-khảo)

---

## 1. Bài toán và thuật ngữ

### 1.1. Phát biểu

Cho bản đồ tĩnh 2D, N robot hình đĩa bán kính r, vận tốc tối đa `v_max`. Robot i có điểm đầu `s_i` và điểm đích `g_i`. Cần tìm N **quỹ đạo** `τ_i(t)` (vị trí theo thời gian) sao cho:
- không quỹ đạo nào chạm vật cản tĩnh;
- `‖τ_i(t) − τ_j(t)‖ ≥ 2r` với mọi t và mọi cặp i ≠ j;
- `‖τ̇_i(t)‖ ≤ v_max`; robot được phép đứng chờ, và đứng yên ở đích sau khi tới.

Trong tài liệu, *MAPF* (multi-agent path finding) chỉ bài toán trên đồ thị/lưới với thời gian rời rạc, còn *MRMP* (multi-robot motion planning) chỉ bài toán trong không gian liên tục như trên. Đồ án thuộc loại MRMP, nhưng nhiều ý tưởng quan trọng (CBS, ưu tiên, safe interval) đến từ MAPF.

### 1.2. Các loại xung đột

Stern và cộng sự [1] chuẩn hoá các loại xung đột trong MAPF: **điểm** (hai agent cùng ở một đỉnh tại cùng thời điểm), **cạnh** (cùng đi một cạnh), **theo sau** (vào đỉnh mà agent khác vừa rời), **vòng** và **đổi chỗ** (hai agent đổi vị trí cho nhau trong một bước). Bài cũng lưu ý xung đột đổi chỗ thường bị gọi nhầm là "xung đột cạnh" trong các bài trước đó, nên khi đọc tài liệu cũ cần xem định nghĩa cụ thể [1].

Trong không gian liên tục chỉ còn một điều kiện: khoảng cách hai tâm < 2r tại một thời điểm nào đó. Điều kiện này bao trùm mọi loại trên. Tuy vậy, khi phân tích kết quả vẫn nên phân loại xung đột (giao nhau, đi ngược chiều, robot đứng ở đích chắn đường), vì mỗi loại cần cơ chế xử lý khác nhau (mục 4).

### 1.3. Mục tiêu tối ưu

- **Sum of costs (SOC):** tổng thời gian đến đích của các robot.
- **Makespan:** thời điểm robot cuối cùng đến đích.

Yu và LaValle [4] chứng minh rằng trên đồ thị, tối ưu makespan, tổng thời gian đến đích hay tổng quãng đường đều NP-hard, và các mục tiêu này không thể tối ưu đồng thời (chúng tạo thành cấu trúc Pareto). Vì vậy đồ án nên chọn một mục tiêu chính (đề xuất SOC) và báo cáo cả hai.

### 1.4. Giả định mặc định

| Giả định | Giá trị đề xuất | Lý do |
|---|---|---|
| Không gian | 2D (x, y) | Tia LEA và APF của RRT-FConnect đều làm trên mặt phẳng |
| Robot | Đĩa bán kính r = 0.5 m | Bằng độ phân giải bản đồ |
| Động học | Holonomic, `v_max` = 1 m/s, được đứng chờ | Đủ để làm rõ bài toán phối hợp; động học xe để sau |
| Sau khi đến đích | Đứng yên mãi | Chuẩn của MAPF [1] |
| Kiểu lập kế hoạch | Offline, tập trung (một máy biết mọi robot) | Phù hợp phạm vi đồ án |

---

## 2. Giới hạn lý thuyết và hệ quả

- Hopcroft, Schwartz và Sharir [2] chứng minh bài toán phối hợp chuyển động của nhiều vật thể độc lập ("warehouseman's problem", các hình chữ nhật cần dời về vị trí đích) là **PSPACE-hard**.
- Spirakis và Yap [3], theo tiêu đề bài (*Strong NP-hardness of moving many discs*), cho kết quả NP-hard mạnh cho bài toán dời nhiều đĩa, gần với mô hình robot đĩa của đồ án.
- Ngay cả trên đồ thị, tìm lời giải **tối ưu** đã là NP-hard [4].

**Hệ quả cho đồ án.** Không có thuật toán nào vừa đầy đủ (complete), vừa tối ưu, vừa mở rộng tốt theo N. Mỗi họ phương pháp ở mục 4 chọn một điểm đánh đổi giữa ba tính chất đó. Báo cáo cần nói rõ đồ án chọn điểm đánh đổi nào, và dùng thí nghiệm để đo cái giá phải trả.

---

## 3. Tiêu chí chọn hướng cho đồ án

| # | Tiêu chí | Vì sao quan trọng với đồ án này |
|---|---|---|
| C1 | **Giữ được đóng góp "lấy mẫu có định hướng"**: RRT-FConnect (heuristic chọn cặp, LEA, APF) phải là phần lõi | Đây là tên và trọng tâm của đề tài |
| C2 | **Xử lý được nút thắt** (narrow passage, khe của flappy, cửa của rooms) | Xung đột giữa robot dồn vào các nút thắt. Đây cũng là nơi RRT-FConnect mạnh nhất: ở Highly và Narrow, nó nhanh hơn RRT-Connect 13–38% ([KET_QUA_REPRODUCE.md](KET_QUA_REPRODUCE.md), lần 3) |
| C3 | **Mở rộng theo N** (mục tiêu 2–16 robot, thử 32) | Mức thường thấy trong các bài MRMP dạng sampling [12][32][33] |
| C4 | **Có đảm bảo hoặc mô tả được khi nào đúng** | Cần cho phần lý thuyết của báo cáo |
| C5 | **Khả thi** trong thời gian đồ án, trên code C++/ROS1 hiện có | Ràng buộc thực tế |
| C6 | **Có baseline công bằng, có code tham chiếu** | Để kết quả có giá trị so sánh |

---

## 4. Các họ phương pháp và chứng cứ

### 4.1. Ghép (coupled): lập kế hoạch trong không gian tích của mọi robot

**Ý tưởng.** Coi N robot là một robot trong không gian 2N chiều, chạy một planner duy nhất. Lời giải tìm được tự động không va chạm.

| Phương pháp | Công bố | Điểm chính và chứng cứ |
|---|---|---|
| Coordinated PRM | Švestka & Overmars, RAS 1998 [5] | Ghép roadmap của từng robot thành "super-graph" |
| PRM tập trung và tách rời | Sánchez & Latombe, IJRR 2002 [6] | Theo tóm tắt: trên trạm hàn mô phỏng 6 robot (36 bậc tự do), lập kế hoạch tập trung hiệu quả hơn nhiều so với tách rời |
| MA-RRT\* | Čáp và cộng sự, AAMAS 2013 [7] | RRT\* trong không gian chung; mở rộng tốt hơn tìm kiếm kiểu A\* trong môi trường lớn nhưng thưa |
| dRRT | Solovey, Salzman, Halperin, IJRR 2016 [8] | RRT trên tích *ẩn* của các roadmap đơn robot; thử đến 60 bậc tự do; nhanh hơn ít nhất 10 lần so với các thuật toán tác giả biết (bản arXiv ghi 15 lần) |
| dRRT\* | Shome và cộng sự, Autonomous Robots 2020 [9] | Bản tối ưu tiệm cận của dRRT có heuristic; mở rộng được tới số robot lớn hơn ở những nơi các phương án khác thất bại |

**Chứng cứ về giới hạn.** Trong thí nghiệm của K-CBS [33], RRT chạy thẳng trên không gian chung (cRRT) chỉ giải được trường hợp 2 robot ở môi trường mở, với tỉ lệ thành công thấp, và luôn thất bại ở hành lang hẹp. Các phương pháp ghép chạy được ở quy mô lớn (dRRT, dRRT\*) đều không dùng một cây RRT thô trên không gian chung, mà dựa vào **roadmap riêng của từng robot**.

**Đánh giá.** Đầy đủ theo xác suất và có bản tối ưu tiệm cận, nhưng **kém ở C1**: tia LEA và lực APF được định nghĩa trong mặt phẳng của một robot, không có nghĩa trong không gian 2N chiều. Muốn dùng thì phải đổi RRT-FConnect thành bộ sinh roadmap cho từng robot, tức là bỏ cấu trúc hai cây. Không nên làm hướng chính; có thể nhắc như hướng đầy đủ và tối ưu để so sánh về lý thuyết.

### 4.2. Ghép khi cần (hybrid)

**Ý tưởng.** Lập kế hoạch riêng cho từng robot, và chỉ ghép không gian của những robot thực sự xung đột với nhau.

| Phương pháp | Công bố | Điểm chính và chứng cứ |
|---|---|---|
| Subdimensional expansion (sRRT, sPRM) | Wagner, Kang, Choset, ICRA 2012 [10] | Chỉ ghép khi các robot ở gần nhau; nhanh hơn hơn 10 lần so với RRT/PRM gốc; giải bài 32 robot (128 bậc tự do) dưới 10 phút |
| Mở rộng sampling dẫn đường bởi MAPF | Le & Plaku, IJCAI 2018 [11] | Cây mở rộng trong không gian liên tục của mọi robot, được dẫn bởi tìm kiếm MAPF trên một trừu tượng rời rạc |
| SSSP | Okumura & Défago, IJCAI 2023 [12] | Xây roadmap đồng thời với tìm kiếm MAPF; đảm bảo tìm được lời giải nếu có; chạy với 32 robot trong tình huống dày đặc |
| Merge trong K-CBS | [33] | Hai robot xung đột lặp lại nhiều lần thì được gộp thành một "meta-robot" (xem 4.5) |

**Đánh giá.** Đây là cách tốt để vá điểm yếu của hướng tách rời ở nút thắt, nhưng phức tạp. Phù hợp làm **cải tiến** sau khi đã có hướng chính, không phải điểm bắt đầu.

### 4.3. Tách đường và vận tốc (path coordination)

**Ý tưởng.** Bước 1: tìm đường hình học cho từng robot, bỏ qua robot khác. Bước 2: chỉ chỉnh thời gian dọc theo các đường đó (chờ, chậm lại) để tránh nhau.

| Phương pháp | Công bố | Điểm chính và chứng cứ |
|---|---|---|
| Path–velocity decomposition | Kant & Zucker, IJRR 1986 [13] | Đề xuất tách bài toán thành đường đi + vận tốc |
| Phối hợp hai tay máy | O'Donnell & Lozano-Pérez, ICRA 1989 [14] | Lập lịch phối hợp hai tay máy, không va chạm và không deadlock |
| Coordination diagram | Leroy, Laumond, Siméon, IJCAI 1999 [15] | Biểu diễn vật cản trong coordination diagram bằng hình hộp bao; đầy đủ theo độ phân giải; ví dụ với hơn 100 robot |
| MAPF-POST | Hönig và cộng sự, ICAPS 2016 [16] | Hậu xử lý lời giải MAPF bằng mạng thời gian đơn (simple temporal network) trong thời gian đa thức, để tôn trọng giới hạn vận tốc và khoảng cách an toàn |

**Lập luận về giới hạn.** Đường đã cố định thì chỉ còn chỉnh được thời gian, nên tính đầy đủ chỉ là *đầy đủ so với các đường đã cho* [15]. Phương pháp thất bại khi không có thứ tự đi nào hợp lệ. Ví dụ điển hình: đích của robot A nằm trên đường của robot B và đích của B nằm trên đường của A. Robot nào đến trước cũng đứng yên mãi và chặn robot kia. Trường hợp đi ngược chiều qua một hành lang thì thường vẫn giải được: một robot chờ ngoài hành lang cho robot kia đi qua.

**Đánh giá.** Tốt nhất ở C1 (dùng nguyên RRT-FConnect), C3 và C5. Yếu ở C2 khi các robot chặn nhau. Nên làm **baseline đầu tiên**: rẻ, và buộc phải xây trước toàn bộ hạ tầng (quỹ đạo, kiểm tra va chạm, metric).

### 4.4. Ưu tiên (prioritized planning) trong không gian–thời gian

**Ý tưởng.** Sắp thứ tự các robot. Lập kế hoạch lần lượt: robot k tìm quỹ đạo trong không gian–thời gian, coi quỹ đạo của robot 1..k−1 là **vật cản động** [17].

**Khung và cách chọn thứ tự:**

| Phương pháp | Công bố | Điểm chính và chứng cứ |
|---|---|---|
| Prioritized planning gốc | Erdmann & Lozano-Pérez, Algorithmica 1987 [17] | Gán ưu tiên, lập kế hoạch từng vật thể trong không gian cấu hình–thời gian |
| Prioritized motion planning | van den Berg & Overmars, IROS 2005 [18] | Thường được trích như một bài chuẩn về prioritized planning cho nhiều robot (nội dung chưa đối chiếu) |
| Tìm thứ tự ưu tiên | Bennewitz, Burgard, Thrun, RAS 2002 [19] | Một thứ tự cố định có thể thất bại dù lời giải tồn tại; dùng leo đồi ngẫu nhiên trên không gian thứ tự để tìm thứ tự giải được và đường ngắn |
| DMA-RRT | Desaraju & How, Autonomous Robots 2012 [39] | Phiên bản phân tán dựa trên RRT; thứ tự lập lại kế hoạch quyết định bằng "token" theo lợi ích |
| **Revised PP (RPP)** | Čáp và cộng sự, IEEE T-ASE 2015 [20] | Xem phân tích bên dưới |
| **PBS** | Ma và cộng sự, AAAI 2019 [21] | Tìm kiếm trên các thứ tự ưu tiên *từng phần*, chỉ thêm quan hệ ưu tiên khi gặp xung đột; chất lượng lời giải và tỉ lệ thành công thuộc loại tốt nhất, thời gian chạy thường tương đương; có kết quả lý thuyết về giới hạn đầy đủ và tối ưu của prioritized planning |
| Ưu tiên theo "path prospects" | Wu, Bhattacharya, Prorok, ICRA 2020 [22] | Robot có ít lựa chọn đường (ít lớp đồng luân) được ưu tiên trước; tỉ lệ thành công cao nhất trong 5 phương án so sánh trên lưới |

**Vì sao prioritized planning không đầy đủ, và cách vá (Čáp và cộng sự [20]).** Bài chỉ ra hai kiểu thất bại:
- **Kiểu A:** robot bị chặn bởi một robot ưu tiên cao hơn *đã đứng ở đích*.
- **Kiểu B:** robot bị robot ưu tiên cao hơn "đâm" khi đang đứng ở điểm đầu chờ xuất phát, vì robot ưu tiên cao không biết robot ưu tiên thấp tồn tại.

RPP sửa kiểu B bằng một luật: khi lập kế hoạch robot i, **tránh luôn vùng điểm đầu của mọi robot ưu tiên thấp hơn**. RPP được chứng minh luôn thành công nếu mỗi robot có đường đi tránh được điểm đầu của các robot ưu tiên thấp hơn và điểm đích của các robot ưu tiên cao hơn, và planner đơn robot là đầy đủ. Điều kiện này luôn đúng trong các "hạ tầng hợp lệ": có một tập điểm đầu/đích sao cho **mọi cặp điểm nối được bằng một đường cách vật cản ≥ r và cách các điểm đầu/đích khác ≥ 2r**. Bài lập luận rằng văn phòng, nhà kho và mạng đường có cấu trúc như vậy.

Kết quả thí nghiệm của bài: với nhiệm vụ kiểu hạ tầng, RPP giải hết mọi instance, kể cả những instance mà PP gốc và ORCA không giải được. Với nhiệm vụ tự do, RPP giải được ít hơn PP. Đường của RPP dài hơn một chút.

Bản đồ **rooms** của đồ án chính là một hạ tầng kiểu văn phòng. Nếu sinh điểm đầu/đích theo điều kiện trên thì kiểm chứng được định lý này bằng thực nghiệm (mục 7, RQ3).

**Planner đơn robot trong không gian–thời gian (thành phần quyết định chất lượng):**

| Phương pháp | Công bố | Cách xử lý thời gian | Chứng cứ |
|---|---|---|---|
| SIPP | Phillips & Likhachev, ICRA 2011 [23] | Tìm kiếm trên cặp (cấu hình, **khoảng an toàn**). Số bước thời gian an toàn có thể vô hạn, nhưng số khoảng an toàn thì hữu hạn và thường rất nhỏ | Cùng đảm bảo tối ưu và đầy đủ như khi thêm chiều thời gian, nhưng nhanh hơn đáng kể (mô phỏng với tới 200 vật cản động); thử trên robot PR2 |
| TB-RRT | Sintov & Shapiro, ICRA 2014 [24] | Thêm thời gian vào trạng thái; **thời gian đến đích cố định chọn trước**; một chiều | Bài toán gặp nhau (rendezvous) với mục tiêu động |
| ST-RRT\* | Grothe, Hartmann, Orthey, Toussaint, ICRA 2022 [25] | **Hai chiều**. Cây goal có nhiều gốc: đích ở các thời điểm lấy mẫu. Khoảng thời gian được nới dần theo từng đợt; sau khi có lời giải thì cận trên bằng thời điểm đến của lời giải. Lấy mẫu có điều kiện: chỉ lấy trạng thái vừa tới được từ start vừa còn kịp về đích (giao của hai "nón" vận tốc). Khoảng cách bằng ∞ nếu ngược thời gian hoặc vượt vận tốc | Lời giải đầu tiên nhanh hơn RRT-Connect và RRT\* trong hầu hết trường hợp, chi phí cuối thấp hơn hoặc bằng; có thí nghiệm "tuần tự" lập kế hoạch cho robot thứ 6 và thứ 11 khi các robot trước đã có quỹ đạo (đúng kịch bản prioritized); có sẵn trong OMPL |
| **SI-RRT** | Kerimov, Onegin, Yakovlev, ICAPS 2025 [26] | **RRT-Connect + khoảng an toàn.** Node = (cấu hình, khoảng an toàn, thời điểm đến). Cây start tìm thời điểm đến *sớm nhất*. Cây goal có gốc là đích với khoảng an toàn cuối cùng và thời điểm `t_max`, suy luận ngược để tìm thời điểm đến *muộn nhất* còn khả thi. Mở rộng kiểu "chờ rồi đi". Khi hai cây gặp nhau thì đảo chiều nhánh goal và cắt bớt thời gian chờ thừa | Ở 200 vật cản động: thành công 77% so với 42% của ST-RRT\*; ở 100–120 vật cản: nhanh hơn ST-RRT\* khoảng 10 lần; thời điểm đến sớm hơn. Giới hạn tự nêu: chưa có phân tích lý thuyết, không anytime, mới thử một agent |
| SI-RRT và ST-RRT\* trong prioritized | Kerimov, Onegin, Yakovlev, ICR 2025 [27] | So sánh hai planner trên trong prioritized planning cho nhiều tay máy (2–8 robot) | Trên benchmark TAPAS: SI-RRT thành công nhiều hơn ở mọi kịch bản, nhanh hơn, makespan thấp hơn. Trên kịch bản tổng hợp, SI-RRT chậm hơn khi có 6–8 robot. Bài nhấn mạnh rằng trong prioritized planning, **planner đơn robot quyết định hiệu năng** |

**Chứng cứ về giới hạn của prioritized với planner sampling.** Trong [33], prioritized RRT (pRRT) đạt 100% ở môi trường mở, nhưng thời gian tăng rất nhanh theo số robot (vì phải kiểm tra va chạm với mọi robot trước). Nó gặp khó ở hành lang hẹp, không vượt quá 7 robot ở môi trường nhiều vật cản, và thất bại ở mọi instance của môi trường lớn. Như vậy, prioritized + RRT thô không đủ ở nút thắt; cần planner đơn robot tốt hơn và cơ chế chọn thứ tự (RPP, PBS).

**Đánh giá.** Rất tốt ở C1: mỗi robot là một lần gọi planner đơn robot trong không gian–thời gian, đúng chỗ để LEA, APF và heuristic chọn cặp phát huy. Tốt ở C3 và C6 (ST-RRT\* có trong OMPL [25], SI-RRT có code công khai [26]). C4 có điều kiện đảm bảo rõ ràng (RPP [20]). C2 ở mức trung bình, phải vá bằng RPP, tìm thứ tự (PBS) hoặc tầng CBS.

### 4.5. Họ Conflict-Based Search (CBS)

**Ý tưởng.** Tìm kiếm hai tầng. Tầng dưới lập kế hoạch cho từng robot riêng lẻ. Tầng trên là cây ràng buộc: tìm xung đột sớm nhất giữa hai robot, tách thành hai nhánh, mỗi nhánh cấm một robot đi vào vị trí và thời điểm xung đột, rồi cho robot đó lập lại kế hoạch.

| Phương pháp | Công bố | Điểm chính và chứng cứ |
|---|---|---|
| CBS | Sharon và cộng sự, AAAI 2012; AIJ 2015 [28] | Tối ưu; trong nhiều trường hợp xét ít trạng thái hơn A\*; nhanh hơn tới một bậc độ lớn so với các phương pháp trước. Bản SoCS 2012 [29] nhận xét: **CBS rất tốt khi có nhiều hành lang dài và nút thắt, kém khi có nhiều vùng mở rộng** |
| ECBS | Barer và cộng sự, SoCS 2014 [30] | Bản dưới tối ưu có cận (hệ số w ≥ 1): giảm mạnh thời gian chạy, chất lượng giảm ít |
| CCBS | Andreychuk và cộng sự, AIJ 2022 [31] | Thời gian liên tục; kết hợp SIPP (tầng dưới) với CBS; tối ưu theo SOC; thử trên lưới và roadmap |
| CBS-MP | Solis và cộng sự, RA-L 2021 [32] | CBS cho MRMP liên tục với roadmap; tới 32 agent, 8 tay máy nhiều bậc tự do, đội robot khác loại |
| **K-CBS** | Kottinger, Almagor, Lahijanian, IROS 2022 [33] | Tầng dưới là **bất kỳ planner cây dạng sampling** (RRT, KPIECE). Ràng buộc = thân robot kia trong khoảng thời gian xung đột, coi như vật cản động. Thừa hưởng tính đầy đủ xác suất của tầng dưới. Có gộp robot khi xung đột lặp lại |
| db-CBS | Moldagalieva và cộng sự, ICRA 2024 [34] | Ba tầng, có tối ưu quỹ đạo; anytime, đầy đủ xác suất, tối ưu tiệm cận; tốt hơn các phương pháp trước cho robot có động học |

**Số liệu của K-CBS [33]** (robot động học xe bậc 2):
- Môi trường mở: nhanh hơn pRRT 1–2 bậc độ lớn.
- Môi trường lớn 33×33: 10 robot thành công 92–100%; 20 robot thành công 30–74%; thời gian trung bình khoảng 105–291 s.
- Hành lang hẹp: gộp robot sớm (merge bound thấp) cho tỉ lệ thành công cao. Ở môi trường mở thì nên gộp muộn.

**Đánh giá.** Tốt ở C1 (tầng dưới là planner đơn robot), C2 (mạnh ở nút thắt [29], có gộp [33]) và C4. Khó ở C5: tầng dưới phải nhận tập ràng buộc tùy ý theo thời gian, và số lần gọi tầng dưới có thể rất lớn. Phù hợp làm **tầng nâng cao**, dùng lại planner của 4.4.

### 4.6. Tránh va chạm phản ứng (reactive)

**ORCA** (van den Berg, Guy, Lin, Manocha [36]): mỗi robot tự chọn vận tốc bằng một quy hoạch tuyến tính nhỏ, không cần liên lạc. Rất nhanh, nhưng là cơ chế cục bộ, không lập kế hoạch toàn cục. Čáp và cộng sự [20] cho thấy ORCA không giải được những instance kiểu hạ tầng mà RPP giải được. Vai trò hợp lý: baseline, hoặc lớp an toàn khi chạy robot thật. Không phù hợp làm hướng chính (yếu ở C1, C2, C4).

### 4.7. Học máy

**PRIMAL** (Sartoretti và cộng sự [37]) học chính sách phân tán bằng học tăng cường kết hợp bắt chước một planner MAPF, chạy được tới 1024 agent trên lưới quan sát một phần. Cần huấn luyện, làm trên lưới, và không giữ được C1. Chỉ nên nhắc trong phần tổng quan.

### 4.8. Tối ưu hoá

**ST-GCS** (Tang và cộng sự, 2025 [38]) mở rộng Graphs of Convex Sets sang chiều thời gian, tính quỹ đạo tối ưu thời gian bằng tối ưu lồi, và dùng **khung ưu tiên** với quỹ đạo của robot ưu tiên cao làm vật cản không gian–thời gian. Đây là chứng cứ cho thấy khung ưu tiên + vật cản không gian–thời gian vẫn là hướng nghiên cứu đang được quan tâm (2025). Kỹ thuật này khác hẳn sampling nên nằm ngoài phạm vi; có thể dùng làm mốc so sánh nếu có code.

### 4.9. MAPF trên lưới: mốc so sánh chất lượng

Các bộ giải MAPF như CBS/ECBS [28][30], PBS [21] và LaCAM [35] (đầy đủ, xử lý được hàng trăm agent trở lên) chạy được trên chính lưới chiếm chỗ 0.5 m của bản đồ. Kết quả của chúng là **mốc tham chiếu** cho SOC và makespan ở phiên bản rời rạc của bài toán, và có thể đưa sang robot liên tục bằng MAPF-POST [16]. Không phải phương pháp của đồ án, nhưng giúp trả lời câu hỏi "lời giải dạng sampling cách lời giải gần tối ưu bao xa".

---

## 5. Bảng so sánh tổng hợp

| Họ | Đầy đủ | Tối ưu | C1 giữ RRT-FConnect | C2 nút thắt | C3 mở rộng N | C4 lý thuyết | C5 khả thi | C6 baseline/code |
|---|---|---|---|---|---|---|---|---|
| Ghép [5–9] | Xác suất | Tiệm cận (dRRT\*) | Kém | Tốt | Kém nếu dùng RRT thô [33] | Tốt | Trung bình | Có bài; code chưa kiểm tra |
| Ghép khi cần [10–12] | Có (SSSP) | Tuỳ | Trung bình | Tốt | Khá (32 robot) | Tốt | Khó | Ít |
| Tách đường – vận tốc [13–16] | So với đường đã cho | Không | **Rất tốt** | Kém khi chặn nhau | Rất tốt (>100 robot [15]) | Có (đầy đủ theo độ phân giải) | **Dễ** | Tự xây |
| **Ưu tiên + planner không gian–thời gian** [17–27] | Không (RPP: có điều kiện [20]) | Không | **Rất tốt** | Trung bình, vá bằng RPP/PBS | Tốt | Có điều kiện rõ [20] | Trung bình | **Tốt**: ST-RRT\* trong OMPL, SI-RRT có code |
| CBS + sampling [28–34] | Xác suất (K-CBS) | Gần tối ưu | Tốt (tầng dưới) | **Tốt** [29][33] | Khá (20 robot, 30–74% [33]) | Tốt | Khó | Có |
| Phản ứng (ORCA) [36] | Không | Không | Kém | Kém [20] | Rất tốt | Không | Dễ (RVO2) | Có |
| Học máy [37] | Không | Không | Kém | Tuỳ | Rất tốt | Không | Khó | Có |

Ghi chú: "Kém/Tốt" là đánh giá tương đối cho mục tiêu của đồ án, dựa trên chứng cứ trích dẫn trong ô.

---

## 6. Lập luận chọn hướng

**L1 (giới hạn).** Không thể đạt đồng thời đầy đủ, tối ưu và mở rộng tốt (mục 2). Phải chọn điểm đánh đổi.

**L2 (đóng góp).** Đóng góp của đề tài là *lấy mẫu có định hướng*. Hướng được chọn phải để RRT-FConnect làm lõi (C1). Điều này loại họ ghép (4.1), phản ứng (4.6) và học máy (4.7) khỏi vai trò hướng chính. Còn lại ba họ có lõi là planner đơn robot: tách đường – vận tốc (4.3), ưu tiên (4.4) và CBS (4.5).

**L3 (thời gian là bắt buộc).** Tách đường – vận tốc không đổi được đường, nên thất bại khi robot chặn nhau (4.3). Ưu tiên và CBS đều cần một **planner đơn robot trong không gian–thời gian, có vật cản động**. Thành phần này xuất hiện trong cả hai, nên đầu tư vào nó được dùng lại cho cả hai.

**L4 (planner đơn robot quyết định hiệu năng).** Trong prioritized planning, planner đơn robot quyết định kết quả [27]. pRRT thô thất bại ở hành lang hẹp và khi N lớn [33]. Như vậy, cải thiện planner đơn robot là đóng góp **đo được trực tiếp**, đúng chỗ mạnh của RRT-FConnect (nhanh hơn RRT-Connect 13–38% ở Highly và Narrow, đường ngắn hơn 3–10%, ít node hơn 12–33%; [KET_QUA_REPRODUCE.md](KET_QUA_REPRODUCE.md)).

**L5 (giữ được hai chiều).** Có thể nghĩ rằng cây goal không biết thời điểm đến nên không thể giữ cấu trúc hai cây. Hai công trình gần đây cho thấy làm được:
- ST-RRT\* [25]: nhiều gốc goal theo thời gian, nới dần khoảng thời gian.
- SI-RRT [26]: gốc goal là khoảng an toàn cuối cùng của đích, suy luận ngược để tìm thời điểm đến muộn nhất.

SI-RRT xây trên **RRT-Connect**, cùng gốc với RRT-FConnect (hai cây, CONNECT tham lam, SWAP), và có chứng cứ thắng ST-RRT\* cả khi chạy một robot (77% so với 42% thành công, nhanh hơn khoảng 10 lần [26]) lẫn trong prioritized nhiều robot (thành công cao hơn trên TAPAS [27]). Vì vậy **khung SI-RRT là nền phù hợp nhất để đặt RRT-FConnect vào không gian–thời gian.**

**L6 (vá điểm yếu ở nút thắt bằng cơ chế đã được chứng minh).**
- Luật RPP (tránh điểm đầu của robot ưu tiên thấp hơn) cho đảm bảo trong hạ tầng hợp lệ [20].
- Tìm thứ tự ưu tiên (đổi thứ tự ngẫu nhiên [19], rồi PBS [21]).
- Nếu vẫn chưa đủ thì dùng CBS kiểu K-CBS [33], vì CBS mạnh ở nút thắt [29].

PBS nên làm trước CBS: nó dùng lại **đúng** lời gọi tầng dưới của prioritized ("lập kế hoạch robot i tránh tập robot ưu tiên cao hơn"), trong khi CBS cần tầng dưới nhận tập ràng buộc tùy ý.

**L7 (tính mới).** Bản thân việc kết hợp RRT-Connect không gian–thời gian với prioritized planning **không mới**: [27] đã làm với SI-RRT. Các planner không gian–thời gian dạng sampling đã tìm thấy [25][26][27] **đều lấy mẫu đều**. Ngược lại, các bài cải tiến RRT-Connect bằng heuristic mà lần tìm mở rộng (28/10/2026) trả về đều chỉ cho một robot. Chưa thấy công trình nào đưa heuristic chọn cặp, LEA và APF vào không gian–thời gian, hay cho tia LEA "nhìn thấy" robot khác theo thời gian. **Đây là khoảng trống đồ án có thể lấp.** Kết luận này phụ thuộc vào phạm vi tra cứu; cần tìm thêm (Google Scholar, IEEE Xplore) trước khi viết vào báo cáo.

### Kết luận lựa chọn

| Vai trò | Hướng | Căn cứ |
|---|---|---|
| **Hướng chính** | Prioritized planning (luật RPP) + **SI-RRT-FConnect**: RRT-FConnect đặt trong khung khoảng an toàn của SI-RRT | L2–L7 |
| Baseline tối thiểu | Tách đường – vận tốc với RRT-FConnect | 4.3: rẻ, dựng hạ tầng |
| Baseline ngoài | Prioritized + ST-RRT\* (OMPL) | [25], công bằng, có sẵn |
| Baseline trong (ablation) | Prioritized + SI-RRT-Connect (cùng khung, bỏ heuristic/LEA/APF) | Đo riêng đóng góp của phần "có định hướng" |
| Tầng nâng cao | PBS, rồi K-CBS nếu còn thời gian | L6 |
| Mốc chất lượng (tùy chọn) | ECBS/LaCAM trên lưới 0.5 m | 4.9 |

---

## 7. Câu hỏi nghiên cứu và giả thuyết

| # | Câu hỏi | Giả thuyết kiểm chứng được | Thí nghiệm |
|---|---|---|---|
| RQ1 | Ưu thế của RRT-FConnect có còn khi có vật cản động? | Một robot giữa các robot đã lập kế hoạch: SI-RRT-FConnect nhanh hơn SI-RRT-Connect và ST-RRT\*, đường ngắn hơn, nhất là ở nút thắt. **Rủi ro đã thấy ở single robot:** trên flappy/rooms (tường có khe), RRT-FConnect ít vòng lặp hơn nhưng chậm hơn RRT-Connect tới 1.9 lần (riêng rooms_20 ngang), vì số vòng giảm không đủ bù chi phí mỗi vòng cao gấp khoảng 2 lần (lần 9 của KET_QUA_REPRODUCE.md). Vì vậy cần kiểm tra cả khả năng kết quả ngược lại | Chạy robot thứ k (k = 2, 6, 11, như [25]) trên mọi bản đồ |
| RQ2 | LEA và APF "biết robot khác" có giúp không? | Bật tia nhìn thấy robot khác theo thời gian thì giảm số vòng lặp và tăng thành công khi N lớn | Ablation từng cờ |
| RQ3 | Luật RPP có cho đảm bảo trên bản đồ rooms? | Với điểm đầu/đích sinh theo điều kiện hạ tầng hợp lệ [20], prioritized + RPP thành công gần 100% (giới hạn chỉ do planner xác suất và giới hạn thời gian) | Kịch bản "hạ tầng" trên rooms |
| RQ4 | Tầng tìm thứ tự (PBS) bù được bao nhiêu khi N tăng? | Thành công của PBS cao hơn prioritized thứ tự cố định, chênh lệch tăng theo N và ở nút thắt | N = 2–16 (32) |
| RQ5 | Lời giải cách mốc gần tối ưu bao xa? | SOC của hướng chính trong khoảng vài chục phần trăm so với ECBS trên lưới | Tùy chọn |

---

## 8. Thiết kế kỹ thuật của hướng được chọn

### 8.1. Hạ tầng chung

- **`Trajectory`**: các waypoint `(p_k, t_k)`, nội suy tuyến tính, đứng yên sau điểm cuối.
- **Kiểm tra va chạm chính xác giữa hai đoạn chuyển động đều.** Với hiệu vị trí đầu khoảng `Δa` và hiệu vận tốc `Δu` trên khoảng `[t_a, t_b]`:
  `τ* = clamp(−(Δa·Δu)/‖Δu‖², 0, t_b − t_a)`, rồi `d_min = ‖Δa + Δu·τ*‖`. Có xung đột nếu `d_min < 2r + margin`.
- **Khoảng an toàn tại một điểm p (giải tích, cho robot đĩa 2D).** Với mỗi đoạn quỹ đạo `c(t) = a + u(t − t₀)` của robot khác, phương trình `‖p − c(t)‖² = (2r + m)²` là phương trình bậc hai theo t. Khoảng giữa hai nghiệm là khoảng *không an toàn*. Robot đứng ở đích tạo khoảng không an toàn `[t_đến, ∞)` nếu đích gần p. Hợp các khoảng không an toàn, lấy phần bù trong `[0, t_max]`, được danh sách khoảng an toàn.
  SI-RRT tính khoảng an toàn bằng cách kiểm tra va chạm tại từng bước thời gian (30 lần mỗi giây) [26]. Với robot đĩa 2D, tính giải tích như trên là chính xác và rẻ hơn. Đây là một điểm cải tiến kỹ thuật nhỏ.
- **`ValidityChecker`** thay cho các lời gọi trực tiếp `map_ptr_->isStateValid/isSegmentValid` trong planner, để chèn vật cản động mà không sửa logic thuật toán. Kiểm thử hồi quy: với vật cản tĩnh, kết quả reproduce phải giữ nguyên.
- **Nới rộng vật cản tĩnh** theo r trong `OccMap` (`inflate_radius`).
- **Kịch bản N robot**, ba chế độ: ngẫu nhiên; "đổi chỗ" (hai nhóm đi ngược chiều qua nút thắt); "hạ tầng" (cho RQ3: chọn tập điểm đầu/đích sao cho mọi cặp nối được bằng đường cách vật cản ≥ r và cách các điểm khác ≥ 2r [20]; kiểm tra bằng planner trên bản đồ đã nới rộng thêm r quanh các điểm đó).
- **Bản đồ có nút thắt thật sự.** Khe của narrow, flappy và cửa của rooms hiện rộng 20 m (`gap_width` trong `dataset_map_generator.py`; narrow viết cứng `gap_w = 20`). So với robot rộng 1 m, các khe này rất rộng, nên robot đi qua nhau thoải mái. Cần sinh thêm biến thể khe 2–4 m (khoảng 2–4 lần đường kính robot) để có xung đột thật.
- **RViz**: phát lại quỹ đạo N robot theo thời gian. Đây là công cụ debug quan trọng nhất.

### 8.2. SI-RRT-FConnect

| Thành phần | SI-RRT [26] | Thêm phần của RRT-FConnect |
|---|---|---|
| Node | (p, khoảng an toàn, t đến) | Giữ nguyên |
| Gốc cây start / goal | (s, khoảng đầu, t = 0) / (g, khoảng cuối, `t_max`), suy luận ngược | Giữ nguyên |
| Chọn node để mở rộng | Nearest neighbor của mẫu đều | **Heuristic chọn cặp + kho 100 cặp** với ρ thích nghi (Eq. 1–3). Cặp không khả thi về thời gian (`t_goal − t_start < ‖Δp‖/v_max`) có h = ∞ |
| Lấy mẫu | Đều | **LEA**: tia từ `(p, t)` dừng ở vật cản tĩnh, **hoặc robot khác tại thời điểm `t ± s/v_max`** (cây goal dùng dấu trừ) |
| Steer | Thẳng | **APF**: hút về node dẫn đường, xoay tiếp tuyến quanh vật cản; thêm xoay quanh robot khác |
| Thời gian | Chờ rồi đi, đến sớm nhất trong khoảng an toàn | Giữ nguyên |
| CONNECT | Tham lam, đảo nhánh goal, cắt thời gian chờ thừa | Giữ nguyên |
| Chế độ exploration (20/50) | Không có | Giữ, vì [KET_QUA_REPRODUCE.md](KET_QUA_REPRODUCE.md) cho thấy nó cần thiết |
| Mẫu nằm trong vật cản | Bỏ và lấy mẫu lại | Không bỏ, vẫn steer về phía mẫu: đây là cấu hình gần paper nhất ([paper_table1_final.json](../src/path_finder/config/paper_table1_final.json)). Trong không gian–thời gian, mẫu "chạm robot khác" cũng nên được xử lý giống vậy; nên đo cả hai cách |

Mọi phần thêm đều đặt sau cờ (như lần reproduce), để chạy được SI-RRT-Connect (tắt hết) làm baseline trong.

**Điểm yếu cần lưu ý: heuristic chọn cặp không biết tường.**
- Heuristic dùng khoảng cách đường thẳng. Trên flappy/rooms, cặp node gần nhau nhất thường nằm hai bên một bức tường. Kết quả: số vòng lặp chỉ giảm 18–45% so với RRT-Connect (blobs_40: 60%), trong khi mỗi vòng lặp tốn khoảng gấp đôi, nên tổng thời gian chậm hơn.
- Bản đồ khe 2–4 m cho multi-robot (8.1) còn khó hơn.
- Phương án nên đo: khoảng cách trên lưới (geodesic) tính trước một lần bằng BFS từ start và từ goal, dùng thay khoảng cách đường thẳng trong h. Giống mọi phần thêm khác, phương án này đặt sau cờ.

### 8.3. Prioritized + RPP + tìm thứ tự

```
order ← heuristic (mặc định: đường đơn robot dài trước; thử "path prospects" [22])
for lần thử = 1..R:
    for robot k theo order:
        others ← quỹ đạo các robot trước (kể cả đứng ở đích mãi)
        cấm thêm vùng điểm đầu của các robot sau k (luật RPP [20])
        τ_k ← SI-RRT-FConnect(s_k, g_k, others)
        nếu thất bại: đưa k lên đầu (hoặc xáo ngẫu nhiên [19]) và thử lại
PBS [21]: thay vòng thử lại bằng tìm kiếm trên thứ tự ưu tiên từng phần
```

### 8.4. CBS kiểu K-CBS (nếu còn thời gian)

Ràng buộc = thân robot kia trong khoảng thời gian xung đột, coi như vật cản động [33]. Tầng dưới đã hỗ trợ vật cản động thì chỉ cần cộng thêm ràng buộc vào tập `others`. Gộp hai robot khi chúng xung đột quá B lần [33].

---

## 9. Thiết kế thí nghiệm

| Hạng mục | Lựa chọn |
|---|---|
| Bản đồ | blobs_10/20/40, narrow_10, flappy_10/20/40, rooms_10/20/40 (có sẵn) + biến thể khe 2–4 m (8.1) |
| Số robot N | 2, 4, 8, 16 (32 nếu kịp) |
| Kịch bản | Ngẫu nhiên, đổi chỗ, hạ tầng; mỗi bản đồ × N: 10 kịch bản × 5 lần chạy |
| Thuật toán | Tách đường – vận tốc · PP + ST-RRT\* (OMPL) · PP + SI-RRT-Connect · PP + SI-RRT-FConnect · + RPP · + PBS · (K-CBS) · (ECBS trên lưới) |
| Metric | Thành công (kiểm lại độc lập bằng bộ kiểm tra va chạm của 8.1, không tin planner), SOC, makespan, thời gian lập kế hoạch, số node/vòng lặp, số lần thử lại/nút PBS |
| Giới hạn thời gian | 10 s (bảng chính), 60 s (xem giới hạn) |
| Báo cáo | Trung vị và khoảng tứ phân vị cho thời gian và chi phí; tỉ lệ thành công kèm khoảng tin cậy |

**Bài học từ lần reproduce, áp dụng ngay:**
- Mọi thay đổi đặt sau cờ, mặc định là hành vi cũ.
- **Đo thời gian bằng đồng hồ đơn điệu** (`ros::SteadyTime`, `std::chrono::steady_clock`), không dùng `ros::Time::now()`. Trước khi chạy, kiểm tra đồng hồ WSL (mục 2b của HowToRunCode.md). Ở máy chạy chính, đồng hồ thực từng bị chỉnh nhảy +1,4 s mỗi 34 s, làm nhiễu toàn bộ số liệu thời gian của 6 lần chạy đầu.
- Chạy baseline xen kẽ trong cùng lần chạy và so tỉ lệ thời gian, vì tốc độ CPU của laptop thay đổi theo điện năng và nhiệt.
- Dùng `check_timing.py` để tìm lần chạy bị nhiễu (so với các lần chạy có số vòng lặp và số node tương đương) và chạy lại cả map, không ước lượng, không lọc bỏ lần chạy dài.
- **Kiểm tra tính khả thi của kịch bản** (BFS trên lưới chiếm chỗ) khi sinh kịch bản. Dataset hiện tại không kiểm tra: flappy_40 có 1/100 bài vô nghiệm, rooms_40 có 2/100. Với nhiều robot, mỗi robot phải có đường đi khi đứng một mình.
- Dùng giới hạn thời gian, không dùng giới hạn vòng lặp. Ở single robot, giới hạn 100.000 vòng lặp làm thuật toán có vòng lặp rẻ (RRT-Connect) dừng sớm hơn về thời gian, nên success ở flappy_40/rooms_40 không so được theo cùng ngân sách thời gian.
- Giữ máy thức, cắm sạc khi chạy dài.
- Cố định seed và ghi seed vào JSON.

---

## 10. Lộ trình và bảng theo dõi

Đánh dấu `[x]` khi xong, ghi ngày vào cột Ghi chú. Ước lượng tính cho một người làm toàn thời gian.

### Giai đoạn 0: hoàn tất single robot
| # | Việc | Ước lượng | Trạng thái | Ghi chú |
|---|---|---|---|---|
| 0.1 | Reproduce, có cấu hình khớp paper | – | [x] | `paper_table1_final.json`; bảng đầy đủ 10 môi trường ở lần 9 của KET_QUA_REPRODUCE.md (đo lại sau khi sửa đồng hồ) |
| 0.2 | Giải thích phần số vòng lặp còn lệch | 1–2 ngày | [x] | 28/10: do code bỏ mẫu nằm trong vật cản (H9) |
| 0.3 | Chốt giả định 1.4, ghi vào nhật ký quyết định | 0.5 ngày | [ ] | |
| 0.4 | Seed cố định được | 0.5 ngày | [ ] | |
| 0.5 | Đọc kỹ [20], [25], [26], [33] | 2–3 ngày | [ ] | Ưu tiên [26] |

### Giai đoạn 1: hạ tầng (8.1)
| # | Việc | Ước lượng | Trạng thái | Ghi chú |
|---|---|---|---|---|
| 1.1 | `Trajectory` + kiểm tra va chạm giải tích + kiểm thử (đi ngược chiều, song song cách 3r, qua cùng điểm cách 2 s, đứng ở đích chắn đường) | 2 ngày | [ ] | |
| 1.2 | Khoảng an toàn giải tích + kiểm thử | 1–2 ngày | [ ] | |
| 1.3 | `ValidityChecker`, kiểm thử hồi quy với reproduce | 1–2 ngày | [ ] | |
| 1.4 | Nới rộng vật cản | 1 ngày | [ ] | |
| 1.5 | Sinh kịch bản (3 chế độ) + bản đồ khe 2–4 m | 1–2 ngày | [ ] | |
| 1.6 | Node chạy hàng loạt + phân tích kết quả | 2 ngày | [ ] | Dựa trên script hiện có |
| 1.7 | RViz phát lại quỹ đạo | 1–2 ngày | [ ] | Có thể giao người thứ hai |

### Giai đoạn 2: baseline tách đường – vận tốc (4.3)
| # | Việc | Ước lượng | Trạng thái | Ghi chú |
|---|---|---|---|---|
| 2.1 | RRT-FConnect từng robot + trễ xuất phát/chờ tại waypoint | 2 ngày | [ ] | |
| 2.2 | Thí nghiệm N = 2–8, ghi các ca thất bại do chặn nhau | 1 ngày | [ ] | Dùng làm lập luận cho giai đoạn 3 |

### Giai đoạn 3: hướng chính (8.2, 8.3)
| # | Việc | Ước lượng | Trạng thái | Ghi chú |
|---|---|---|---|---|
| 3.1 | SI-RRT-Connect (tái hiện khung [26] cho robot đĩa 2D) | 4–5 ngày | [ ] | Baseline trong |
| 3.2 | Tích hợp ST-RRT\* của OMPL làm baseline ngoài | 2 ngày | [ ] | Kiểm tra gói `ros-noetic-ompl` |
| 3.3 | SI-RRT-FConnect: heuristic chọn cặp + kho cặp | 2–3 ngày | [ ] | |
| 3.4 | + LEA theo thời gian | 2 ngày | [ ] | |
| 3.5 | + APF với robot khác | 2 ngày | [ ] | |
| 3.6 | Prioritized + RPP + thử lại thứ tự | 2 ngày | [ ] | |
| 3.7 | Thí nghiệm RQ1–RQ3 | 3 ngày | [ ] | Bảng chính của báo cáo |

### Giai đoạn 4: nâng cao
| # | Việc | Ước lượng | Trạng thái | Ghi chú |
|---|---|---|---|---|
| 4.1 | PBS | 3 ngày | [ ] | RQ4 |
| 4.2 | K-CBS (tùy thời gian) | 4–5 ngày | [ ] | |
| 4.3 | ECBS/LaCAM trên lưới làm mốc (tùy chọn) | 2 ngày | [ ] | RQ5 |

### Giai đoạn 5: báo cáo
| # | Việc | Trạng thái |
|---|---|---|
| 5.1 | Chương tổng quan: dùng mục 1–5 của tài liệu này | [ ] |
| 5.2 | Chương reproduce single robot | [ ] |
| 5.3 | Chương phương pháp: mục 6, 8 | [ ] |
| 5.4 | Chương thí nghiệm: RQ1–RQ5 | [ ] |

### Nhật ký quyết định
| Ngày | Quyết định | Căn cứ | Người |
|---|---|---|---|
| 28/10/2026 | Hướng chính: prioritized + RPP + SI-RRT-FConnect; baseline tách đường – vận tốc, ST-RRT\*, SI-RRT-Connect; nâng cao PBS, rồi K-CBS | Mục 6 (L1–L7) | |
| 28/10/2026 | Đo thời gian bằng đồng hồ đơn điệu; trên máy chạy WSL thì mask `systemd-timesyncd` | Lần 7 của KET_QUA_REPRODUCE.md | |

### Nhật ký tiến độ
| Ngày | Đã làm | Kết quả / vấn đề | Việc tiếp theo |
|---|---|---|---|
| 28/10/2026 | Reproduce: cấu hình cuối (H9: không bỏ mẫu nằm trong vật cản); tìm ra và sửa lỗi đồng hồ WSL | Số vòng lặp và tỉ lệ thời gian gần paper; số liệu thời gian cũ bị nhiễu | Chạy lại sạch (lần 9), rồi giai đoạn 0.3–0.5 |
| 28/10/2026 | Chạy lại sạch (lần 9): 10 môi trường, EBi-RRT trên flappy/rooms, các vòng quét | 4 môi trường của paper: tái hiện mọi kết luận của Table 2. Flappy/rooms: RRT-FConnect chậm hơn RRT-Connect tới 1.9 lần nhưng giải được nhiều bài nhất ở mật độ 40% | Giai đoạn 0.3–0.5 |

---

## 11. Tài liệu tham khảo

Tra cứu ngày 28/10/2026. ✓ = đã đối chiếu nội dung; ~ = chỉ xác nhận thư mục hoặc qua nguồn thứ cấp.

**Định nghĩa và độ phức tạp**
1. ✓ R. Stern et al., *Multi-Agent Pathfinding: Definitions, Variants, and Benchmarks*, SoCS 2019. [arXiv:1906.08291](https://arxiv.org/abs/1906.08291)
2. ~ J. Hopcroft, J. Schwartz, M. Sharir, *On the Complexity of Motion Planning for Multiple Independent Objects; PSPACE-Hardness of the "Warehouseman's Problem"*, 1984. [bản scan](https://usercontent.one/wp/sokoban.dk/wp-content/uploads/2016/02/Hopcroft-et-Al-On-the-compelxity-of-motion-planning.pdf)
3. ~ P. Spirakis, C. Yap, *Strong NP-hardness of moving many discs*, Information Processing Letters 19(1):55–59, 1984. [DOI](https://doi.org/10.1016/0020-0190(84)90130-3)
4. ✓ J. Yu, S. LaValle, *Structure and Intractability of Optimal Multi-Robot Path Planning on Graphs*, AAAI 2013. [AAAI](https://ojs.aaai.org/index.php/AAAI/article/view/8541)

**Ghép và ghép khi cần**

5. ~ P. Švestka, M. Overmars, *Coordinated path planning for multiple robots*, Robotics and Autonomous Systems 23:125–152, 1998. [báo cáo kỹ thuật](https://dspace.library.uu.nl/handle/1874/18349)
6. ~ G. Sánchez, J.-C. Latombe, *On Delaying Collision Checking in PRM Planning – Application to Multi-Robot Coordination*, IJRR 21(1):5–26, 2002. [PDF](https://ai.stanford.edu/~latombe/cs326/2002/delayedcc.pdf)
7. ✓ M. Čáp, P. Novák, J. Vokřínek, M. Pěchouček, *Multi-agent RRT\*: Sampling-based Cooperative Pathfinding*, AAMAS 2013 (extended abstract). [arXiv:1302.2828](https://arxiv.org/abs/1302.2828)
8. ✓ K. Solovey, O. Salzman, D. Halperin, *Finding a needle in an exponential haystack: Discrete RRT for exploration of implicit roadmaps in multi-robot motion planning*, IJRR 35(5):501–513, 2016. [arXiv:1305.2889](https://arxiv.org/abs/1305.2889)
9. ✓ R. Shome, K. Solovey, A. Dobson, D. Halperin, K. Bekris, *dRRT\*: Scalable and Informed Asymptotically-Optimal Multi-Robot Motion Planning*, Autonomous Robots 44:443–467, 2020. [arXiv:1903.00994](https://arxiv.org/abs/1903.00994)
10. ✓ G. Wagner, M. Kang, H. Choset, *Probabilistic path planning for multiple robots with subdimensional expansion*, ICRA 2012. [CMU](https://www.ri.cmu.edu/publications/probabilistic-path-planning-for-multiple-robots-with-subdimensional-expansion)
11. ✓ D. Le, E. Plaku, *Multi-Robot Motion Planning with Dynamics Guided by Multi-Agent Search*, IJCAI 2018. [IJCAI](https://www.ijcai.org/proceedings/2018/744)
12. ✓ K. Okumura, X. Défago, *Quick Multi-Robot Motion Planning by Combining Sampling and Search*, IJCAI 2023. [arXiv:2203.00315](https://arxiv.org/abs/2203.00315)

**Tách đường – vận tốc**

13. ~ K. Kant, S. Zucker, *Toward efficient trajectory planning: the path-velocity decomposition*, IJRR, 1986 (chưa đối chiếu số trang).
14. ~ P. O'Donnell, T. Lozano-Pérez, *Deadlock-free and collision-free coordination of two robot manipulators*, ICRA 1989, pp. 484–489.
15. ✓ S. Leroy, J.-P. Laumond, T. Siméon, *Multiple Path Coordination for Mobile Robots: A Geometric Algorithm*, IJCAI 1999. [mlanthology](https://mlanthology.org/ijcai/1999/leroy1999ijcai-multiple). Bản tạp chí: *Path coordination for multiple mobile robots: a resolution-complete algorithm* [HAL](https://hal.laas.fr/hal-01987712)
16. ✓ W. Hönig et al., *Multi-Agent Path Finding with Kinematic Constraints*, ICAPS 2016. [AAAI](https://ojs.aaai.org/index.php/ICAPS/article/view/13796)

**Ưu tiên và planner không gian–thời gian**

17. ✓ M. Erdmann, T. Lozano-Pérez, *On Multiple Moving Objects*, Algorithmica 2:477–521, 1987. [CMU](https://www.ri.cmu.edu/publications/on-multiple-moving-objects)
18. ~ J. van den Berg, M. Overmars, *Prioritized motion planning for multiple robots*, IROS 2005, pp. 430–435.
19. ~ M. Bennewitz, W. Burgard, S. Thrun, *Finding and optimizing solvable priority schemes for decoupled path planning techniques for teams of mobile robots*, Robotics and Autonomous Systems, 2002. [PDF](https://www.hrl.uni-bonn.de/papers/bennewitz02ras.pdf)
20. ✓ M. Čáp, P. Novák, A. Kleiner, M. Selecký, *Prioritized Planning Algorithms for Trajectory Coordination of Multiple Mobile Robots*, IEEE T-ASE 12(3):835–849, 2015. [arXiv:1409.2399](https://arxiv.org/abs/1409.2399)
21. ✓ H. Ma, D. Harabor, P. Stuckey, J. Li, S. Koenig, *Searching with Consistent Prioritization for Multi-Agent Path Finding*, AAAI 2019. [AAAI](https://ojs.aaai.org/index.php/AAAI/article/view/4758)
22. ✓ W. Wu, S. Bhattacharya, A. Prorok, *Multi-Robot Path Deconfliction through Prioritization by Path Prospects*, ICRA 2020. [arXiv:1908.02361](https://arxiv.org/abs/1908.02361)
23. ✓ M. Phillips, M. Likhachev, *SIPP: Safe Interval Path Planning for Dynamic Environments*, ICRA 2011. [CMU](https://www.ri.cmu.edu/publications/sipp-safe-interval-path-planning-for-dynamic-environments)
24. ✓ A. Sintov, A. Shapiro, *Time-based RRT algorithm for rendezvous planning of two dynamic systems*, ICRA 2014. [PDF](https://robotics.sites.tau.ac.il/_files/ugd/c27470_d9e131e639c5490c962892ff9a2b6dc0.pdf)
25. ✓ F. Grothe, V. Hartmann, A. Orthey, M. Toussaint, *ST-RRT\*: Asymptotically-Optimal Bidirectional Motion Planning through Space-Time*, ICRA 2022. [arXiv:2203.02176](https://arxiv.org/abs/2203.02176)
26. ✓ N. Kerimov, A. Onegin, K. Yakovlev, *Safe Interval Randomized Path Planning For Manipulators*, ICAPS 2025. [arXiv:2412.19567](https://arxiv.org/abs/2412.19567)
27. ✓ N. Kerimov, A. Onegin, K. Yakovlev, *SI-RRT and ST-RRT\* For Prioritized Multi-Manipulator Planning: Empirical Evaluation*, ICR 2025. [trang bài](https://ia.spcras.ru/index.php/icr2026/article/view/17212)

**Họ CBS**

28. ✓ G. Sharon, R. Stern, A. Felner, N. Sturtevant, *Conflict-based search for optimal multi-agent pathfinding*, Artificial Intelligence 219:40–66, 2015 (bản đầu: AAAI 2012). [PDF](https://people.engr.tamu.edu/guni/pistar/Papers/CBS-AIJ15.pdf)
29. ~ G. Sharon et al., bản SoCS 2012 của CBS (nhận xét về nút thắt và vùng mở). [AAAI](https://ojs.aaai.org/index.php/SOCS/article/download/18222/18013/21738)
30. ✓ M. Barer, G. Sharon, R. Stern, A. Felner, *Suboptimal Variants of the Conflict-Based Search Algorithm for the Multi-Agent Pathfinding Problem*, SoCS 2014. [AAAI](https://ojs.aaai.org/index.php/SOCS/article/view/18315)
31. ✓ A. Andreychuk, K. Yakovlev, P. Surynek, D. Atzmon, R. Stern, *Multi-agent pathfinding with continuous time*, Artificial Intelligence 305:103662, 2022. [arXiv:1901.05506](https://arxiv.org/abs/1901.05506)
32. ✓ I. Solis, J. Motes, R. Sandström, N. Amato, *Representation-Optimal Multi-Robot Motion Planning Using Conflict-Based Search*, RA-L 6(3):4608–4615, 2021. [arXiv:1909.13352](https://arxiv.org/abs/1909.13352)
33. ✓ J. Kottinger, S. Almagor, M. Lahijanian, *Conflict-Based Search for Multi-Robot Motion Planning with Kinodynamic Constraints*, IROS 2022. [arXiv:2207.00576](https://arxiv.org/abs/2207.00576)
34. ✓ A. Moldagalieva, J. Ortiz-Haro, M. Toussaint, W. Hönig, *db-CBS: Discontinuity-Bounded Conflict-Based Search for Multi-Robot Kinodynamic Motion Planning*, ICRA 2024. [arXiv:2309.16445](https://arxiv.org/abs/2309.16445)
35. ✓ K. Okumura, *LaCAM: Search-Based Algorithm for Quick Multi-Agent Pathfinding*, AAAI 2023. [arXiv:2211.13432](https://arxiv.org/abs/2211.13432)

**Phản ứng, học máy, tối ưu hoá, phân tán**

36. ✓ J. van den Berg, S. Guy, M. Lin, D. Manocha, *Reciprocal n-Body Collision Avoidance*, ISRR 2009 (xuất bản 2011). [DOI](https://doi.org/10.1007/978-3-642-19457-3_1)
37. ✓ G. Sartoretti et al., *PRIMAL: Pathfinding via Reinforcement and Imitation Multi-Agent Learning*, RA-L 4(3):2378–2385, 2019. [arXiv:1809.03531](https://arxiv.org/abs/1809.03531)
38. ✓ J. Tang, Z. Mao, L. Yang, H. Ma, *Space-Time Graphs of Convex Sets for Multi-Robot Motion Planning*, arXiv 2025. [arXiv:2503.00583](https://arxiv.org/abs/2503.00583)
39. ✓ V. Desaraju, J. How, *Decentralized path planning for multi-agent teams with complex constraints*, Autonomous Robots 32:385–403, 2012. [MIT](https://dspace.mit.edu/handle/1721.1/62480)

**Tổng quan và nền tảng single robot**

40. ✓ A. Orthey, C. Chamzas, L. Kavraki, *Sampling-Based Motion Planning: A Comparative Review*, Annual Review of Control, Robotics, and Autonomous Systems 7:285–310, 2024. [arXiv:2309.13119](https://arxiv.org/abs/2309.13119)
41. ✓ H.-D. Bui, *A Survey of Multi-Robot Motion Planning*, arXiv 2023 (báo cáo kỳ thi tiến sĩ). [arXiv:2310.08599](https://arxiv.org/abs/2310.08599)
42. J. Kuffner, S. LaValle, *RRT-Connect: An efficient approach to single-query path planning*, ICRA 2000 (tài liệu [8] của paper RRT-FConnect).
43. T. Q. Chi, K. Van Xuan, L. H. Trang, *EBi-RRT: An Enhanced Bidirectional RRT via Target-Biased Sampling*, Springer 2026 (tài liệu [1] của paper RRT-FConnect).
44. S. Yu et al., *SOF-RRT\*: An improved path planning algorithm using spatial offset sampling*, Engineering Applications of Artificial Intelligence 126:106875, 2023 (tài liệu [14] của paper RRT-FConnect).
