# ACM-LOS: Adaptive Coverage Motion (cảm biến có che khuất)

nhóm Kiên & Phát, code lại thuật toán ACM-LOS theo bản thảo "Adaptive (Coverage) Motion for Path- and Turn-Efficient Online Complete Coverage in Unknown Static Environments"

file code: `ACM_LOS_FINAL.py`

> lưu ý: bản thảo + code chỉ dùng trong nhóm, ko push public lên GitHub (private thì được), gửi thẳng cho thầy/cô

---

## 1. Quy ước chung (đọc cái này trước)

- `grid[dòng][cột] == 1` là vật cản, `== 0` là ô trống. Viền ngoài bản đồ phải là vật cản hết
- 1 ô viết dạng tuple `o = (dòng, cột)`, `dòng` tăng khi đi xuống
- **hướng tuyệt đối**: `'N'` lên, `'E'` phải, `'S'` xuống, `'W'` trái. Xếp theo thứ tự `N, E, S, W` nên quay trái = `-1`, quay phải = `+1`, quay đầu = `+2` (mod 4)
- **hướng tương đối**: `'F'` trước, `'L'` trái, `'R'` phải, `'B'` sau. Robot chỉ được chọn trong `D = {F, L, R}` (8), `'B'` chỉ dùng ở ô xuất phát (Thuật toán 1, dòng 8 - 15)
- góc: chọn hướng Đông làm 0, `goc = atan2(dDong, dCot)` -> `E = 0`, `S = π/2`, `W = π`, `N = -π/2`
- trạng thái robot lưu trong class:
  - `self.pos` / `self.heading`: ô đang đứng + hướng đang quay
  - `self.covered`: set các ô đã đi qua (C_covered)
  - `self.L`: list các cặp quay lui `(c_b, c_u)` (L_t)
  - `self.path`: list cấu hình `(dòng, cột, góc)`, ứng với P = [q0, q1, ..., qT] (10)

---

## 2. Các hàm theo từng stage

trong code chỉ đánh dấu `# stage1` và `# stage 3`, README chia nhỏ ra thêm cho dễ đọc, thứ tự y như trong file

### Stage 0: hằng số

| tên | giá trị | công dụng |
|---|---|---|
| `DANH_SACH_HUONG` | `['N', 'E', 'S', 'W']` | thứ tự hướng để quay bằng cộng/trừ mod 4 |
| `BUOC_DI` | `{'N': (-1, 0), 'E': (0, 1), 'S': (1, 0), 'W': (0, -1)}` | 1 bước theo mỗi hướng tuyệt đối |
| `GOC_CUA_HUONG` | `{'N': -π/2, 'E': 0, 'S': π/2, 'W': π}` | góc θ của mỗi hướng, dùng để ghi vào `path` |

### Stage 1: hàm hỗ trợ (nằm ngoài class)

| hàm | tham số | trả về | công dụng |
|---|---|---|---|
| `turnRobot(huong_hien_tai, kieuQuay)` | `huong_hien_tai`: `'N'/'E'/'S'/'W'`; `kieuQuay`: `'L'/'R'/'B'` | hướng tuyệt đối mới | quay robot trái/phải/quay đầu, kiểu khác thì giữ nguyên |
| `getAbsoluteDirection(huong_hien_tai, huongTuongDoi)` | `huongTuongDoi`: `'F'/'L'/'R'/'B'` | hướng tuyệt đối | đổi hướng tương đối -> tuyệt đối, chỉ nhìn chứ ko quay thật |
| `moveCell(o, buoc, k = 1)` | `o`: ô gốc; `buoc`: vector 1 bước; `k`: số bước (âm là lùi) | ô mới | lấy ô cách `o` đúng `k` bước theo `buoc` |
| `getNeighbors4(o)` | `o`: ô | list 4 ô | 4 ô kề cạnh N4 (5), dùng trong BFS |
| `normalizeAngle(goc)` | `goc`: radian | góc trong (-π, π] | chuẩn hóa góc cho (40), (43) |
| `getFreeCells(grid)` | `grid`: bản đồ | set ô trống | lấy C_free (2), dùng để so số ô đã phủ |

### Stage 2: khởi tạo, cảm biến LOS, bao phủ thích nghi (mục 4.1, 4.3)

| hàm | tham số | trả về | công dụng |
|---|---|---|---|
| `ACM_LOS(grid, start, rho = 3.0, huongBanDau = 'N')` | `grid`: bản đồ; `start`: ô bắt đầu `(dòng, cột)`; `rho`: tầm nhìn ρ; `huongBanDau`: hướng lúc đầu | object robot | khởi tạo robot, `covered = {start}`, `L = []`, `path = [q0]`, reset bộ đếm. Báo `ValueError` nếu `rho < √2`, start nằm ngoài map hoặc start là vật cản |
| `isInsideMap(o)` | `o`: ô | `True/False` | ô có nằm trong bản đồ ko |
| `getCellsOnLine(A, B)` | `A`, `B`: 2 ô | list ô | lấy các ô mà đoạn thẳng tâm A -> tâm B đi xuyên qua (ko tính A, B) |
| `isSightBlocked(A, B)` | `A`, `B`: 2 ô | `True/False` | đường nhìn A -> B có bị vật cản che ko |
| `senseCell(o, diemDung = None)` | `o`: ô cần nhìn; `diemDung`: chỗ đứng nhìn (mặc định `self.pos`) | `'free'/'blocked'/'unknown'` | phép quan sát Z_t (3) kiểu LOS: ngoài map -> `blocked`, xa hơn ρ hoặc bị che -> `unknown`, còn lại đọc grid |
| `isFreeUncovered(o, diemDung = None)` | như trên | `True/False` | ô tự do chưa bao phủ, điều kiện (19) |
| `isBlocked(o, diemDung = None)` | như trên | `True/False` | ô chặn, điều kiện (20): `blocked` hoặc đã covered. Ko phải phủ định của hàm trên vì còn `unknown` |
| `getCellAt(huongTuongDoi, i=1, viTri=None, huong=None)` | `huongTuongDoi`: `'F'/'L'/'R'/'B'`; `i`: số bước; `viTri`, `huong`: mặc định lấy của robot | ô | lấy ô c^a_i (6) |
| `measureDirection(huongTuongDoi, viTri = None, huong = None)` | như trên | số nguyên | độ dài quan sát ℓ_t(a) (13): đếm ô trống chưa phủ liên tiếp, tối đa ⌊ρ⌋ |
| `getFeasibleDirections(viTri = None, huong = None)` | vị trí, hướng (mặc định của robot) | `(doDai, danhSachHuong)` | `doDai` là dict ℓ_t của F, L, R; `danhSachHuong` là tập A_t (14) |
| `chooseDirection(doDai, danhSachHuong)` | output của hàm trên (A_t khác rỗng) | `'F'/'L'/'R'` | chọn a* theo (15), bằng nhau thì ưu tiên F > L > R (16). Là `@staticmethod` |
| `turnToAngle(goc)` | `goc`: radian | ko | quay robot tới góc, nếu góc thật sự đổi thì ghi 1 cấu hình mới vào `path` |
| `turnTo(huong)` | `huong`: hướng tuyệt đối | ko | cập nhật `self.heading` + gọi `turnToAngle` |
| `moveToCell(o)` | `o`: ô kề cạnh | ko | bước sang `o`, thêm vào covered (18), ghi `path`, xóa các cặp có `c_u == o` khỏi L (33)(34) |

### Stage 3: cặp quay lui + phát hiện và bao phủ hole (mục 4.4, 4.5.1)

| hàm | tham số | trả về | công dụng |
|---|---|---|---|
| `checkBacktrackCondition(oTrong, oChan)` | `oTrong`: c^u; `oChan`: c^x | `1/0` | hàm χ_t(c^u, c^x) (29) |
| `findBacktrackPairs()` | ko | ko | xét 6 cấu hình Hình 3, tính κ_t (30), thêm cặp mới B_t (32) vào `self.L` (ko trùng) |
| `findHoleCandidates(boiCanh, huongSeQuay = None)` | `boiCanh`: `'forward'` (TH 1, 2) hoặc chữ khác, vd `'turn'` (TH 3, 4); `huongSeQuay`: `'L'/'R'` khi `boiCanh` là turn | list `(ben, cs)` | tìm ô ứng viên hole c_s theo 4 trường hợp mục 4.4.1 |
| `buildGate(oUngVien)` | `oUngVien`: c_s | `(iδ, Γt)` hoặc `None` | dò tia từ c_s theo hướng F, tìm ô chặn đầu tiên (21)(22), cổng Γt (23). Hết tầm nhìn ko thấy ô chặn -> `None` (loại ứng viên) |
| `buildLocalMap()` | ko | set ô | bản đồ cục bộ tạm M^H_t (24) |
| `getCellState(o)` | `o`: ô | `'free-covered'/'blocked'/'free-uncovered'/'unknown'` | trạng thái S^H_t (25) |
| `verifyHoleBFS(oUngVien, danhSachOCuaCong, banDoNho)` | c_s, Γt, M^H_t | `(vungHole, laHoleThat)` | Thuật toán 2. Hàm con `laCanhBiCam(o1, o2)` = CrossGate, cấm cạnh từ ô cổng về phía robot. Gặp `unknown` hoặc tràn ra ngoài M^H_t -> ko phải hole |
| `detectHole(boiCanh, huongSeQuay = None)` | như `findHoleCandidates` | `(vungHole, cs, iδ)` hoặc `None` | gộp: tìm ứng viên -> xây cổng -> BFS. Cả 2 bên đều là hole thì chọn |H| nhỏ hơn, bằng nhau thì ưu tiên L |
| `coverHole(vungHole, oUngVien, doDaiCong)` | H, c_s, iδ (lấy từ `detectHole`) | `True` nếu q_resume cạn, `False` nếu đã quay về q_resume | mục 4.4.4: xét (26), nếu thỏa thì đi thẳng thêm (27) và vào hole ở đầu xa (28); lưu q_resume; quét H bằng chọn hướng thích nghi + quay lui cục bộ; quét xong kiểm tra F, L, R của q_resume, còn việc thì Theta* quay về và khôi phục heading |

hàm con bên trong `coverHole`:

| hàm con | công dụng |
|---|---|
| `conPhaiGhe(o)` | `o` thuộc H và chưa covered |
| `demTrongHole(huongTuongDoi)` | giống `measureDirection` nhưng chỉ đếm ô trong H |
| `ghiCapCucBo()` | ghi cặp quay lui cục bộ trong H (Hình 3, ô chặn xét tương đối với H). Đồng thời ô trống nằm **ngoài** H vẫn được ghi vào L toàn cục để ko bị bỏ sót (Bổ đề 5.3) |

### Stage 4: lập đường quay lui bằng Theta* (mục 4.5.2)

| hàm | tham số | trả về | công dụng |
|---|---|---|---|
| `canGoStraight(A, B, vungChoPhep = None)` | `A`, `B`: 2 ô; `vungChoPhep`: mặc định `self.covered` | `True/False` | kiểm tra line-of-sight của Theta*: đoạn thẳng A -> B chỉ đi trong vùng cho phép, ko cắt góc vật cản |
| `thetaStar(dichDen, vungChoPhep = None)` | `dichDen`: set các điểm quay lui c_b; `vungChoPhep`: mặc định `self.covered` | list ô `[c_b0, ..., c_bm]` (38) hoặc `None` | Theta* nhiều đích, heuristic (36) = khoảng cách Euclid tới đích gần nhất, f = g + h (37). Hàm con `uocLuong(o)` là h_t |
| `followPath(duongDi)` | `duongDi`: output của `thetaStar` | ko | đi theo từng đoạn: tính β (39), quay α (40), đi thẳng l (41). Ko đổi covered (42) |
| `enterBranch(oCanGhe)` | `oCanGhe`: c^u kề cạnh ô đang đứng | ko | quay về phía c^u rồi bước vào (43)(44), kết thúc quay lui, quay lại chế độ bao phủ |
| `globalBacktrack()` | ko | `False` nếu ko quay lui được, `True` nếu xong, `'exhausted'` nếu sau khi vào nhánh gặp hole mà q_resume cạn | lọc L, Theta* tới c_b gần nhất, nhiều cặp cùng c_b thì lấy cặp xuất hiện đầu tiên, vào c^u, ghi cặp mới, kiểm tra hole TH 1, 2 |

### Stage 5: vòng lặp chính + đánh giá (Thuật toán 1, mục 4.2)

| hàm | tham số | trả về | công dụng |
|---|---|---|---|
| `runStraight(huongChon)` | `huongChon`: a* (`'F'/'L'/'R'`) | `None` khi F hết khả thi, `'exhausted'` khi hole xong mà q_resume cạn | 1 đoạn di chuyển thẳng liên tục: mỗi ô ghi cặp quay lui, kiểm hole TH 1, 2, giữ hướng F chừng nào ℓ_t(F) > 0 (17) |
| `run()` | ko | dict kết quả (xem `getResult`) | toàn bộ thuật toán: xử lý ô sau lưng ở c0, rồi lặp: A_t khác rỗng -> kiểm hole TH 3, 4 (nếu rẽ) -> chọn hướng -> `runStraight`; A_t rỗng mà L còn -> `globalBacktrack`; cả hai rỗng -> dừng (45). q_resume cạn -> quay lui toàn cục luôn |
| `getResult()` | ko | dict | tính L(P) (11) và NT(P) (12), gom các bộ đếm |

`ACM = ACM_LOS`: tên gọi tắt, dùng `ACM(...)` hay `ACM_LOS(...)` đều được

các key trong dict kết quả:

| key | ý nghĩa |
|---|---|
| `'path'` | list cấu hình `(dòng, cột, góc)` |
| `'covered'` | set ô đã bao phủ |
| `'L(P)'` | tổng chiều dài đường đi (11) |
| `'NT(P)'` | số lần đổi hướng (12) |
| `'global_backtracks'` | số lần quay lui toàn cục |
| `'holes_covered'` | số hole đã bao phủ |
| `'resume_exhausted'` | số lần q_resume bị cạn |
| `'local_backtracks'` | số lần quay lui cục bộ trong hole |
| `'cond26'` | số lần thỏa điều kiện (26) |

---

## 3. Hướng dẫn chạy

### yêu cầu

- Python 3.8 trở lên
- code ACM chỉ dùng thư viện có sẵn (`math`, `heapq`, `collections`), ko cần cài gì
- muốn vẽ đường đi thì cài thêm matplotlib: `pip install matplotlib`

### bước 1: tạo file `run_demo.py` cùng thư mục với `ACM_LOS_FINAL.py`

```python
from ACM_LOS_FINAL import ACM_LOS, getFreeCells

grid = [
    [1,1,1,1,1,1,1,1,1,1],
    [1,0,0,0,0,0,0,0,0,1],
    [1,0,1,1,0,0,1,0,0,1],
    [1,0,1,0,0,0,1,0,0,1],
    [1,0,0,0,1,0,0,0,0,1],
    [1,0,0,0,1,0,1,1,0,1],
    [1,0,1,0,0,0,0,0,0,1],
    [1,1,1,1,1,1,1,1,1,1],
]

robot = ACM_LOS(grid, start=(6, 1), rho=3.0, huongBanDau='N')
ketQua = robot.run()

print("so o trong      :", len(getFreeCells(grid)))
print("so o da bao phu :", len(ketQua['covered']))
print("L(P)            :", round(ketQua['L(P)'], 3))
print("NT(P)           :", ketQua['NT(P)'])
print("quay lui toan cuc:", ketQua['global_backtracks'])
print("so hole         :", ketQua['holes_covered'])
```

### bước 2: chạy

```
python run_demo.py
```

kết quả ra như này:

```
so o trong      : 38
so o da bao phu : 38
L(P)            : 41.414
NT(P)           : 18
quay lui toan cuc: 1
so hole         : 2
```

`so o trong` = `so o da bao phu` là bao phủ đủ (bản đồ này ô trống nào cũng tới được từ start). Nếu bản đồ có vùng bị vật cản cô lập thì robot chỉ phủ C_acc, số ô phủ sẽ nhỏ hơn số ô trống, vậy là đúng

### bước 3 (ko bắt buộc): vẽ đường đi

tạo file `ve_duong_di.py`:

```python
import matplotlib.pyplot as plt
from run_demo import grid, ketQua

dong = [p[0] for p in ketQua['path']]
cot = [p[1] for p in ketQua['path']]

plt.figure(figsize=(6, 5))
plt.imshow(grid, cmap='Greys', origin='upper')
plt.plot(cot, dong, '-', color='tab:blue', linewidth=1.5)
plt.plot(cot[0], dong[0], 'go', markersize=10)     # điểm bắt đầu
plt.plot(cot[-1], dong[-1], 'ro', markersize=10)   # điểm kết thúc
plt.title('ACM-LOS  L(P)=%.2f  NT(P)=%d' % (ketQua['L(P)'], ketQua['NT(P)']))
plt.savefig('duong_di_acm_los.png', dpi=150)
plt.show()
```

chạy `python ve_duong_di.py`. Đoạn thẳng dọc/ngang là bao phủ, đoạn chéo là quay lui Theta* đi qua vùng đã phủ

### lưu ý khi tự tạo bản đồ

- viền ngoài phải toàn số 1, ko thì robot có thể nhìn ra ngoài map
- `start` phải là ô 0, `rho >= √2` (≈ 1.415), ko thì báo `ValueError`
- đổi `huongBanDau` (`'N'/'E'/'S'/'W'`) và `rho` sẽ ra đường đi khác nhau, khi so với BA* nhớ giữ cùng start, cùng hướng, cùng ρ
