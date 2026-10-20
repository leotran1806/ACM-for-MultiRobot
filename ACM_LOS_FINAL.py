# # `grid[dòng][cột] == 1` là vật cản, `== 0`. Viền ngoài bản đồ phải là vật cản hết
# # **Hướng tuyệt đối.** `'N'` lên, `'E'` phải, `'S'` xuống, `'W'` trái, Xếp đúng thứ tự này thì quay trái = `-1`, quay phải = `+1`, quay đầu = `+2` (modulo 4), khỏi phải viết `if` lằng nhằng.
# # **Hướng tương đối.** `'F'` trước, `'L'` trái, `'R'` phải, `'B'` sau. Bản thảo công thức (8): `D = {F, L, R}` — robot **chỉ được chọn** 3 hướng này. `'B'` không nằm trong tập lựa chọn. Trừ ngoại lệ ở xuất phát
# # note: 
# # self.covered: set các ô đã đi qua
# #self.L: list các ô quay lui (c_b, c_u)
# #self.pos/ self.heading: ô đang đứng + hướng quay

# import math
# import heapq
# from collections import deque
# #initilize

# DANH_SACH_HUONG = ['N', 'E', 'S', 'W']
# # dựa trên bản đồ thực tế
# BUOC_DI = {
#     'N': (-1, 0),
#     'E': (0, 1),
#     'S': (1, 0),
#     'W': (0, -1),
# }
# # chọn hướng Đông làm 0 độ, tự quy ước
# GOC_CUA_HUONG = {}
# for _h in DANH_SACH_HUONG:
#     GOC_CUA_HUONG[_h] = math.atan2(BUOC_DI[_h][0], BUOC_DI[_h][1])

# # stage1
# # thật sự xoay robot theo kieuQuay
# def turnRobot(huong_hien_tai, kieuQuay):
#     viTri = DANH_SACH_HUONG.index(huong_hien_tai)
#     if kieuQuay == 'L':
#         return DANH_SACH_HUONG[(viTri - 1) % 4]
#     elif kieuQuay == 'R':
#         return DANH_SACH_HUONG[(viTri + 1) % 4]
#     elif kieuQuay == 'B':
#         return DANH_SACH_HUONG[(viTri + 2) % 4]
#     return huong_hien_tai   

# # giống y chang turnRobot nhưng chỉ nhìn hướng quay mà ko quay thật
# def getAbsoluteDirection(huong_hien_tai, huongTuongDoi):
#     if huongTuongDoi == 'F': 
#         return huong_hien_tai
#     return turnRobot(huong_hien_tai, huongTuongDoi)

# # o: vị trí hiện tại, buoc: hướng sắp bước theo, k = 1 mặc định bước 1 bước
# def moveCell(o, buoc, k = 1):
#     return (o[0] + buoc[0] * k, o[1] + buoc[1] * k)

# def getNeighbors4(o):
#     dong, cot  = o
#     return [(dong - 1, cot), (dong + 1, cot), (dong, cot - 1), (dong, cot + 1)]

# def normalizeAngle(goc):
#     goc = (goc + math.pi) % (2 * math.pi) - math.pi
#     if goc <= - math.pi:
#         goc = goc + 2 * math.pi
#     return goc

# def getFreeCells(grid):
#     ketQua = set()
#     for dong in range(len(grid)):
#         for cot in range(len(grid[0])):
#             if grid[dong][cot] == 0:
#                 ketQua.add((dong, cot))
#     return ketQua

# class ACM_LOS:
#     # _init_ khởi tạo robot
#     # grid: bản đồ (1 là vật cản, 0 là trống)
#     # start: ô bắt đầu (dòng, cột)
#     # rho: p (trong research), tầm nhìn của cảm biến
#     # huongBanDau: huong robot quay ban đầu
#     def __init__(self, grid, start, rho = 3.0, huongBanDau = 'N'):
#         if rho < math.sqrt(2):
#             raise ValueError("rho qua nho")
#         if not (0 <= start[0] < len(grid) and 0 <= start[1] <len(grid[0])):
#             raise ValueError ("nam ngoai ban do")
#         if grid[start[0]][start[1]] == 1:
#             raise ValueError("o bat dau la vat can")

#         # Initialize
#         self.grid = grid
#         self.soDong = len(grid)
#         self.soCot = len(grid[0])
#         self.rho = rho
#         # model đang test
#         self.mode = 'LOS'

#         # trạng thái robot
#         self.pos = start 
#         self.heading = huongBanDau

#         # tập hợp những ô đã bao phủ
#         self.covered = set()
#         self.covered.add(start)

#         # tập hợp quay lui
#         self.L = []
#         # đường đi đầy đủ 
#         self.path = [(start[0], start[1], GOC_CUA_HUONG[huongBanDau])]

#         # bộ đếm
#         self.soLanQuayLuiToanCuc = 0
#         self.soHoleDaBaoPhu = 0
#         self.soLanResumeCan = 0
#         self.soLanQuayLuiCucBo = 0
#         self.soLanDungDieuKien26 = 0

#         # chống loop vô hạn
#         self.demVongLap = 0

#     def isInsideMap(self, o):
#         return 0 <= o[0] < self.soDong and 0 <= o[1] < self.soCot

#     # thuật toán Bresenham lấy tất cả các ô từ vị trí A -> B, mục đích để check có ô nào blocked hay covered không
#     def getCellsOnLine(self, A, B):
#         dong0, cot0 = A
#         dong1, cot1 = B
#         dDong = dong1 - dong0
#         dCot = cot1 - cot0
#     # 0.0 đại diện cho A, 1.0 đại diện cho B
#         danhsachT = {0.0, 1.0}
#         if dDong != 0:
#             for k in range(min(dong0, dong1), max(dong0, dong1) + 1):
#                 for j in (k - 0.5, k + 0.5):
#                     temp = (j - dong0)/dDong
#                     if 0.0 < temp < 1.0:
#                         danhsachT.add(temp)

#         if dCot != 0:
#             for k in range(min(cot0, cot1), max(cot0, cot1) + 1):
#                 for j in (k - 0.5, k + 0.5):
#                     temp = (j - cot0)/dCot
#                     if 0.0 < temp < 1.0:
#                         danhsachT.add(temp)

#         danhsachT = sorted(danhsachT)
#         # lấy trung điểm giữa 2 temp liên tục -> tìm được ô đoạn thẳng xuyên qua
#         ketQua = []
#         for i in range (len(danhsachT) - 1):
#             t = (danhsachT[i] + danhsachT[i+1])/2.0
#             o = (round(dong0 + dDong * t), round(cot0 + dCot * t))
#             if o != A and o != B and o not in ketQua:
#                 ketQua.append(o)
#         return ketQua

#     # kiểm tra tầm nhìn từ A->B có bị blocked hay covered không
#     # quy chung o là ô cần check
#     def isSightBlocked(self, A, B):
#         for o in self.getCellsOnLine(A, B):
#             if not self.isInsideMap(o) or self.grid[o[0]][o[1]]:
#                 return True
#         return False

#     # dành cho LOS vì có trạng thái unknown
#     def senseCell(self, o, diemDung = None):
#         if diemDung is None:
#             diemDung = self.pos
#         # không nằm trong map tức biên
#         if not self.isInsideMap(o):
#             return 'blocked'
#         # xa hơn rho (p)
#         euclidean_distance = math.hypot(o[0] - diemDung[0], o[1] - diemDung[1])
#         if euclidean_distance > self.rho:
#             return 'unknown'
#         # check LOS 
#         if self.isSightBlocked(o, diemDung):
#             return 'unknown'
#         if self.grid[o[0]][o[1]] == 1:
#             return 'blocked'
#         # nếu không phải tất cả trường hợp thì free
#         return 'free'

#     # diemDung là điểm nhìn, o là điểm bị nhìn, tức là check xem 
#     def isFreeUncovered(self, o, diemDung = None):
#         if self.senseCell(o, diemDung) != 'free':
#             return False
#         if o in self.covered:
#             return False
#         return True

#     # hàm này không phải phủ định isFreeCovered, vì có trường hợp unknown ở LOS
#     # phản ví dụ: nếu isFreeCovered nhận unknown trả False -> isBlocked trả True -> sai LOS
#     def isBlocked(self, o, diemDung = None):
#         if self.senseCell(o, diemDung) == 'blocked':
#             return True
#         if o in self.covered:
#             return True
#         return False

#     # lấy ô nằm cách robot i bước theo 1 hướng tương đối 
#     # công thức (5) (6) theo research
#     def getCellAt(self, huongTuongDoi, i=1, viTri=None, huong=None):
#         if viTri is None:
#             viTri = self.pos
#         if huong is None:
#             huong = self.heading
#         huongThat = getAbsoluteDirection(huong, huongTuongDoi)
#         return moveCell(viTri, BUOC_DI[huongThat], i)

#     # đếm xem theo một hướng có bao nhiêu ô trống liên tiếp
#     # mục đích dùng để chọn hướng có số ô trống trong range (1, rho) tối đa để đi theo
#     def measureDirection(self, huongTuongDoi, viTri = None, huong = None):
#         count = 0   
#         tamNhin = int(math.floor(self.rho))
#         for i in range(1, tamNhin + 1):
#             o = self.getCellAt(huongTuongDoi, i, viTri, huong)
#             if self.isFreeUncovered(o, viTri):
#                 count = i
#             else:
#                 break
#         return count

#     # xem robot đi được những hướng nào, tương ứng doDai[huongTuongDoi] > 0
#     # Ứng với công thức (14): tập A_t
#     def getFeasibleDirections(self, viTri = None, huong = None):
#         doDai = {}
#         for huongTuongDoi in ('F', 'L', 'R'):
#             doDai[huongTuongDoi] = self.measureDirection(huongTuongDoi, viTri, huong)

#         danhSachHuong = []
#         for huongTuongDoi in ('F', 'L', 'R'):
#             if doDai[huongTuongDoi] > 0:
#                 danhSachHuong.append(huongTuongDoi)

#         return doDai, danhSachHuong

#     # chọn hướng tốt nhất trong 3 hướng {F, L, R} theo độ dài xa nhất có thể đi
#     # nếu độ dài tối đa có thể đi bằng nhau hết thì ưu tiên F > L > R -> chọn F: huongTotNhat
#     @staticmethod
#     def chooseDirection(doDai, danhSachHuong):
#         thuTuUuTien = {'F': 2, 'L': 1, 'R': 0}
#         huongTotNhat = danhSachHuong[0]
#         for huongTuongDoi in danhSachHuong:
#             daiHon = doDai[huongTuongDoi] > doDai[huongTotNhat]
#             bangNhauNhungUuTienHon = (
#                 doDai[huongTuongDoi] == doDai[huongTotNhat]
#                 and thuTuUuTien[huongTuongDoi] > thuTuUuTien[huongTotNhat]
#             )
#             if daiHon or bangNhauNhungUuTienHon:
#                 huongTotNhat = huongTuongDoi
#         return huongTotNhat

#     # quay robot một góc cụ thể, nếu thật sự thay đổi -> ghi lại
#     def turnToAngle(self, goc):
#         gocHienTai = self.path[-1][2]
#         if abs(normalizeAngle(goc - gocHienTai)) > 1e-9:
#             self.path.append((self.pos[0], self.pos[1], goc))
#     # quay robot sang một hướng tuyệt đối ('N', 'E', 'S', 'W')
#     def turnTo(self, huong):
#         self.heading = huong
#         self.turnToAngle(GOC_CUA_HUONG[huong])

#     # robot bước sang 1 ô
#     # cập nhật vị trí và đánh dấu covered (18)
#     # ghi đường đi
#     # xóa các cặp quay lui trỏ đến ô đã ghé (33) (34)
#     def moveToCell(self, o):
#         self.pos = o
#         self.covered.add(o)
#         self.path.append((o[0], o[1], self.path[-1][2]))

#         capConLai = []
#         for (c_b, c_u) in self.L:
#             if c_u != o:
#                 capConLai.append((c_b, c_u))
#         self.L = capConLai

#     # stage 3
#     # dùng để hỗ trợ hàm findBacktrackPairs() để add vào L (29)
#     def checkBacktrackCondition(self, oTrong, oChan):
#         if self.isFreeUncovered(oTrong) and self.isBlocked(oChan):
#             return 1
#         return 0

#     # Tìm các cặp quay lui tại vị trí robot đang đứng
#     def findBacktrackPairs(self):
#         # lấy 8 ô xung quanh ô robot đang đứng
#         oTrai = self.getCellAt('L')
#         oPhai = self.getCellAt('R')
#         oTruoc = self.getCellAt('F')
#         buocTien = BUOC_DI[self.heading]
#         oTraiTruoc = moveCell(oTrai, buocTien)
#         oTraiSau = moveCell(oTrai, buocTien, - 1)
#         oPhaiTruoc = moveCell(oPhai, buocTien)
#         oPhaiSau = moveCell(oPhai, buocTien, - 1)

#         capMoi = []
#         traiHopLe = (self.checkBacktrackCondition(oTrai, oTraiTruoc) == 1
#                      or self.checkBacktrackCondition(oTrai, oTraiSau) == 1
#                      or self.checkBacktrackCondition(oTrai, oTruoc) == 1)
#         if traiHopLe:
#             capMoi.append((self.pos, oTrai))
#         phaiHopLe = (self.checkBacktrackCondition(oPhai, oPhaiTruoc) == 1
#                      or self.checkBacktrackCondition(oPhai, oPhaiSau) == 1
#                      or self.checkBacktrackCondition(oPhai, oTruoc) == 1)
#         if phaiHopLe:
#             capMoi.append((self.pos, oPhai))

#         for i in capMoi:
#             if i not in self.L:
#                 self.L.append(i)

#     # tìm hole candidate, 2 case
#     # 1 là đi thẳng check 2 bên -> ô kè trống, ô phía sau ô kề blocked
#     # 2 là trước khi quay phải/trái check bên còn lại theo logic 1
#     def findHoleCandidates(self, boiCanh, huongSeQuay = None):
#         oTrai = self.getCellAt('L')
#         oPhai = self.getCellAt('R')
#         oTruoc = self.getCellAt('F')
#         buocTien = BUOC_DI[self.heading]
#         oTraiSau = moveCell(oTrai, buocTien, -1)
#         oPhaiSau = moveCell(oPhai, buocTien, -1)

#         ketQua = []
#         if boiCanh == 'forward':
#             # bên trái trống, đằng sau bị chặn -> candidate hole
#             if self.isFreeUncovered(oTrai) and self.isBlocked(oTraiSau):
#                 ketQua.append(('L', oTrai))
#             # bên phải trống, đằng sau bị chặn -> candidate hole
#             if self.isFreeUncovered(oPhai) and self.isBlocked(oPhaiSau):
#                 ketQua.append(('R', oPhai))
#         # check candidate hole trước khi quay phải/trái
#         else:
#             if huongSeQuay == 'R':
#                 if self.isFreeUncovered(oTrai) and self.isBlocked(oTruoc):
#                     ketQua.append(('L', oTrai))
#             elif huongSeQuay == 'L':
#                 if self.isFreeUncovered(oPhai) and self.isBlocked(oTruoc):
#                     ketQua.append(('R', oPhai))
#         return ketQua
#     # xây cổng cho hole
#     # cổng là các ô liên tục tính từ ô ứng viên theo hướng robot đang đang đi, dài cho tới khi gặp ô blocked
#     # nếu hết tầm nhìn ko thấy ô blocked -> loại ứng viên
#     def buildGate(self, oUngVien):
#         buocTien = BUOC_DI[self.heading]
#         i = 1
#         while i * i + 1 <= self.rho * self.rho:
#             if self.isBlocked(moveCell(oUngVien, buocTien, i)):
#                 danhSachOCuaCong = []
#                 for j in range (i):
#                     danhSachOCuaCong.append(moveCell(oUngVien, buocTien, j))
#                 return i, danhSachOCuaCong
#             i = i + 1
#         return None

#     # bản đồ nhỏ xung quanh robot, gồm các ô trong tầm nhìn rho(p) 
#     # dùng để kiểm tra hole trong bản đồ này, ngoài bản đồ -> unknown
#     #  (24)
#     def buildLocalMap(self):
#         dongRobot, cotRobot = self.pos
#         banKinh = int(math.floor(self.rho))
#         banDoNho = set()
#         for dong in range(dongRobot - banKinh, dongRobot + banKinh + 1):
#             for cot in range(cotRobot - banKinh, cotRobot + banKinh + 1):
#                 distance = (dong - dongRobot)**2 + (cot - cotRobot)**2
#                 if distance <= self.rho**2 and self.isInsideMap((dong, cot)):
#                     banDoNho.add((dong, cot))
#         return banDoNho
#     # tìm trạng thái của 1 ô trong bản đồ nhỏ
#     def getCellState(self, o):
#         if o in self.covered:
#             return 'free-covered'
#         trangThai = self.senseCell(o)
#         if trangThai == 'blocked':
#             return 'blocked'
#         if trangThai == 'free':
#             return 'free-uncovered'
#         return 'unknown'

#     # BFS kiểm tra xem vùng này có phải là hole thật không 
#     # lan tỏa ô ứng viên ra các ô xung quanh
#     # Nếu: lan tỏa ko gặp ô nào 'unknown', ko tràn ra ngoài bản đồ nhỏ
#     # ko cho hole lan ra bên ngoài, check huongRaNgoai
#     def verifyHoleBFS(self, oUngVien, danhSachOCuaCong, banDoNho):
#         huongRaNgoai = (self.pos[0] - oUngVien[0], self.pos[1] - oUngVien[1])
#         tapCong = set(danhSachOCuaCong)

#         def laCanhBiCam(o1, o2):
#             if o1 in tapCong and o2 == moveCell(o1, huongRaNgoai):
#                 return True
#             if o2 in tapCong and o1 == moveCell(o2, huongRaNgoai):
#                 return True
#             return False

#         vungHole = set()
#         vungHole.add(oUngVien)
#         hangDoi = deque()
#         hangDoi.append(oUngVien)

#         while len(hangDoi) > 0:
#             o = hangDoi.popleft()
#             for oKe in getNeighbors4(o):
#                 if laCanhBiCam(o, oKe):
#                     continue
#                 if oKe not in banDoNho:
#                     return vungHole, False
#                 trangThai = self.getCellState(oKe)
#                 if trangThai == 'unknown':
#                     return vungHole, False
#                 if trangThai == 'free-uncovered' and oKe not in vungHole:
#                     vungHole.add(oKe)
#                     hangDoi.append(oKe)
#         return vungHole, True

#     # gộp các hàm đã viết: tìm candidate, xây cổng, BFS xác nhận hole
#     # nếu cả 2 bên đều có hole thì ưu tiên bên hole nhỏ, nếu bằng thì ưu tiên trái
#     def detectHole(self, boiCanh, huongSeQuay = None):
#         banDoNho = self.buildLocalMap()
#         danhSachTimDuoc = []
#         for (ben, oUngVien) in self.findHoleCandidates(boiCanh, huongSeQuay):
#             ketQuaCong = self.buildGate(oUngVien)
#             if ketQuaCong is None:
#                 continue
#             doDaiCong, danhSachOCuaCong = ketQuaCong

#             vungHole, laHoleThat = self.verifyHoleBFS(oUngVien, danhSachOCuaCong, banDoNho)
#             if laHoleThat:
#                 if ben == 'L':
#                     diemUuTien = 0
#                 else:
#                     diemUuTien = 1
#                 danhSachTimDuoc.append(
#                     (len(vungHole), diemUuTien, vungHole, oUngVien, doDaiCong))
#         if len(danhSachTimDuoc) == 0:
#             return None
#         danhSachTimDuoc.sort(key = lambda x: (x[0], x[1]))
#         chonDuoc = danhSachTimDuoc[0]
#         return chonDuoc[2], chonDuoc[3], chonDuoc[4]
    
#     # chui vào hole để quét sạch rồi quay ra
#     # kiểm tra (26), nếu độ dài có thể đi thẳng trước khi chạm blocked = độ dài cổng - 1 thì đi thẳng trước rồi mới rẽ vào cổng
#     # ghi nhớ ô q_resume và hướng ban đầu để quay về
#     # vào hole thì hoạt động theo logic bao phủ thích nghi
#     # quét xong hole thì kiểm q_resume còn cần làm gì ko, hết thì True khỏi quay về, nếu còn thì quay lại q_resume
#     def coverHole(self, vungHole, oUngVien, doDaiCong):
#         self.soHoleDaBaoPhu = self.soHoleDaBaoPhu + 1
#         buocTien = BUOC_DI[self.heading]

#         soODiThangDuoc = self.measureDirection('F')
#         oSauCong = moveCell(self.pos, buocTien, doDaiCong)
#         thoaDieuKien26 = (doDaiCong > 1
#                           and soODiThangDuoc == doDaiCong - 1
#                           and self.isBlocked(oSauCong))

#         if thoaDieuKien26:
#             self.soLanDungDieuKien26 = self.soLanDungDieuKien26 + 1
#             for k in range(1, doDaiCong):
#                 self.moveToCell(moveCell(self.pos, buocTien))
#                 self.findBacktrackPairs()
#             oChuiVao = moveCell(oUngVien, buocTien, doDaiCong - 1)
#         else:
#             oChuiVao = oUngVien
#         # ghi lại vị trí quay về và hướng ban đầu
#         viTriQuayVe = self.pos
#         huongQuayVe = self.heading

#         # vào hole
#         huongChuiVao = None
#         buocCanDi = (oChuiVao[0] - self.pos[0], oChuiVao[1] - self.pos[1])
#         for ten, buoc in BUOC_DI.items():
#             if buoc == buocCanDi:
#                 huongChuiVao = ten
#         self.turnTo(huongChuiVao)
#         self.moveToCell(oChuiVao)

#         # quét bên trong hole
#         capQuayLuiCucBo = []
#         def conPhaiGhe(o):
#             return o in vungHole and o not in self.covered

#         def demTrongHole(huongTuongDoi):
#             soO = 0
#             for i in range(1, int(math.floor(self.rho)) + 1):
#                 if conPhaiGhe(self.getCellAt(huongTuongDoi, i)):
#                     soO = i
#                 else:
#                     break
#             return soO

#         def ghiCapCucBo():
#             # ô trống nằm ngoài hole vẫn ghi vào L toàn cục, ko thì bị bỏ sót (Bổ đề 5.3)
#             soCapCu = len(self.L)
#             self.findBacktrackPairs()
#             self.L[soCapCu:] = [cap for cap in self.L[soCapCu:] if cap[1] not in vungHole]
#             oTrai = self.getCellAt('L')
#             oPhai = self.getCellAt('R')
#             oTruoc = self.getCellAt('F')
#             buoc = BUOC_DI[self.heading]
#             danhSachXet = [
#                 (oTrai, [moveCell(oTrai, buoc), moveCell(oTrai, buoc, -1), oTruoc]),
#                 (oPhai, [moveCell(oPhai, buoc), moveCell(oPhai, buoc, -1), oTruoc])
#             ]
#             for (oTrong, danhSachOChan) in danhSachXet:
#                 if not conPhaiGhe(oTrong):
#                     continue
#                 coOChan = False
#                 for oChan in danhSachOChan:
#                     if not conPhaiGhe(oChan):
#                         coOChan = True
#                 if coOChan and (self.pos, oTrong) not in capQuayLuiCucBo:
#                     capQuayLuiCucBo.append((self.pos, oTrong))

#         demVong = 0
#         while demVong < 20000:
#             demVong = demVong + 1
#             ghiCapCucBo()

#             doDai = {}
#             for huongTuongDoi in ('F', 'L', 'R'):
#                 doDai[huongTuongDoi] = demTrongHole(huongTuongDoi)
#             danhSachHuong = []
#             for huongTuongDoi in ('F', 'L', 'R'):
#                 if doDai[huongTuongDoi] > 0:
#                     danhSachHuong.append(huongTuongDoi)

#             if len(danhSachHuong) > 0:
#                 # chọn hướng và đi thẳng trong hole
#                 huongChon = self.chooseDirection(doDai, danhSachHuong)
#                 if huongChon != 'F':
#                     self.turnTo(turnRobot(self.heading, huongChon))
#                 while conPhaiGhe(self.getCellAt('F')):
#                     self.moveToCell(self.getCellAt('F'))
#                     conLai = []
#                     for (c_b, c_u) in capQuayLuiCucBo:
#                         if c_u != self.pos:
#                             conLai.append((c_b, c_u))
#                     capQuayLuiCucBo[:] = conLai
#                     ghiCapCucBo()
#             else:
#                 # đi trong hole bị kẹt -> quay lại cục bộ trong hole
#                 conLai = []
#                 for (c_b, c_u) in capQuayLuiCucBo:
#                     if conPhaiGhe(c_u):
#                         conLai.append((c_b, c_u))
#                 capQuayLuiCucBo[:] = conLai

#                 if len(capQuayLuiCucBo) == 0:
#                     break
#                 dichDen = set()
#                 for (c_b, c_u) in capQuayLuiCucBo:
#                     dichDen.add(c_b)

#                 # đường quay lui cục bổ chỉ dùng trong hole
#                 vungDuocDi = set()
#                 for o in vungHole:
#                     if o in self.covered:
#                         vungDuocDi.add(o)
#                 vungDuocDi.add(self.pos)

#                 duongDi = self.thetaStar(dichDen, vungDuocDi)
#                 if duongDi is None:
#                     break
#                 self.soLanQuayLuiCucBo = self.soLanQuayLuiCucBo + 1
#                 self.followPath(duongDi)

#                 oDangDung = self.pos
#                 oCanGhe = None
#                 for (c_b, c_u) in capQuayLuiCucBo:
#                     if c_b == oDangDung and oCanGhe is None:
#                         oCanGhe = c_u
#                 self.enterBranch(oCanGhe)

#         # kiểm tra quay về con việc không
#         conViec = False
#         for huongTuongDoi in ('F', 'L', 'R'):
#             huongThat = getAbsoluteDirection(huongQuayVe, huongTuongDoi)
#             oKe = moveCell(viTriQuayVe, BUOC_DI[huongThat])
#             if self.isFreeUncovered(oKe, viTriQuayVe):
#                 conViec = True

#         if not conViec:
#             self.soLanResumeCan = self.soLanResumeCan + 1
#             return True

#         # quay về chỗ cũ, dùng hướng cũ
#         if self.pos !=  viTriQuayVe:
#             duongVe = self.thetaStar({viTriQuayVe})
#             if duongVe is None:
#                 return True
#             self.followPath(duongVe)
#         self.turnTo(huongQuayVe)
#         return False
#     # tìm đường quay lui bằng thuật Theta*
#     # thuật Bresenham
#     def canGoStraight(self, A, B, vungChoPhep = None):
#         if vungChoPhep is None:
#             vungChoPhep = self.covered

#         dong0, cot0 = A
#         dong1, cot1 = B
#         dDong = dong1 - dong0
#         dCot = cot1 - cot0

#         danhSachT = {0.0, 1.0}
#         for k in range(min(dong0, dong1), max(dong0, dong1)+1):
#             if dDong != 0:
#                 for bien in (k - 0.5, k + 0.5):
#                     t = (bien - dong0)/dDong
#                     if 0 < t < 1:
#                         danhSachT.add(t)
#         for k in range(min(cot0, cot1), max(cot0, cot1)+1):
#             if dCot != 0:
#                 for bien in (k - 0.5, k + 0.5):
#                     t = (bien - cot0)/dCot
#                     if 0 < t < 1:
#                         danhSachT.add(t)
#         danhSachT = sorted(danhSachT)
#         # kiểm tra điểm cắt và trung điểm
#         diemKiemTra = list(danhSachT)
#         for i in range(len(danhSachT) - 1):
#             diemKiemTra.append((danhSachT[i] + danhSachT[i + 1]) / 2)

#         saiSo = 1e-7
#         for t in diemKiemTra:
#             dongThuc = dong0 + dDong * t
#             cotThuc = cot0 + dCot * t
#             dongDau = math.ceil(dongThuc - 0.5 - saiSo)
#             dongCuoi = math.floor(dongThuc + 0.5 + saiSo)
#             cotDau = math.ceil(cotThuc - 0.5 - saiSo)
#             cotCuoi = math.floor(cotThuc + 0.5 + saiSo)
#             for dong in range(dongDau, dongCuoi + 1):
#                 for cot in range(cotDau, cotCuoi + 1):
#                     if (dong, cot) not in vungChoPhep:
#                         return False
#         return True

#     # thetaStar: tìm đường ngắn nhất quay về, đi qua covered
#     def thetaStar(self, dichDen, vungChoPhep = None):
#         if vungChoPhep is None:
#             vungChoPhep = self.covered

#         if self.pos in dichDen:
#             return [self.pos]

#         danhSachDich = list(dichDen)
#         # ước lượng khoảng cách đến đích gần nhất (36)
#         def uocLuong(o):
#             nhoNhat = None
#             for dich in danhSachDich:
#                 d = math.hypot(o[0] - dich[0], o[1] - dich[1])
#                 if nhoNhat is None or d < nhoNhat:
#                     nhoNhat = d
#             return nhoNhat

#         batDau = self.pos
#         chiPhi = {batDau: 0.0}          # g: cost đã đi
#         oCha = {batDau: batDau}
#         hangDoiUuTien = [(uocLuong(batDau), batDau)]
#         daXet = set()

#         while len(hangDoiUuTien) > 0:
#             _, o = heapq.heappop(hangDoiUuTien)
#             if o in daXet:
#                 continue

#             if o in dichDen:
#                 # Truy ngược lấy đường đi
#                 duongDi = [o]
#                 while oCha[duongDi[-1]] != duongDi[-1]:
#                     duongDi.append(oCha[duongDi[-1]])
#                 duongDi.reverse()
#                 return duongDi

#             daXet.add(o)

#             # Xét 8 ô xung quanh
#             for dDong in (-1, 0, 1):
#                 for dCot in (-1, 0, 1):
#                     if dDong == 0 and dCot == 0:
#                         continue
#                     oKe = (o[0] + dDong, o[1] + dCot)
#                     if oKe in daXet or oKe not in vungChoPhep:
#                         continue

#                     # Đi chéo ko được cắt góc
#                     if dDong != 0 and dCot != 0:
#                         if (o[0] + dDong, o[1]) not in vungChoPhep:
#                             continue
#                         if (o[0], o[1] + dCot) not in vungChoPhep:
#                             continue

#                     cha = oCha[o]
#                     if self.canGoStraight(cha, oKe, vungChoPhep):
#                         # Nối ô cha -> đường ngắn hơn 
#                         # phân biệt Theta* với A*
#                         chiPhiMoi = chiPhi[cha] + math.hypot(oKe[0] - cha[0], oKe[1] - cha[1])
#                         chaMoi = cha
#                     else:
#                         chiPhiMoi = chiPhi[o] + math.hypot(dDong, dCot)
#                         chaMoi = o

#                     chiPhiCu = chiPhi.get(oKe, float('inf'))
#                     if chiPhiMoi < chiPhiCu - 1e-12:
#                         chiPhi[oKe] = chiPhiMoi
#                         oCha[oKe] = chaMoi
#                         heapq.heappush(hangDoiUuTien, (chiPhiMoi + uocLuong(oKe), oKe))

#         return None
#     # robot đi theo hướng quay lui đã tìm được
#     # mỗi đoạn robot quay đúng hướng -> đi thẳng
#     # tăng số ô bao phủ (42)
#     # (39)(40)(41)
#     def followPath(self, duongDi):
#         for k in range(len(duongDi) - 1):
#             oHienTai = duongDi[k]
#             oKeTiep = duongDi[k + 1]
#             goc = math.atan2(oKeTiep[0] - oHienTai[0], oKeTiep[1] - oHienTai[1])
#             self.turnToAngle(goc)
#             self.pos = oKeTiep
#             self.path.append((oKeTiep[0], oKeTiep[1], goc))

#     # từ điểm quay lui vào ô trống bên cạnh
#     # kết thúc quay lui và bắt đầu bao phủ
#     # (43)(44)
#     def enterBranch(self, oCanGhe):
#         buocCanDi = (oCanGhe[0] - self.pos[0], oCanGhe[1] - self.pos[1])
#         huongMoi = None
#         for ten, buoc in BUOC_DI.items():
#             if buoc == buocCanDi:
#                 huongMoi = ten
#         self.turnTo(huongMoi)
#         self.moveToCell(oCanGhe)

#     # quay lui toàn cục
#     # ô đích đã được ghé
#     # Theta* tìm đường ngắn nhất đếm 1 trong các điểm quay lui
#     # đi đến ô tìm được, bước vào ô trống bên cạnh
#     # ghi lại cặp quay lui mới + kiểm tra hole
#     def globalBacktrack(self):
#         # bỏ các cặp hết giá trị
#         capConHieuLuc = []
#         for (c_b, c_u) in self.L:
#             if c_u not in self.covered:
#                 capConHieuLuc.append((c_b, c_u))
#         self.L = capConHieuLuc

#         if len(self.L) == 0:
#             return False

#         # tìm đường đến quay lui gần nhất
#         dichDen = set()
#         for (c_b, c_u) in self.L:
#             dichDen.add(c_b)

#         duongDi = self.thetaStar(dichDen)
#         if duongDi is None:
#             self.L = []
#             return False

#         self.soLanQuayLuiToanCuc = self.soLanQuayLuiToanCuc + 1
#         self.followPath(duongDi)

#         # vào ô trống bên cạnh
#         # nhiều cặp cùng điểm quay lui -> lấy cặp đầu tiên(A4)
#         oDangDung = self.pos
#         oCanGhe = None
#         for (c_b, c_u) in self.L:
#             if c_b == oDangDung and oCanGhe is None:
#                 oCanGhe = c_u
#         self.enterBranch(oCanGhe)

#         # ghi lại cặp mới, kiểm tra hole
#         self.findBacktrackPairs()
#         hole = self.detectHole('forward')
#         if hole is not None:
#             daCan = self.coverHole(hole[0], hole[1], hole[2])
#             if daCan:
#                 return 'exhausted'

#         return True

#     # chạy thẳng theo 1 hướng
#     # mỗi bước ghi nhớ cặp quay lui
#     # kiểm tra hole bên cạnh 
#     # dừng di gặp blocked
#     def runStraight(self, huongChon):
#         if huongChon != 'F':
#             self.turnTo(turnRobot(self.heading, huongChon))

#         while True:
#             self.demVongLap = self.demVongLap + 1
#             if self.demVongLap > 200000:
#                 return None                          # chong lap vo han

#             self.findBacktrackPairs()

#             # detect hole trường hợp 1/2
#             # nếu đi đến ô blocked, chuyển qua detect hole trường hợp 3/4
#             if self.measureDirection('F') == 0:
#                 return None

#             hole = self.detectHole('forward')
#             if hole is not None:
#                 daCan = self.coverHole(hole[0], hole[1], hole[2])
#                 if daCan:
#                     return 'exhausted'
#                 continue                             # da quay ve, di tiep

#             self.moveToCell(self.getCellAt('F'))

#     # hàm chính chạy toàn bộ thuật toán
#     # loop:
#     # xem hướng nào còn đi được
#     # Còn thì chọn hướng rồi chạy thẳng
#     # Hết thì quay lui nhánh chưa đi
#     # nếu hết đường đi hết luôn quay lui -> xong
#     def run(self):
#         # Xử lý riêng case ô bắt đầu
#         # Xét ô sau lưng tránh trường hợp bỏ quên mãi mãi
#         # Thuật toán 1, dòng 8 - 15
#         doDai, danhSachHuong = self.getFeasibleDirections()
#         oPhiaSau = self.getCellAt('B')

#         if self.isFreeUncovered(oPhiaSau):
#             if len(danhSachHuong) > 0:
#                 # Còn hướng khác -> nhớ ô sau lưng
#                 self.L.append((self.pos, oPhiaSau))
#             else:
#                 # Không còn hướng thì quay đầu lại
#                 self.turnTo(turnRobot(self.heading, 'B'))
#                 doDai, danhSachHuong = self.getFeasibleDirections()


#         self.findBacktrackPairs()

#         # loop chính
#         while self.demVongLap < 200000:
#             self.demVongLap = self.demVongLap + 1

#             doDai, danhSachHuong = self.getFeasibleDirections()

#             if len(danhSachHuong) > 0:
#                 huongChon = self.chooseDirection(doDai, danhSachHuong)

#                 # Trước khi quay trái phải, kiểm tra hole case 3,4
#                 if 'F' not in danhSachHuong and huongChon in ('L', 'R'):
#                     hole = self.detectHole('turn', huongChon)
#                     if hole is not None:
#                         daCan = self.coverHole(hole[0], hole[1], hole[2])
#                         if daCan:
#                             while len(self.L) > 0 and self.globalBacktrack() == 'exhausted':
#                                 pass
#                         continue

#                 ketQua = self.runStraight(huongChon)

#                 # Muc 4.4.4 va 4.5: nếu quay về hết việc -> đi thẳng
#                 # chuyển qua quay lui toàn cục
#                 if ketQua == 'exhausted':
#                     while len(self.L) > 0 and self.globalBacktrack() == 'exhausted':
#                         pass

#             elif len(self.L) > 0:
#                 ketQua = self.globalBacktrack()
#                 if not ketQua:
#                     break
#                 while ketQua == 'exhausted' and len(self.L) > 0:
#                     ketQua = self.globalBacktrack()
#             else:
#                 break                                 # xong, khong con gi de lam

#         return self.getResult()

#     # đánh giá
#     def getResult(self):
#         # Tổng quãng đường = tổng độ dài từng đoạn
#         tongQuangDuong = 0.0
#         for i in range(len(self.path) - 1):
#             dong1, cot1, _ = self.path[i]
#             dong2, cot2, _ = self.path[i + 1]
#             tongQuangDuong = tongQuangDuong + math.hypot(dong2 - dong1, cot2 - cot1)

#         # Số lần quay đầu = đếm số lần góc thay đổi
#         soLanQuayDau = 0
#         for i in range(len(self.path) - 1):
#             goc1 = self.path[i][2]
#             goc2 = self.path[i + 1][2]
#             if abs(normalizeAngle(goc2 - goc1)) > 1e-9:
#                 soLanQuayDau = soLanQuayDau + 1

#         return {
#             'path': self.path,
#             'covered': self.covered,
#             'L(P)': tongQuangDuong,
#             'NT(P)': soLanQuayDau,
#             'global_backtracks': self.soLanQuayLuiToanCuc,
#             'holes_covered': self.soHoleDaBaoPhu,
#             'resume_exhausted': self.soLanResumeCan,
#             'local_backtracks': self.soLanQuayLuiCucBo,
#             'cond26': self.soLanDungDieuKien26,
#         }

# ACM = ACM_LOS
# `grid[dòng][cột] == 1` là vật cản, `== 0`. Viền ngoài bản đồ phải là vật cản hết
# **Hướng tuyệt đối.** `'N'` lên, `'E'` phải, `'S'` xuống, `'W'` trái, Xếp đúng thứ tự này thì quay trái = `-1`, quay phải = `+1`, quay đầu = `+2` (modulo 4), khỏi phải viết `if` lằng nhằng.
# **Hướng tương đối.** `'F'` trước, `'L'` trái, `'R'` phải, `'B'` sau. Bản thảo công thức (8): `D = {F, L, R}` — robot **chỉ được chọn** 3 hướng này. `'B'` không nằm trong tập lựa chọn. Trừ ngoại lệ ở xuất phát
# note: 
# self.covered: set các ô đã đi qua
#self.L: list các ô quay lui (c_b, c_u)
#self.pos/ self.heading: ô đang đứng + hướng quay

import math
import heapq
from collections import deque
#initilize

DANH_SACH_HUONG = ['N', 'E', 'S', 'W']
# dựa trên bản đồ thực tế
BUOC_DI = {
    'N': (-1, 0),
    'E': (0, 1),
    'S': (1, 0),
    'W': (0, -1),
}
# chọn hướng Đông làm 0 độ, tự quy ước
GOC_CUA_HUONG = {}
for _h in DANH_SACH_HUONG:
    GOC_CUA_HUONG[_h] = math.atan2(BUOC_DI[_h][0], BUOC_DI[_h][1])

# stage1
# thật sự xoay robot theo kieuQuay
def turnRobot(huong_hien_tai, kieuQuay):
    viTri = DANH_SACH_HUONG.index(huong_hien_tai)
    if kieuQuay == 'L':
        return DANH_SACH_HUONG[(viTri - 1) % 4]
    elif kieuQuay == 'R':
        return DANH_SACH_HUONG[(viTri + 1) % 4]
    elif kieuQuay == 'B':
        return DANH_SACH_HUONG[(viTri + 2) % 4]
    return huong_hien_tai   

# giống y chang turnRobot nhưng chỉ nhìn hướng quay mà ko quay thật
def getAbsoluteDirection(huong_hien_tai, huongTuongDoi):
    if huongTuongDoi == 'F': 
        return huong_hien_tai
    return turnRobot(huong_hien_tai, huongTuongDoi)

# o: vị trí hiện tại, buoc: hướng sắp bước theo, k = 1 mặc định bước 1 bước
def moveCell(o, buoc, k = 1):
    return (o[0] + buoc[0] * k, o[1] + buoc[1] * k)

def getNeighbors4(o):
    dong, cot  = o
    return [(dong - 1, cot), (dong + 1, cot), (dong, cot - 1), (dong, cot + 1)]

def normalizeAngle(goc):
    goc = (goc + math.pi) % (2 * math.pi) - math.pi
    if goc <= - math.pi:
        goc = goc + 2 * math.pi
    return goc

def getFreeCells(grid):
    ketQua = set()
    for dong in range(len(grid)):
        for cot in range(len(grid[0])):
            if grid[dong][cot] == 0:
                ketQua.add((dong, cot))
    return ketQua

class ACM_LOS:
    # _init_ khởi tạo robot
    # grid: bản đồ (1 là vật cản, 0 là trống)
    # start: ô bắt đầu (dòng, cột)
    # rho: p (trong research), tầm nhìn của cảm biến
    # huongBanDau: huong robot quay ban đầu
    # mode: kieu cam bien - 'LOS' (line of sight, bi chan boi vat can) hoac
    #       'OFS' (occlusion-free sensing, khong bi chan boi vat can, chi gioi han boi rho va bien ban do)
    def __init__(self, grid, start, rho = 3.0, huongBanDau = 'N', mode = 'LOS'):
        if rho < math.sqrt(2):
            raise ValueError("rho qua nho")
        if not (0 <= start[0] < len(grid) and 0 <= start[1] <len(grid[0])):
            raise ValueError ("nam ngoai ban do")
        if grid[start[0]][start[1]] == 1:
            raise ValueError("o bat dau la vat can")
        if mode not in ('LOS', 'OFS'):
            raise ValueError("mode phai la 'LOS' hoac 'OFS'")

        # Initialize
        self.grid = grid
        self.soDong = len(grid)
        self.soCot = len(grid[0])
        self.rho = rho
        # model dang test: 'LOS' hoac 'OFS'
        self.mode = mode

        # trạng thái robot
        self.pos = start 
        self.heading = huongBanDau

        # tập hợp những ô đã bao phủ
        self.covered = set()
        self.covered.add(start)

        # tập hợp quay lui
        self.L = []
        # đường đi đầy đủ 
        self.path = [(start[0], start[1], GOC_CUA_HUONG[huongBanDau])]

        # bộ đếm
        self.soLanQuayLuiToanCuc = 0
        self.soHoleDaBaoPhu = 0
        self.soLanResumeCan = 0
        self.soLanQuayLuiCucBo = 0
        self.soLanDungDieuKien26 = 0

        # chống loop vô hạn
        self.demVongLap = 0

    def isInsideMap(self, o):
        return 0 <= o[0] < self.soDong and 0 <= o[1] < self.soCot

    # thuật toán Bresenham lấy tất cả các ô từ vị trí A -> B, mục đích để check có ô nào blocked hay covered không
    def getCellsOnLine(self, A, B):
        dong0, cot0 = A
        dong1, cot1 = B
        dDong = dong1 - dong0
        dCot = cot1 - cot0
    # 0.0 đại diện cho A, 1.0 đại diện cho B
        danhsachT = {0.0, 1.0}
        if dDong != 0:
            for k in range(min(dong0, dong1), max(dong0, dong1) + 1):
                for j in (k - 0.5, k + 0.5):
                    temp = (j - dong0)/dDong
                    if 0.0 < temp < 1.0:
                        danhsachT.add(temp)

        if dCot != 0:
            for k in range(min(cot0, cot1), max(cot0, cot1) + 1):
                for j in (k - 0.5, k + 0.5):
                    temp = (j - cot0)/dCot
                    if 0.0 < temp < 1.0:
                        danhsachT.add(temp)

        danhsachT = sorted(danhsachT)
        # lấy trung điểm giữa 2 temp liên tục -> tìm được ô đoạn thẳng xuyên qua
        ketQua = []
        for i in range (len(danhsachT) - 1):
            t = (danhsachT[i] + danhsachT[i+1])/2.0
            o = (round(dong0 + dDong * t), round(cot0 + dCot * t))
            if o != A and o != B and o not in ketQua:
                ketQua.append(o)
        return ketQua

    # kiểm tra tầm nhìn từ A->B có bị blocked hay covered không
    # quy chung o là ô cần check
    def isSightBlocked(self, A, B):
        for o in self.getCellsOnLine(A, B):
            if not self.isInsideMap(o) or self.grid[o[0]][o[1]]:
                return True
        return False

    # trạng thái unknown chỉ có thể xảy ra ở mode LOS (bị che khuất)
    # ở mode OFS cảm biến xuyên vật cản trong tầm rho -> ko có unknown do bị che (25)
    def senseCell(self, o, diemDung = None):
        if diemDung is None:
            diemDung = self.pos
        # không nằm trong map tức biên
        if not self.isInsideMap(o):
            return 'blocked'
        # xa hơn rho (p)
        euclidean_distance = math.hypot(o[0] - diemDung[0], o[1] - diemDung[1])
        if euclidean_distance > math.floor(self.rho):
            return 'unknown'
        # check LOS - chỉ áp dụng cho mode LOS, OFS bỏ qua che khuất
        if self.mode == 'LOS' and self.isSightBlocked(o, diemDung):
            return 'unknown'
        if self.grid[o[0]][o[1]] == 1:
            return 'blocked'
        # nếu không phải tất cả trường hợp thì free
        return 'free'

    # diemDung là điểm nhìn, o là điểm bị nhìn, tức là check xem 
    def isFreeUncovered(self, o, diemDung = None):
        if self.senseCell(o, diemDung) != 'free':
            return False
        if o in self.covered:
            return False
        return True

    # hàm này không phải phủ định isFreeCovered, vì có trường hợp unknown ở LOS
    # phản ví dụ: nếu isFreeCovered nhận unknown trả False -> isBlocked trả True -> sai LOS
    def isBlocked(self, o, diemDung = None):
        if self.senseCell(o, diemDung) == 'blocked':
            return True
        if o in self.covered:
            return True
        return False

    # lấy ô nằm cách robot i bước theo 1 hướng tương đối 
    # công thức (5) (6) theo research
    def getCellAt(self, huongTuongDoi, i=1, viTri=None, huong=None):
        if viTri is None:
            viTri = self.pos
        if huong is None:
            huong = self.heading
        huongThat = getAbsoluteDirection(huong, huongTuongDoi)
        return moveCell(viTri, BUOC_DI[huongThat], i)

    # đếm xem theo một hướng có bao nhiêu ô trống liên tiếp
    # mục đích dùng để chọn hướng có số ô trống trong range (1, rho) tối đa để đi theo
    def measureDirection(self, huongTuongDoi, viTri = None, huong = None):
        count = 0   
        tamNhin = int(math.floor(self.rho))
        for i in range(1, tamNhin + 1):
            o = self.getCellAt(huongTuongDoi, i, viTri, huong)
            if self.isFreeUncovered(o, viTri):
                count = i
            else:
                break
        return count

    # xem robot đi được những hướng nào, tương ứng doDai[huongTuongDoi] > 0
    # Ứng với công thức (14): tập A_t
    def getFeasibleDirections(self, viTri = None, huong = None):
        doDai = {}
        for huongTuongDoi in ('F', 'L', 'R'):
            doDai[huongTuongDoi] = self.measureDirection(huongTuongDoi, viTri, huong)

        danhSachHuong = []
        for huongTuongDoi in ('F', 'L', 'R'):
            if doDai[huongTuongDoi] > 0:
                danhSachHuong.append(huongTuongDoi)

        return doDai, danhSachHuong

    # chọn hướng tốt nhất trong 3 hướng {F, L, R} theo độ dài xa nhất có thể đi
    # nếu độ dài tối đa có thể đi bằng nhau hết thì ưu tiên F > L > R -> chọn F: huongTotNhat
    @staticmethod
    def chooseDirection(doDai, danhSachHuong):
        huongTotNhat = danhSachHuong[0]
        for huongTuongDoi in danhSachHuong:
            if doDai[huongTuongDoi] > doDai[huongTotNhat]:
                huongTotNhat = huongTuongDoi
        return huongTotNhat

    # quay robot một góc cụ thể, nếu thật sự thay đổi -> ghi lại
    def turnToAngle(self, goc):
        gocHienTai = self.path[-1][2]
        if abs(normalizeAngle(goc - gocHienTai)) > 1e-9:
            self.path.append((self.pos[0], self.pos[1], goc))
    # quay robot sang một hướng tuyệt đối ('N', 'E', 'S', 'W')
    def turnTo(self, huong):
        self.heading = huong
        self.turnToAngle(GOC_CUA_HUONG[huong])

    # robot bước sang 1 ô
    # cập nhật vị trí và đánh dấu covered (18)
    # ghi đường đi
    # xóa các cặp quay lui trỏ đến ô đã ghé (33) (34)
    def moveToCell(self, o):
        self.pos = o
        self.covered.add(o)
        self.path.append((o[0], o[1], self.path[-1][2]))

        capConLai = []
        for (c_b, c_u) in self.L:
            if c_u != o:
                capConLai.append((c_b, c_u))
        self.L = capConLai

    # stage 3
    # dùng để hỗ trợ hàm findBacktrackPairs() để add vào L (29)
    def checkBacktrackCondition(self, oTrong, oChan):
        if self.isFreeUncovered(oTrong) and self.isBlocked(oChan):
            return 1
        return 0

    # Tìm các cặp quay lui tại vị trí robot đang đứng
    def findBacktrackPairs(self):
        # lấy 8 ô xung quanh ô robot đang đứng
        oTrai = self.getCellAt('L')
        oPhai = self.getCellAt('R')
        oTruoc = self.getCellAt('F')
        buocTien = BUOC_DI[self.heading]
        oTraiTruoc = moveCell(oTrai, buocTien)
        oTraiSau = moveCell(oTrai, buocTien, - 1)
        oPhaiTruoc = moveCell(oPhai, buocTien)
        oPhaiSau = moveCell(oPhai, buocTien, - 1)

        capMoi = []
        traiHopLe = (self.checkBacktrackCondition(oTrai, oTraiTruoc) == 1
                     or self.checkBacktrackCondition(oTrai, oTraiSau) == 1
                     or self.checkBacktrackCondition(oTrai, oTruoc) == 1)
        if traiHopLe:
            capMoi.append((self.pos, oTrai))
        phaiHopLe = (self.checkBacktrackCondition(oPhai, oPhaiTruoc) == 1
                     or self.checkBacktrackCondition(oPhai, oPhaiSau) == 1
                     or self.checkBacktrackCondition(oPhai, oTruoc) == 1)
        if phaiHopLe:
            capMoi.append((self.pos, oPhai))

        for i in capMoi:
            if i not in self.L:
                self.L.append(i)

    # tìm hole candidate, 2 case
    # 1 là đi thẳng check 2 bên -> ô kè trống, ô phía sau ô kề blocked
    # 2 là trước khi quay phải/trái check bên còn lại theo logic 1
    def findHoleCandidates(self, boiCanh, huongSeQuay = None):
        oTrai = self.getCellAt('L')
        oPhai = self.getCellAt('R')
        oTruoc = self.getCellAt('F')
        buocTien = BUOC_DI[self.heading]
        oTraiSau = moveCell(oTrai, buocTien, -1)
        oPhaiSau = moveCell(oPhai, buocTien, -1)

        ketQua = []
        if boiCanh == 'forward':
            # bên trái trống, đằng sau bị chặn -> candidate hole
            if self.isFreeUncovered(oTrai) and self.isBlocked(oTraiSau):
                print(f"Mode: {self.mode}, Candidate({oTrai})")
                ketQua.append(('L', oTrai))
            # bên phải trống, đằng sau bị chặn -> candidate hole
            if self.isFreeUncovered(oPhai) and self.isBlocked(oPhaiSau):
                print(f"Mode: {self.mode}, Candidate({oPhai})")
                ketQua.append(('R', oPhai))
        # check candidate hole trước khi quay phải/trái
        else:
            if huongSeQuay == 'R':
                if self.isFreeUncovered(oTrai) and self.isBlocked(oTruoc):
                    print(f"Mode: {self.mode}, Candidate({oTrai})")
                    ketQua.append(('L', oTrai))
            elif huongSeQuay == 'L':
                if self.isFreeUncovered(oPhai) and self.isBlocked(oTruoc):
                    print(f"Mode: {self.mode}, Candidate({oPhai})")
                    ketQua.append(('R', oPhai))
        return ketQua
    # xây cổng cho hole
    # cổng là các ô liên tục tính từ ô ứng viên theo hướng robot đang đang đi, dài cho tới khi gặp ô blocked
    # nếu hết tầm nhìn ko thấy ô blocked -> loại ứng viên
    def buildGate(self, oUngVien):
        buocTien = BUOC_DI[self.heading]
        i = 1
        while i * i + 1 <= self.rho * self.rho:
            if self.isBlocked(moveCell(oUngVien, buocTien, i)):
                danhSachOCuaCong = []
                for j in range (i):
                    danhSachOCuaCong.append(moveCell(oUngVien, buocTien, j))
                return i, danhSachOCuaCong
            i = i + 1
        return None

    # bản đồ nhỏ xung quanh robot, gồm các ô trong tầm nhìn rho(p) 
    # dùng để kiểm tra hole trong bản đồ này, ngoài bản đồ -> unknown
    #  (24)
    def buildLocalMap(self):
        dongRobot, cotRobot = self.pos
        banKinh = int(math.floor(self.rho))
        banDoNho = set()
        for dong in range(dongRobot - banKinh, dongRobot + banKinh + 1):
            for cot in range(cotRobot - banKinh, cotRobot + banKinh + 1):
                distance = (dong - dongRobot)**2 + (cot - cotRobot)**2
                if distance <= self.rho**2 and self.isInsideMap((dong, cot)):
                    banDoNho.add((dong, cot))
        return banDoNho
    # tìm trạng thái của 1 ô trong bản đồ nhỏ
    def getCellState(self, o):
        if o in self.covered:
            return 'free-covered'
        trangThai = self.senseCell(o)
        if trangThai == 'blocked':
            return 'blocked'
        if trangThai == 'free':
            return 'free-uncovered'
        return 'unknown'

    # BFS kiểm tra xem vùng này có phải là hole thật không 
    # lan tỏa ô ứng viên ra các ô xung quanh
    # Nếu: lan tỏa ko gặp ô nào 'unknown', ko tràn ra ngoài bản đồ nhỏ
    # ko cho hole lan ra bên ngoài, check huongRaNgoai
    def verifyHoleBFS(self, oUngVien, danhSachOCuaCong, banDoNho):
        huongRaNgoai = (self.pos[0] - oUngVien[0], self.pos[1] - oUngVien[1])
        tapCong = set(danhSachOCuaCong)

        def laCanhBiCam(o1, o2):
            if o1 in tapCong and o2 == moveCell(o1, huongRaNgoai):
                return True
            if o2 in tapCong and o1 == moveCell(o2, huongRaNgoai):
                return True
            return False

        vungHole = []
        vungHole.append(oUngVien)
        hangDoi = deque()
        hangDoi.append(oUngVien)

        while len(hangDoi) > 0:
            o = hangDoi.popleft()
            for oKe in getNeighbors4(o):
                if laCanhBiCam(o, oKe):
                    continue
                if oKe not in banDoNho:
                    return vungHole, False
                trangThai = self.getCellState(oKe)
                if trangThai == 'unknown':
                    return vungHole, False
                if trangThai == 'free-uncovered' and oKe not in vungHole:
                    vungHole.append (oKe)
                    hangDoi.append(oKe)
        print(f"Mode: {self.mode}, Hole({vungHole})")
        return vungHole, True

    # gộp các hàm đã viết: tìm candidate, xây cổng, BFS xác nhận hole
    # nếu cả 2 bên đều có hole thì ưu tiên bên hole nhỏ, nếu bằng thì ưu tiên trái
    # vungGioiHan (mặc định None): chỉ nhận hole nằm gọn trong vùng này
    # dùng khi đang quét bên trong 1 hole (hole con phải nằm trong hole cha)
    def detectHole(self, boiCanh, huongSeQuay = None, vungGioiHan = None):
        banDoNho = self.buildLocalMap()
        danhSachTimDuoc = []
        for (ben, oUngVien) in self.findHoleCandidates(boiCanh, huongSeQuay):
            ketQuaCong = self.buildGate(oUngVien)
            if ketQuaCong is None:
                continue
            doDaiCong, danhSachOCuaCong = ketQuaCong

            vungHole, laHoleThat = self.verifyHoleBFS(oUngVien, danhSachOCuaCong, banDoNho)
            if laHoleThat:
                if vungGioiHan is not None:
                    ngoaiVung = False
                    for o in vungHole:
                        if o not in vungGioiHan:
                            ngoaiVung = True
                    if ngoaiVung:
                        continue
                if ben == 'L':
                    diemUuTien = 0
                else:
                    diemUuTien = 1
                danhSachTimDuoc.append(
                    (len(vungHole), diemUuTien, vungHole, oUngVien, doDaiCong))
        if len(danhSachTimDuoc) == 0:
            return None
        danhSachTimDuoc.sort(key = lambda x: (x[0], x[1]))
        chonDuoc = danhSachTimDuoc[0]
        return chonDuoc[2], chonDuoc[3], chonDuoc[4]
    
    # chui vào hole để quét sạch rồi quay ra
    # kiểm tra (26), nếu độ dài có thể đi thẳng trước khi chạm blocked = độ dài cổng - 1 thì đi thẳng trước rồi mới rẽ vào cổng
    # ghi nhớ ô q_resume và hướng ban đầu để quay về
    # vào hole thì hoạt động theo logic bao phủ thích nghi
    # quét xong hole thì kiểm q_resume còn cần làm gì ko, hết thì True khỏi quay về, nếu còn thì quay lại q_resume
    def coverHole(self, vungHole, oUngVien, doDaiCong, khungCha = None):
        self.soHoleDaBaoPhu = self.soHoleDaBaoPhu + 1
        vungHole = set(vungHole)

        # khungCha: danh sách các hole cha đang bao quanh hole này, mỗi phần tử là
        # (vùng hole cha, hàm quét cặp cục bộ của hole cha), hole cha gần nhất đứng trước
        # hole ở hành lang chính -> khungCha rỗng -> hành vi giống hệt bản chỉ có 1 cấp hole
        if khungCha is None:
            khungCha = []

        # ----------------------------------------------------------------
        # các hàm hỗ trợ: bên trong hole robot hoạt động giống hành lang chính
        # nhưng bị giới hạn trong vungHole và dùng danh sách cặp cục bộ thay cho self.L
        # ----------------------------------------------------------------
        capQuayLuiCucBo = []

        def conPhaiGhe(o):
            return o in vungHole and o not in self.covered

        # đếm số ô còn phải ghé liên tiếp theo 1 hướng tương đối (thay cho measureDirection)
        def demTrongHole(huongTuongDoi):
            soO = 0
            for i in range(1, int(math.floor(self.rho)) + 1):
                if conPhaiGhe(self.getCellAt(huongTuongDoi, i)):
                    soO = i
                else:
                    break
            return soO

        # quét cặp quay lui cục bộ tại vị trí robot đang đứng (thay cho findBacktrackPairs)
        # chỉ quét hình học, ghi vào capQuayLuiCucBo, không đụng self.L
        def quetCapCucBo():
            oTrai = self.getCellAt('L')
            oPhai = self.getCellAt('R')
            oTruoc = self.getCellAt('F')
            buoc = BUOC_DI[self.heading]
            danhSachXet = [
                (oTrai, [moveCell(oTrai, buoc), moveCell(oTrai, buoc, -1), oTruoc]),
                (oPhai, [moveCell(oPhai, buoc), moveCell(oPhai, buoc, -1), oTruoc])
            ]
            for (oTrong, danhSachOChan) in danhSachXet:
                if not conPhaiGhe(oTrong):
                    continue
                coOChan = False
                for oChan in danhSachOChan:
                    if not conPhaiGhe(oChan):
                        coOChan = True
                if coOChan and (self.pos, oTrong) not in capQuayLuiCucBo:
                    capQuayLuiCucBo.append((self.pos, oTrong))

        # cặp mới sinh ra trong self.L mà trỏ vào hole này / hole cha thì do hole đó tự quản lý
        # (bằng danh sách cục bộ), bỏ khỏi self.L. Ô trống nằm ngoài mọi hole vẫn giữ trong L
        # toàn cục, ko thì bị bỏ sót (Bổ đề 5.3)
        def locCapMoiTrongL(soCapCu, vungCuaMinh):
            conLai = []
            for cap in self.L[soCapCu:]:
                if vungCuaMinh is not None and cap[1] in vungCuaMinh:
                    continue
                trongHoleCha = False
                for (vungCha, _) in khungCha:
                    if cap[1] in vungCha:
                        trongHoleCha = True
                if not trongHoleCha:
                    conLai.append(cap)
            self.L[soCapCu:] = conLai

        # tương đương findBacktrackPairs() ở hành lang chính, dùng khi robot đi bên trong hole
        def ghiCapCucBo():
            soCapCu = len(self.L)
            self.findBacktrackPairs()
            locCapMoiTrongL(soCapCu, vungHole)
            quetCapCucBo()
            for (_, quetCha) in khungCha:
                quetCha()

        # như trên nhưng dùng trong lúc đi thẳng tới cổng (điều kiện 26), robot chưa vào hole này
        # hole ở hành lang chính -> khungCha rỗng -> chỉ là findBacktrackPairs() như cũ
        def ghiCapKhiDiTheoCong():
            soCapCu = len(self.L)
            self.findBacktrackPairs()
            locCapMoiTrongL(soCapCu, None)
            for (_, quetCha) in khungCha:
                quetCha()

        # phát hiện hole con nằm gọn trong hole này (tương đương detectHole ở hành lang chính)
        def phatHienHoleCon(boiCanh, huongSeQuay = None):
            return self.detectHole(boiCanh, huongSeQuay, vungGioiHan = vungHole)

        # xử lí hole con bằng đệ quy, trả về True nếu quay về q_resume của hole con hết việc
        def chuiVaoHoleCon(hole):
            return self.coverHole(hole[0], hole[1], hole[2],
                                  khungCha = [(vungHole, quetCapCucBo)] + khungCha)

        # ----------------------------------------------------------------
        # đi tới cổng, vào hole
        # ----------------------------------------------------------------
        buocTien = BUOC_DI[self.heading]

        soODiThangDuoc = self.measureDirection('F')
        oSauCong = moveCell(self.pos, buocTien, doDaiCong)
        thoaDieuKien26 = (doDaiCong > 1
                          and soODiThangDuoc == doDaiCong - 1
                          and self.isBlocked(oSauCong))

        if thoaDieuKien26:
            self.soLanDungDieuKien26 = self.soLanDungDieuKien26 + 1
            for k in range(1, doDaiCong):
                self.moveToCell(moveCell(self.pos, buocTien))
                ghiCapKhiDiTheoCong()
            oChuiVao = moveCell(oUngVien, buocTien, doDaiCong - 1)
        else:
            oChuiVao = oUngVien
        # ghi lại vị trí quay về và hướng ban đầu
        viTriQuayVe = self.pos
        huongQuayVe = self.heading

        # vào hole
        huongChuiVao = None
        buocCanDi = (oChuiVao[0] - self.pos[0], oChuiVao[1] - self.pos[1])
        for ten, buoc in BUOC_DI.items():
            if buoc == buocCanDi:
                huongChuiVao = ten
        self.turnTo(huongChuiVao)
        self.moveToCell(oChuiVao)

        # ----------------------------------------------------------------
        # quét bên trong hole, cùng khung logic với run() / runStraight() / globalBacktrack()
        # ----------------------------------------------------------------
        # đi thẳng trong hole (tương đương runStraight)
        # mỗi bước: ghi cặp cục bộ -> hết đường thì dừng -> kiểm tra hole con (case 1/2) -> đi
        def runStraightLocal(huongChon):
            if huongChon != 'F':
                self.turnTo(turnRobot(self.heading, huongChon))

            soBuoc = 0
            while soBuoc < 20000:                        # chống lặp vô hạn
                soBuoc = soBuoc + 1
                ghiCapCucBo()
                if not conPhaiGhe(self.getCellAt('F')):
                    return None

                hole = phatHienHoleCon('forward')
                if hole is not None:
                    daCan = chuiVaoHoleCon(hole)
                    if daCan:
                        return 'exhausted'
                    continue                             # đã quay về, đi tiếp

                self.moveToCell(self.getCellAt('F'))
                # xóa cặp cục bộ trỏ đến ô vừa ghé (giống moveToCell xóa cặp trong self.L)
                conLai = []
                for (c_b, c_u) in capQuayLuiCucBo:
                    if c_u != self.pos:
                        conLai.append((c_b, c_u))
                capQuayLuiCucBo[:] = conLai
            return None

        # quay lui cục bộ (tương đương globalBacktrack), chỉ đi trong hole
        # trả về False nếu hết việc, True nếu đã vào được 1 nhánh, 'exhausted' nếu hole con vừa xử lí hết việc
        def quayLuiCucBo():
            # bỏ các cặp hết giá trị
            conLai = []
            for (c_b, c_u) in capQuayLuiCucBo:
                if conPhaiGhe(c_u):
                    conLai.append((c_b, c_u))
            capQuayLuiCucBo[:] = conLai

            if len(capQuayLuiCucBo) == 0:
                return False

            dichDen = set()
            for (c_b, c_u) in capQuayLuiCucBo:
                dichDen.add(c_b)

            # đường quay lui cục bộ chỉ dùng trong hole
            vungDuocDi = set()
            for o in vungHole:
                if o in self.covered:
                    vungDuocDi.add(o)
            vungDuocDi.add(self.pos)

            duongDi = self.thetaStar(dichDen, vungDuocDi)
            if duongDi is None:
                capQuayLuiCucBo[:] = []
                return False

            self.soLanQuayLuiCucBo = self.soLanQuayLuiCucBo + 1
            self.followPath(duongDi)

            # vào ô trống bên cạnh, nhiều cặp cùng điểm quay lui -> lấy cặp đầu tiên
            oDangDung = self.pos
            oCanGhe = None
            for (c_b, c_u) in capQuayLuiCucBo:
                if c_b == oDangDung and oCanGhe is None:
                    oCanGhe = c_u
            self.enterBranch(oCanGhe)

            # ghi lại cặp mới, kiểm tra hole con
            ghiCapCucBo()
            hole = phatHienHoleCon('forward')
            if hole is not None:
                daCan = chuiVaoHoleCon(hole)
                if daCan:
                    return 'exhausted'
            return True

        # vòng lặp chính trong hole (tương đương run)
        ghiCapCucBo()
        demVong = 0
        while demVong < 20000:
            demVong = demVong + 1

            doDai = {}
            for huongTuongDoi in ('F', 'L', 'R'):
                doDai[huongTuongDoi] = demTrongHole(huongTuongDoi)
            danhSachHuong = []
            for huongTuongDoi in ('F', 'L', 'R'):
                if doDai[huongTuongDoi] > 0:
                    danhSachHuong.append(huongTuongDoi)

            if len(danhSachHuong) > 0:
                huongChon = self.chooseDirection(doDai, danhSachHuong)

                # trước khi quay trái phải, kiểm tra hole con case 3,4
                if 'F' not in danhSachHuong and huongChon in ('L', 'R'):
                    hole = phatHienHoleCon('turn', huongChon)
                    if hole is not None:
                        daCan = chuiVaoHoleCon(hole)
                        if daCan:
                            while len(capQuayLuiCucBo) > 0 and quayLuiCucBo() == 'exhausted':
                                pass
                        continue

                ketQua = runStraightLocal(huongChon)

                # hole con hết việc -> chuyển sang quay lui cục bộ
                if ketQua == 'exhausted':
                    while len(capQuayLuiCucBo) > 0 and quayLuiCucBo() == 'exhausted':
                        pass

            elif len(capQuayLuiCucBo) > 0:
                ketQua = quayLuiCucBo()
                if not ketQua:
                    break
                while ketQua == 'exhausted' and len(capQuayLuiCucBo) > 0:
                    ketQua = quayLuiCucBo()
            else:
                break                                    # xong hole, không còn gì để làm

        # kiểm tra quay về con việc không
        conViec = False
        for huongTuongDoi in ('F', 'L', 'R'):
            huongThat = getAbsoluteDirection(huongQuayVe, huongTuongDoi)
            oKe = moveCell(viTriQuayVe, BUOC_DI[huongThat])
            if self.isFreeUncovered(oKe, viTriQuayVe):
                conViec = True

        if not conViec:
            self.soLanResumeCan = self.soLanResumeCan + 1
            return True

        # quay về chỗ cũ, dùng hướng cũ
        if self.pos !=  viTriQuayVe:
            duongVe = self.thetaStar({viTriQuayVe})
            if duongVe is None:
                return True
            self.followPath(duongVe)
        self.turnTo(huongQuayVe)
        return False
    # tìm đường quay lui bằng thuật Theta*
    # thuật Bresenham
    def canGoStraight(self, A, B, vungChoPhep = None):
        if vungChoPhep is None:
            vungChoPhep = self.covered

        dong0, cot0 = A
        dong1, cot1 = B
        dDong = dong1 - dong0
        dCot = cot1 - cot0

        danhSachT = {0.0, 1.0}
        for k in range(min(dong0, dong1), max(dong0, dong1)+1):
            if dDong != 0:
                for bien in (k - 0.5, k + 0.5):
                    t = (bien - dong0)/dDong
                    if 0 < t < 1:
                        danhSachT.add(t)
        for k in range(min(cot0, cot1), max(cot0, cot1)+1):
            if dCot != 0:
                for bien in (k - 0.5, k + 0.5):
                    t = (bien - cot0)/dCot
                    if 0 < t < 1:
                        danhSachT.add(t)
        danhSachT = sorted(danhSachT)
        # kiểm tra điểm cắt và trung điểm
        diemKiemTra = list(danhSachT)
        for i in range(len(danhSachT) - 1):
            diemKiemTra.append((danhSachT[i] + danhSachT[i + 1]) / 2)

        saiSo = 1e-7
        for t in diemKiemTra:
            dongThuc = dong0 + dDong * t
            cotThuc = cot0 + dCot * t
            dongDau = math.ceil(dongThuc - 0.5 - saiSo)
            dongCuoi = math.floor(dongThuc + 0.5 + saiSo)
            cotDau = math.ceil(cotThuc - 0.5 - saiSo)
            cotCuoi = math.floor(cotThuc + 0.5 + saiSo)
            for dong in range(dongDau, dongCuoi + 1):
                for cot in range(cotDau, cotCuoi + 1):
                    if (dong, cot) not in vungChoPhep:
                        return False
        return True

    # thetaStar: tìm đường ngắn nhất quay về, đi qua covered
    def thetaStar(self, dichDen, vungChoPhep = None):
        if vungChoPhep is None:
            vungChoPhep = self.covered

        if self.pos in dichDen:
            return [self.pos]

        danhSachDich = list(dichDen)
        # ước lượng khoảng cách đến đích gần nhất (36)
        def uocLuong(o):
            nhoNhat = None
            for dich in danhSachDich:
                d = math.hypot(o[0] - dich[0], o[1] - dich[1])
                if nhoNhat is None or d < nhoNhat:
                    nhoNhat = d
            return nhoNhat

        batDau = self.pos
        chiPhi = {batDau: 0.0}          # g: cost đã đi
        oCha = {batDau: batDau}
        hangDoiUuTien = [(uocLuong(batDau), batDau)]
        daXet = set()

        while len(hangDoiUuTien) > 0:
            _, o = heapq.heappop(hangDoiUuTien)
            if o in daXet:
                continue

            if o in dichDen:
                # Truy ngược lấy đường đi
                duongDi = [o]
                while oCha[duongDi[-1]] != duongDi[-1]:
                    duongDi.append(oCha[duongDi[-1]])
                duongDi.reverse()
                return duongDi

            daXet.add(o)

            # Xét 8 ô xung quanh
            for dDong in (-1, 0, 1):
                for dCot in (-1, 0, 1):
                    if dDong == 0 and dCot == 0:
                        continue
                    oKe = (o[0] + dDong, o[1] + dCot)
                    if oKe in daXet or oKe not in vungChoPhep:
                        continue

                    # Đi chéo ko được cắt góc
                    if dDong != 0 and dCot != 0:
                        if (o[0] + dDong, o[1]) not in vungChoPhep:
                            continue
                        if (o[0], o[1] + dCot) not in vungChoPhep:
                            continue

                    cha = oCha[o]
                    if self.canGoStraight(cha, oKe, vungChoPhep):
                        # Nối ô cha -> đường ngắn hơn 
                        # phân biệt Theta* với A*
                        chiPhiMoi = chiPhi[cha] + math.hypot(oKe[0] - cha[0], oKe[1] - cha[1])
                        chaMoi = cha
                    else:
                        chiPhiMoi = chiPhi[o] + math.hypot(dDong, dCot)
                        chaMoi = o

                    chiPhiCu = chiPhi.get(oKe, float('inf'))
                    if chiPhiMoi < chiPhiCu - 1e-12:
                        chiPhi[oKe] = chiPhiMoi
                        oCha[oKe] = chaMoi
                        heapq.heappush(hangDoiUuTien, (chiPhiMoi + uocLuong(oKe), oKe))

        return None
    # robot đi theo hướng quay lui đã tìm được
    # mỗi đoạn robot quay đúng hướng -> đi thẳng
    # tăng số ô bao phủ (42)
    # (39)(40)(41)
    def followPath(self, duongDi):
        for k in range(len(duongDi) - 1):
            oHienTai = duongDi[k]
            oKeTiep = duongDi[k + 1]
            goc = math.atan2(oKeTiep[0] - oHienTai[0], oKeTiep[1] - oHienTai[1])
            self.turnToAngle(goc)
            self.pos = oKeTiep
            self.path.append((oKeTiep[0], oKeTiep[1], goc))

    # từ điểm quay lui vào ô trống bên cạnh
    # kết thúc quay lui và bắt đầu bao phủ
    # (43)(44)
    def enterBranch(self, oCanGhe):
        buocCanDi = (oCanGhe[0] - self.pos[0], oCanGhe[1] - self.pos[1])
        huongMoi = None
        for ten, buoc in BUOC_DI.items():
            if buoc == buocCanDi:
                huongMoi = ten
        self.turnTo(huongMoi)
        self.moveToCell(oCanGhe)

    # quay lui toàn cục
    # ô đích đã được ghé
    # Theta* tìm đường ngắn nhất đếm 1 trong các điểm quay lui
    # đi đến ô tìm được, bước vào ô trống bên cạnh
    # ghi lại cặp quay lui mới + kiểm tra hole
    def globalBacktrack(self):
        # bỏ các cặp hết giá trị
        capConHieuLuc = []
        for (c_b, c_u) in self.L:
            if c_u not in self.covered:
                capConHieuLuc.append((c_b, c_u))
        self.L = capConHieuLuc

        if len(self.L) == 0:
            return False

        # tìm đường đến quay lui gần nhất
        dichDen = set()
        for (c_b, c_u) in self.L:
            dichDen.add(c_b)

        duongDi = self.thetaStar(dichDen)
        if duongDi is None:
            self.L = []
            return False

        self.soLanQuayLuiToanCuc = self.soLanQuayLuiToanCuc + 1
        self.followPath(duongDi)

        # vào ô trống bên cạnh
        # nhiều cặp cùng điểm quay lui -> lấy cặp đầu tiên(A4)
        oDangDung = self.pos
        oCanGhe = None
        for (c_b, c_u) in self.L:
            if c_b == oDangDung and oCanGhe is None:
                oCanGhe = c_u
        self.enterBranch(oCanGhe)

        # ghi lại cặp mới, kiểm tra hole
        self.findBacktrackPairs()
        hole = self.detectHole('forward')
        if hole is not None:
            daCan = self.coverHole(hole[0], hole[1], hole[2])
            if daCan:
                return 'exhausted'

        return True

    # chạy thẳng theo 1 hướng
    # mỗi bước ghi nhớ cặp quay lui
    # kiểm tra hole bên cạnh 
    # dừng di gặp blocked
    def runStraight(self, huongChon):
        if huongChon != 'F':
            self.turnTo(turnRobot(self.heading, huongChon))

        while True:
            self.demVongLap = self.demVongLap + 1
            if self.demVongLap > 2000:
                return None                          # chong lap vo han

            self.findBacktrackPairs()
            if self.measureDirection('F') == 0:
                return None

            hole = self.detectHole('forward')
            if hole is not None:
                daCan = self.coverHole(hole[0], hole[1], hole[2])
                if daCan:
                    return 'exhausted'
                continue                             # da quay ve, di tiep

            self.moveToCell(self.getCellAt('F'))

    # hàm chính chạy toàn bộ thuật toán
    # loop:
    # xem hướng nào còn đi được
    # Còn thì chọn hướng rồi chạy thẳng
    # Hết thì quay lui nhánh chưa đi
    # nếu hết đường đi hết luôn quay lui -> xong
    def run(self):
        # Xử lý riêng case ô bắt đầu
        # Xét ô sau lưng tránh trường hợp bỏ quên mãi mãi
        # Thuật toán 1, dòng 8 - 15
        doDai, danhSachHuong = self.getFeasibleDirections()
        oPhiaSau = self.getCellAt('B')

        if self.isFreeUncovered(oPhiaSau):
            if len(danhSachHuong) > 0:
                # Còn hướng khác -> nhớ ô sau lưng
                self.L.append((self.pos, oPhiaSau))
            else:
                # Không còn hướng thì quay đầu lại
                self.turnTo(turnRobot(self.heading, 'B'))
                doDai, danhSachHuong = self.getFeasibleDirections()

        self.findBacktrackPairs()

        # loop chính
        while self.demVongLap < 200000:
            self.demVongLap = self.demVongLap + 1

            doDai, danhSachHuong = self.getFeasibleDirections()

            if len(danhSachHuong) > 0:
                huongChon = self.chooseDirection(doDai, danhSachHuong)

                # Trước khi quay trái phải, kiểm tra hole case 3,4
                if 'F' not in danhSachHuong and huongChon in ('L', 'R'):
                    hole = self.detectHole('turn', huongChon)
                    if hole is not None:
                        daCan = self.coverHole(hole[0], hole[1], hole[2])
                        if daCan:
                            while len(self.L) > 0 and self.globalBacktrack() == 'exhausted':
                                pass
                        continue

                ketQua = self.runStraight(huongChon)

                # Muc 4.4.4 va 4.5: nếu quay về hết việc -> đi thẳng
                # chuyển qua quay lui toàn cục
                if ketQua == 'exhausted':
                    while len(self.L) > 0 and self.globalBacktrack() == 'exhausted':
                        pass

            elif len(self.L) > 0:
                ketQua = self.globalBacktrack()
                if not ketQua:
                    break
                while ketQua == 'exhausted' and len(self.L) > 0:
                    ketQua = self.globalBacktrack()
            else:
                break                                 # xong, khong con gi de lam

        return self.getResult()

    # đánh giá
    def getResult(self):
        # Tổng quãng đường = tổng độ dài từng đoạn
        tongQuangDuong = 0.0
        for i in range(len(self.path) - 1):
            dong1, cot1, _ = self.path[i]
            dong2, cot2, _ = self.path[i + 1]
            tongQuangDuong = tongQuangDuong + math.hypot(dong2 - dong1, cot2 - cot1)

        # Số lần quay đầu = đếm số lần góc thay đổi
        soLanQuayDau = 0
        for i in range(len(self.path) - 1):
            goc1 = self.path[i][2]
            goc2 = self.path[i + 1][2]
            if abs(normalizeAngle(goc2 - goc1)) > 1e-9:
                soLanQuayDau = soLanQuayDau + 1

        return {
            'path': self.path,
            'covered': self.covered,
            'L(P)': tongQuangDuong,
            'NT(P)': soLanQuayDau,
            'global_backtracks': self.soLanQuayLuiToanCuc,
            'holes_covered': self.soHoleDaBaoPhu,
            'resume_exhausted': self.soLanResumeCan,
            'local_backtracks': self.soLanQuayLuiCucBo,
            'cond26': self.soLanDungDieuKien26,
        }

ACM = ACM_LOS


# ACM-OFS: giống hệt ACM_LOS, chỉ khác duy nhất cảm biến kiểu occlusion-free
# (không bị chặn bởi vật cản, chỉ giới hạn bởi rho và biên bản đồ - xem senseCell())
# Không override logic gì khác, chỉ ép mode = 'OFS' ngay tại khởi tạo.
class ACM_OFS(ACM_LOS):
    def __init__(self, grid, start, rho = 3.0, huongBanDau = 'N'):
        super().__init__(grid, start, rho = rho, huongBanDau = huongBanDau, mode = 'OFS')