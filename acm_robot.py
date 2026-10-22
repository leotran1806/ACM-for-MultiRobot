"""ACMRobot — thuật toán ACM (LOS / OFS) của MỘT robot, viết lại để chạy trong hệ đa robot.

Tách từ `ACM_LOS_FINAL.py`. Logic chọn hướng / hole / cặp quay lui / Theta* GIỮ NGUYÊN.
Khác biệt (xem bảng 7.2 của kế hoạch):

  * `self.covered` và `self.L` là object CHUNG của cả đội (SharedState) -> (M2)(M3)(M9).
    Mọi chỗ gán lại `self.L = ...` đổi thành `self.L[:] = ...` để không mất liên kết.
  * Các vòng lặp lồng nhau run -> runStraight -> coverHole -> globalBacktrack -> followPath
    thành GENERATOR. Mỗi lần robot thật sự di chuyển / chờ, nó `yield Intent` cho bộ điều phối,
    nhận lại True/False (được chấp nhận hay không), áp dụng, rồi `yield SYNC` (Bước 5).
  * Đoạn Theta* được rời rạc hóa thành dãy ô kề nhau; `path` (để tính L(P), NT(P)) vẫn chỉ ghi
    các đỉnh của đường Theta* như bản gốc -> L(P), NT(P) không đổi.
  * Không còn `print` debug, không còn trần `demVongLap > 2000` (trần này làm cắt cụt bản đồ lớn;
    giới hạn an toàn nay nằm ở Simulator.max_steps).
  * `globalBacktrack` không còn xóa sạch L khi Theta* thất bại — đa robot sẽ làm mất cặp của cả đội.
"""
import math
import heapq
from collections import deque

from config import Config
from shared_state import Intent, SYNC

DANH_SACH_HUONG = ['N', 'E', 'S', 'W']
BUOC_DI = {'N': (-1, 0), 'E': (0, 1), 'S': (1, 0), 'W': (0, -1)}
GOC_CUA_HUONG = {h: math.atan2(BUOC_DI[h][0], BUOC_DI[h][1]) for h in DANH_SACH_HUONG}


def turnRobot(huong_hien_tai, kieuQuay):
    viTri = DANH_SACH_HUONG.index(huong_hien_tai)
    if kieuQuay == 'L':
        return DANH_SACH_HUONG[(viTri - 1) % 4]
    elif kieuQuay == 'R':
        return DANH_SACH_HUONG[(viTri + 1) % 4]
    elif kieuQuay == 'B':
        return DANH_SACH_HUONG[(viTri + 2) % 4]
    return huong_hien_tai


def getAbsoluteDirection(huong_hien_tai, huongTuongDoi):
    if huongTuongDoi == 'F':
        return huong_hien_tai
    return turnRobot(huong_hien_tai, huongTuongDoi)


def moveCell(o, buoc, k=1):
    return (o[0] + buoc[0] * k, o[1] + buoc[1] * k)


def getNeighbors4(o):
    dong, cot = o
    return [(dong - 1, cot), (dong + 1, cot), (dong, cot - 1), (dong, cot + 1)]


def normalizeAngle(goc):
    goc = (goc + math.pi) % (2 * math.pi) - math.pi
    if goc <= -math.pi:
        goc = goc + 2 * math.pi
    return goc


class ACMRobot:
    def __init__(self, rid, grid, start, shared, rho=3.0, huongBanDau='N', sensing='LOS', config=None):
        if rho < math.sqrt(2):
            raise ValueError("rho qua nho")
        if not (0 <= start[0] < len(grid) and 0 <= start[1] < len(grid[0])):
            raise ValueError("nam ngoai ban do")
        if grid[start[0]][start[1]] == 1:
            raise ValueError("o bat dau la vat can")
        if sensing not in ('LOS', 'OFS'):
            raise ValueError("sensing phai la 'LOS' hoac 'OFS'")

        self.id = rid
        self.grid = grid
        self.soDong = len(grid)
        self.soCot = len(grid[0])
        self.rho = rho
        self.sensing = sensing
        self.cfg = config or Config()
        self.shared = shared # abstract --> t sẽ chỉnh sau khi có file shared_state.py

        self.pos = start
        self.heading = huongBanDau
        self.state = 'COVER'                 # m_k(t): COVER/HOLE/BT_PLAN/BACKTRACK/WAIT/FAILED

        # ---- DÙNG CHUNG (M2) ----
        self.covered = shared.covered # abstract --> sẽ chỉnh sau khi có shared_state.py
        self.L = shared.L # abstract --> sẽ chỉnh sau khi có shared_state.py
        self.covered.add(start)
        shared.covered_by[rid].add(start) # abstract --> sẽ chỉnh sau khi có shared_state.py

        self.path = [(start[0], start[1], GOC_CUA_HUONG[huongBanDau])]   # P (10), chỉ các đỉnh
        self.trace = [start]                                              # ô thật sự đi qua theo từng bước
        self.moves = []                                                   # (src, dst, kind) để vẽ

        self.soLanQuayLuiToanCuc = 0
        self.soHoleDaBaoPhu = 0
        self.soLanResumeCan = 0
        self.soLanQuayLuiCucBo = 0
        self.soLanDungDieuKien26 = 0
        self.demVongLap = 0

        # đa robot
        self.temp_avoid = set()              # ô vừa thua xung đột (loser_policy = replan_first)
        self.maxRunSteps = None              # dùng cho baseline boustrophedon
        self.waitCount = 0                   # w_k: số lần bị từ chối liên tiếp
        self.totalWait = 0
        self.avoidRobots = False             # nấc 1 của thang deadlock
        self.cancelledBacktracks = 0
        self.unreachable = 0
        self._unreachSig = None

    # =====================================================================
    # Cảm biến, điều kiện ô — giữ nguyên như ACM_LOS_FINAL (+ hook đa robot)
    # =====================================================================
    def isInsideMap(self, o):
        return 0 <= o[0] < self.soDong and 0 <= o[1] < self.soCot

    def getCellsOnLine(self, A, B):
        dong0, cot0 = A
        dong1, cot1 = B
        dDong = dong1 - dong0
        dCot = cot1 - cot0
        danhsachT = {0.0, 1.0}
        if dDong != 0:
            for k in range(min(dong0, dong1), max(dong0, dong1) + 1):
                for j in (k - 0.5, k + 0.5):
                    temp = (j - dong0) / dDong
                    if 0.0 < temp < 1.0:
                        danhsachT.add(temp)
        if dCot != 0:
            for k in range(min(cot0, cot1), max(cot0, cot1) + 1):
                for j in (k - 0.5, k + 0.5):
                    temp = (j - cot0) / dCot
                    if 0.0 < temp < 1.0:
                        danhsachT.add(temp)
        danhsachT = sorted(danhsachT)
        ketQua = []
        for i in range(len(danhsachT) - 1):
            t = (danhsachT[i] + danhsachT[i + 1]) / 2.0
            o = (round(dong0 + dDong * t), round(cot0 + dCot * t))
            if o != A and o != B and o not in ketQua:
                ketQua.append(o)
        return ketQua

    def isSightBlocked(self, A, B):
        for o in self.getCellsOnLine(A, B):
            if not self.isInsideMap(o) or self.grid[o[0]][o[1]]:
                return True
        return False

    def senseCell(self, o, diemDung=None):
        if diemDung is None:
            diemDung = self.pos
        if not self.isInsideMap(o):
            return 'blocked'
        euclidean_distance = math.hypot(o[0] - diemDung[0], o[1] - diemDung[1])
        if euclidean_distance > math.floor(self.rho):
            return 'unknown'
        if self.sensing == 'LOS' and self.isSightBlocked(o, diemDung):
            return 'unknown'
        if self.grid[o[0]][o[1]] == 1:
            return 'blocked'
        return 'free'

    def _reservedByOther(self, o):
        """(M3) ô thuộc hole đang được robot khác đặt chỗ (tùy chọn reserve_hole)."""
        if not self.cfg.reserve_hole:
            return False
        r = self.shared.res_hole.get(o)
        return r is not None and r != self.id

    def isFreeUncovered(self, o, diemDung=None):
        if self.senseCell(o, diemDung) != 'free':
            return False
        if o in self.covered:                 # covered = C_cov CHUNG
            return False
        if o in self.temp_avoid:
            return False
        if self._reservedByOther(o):
            return False
        return True

    def isBlocked(self, o, diemDung=None):
        if self.senseCell(o, diemDung) == 'blocked':
            return True
        if o in self.covered:
            return True
        if self._reservedByOther(o):
            return True
        return False

    def getCellAt(self, huongTuongDoi, i=1, viTri=None, huong=None):
        if viTri is None:
            viTri = self.pos
        if huong is None:
            huong = self.heading
        huongThat = getAbsoluteDirection(huong, huongTuongDoi)
        return moveCell(viTri, BUOC_DI[huongThat], i)

    def measureDirection(self, huongTuongDoi, viTri=None, huong=None):
        count = 0
        tamNhin = int(math.floor(self.rho))
        for i in range(1, tamNhin + 1):
            o = self.getCellAt(huongTuongDoi, i, viTri, huong)
            if self.isFreeUncovered(o, viTri):
                count = i
            else:
                break
        return count

    def getFeasibleDirections(self, viTri=None, huong=None):
        doDai = {}
        for huongTuongDoi in ('F', 'L', 'R'):
            doDai[huongTuongDoi] = self.measureDirection(huongTuongDoi, viTri, huong)
        danhSachHuong = [h for h in ('F', 'L', 'R') if doDai[h] > 0]
        return doDai, danhSachHuong

    @staticmethod
    def chooseDirection(doDai, danhSachHuong):
        huongTotNhat = danhSachHuong[0]
        for huongTuongDoi in danhSachHuong:
            if doDai[huongTuongDoi] > doDai[huongTotNhat]:
                huongTotNhat = huongTuongDoi
        return huongTotNhat

    def selectDirection(self, doDai, danhSachHuong):
        """Điểm móc cho baseline: ACM dùng (15)(16)."""
        return self.chooseDirection(doDai, danhSachHuong)

    def turnToAngle(self, goc):
        gocHienTai = self.path[-1][2]
        if abs(normalizeAngle(goc - gocHienTai)) > 1e-9:
            self.path.append((self.pos[0], self.pos[1], goc))

    def turnTo(self, huong):
        self.heading = huong
        self.turnToAngle(GOC_CUA_HUONG[huong])

    def _setState(self, s):
        self.state = s
        self.shared.state[self.id] = s

    def moveToCell(self, o):
        """(18) phủ ô mới; (33)(34)=(M9) xóa cặp có c^u = o khỏi L CHUNG (sửa tại chỗ)."""
        self.moves.append((self.pos, o, 'cover'))
        self.pos = o
        self.covered.add(o)
        self.shared.covered_by[self.id].add(o)
        self.shared.pos[self.id] = o
        self.shared.cover_log.append((self.shared.t, self.id, o))
        self.trace.append(o)
        self.path.append((o[0], o[1], self.path[-1][2]))
        capConLai = []
        for (c_b, c_u) in self.L:
            if c_u != o:
                capConLai.append((c_b, c_u))
        self.L[:] = capConLai

    def _moveNoCover(self, o):
        """(42) quay lui không đổi C_cov."""
        self.moves.append((self.pos, o, 'bt'))
        self.pos = o
        self.shared.pos[self.id] = o
        self.trace.append(o)

    # =====================================================================
    # Cặp quay lui (29)–(35)
    # =====================================================================
    def checkBacktrackCondition(self, oTrong, oChan):
        if self.isFreeUncovered(oTrong) and self.isBlocked(oChan):
            return 1
        return 0

    def findBacktrackPairs(self):
        oTrai = self.getCellAt('L')
        oPhai = self.getCellAt('R')
        oTruoc = self.getCellAt('F')
        buocTien = BUOC_DI[self.heading]
        oTraiTruoc = moveCell(oTrai, buocTien)
        oTraiSau = moveCell(oTrai, buocTien, -1)
        oPhaiTruoc = moveCell(oPhai, buocTien)
        oPhaiSau = moveCell(oPhai, buocTien, -1)

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

    def _recordSkippedBranch(self, o):
        """Lỗ hổng Bổ đề 5.3 (mục 4.10): robot bỏ hướng F còn khả thi vì thua xung đột
        -> ghi thêm cặp (c_t, c^F_1) giống quy tắc ô sau lưng ở Thuật toán 1."""
        if self.senseCell(o) == 'free' and o not in self.covered:
            cap = (self.pos, o)
            if cap not in self.L:
                self.L.append(cap)

    # =====================================================================
    # Hole (19)–(28)
    # =====================================================================
    def findHoleCandidates(self, boiCanh, huongSeQuay=None):
        oTrai = self.getCellAt('L')
        oPhai = self.getCellAt('R')
        oTruoc = self.getCellAt('F')
        buocTien = BUOC_DI[self.heading]
        oTraiSau = moveCell(oTrai, buocTien, -1)
        oPhaiSau = moveCell(oPhai, buocTien, -1)
        ketQua = []
        if boiCanh == 'forward':
            if self.isFreeUncovered(oTrai) and self.isBlocked(oTraiSau):
                ketQua.append(('L', oTrai))
            if self.isFreeUncovered(oPhai) and self.isBlocked(oPhaiSau):
                ketQua.append(('R', oPhai))
        else:
            if huongSeQuay == 'R':
                if self.isFreeUncovered(oTrai) and self.isBlocked(oTruoc):
                    ketQua.append(('L', oTrai))
            elif huongSeQuay == 'L':
                if self.isFreeUncovered(oPhai) and self.isBlocked(oTruoc):
                    ketQua.append(('R', oPhai))
        return ketQua

    def buildGate(self, oUngVien):
        buocTien = BUOC_DI[self.heading]
        i = 1
        while i * i + 1 <= self.rho * self.rho:
            if self.isBlocked(moveCell(oUngVien, buocTien, i)):
                return i, [moveCell(oUngVien, buocTien, j) for j in range(i)]
            i = i + 1
        return None

    def buildLocalMap(self):
        dongRobot, cotRobot = self.pos
        banKinh = int(math.floor(self.rho))
        banDoNho = set()
        for dong in range(dongRobot - banKinh, dongRobot + banKinh + 1):
            for cot in range(cotRobot - banKinh, cotRobot + banKinh + 1):
                distance = (dong - dongRobot) ** 2 + (cot - cotRobot) ** 2
                if distance <= self.rho ** 2 and self.isInsideMap((dong, cot)):
                    banDoNho.add((dong, cot))
        return banDoNho

    def getCellState(self, o):
        if o in self.covered or self._reservedByOther(o):
            return 'free-covered'
        trangThai = self.senseCell(o)
        if trangThai == 'blocked':
            return 'blocked'
        if trangThai == 'free':
            return 'free-uncovered'
        return 'unknown'

    def verifyHoleBFS(self, oUngVien, danhSachOCuaCong, banDoNho):
        huongRaNgoai = (self.pos[0] - oUngVien[0], self.pos[1] - oUngVien[1])
        tapCong = set(danhSachOCuaCong)

        def laCanhBiCam(o1, o2):
            if o1 in tapCong and o2 == moveCell(o1, huongRaNgoai):
                return True
            if o2 in tapCong and o1 == moveCell(o2, huongRaNgoai):
                return True
            return False

        vungHole = [oUngVien]
        hangDoi = deque([oUngVien])
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
                    vungHole.append(oKe)
                    hangDoi.append(oKe)
        return vungHole, True

    def detectHole(self, boiCanh, huongSeQuay=None, vungGioiHan=None):
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
                    if any(o not in vungGioiHan for o in vungHole):
                        continue
                diemUuTien = 0 if ben == 'L' else 1
                danhSachTimDuoc.append((len(vungHole), diemUuTien, vungHole, oUngVien, doDaiCong))
        if len(danhSachTimDuoc) == 0:
            return None
        danhSachTimDuoc.sort(key=lambda x: (x[0], x[1]))
        chonDuoc = danhSachTimDuoc[0]
        return chonDuoc[2], chonDuoc[3], chonDuoc[4]

    # =====================================================================
    # Theta* đa đích (36)–(38)
    # =====================================================================
    def canGoStraight(self, A, B, vungChoPhep=None):
        if vungChoPhep is None:
            vungChoPhep = self.covered
        dong0, cot0 = A
        dong1, cot1 = B
        dDong = dong1 - dong0
        dCot = cot1 - cot0
        danhSachT = {0.0, 1.0}
        for k in range(min(dong0, dong1), max(dong0, dong1) + 1):
            if dDong != 0:
                for bien in (k - 0.5, k + 0.5):
                    t = (bien - dong0) / dDong
                    if 0 < t < 1:
                        danhSachT.add(t)
        for k in range(min(cot0, cot1), max(cot0, cot1) + 1):
            if dCot != 0:
                for bien in (k - 0.5, k + 0.5):
                    t = (bien - cot0) / dCot
                    if 0 < t < 1:
                        danhSachT.add(t)
        danhSachT = sorted(danhSachT)
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

    def thetaStar(self, dichDen, vungChoPhep=None, obstacles=None):
        """obstacles: ô tạm coi là vật cản (robot hỏng; robot đứng yên ở nấc 1 của thang deadlock)."""
        if vungChoPhep is None:
            vungChoPhep = self.covered
        if obstacles:
            vungChoPhep = set(vungChoPhep) - set(obstacles)
        if self.pos in dichDen:
            return [self.pos]
        danhSachDich = list(dichDen)

        def uocLuong(o):
            nhoNhat = None
            for dich in danhSachDich:
                d = math.hypot(o[0] - dich[0], o[1] - dich[1])
                if nhoNhat is None or d < nhoNhat:
                    nhoNhat = d
            return nhoNhat

        batDau = self.pos
        chiPhi = {batDau: 0.0}
        oCha = {batDau: batDau}
        hangDoiUuTien = [(uocLuong(batDau), batDau)]
        daXet = set()
        while len(hangDoiUuTien) > 0:
            _, o = heapq.heappop(hangDoiUuTien)
            if o in daXet:
                continue
            if o in dichDen:
                duongDi = [o]
                while oCha[duongDi[-1]] != duongDi[-1]:
                    duongDi.append(oCha[duongDi[-1]])
                duongDi.reverse()
                return duongDi
            daXet.add(o)
            for dDong in (-1, 0, 1):
                for dCot in (-1, 0, 1):
                    if dDong == 0 and dCot == 0:
                        continue
                    oKe = (o[0] + dDong, o[1] + dCot)
                    if oKe in daXet or oKe not in vungChoPhep:
                        continue
                    if dDong != 0 and dCot != 0:
                        if (o[0] + dDong, o[1]) not in vungChoPhep:
                            continue
                        if (o[0], o[1] + dCot) not in vungChoPhep:
                            continue
                    cha = oCha[o]
                    if self.canGoStraight(cha, oKe, vungChoPhep):
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

    # =====================================================================
    # GENERATOR: các bước di chuyển / chờ (giao thức với bộ điều phối)
    # =====================================================================
    def _act(self, intent):
        ok = yield intent
        yield SYNC
        return ok

    def _turnTo(self, huong):
        """Quay (có thể tốn turn_cost_steps bước, mục 2.3 dòng 12)."""
        if self.cfg.turn_cost_steps > 0 and huong != self.heading:
            for _ in range(self.cfg.turn_cost_steps):
                yield from self._act(Intent(self.id, 'turn', self.pos, self.pos, 0.0, math.pi / 2))
        self.turnTo(huong)

    def stepTo(self, o, kind, v=1.0, theta=0.0, goal=None, ahead=(), d=0.0):
        """Một bước di chuyển tới ô `o` (kề cạnh; kề chéo khi quay lui).
        True nếu đã tới; False nếu phải bỏ bước này (ô đã bị robot khác phủ / buộc lập lại đường)."""
        cover = kind in ('cover', 'hole', 'enter')
        waited = 0
        while True:
            commit = goal is not None and kind == 'backtrack' and self.shared.res.get(goal) != self.id
            it = Intent(self.id, kind, self.pos, o, float(v), float(theta), goal,
                        commit, tuple(ahead), waited, float(d))
            ok = yield it
            if ok:
                if goal is not None and kind == 'backtrack':
                    self.shared.res.setdefault(goal, self.id)      # cam kết đích -> đặt chỗ (M10)
                if cover:
                    self.moveToCell(o)
                else:
                    self._moveNoCover(o)
                self.waitCount = 0
                yield SYNC
                return True
            yield SYNC
            waited += 1
            self.waitCount = waited
            self.totalWait += 1
            if kind in ('cover', 'hole', 'enter'):
                if o in self.covered:
                    return False                                   # robot khác đã phủ -> đo lại ℓ_t
                if kind == 'cover' and self.cfg.loser_policy == 'replan_first':
                    self.temp_avoid = {o}
                    return False
            elif kind == 'backtrack':
                req = self.shared.dodge.pop(self.id, None)
                if req is not None and self.cfg.deadlock_ladder >= 2:
                    moved = yield from self._dodge(req)
                    if moved:
                        return False                               # bị đẩy khỏi đường -> lập lại
                if waited >= self.shared.K and self.cfg.deadlock_ladder >= 1:
                    self.avoidRobots = True                        # nấc 1: coi robot đứng yên là vật cản
                    return False
            else:                                                   # dodge / khác: bỏ cuộc
                return False

    def _dodge(self, avoid):
        """Nấc 2 / 4: dời tới ô covered gần nhất không nằm trên lối đi `avoid`."""
        sh = self.shared
        occupied = {p for j, p in sh.pos.items() if j != self.id} | set(sh.failed_cells)
        start = self.pos
        prev = {start: None}
        dq = deque([start])
        target = None
        while dq:
            c = dq.popleft()
            if c != start and c not in avoid:
                target = c
                break
            for nb in getNeighbors4(c):
                if nb in prev or nb not in self.covered or nb in occupied:
                    continue
                prev[nb] = c
                dq.append(nb)
        if target is None:
            return False
        route = []
        c = target
        while c != start:
            route.append(c)
            c = prev[c]
        route.reverse()
        sh.stats['dodges'] += 1
        moved = False
        for c in route:
            goc = math.atan2(c[0] - self.pos[0], c[1] - self.pos[1])
            self.turnToAngle(goc)
            ok = yield from self.stepTo(c, 'dodge', v=0.0, theta=math.pi)
            if not ok:
                break
            self.path.append((c[0], c[1], goc))
            moved = True
        return moved

    def _waitStep(self):
        self._setState('WAIT')
        req = self.shared.dodge.pop(self.id, None)
        if req is not None and self.cfg.deadlock_ladder >= 4:
            moved = yield from self._dodge(req)
            self._setState('WAIT')
            if moved:
                return
        yield from self._act(Intent(self.id, 'wait', self.pos, self.pos, 0.0, 0.0))

    def _releaseRes(self, goal):
        if goal is not None and self.shared.res.get(goal) == self.id:
            del self.shared.res[goal]

    def _obstacles(self):
        obs = set(self.shared.failed_cells)
        if self.avoidRobots:
            for j, st in self.shared.state.items():
                if j != self.id and j not in self.shared.failed and st == 'WAIT':
                    obs.add(self.shared.pos[j])
        obs.discard(self.pos)
        return obs

    def _candidatePairs(self):
        """(M10) 𝓛^k_flt(t): bỏ cặp đã phủ, đích bị robot khác đặt chỗ, (tùy chọn) c^u sát robot đang phủ."""
        sh = self.shared
        near = set()
        if self.cfg.filter_near_cover:
            for j, st in sh.state.items():
                if j != self.id and j not in sh.failed and st == 'COVER':
                    p = sh.pos[j]
                    for dr in (-1, 0, 1):
                        for dc in (-1, 0, 1):
                            near.add((p[0] + dr, p[1] + dc))
        out = []
        for (b, u) in self.L:
            if u in self.covered or b in sh.failed_cells:
                continue
            r = sh.res.get(b)
            if r is not None and r != self.id:
                continue
            if u in near:
                continue
            out.append((b, u))
        return out

    # ---------------------------------------------------------------------
    def followPath(self, duongDi, goal=None):
        """(39)(40)(41): đi theo đường Theta*, từng ô một. Trả True nếu tới cuối,
        False nếu bị buộc lập lại (cặp bị xóa / chờ quá lâu / bị nhường đường)."""
        if len(duongDi) < 2:
            return True
        self._setState('BACKTRACK')
        doDaiDoan = [math.hypot(duongDi[k + 1][0] - duongDi[k][0], duongDi[k + 1][1] - duongDi[k][1])
                     for k in range(len(duongDi) - 1)]
        for k in range(len(duongDi) - 1):
            a, b = duongDi[k], duongDi[k + 1]
            if self.pos != a:
                return False
            goc = math.atan2(b[0] - a[0], b[1] - a[1])
            alpha = abs(normalizeAngle(goc - self.path[-1][2]))
            self.turnToAngle(goc)                                   # NT(P) giữ nguyên như ACM
            if self.cfg.turn_cost_steps > 0 and alpha > 1e-9:
                for _ in range(self.cfg.turn_cost_steps):
                    yield from self._act(Intent(self.id, 'turn', self.pos, self.pos, 0.0, alpha))
            cells = [a] + self.getCellsOnLine(a, b) + [b]           # rời rạc hóa đoạn thẳng thành ô
            conLaiSau = sum(doDaiDoan[k + 1:])
            for idx in range(1, len(cells)):
                if goal is not None and not any(bb == goal for (bb, _) in self.L):
                    self.cancelledBacktracks += 1                   # mọi cặp tại c^b đã bị robot khác phủ
                    if self.pos != (self.path[-1][0], self.path[-1][1]):
                        self.path.append((self.pos[0], self.pos[1], goc))
                    return False
                c = cells[idx]
                v = math.hypot(self.pos[0] - b[0], self.pos[1] - b[1]) + conLaiSau
                ok = yield from self.stepTo(c, 'backtrack', v=v, theta=(alpha if idx == 1 else 0.0),
                                            goal=goal, ahead=cells[idx + 1:idx + 4], d=v)
                if not ok:
                    if self.pos != (self.path[-1][0], self.path[-1][1]):
                        self.path.append((self.pos[0], self.pos[1], goc))
                    return False
            self.path.append((b[0], b[1], goc))
        return True

    def enterBranch(self, oCanGhe, goal=None):
        """(43)(44): từ c^b bước vào c^u, kết thúc quay lui, trở lại chế độ phủ."""
        buocCanDi = (oCanGhe[0] - self.pos[0], oCanGhe[1] - self.pos[1])
        huongMoi = None
        for ten, buoc in BUOC_DI.items():
            if buoc == buocCanDi:
                huongMoi = ten
        alpha = abs(normalizeAngle(GOC_CUA_HUONG[huongMoi] - self.path[-1][2]))
        yield from self._turnTo(huongMoi)
        ok = yield from self.stepTo(oCanGhe, 'enter', v=1.0, theta=alpha, goal=goal)
        return ok

    def globalBacktrack(self):
        """Quay lui toàn cục. Trả False nếu L rỗng, 'wait' nếu còn cặp nhưng chưa đi được,
        True nếu đã vào nhánh, 'exhausted' nếu sau khi vào nhánh gặp hole mà q_resume cạn."""
        self._setState('BT_PLAN')
        capConHieuLuc = []
        for (c_b, c_u) in self.L:
            if c_u not in self.covered:
                capConHieuLuc.append((c_b, c_u))
        self.L[:] = capConHieuLuc
        if len(self.L) == 0:
            return False

        dauTien = True
        for _ in range(1000):
            cand = self._candidatePairs()
            if not cand:
                return 'wait'
            obs = self._obstacles()
            dichDen = {c_b for (c_b, _) in cand} - obs
            if not dichDen:
                return 'wait'
            sig = (len(self.covered), frozenset(dichDen), frozenset(obs))
            if sig == self._unreachSig:
                return 'wait'
            duongDi = self.thetaStar(dichDen, obstacles=obs)
            if duongDi is None:                      # KHÔNG xóa L (khác bản 1 robot): đội còn cần các cặp này
                self._unreachSig = sig
                self.unreachable += 1
                self.avoidRobots = False
                return 'wait'
            if dauTien:
                self.soLanQuayLuiToanCuc += 1
                dauTien = False
            goal = duongDi[-1]
            toiNoi = yield from self.followPath(duongDi, goal)
            self._releaseRes(goal)                   # tới c^b (hoặc hủy) -> giải phóng đặt chỗ, KHÔNG xóa cặp
            if not toiNoi:
                continue
            self.avoidRobots = False
            oCanGhe = None
            for (c_b, c_u) in self._candidatePairs():   # nhiều cặp cùng c^b -> cặp xuất hiện đầu tiên
                if c_b == self.pos and oCanGhe is None:
                    oCanGhe = c_u
            if oCanGhe is None:
                continue                              # nhánh vừa bị robot khác phủ -> lập lại
            break
        else:
            return 'wait'

        ok = yield from self.enterBranch(oCanGhe, goal)
        self._setState('COVER')
        if not ok:
            return True
        self.findBacktrackPairs()
        hole = self.detectHole('forward')
        if hole is not None:
            daCan = yield from self.coverHole(hole[0], hole[1], hole[2])
            if daCan:
                return 'exhausted'
        return True

    def runStraight(self, huongChon):
        self._setState('COVER')
        maxBuoc, self.maxRunSteps = self.maxRunSteps, None
        daDi = 0
        dauTien = True
        if huongChon != 'F':
            yield from self._turnTo(turnRobot(self.heading, huongChon))
        while True:
            self.demVongLap += 1
            self.findBacktrackPairs()                 # gọi SAU cập nhật đồng thời (Bổ đề 5.3)
            doDaiF = self.measureDirection('F')
            if doDaiF == 0:
                return None
            if maxBuoc is not None and daDi >= maxBuoc:
                return None
            hole = self.detectHole('forward')
            if hole is not None:
                daCan = yield from self.coverHole(hole[0], hole[1], hole[2])
                if daCan:
                    return 'exhausted'
                self._setState('COVER')
                continue
            o = self.getCellAt('F')
            theta = (math.pi / 2) if (dauTien and huongChon != 'F') else 0.0
            ok = yield from self.stepTo(o, 'cover', v=doDaiF, theta=theta)
            dauTien = False
            if not ok:
                if self.cfg.loser_policy == 'replan_first' and o in self.temp_avoid:
                    self._recordSkippedBranch(o)
                    return None
                continue
            daDi += 1

    # ---------------------------------------------------------------------
    def coverHole(self, vungHole, oUngVien, doDaiCong, khungCha=None):
        mine = set()
        prev = self.state
        try:
            r = yield from self._coverHoleBody(vungHole, oUngVien, doDaiCong, khungCha, mine)
        finally:
            for c in mine:
                if self.shared.res_hole.get(c) == self.id:
                    del self.shared.res_hole[c]
        self._setState(prev)
        return r

    def _coverHoleBody(self, vungHole, oUngVien, doDaiCong, khungCha, mine):
        self.soHoleDaBaoPhu += 1
        self._setState('HOLE')
        vungHole = set(vungHole)
        if khungCha is None:
            khungCha = []
        if self.cfg.reserve_hole:                    # đặt chỗ hole (đề xuất của nhóm, M3)
            for c in vungHole:
                if c not in self.shared.res_hole:
                    self.shared.res_hole[c] = self.id
                    mine.add(c)

        capQuayLuiCucBo = []

        def conPhaiGhe(o):
            return o in vungHole and o not in self.covered

        def demTrongHole(huongTuongDoi):
            soO = 0
            for i in range(1, int(math.floor(self.rho)) + 1):
                if conPhaiGhe(self.getCellAt(huongTuongDoi, i)):
                    soO = i
                else:
                    break
            return soO

        def quetCapCucBo():
            oTrai = self.getCellAt('L')
            oPhai = self.getCellAt('R')
            oTruoc = self.getCellAt('F')
            buoc = BUOC_DI[self.heading]
            danhSachXet = [
                (oTrai, [moveCell(oTrai, buoc), moveCell(oTrai, buoc, -1), oTruoc]),
                (oPhai, [moveCell(oPhai, buoc), moveCell(oPhai, buoc, -1), oTruoc]),
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

        def ghiCapCucBo():
            soCapCu = len(self.L)
            self.findBacktrackPairs()
            locCapMoiTrongL(soCapCu, vungHole)
            quetCapCucBo()
            for (_, quetCha) in khungCha:
                quetCha()

        def ghiCapKhiDiTheoCong():
            soCapCu = len(self.L)
            self.findBacktrackPairs()
            locCapMoiTrongL(soCapCu, None)
            for (_, quetCha) in khungCha:
                quetCha()

        def phatHienHoleCon(boiCanh, huongSeQuay=None):
            return self.detectHole(boiCanh, huongSeQuay, vungGioiHan=vungHole)

        def chuiVaoHoleCon(hole):
            return (yield from self.coverHole(hole[0], hole[1], hole[2],
                                              khungCha=[(vungHole, quetCapCucBo)] + khungCha))

        # ---- đi tới cổng, vào hole ----
        buocTien = BUOC_DI[self.heading]
        soODiThangDuoc = self.measureDirection('F')
        oSauCong = moveCell(self.pos, buocTien, doDaiCong)
        thoaDieuKien26 = (doDaiCong > 1
                          and soODiThangDuoc == doDaiCong - 1
                          and self.isBlocked(oSauCong))
        if thoaDieuKien26:
            self.soLanDungDieuKien26 += 1
            for k in range(1, doDaiCong):
                o = moveCell(self.pos, buocTien)
                ok = yield from self.stepTo(o, 'hole', v=doDaiCong - k, theta=0.0)
                if not ok:
                    return False                      # bị robot khác phủ mất ô -> bỏ, đo lại
                ghiCapKhiDiTheoCong()
            oChuiVao = moveCell(oUngVien, buocTien, doDaiCong - 1)
        else:
            oChuiVao = oUngVien
        viTriQuayVe = self.pos
        huongQuayVe = self.heading

        huongChuiVao = None
        buocCanDi = (oChuiVao[0] - self.pos[0], oChuiVao[1] - self.pos[1])
        for ten, buoc in BUOC_DI.items():
            if buoc == buocCanDi:
                huongChuiVao = ten
        alpha = abs(normalizeAngle(GOC_CUA_HUONG[huongChuiVao] - self.path[-1][2]))
        yield from self._turnTo(huongChuiVao)
        ok = yield from self.stepTo(oChuiVao, 'hole', v=1.0, theta=alpha)
        if not ok:
            return False

        # ---- quét bên trong hole (cùng khung logic với run/runStraight/globalBacktrack) ----
        def runStraightLocal(huongChon):
            if huongChon != 'F':
                yield from self._turnTo(turnRobot(self.heading, huongChon))
            soBuoc = 0
            while soBuoc < 20000:
                soBuoc = soBuoc + 1
                ghiCapCucBo()
                if not conPhaiGhe(self.getCellAt('F')):
                    return None
                hole = phatHienHoleCon('forward')
                if hole is not None:
                    daCan = yield from chuiVaoHoleCon(hole)
                    if daCan:
                        return 'exhausted'
                    continue
                o = self.getCellAt('F')
                ok = yield from self.stepTo(o, 'hole', v=demTrongHole('F'), theta=0.0)
                if not ok:
                    return None
                capQuayLuiCucBo[:] = [(b, u) for (b, u) in capQuayLuiCucBo if u != self.pos]
            return None

        def quayLuiCucBo():
            capQuayLuiCucBo[:] = [(b, u) for (b, u) in capQuayLuiCucBo if conPhaiGhe(u)]
            if len(capQuayLuiCucBo) == 0:
                return False
            dichDen = {b for (b, _) in capQuayLuiCucBo}
            vungDuocDi = {o for o in vungHole if o in self.covered}
            vungDuocDi.add(self.pos)
            duongDi = self.thetaStar(dichDen, vungDuocDi)
            if duongDi is None:
                capQuayLuiCucBo[:] = []
                return False
            self.soLanQuayLuiCucBo += 1
            toiNoi = yield from self.followPath(duongDi)
            if not toiNoi:
                return True
            oCanGhe = None
            for (b, u) in capQuayLuiCucBo:
                if b == self.pos and oCanGhe is None:
                    oCanGhe = u
            if oCanGhe is None:
                return True
            ok = yield from self.enterBranch(oCanGhe)
            self._setState('HOLE')
            if not ok:
                return True
            ghiCapCucBo()
            hole = phatHienHoleCon('forward')
            if hole is not None:
                daCan = yield from chuiVaoHoleCon(hole)
                if daCan:
                    return 'exhausted'
            return True

        ghiCapCucBo()
        demVong = 0
        while demVong < 20000:
            demVong = demVong + 1
            doDai = {h: demTrongHole(h) for h in ('F', 'L', 'R')}
            danhSachHuong = [h for h in ('F', 'L', 'R') if doDai[h] > 0]

            if len(danhSachHuong) > 0:
                huongChon = self.chooseDirection(doDai, danhSachHuong)
                if 'F' not in danhSachHuong and huongChon in ('L', 'R'):
                    hole = phatHienHoleCon('turn', huongChon)
                    if hole is not None:
                        daCan = yield from chuiVaoHoleCon(hole)
                        if daCan:
                            while len(capQuayLuiCucBo) > 0:
                                r = yield from quayLuiCucBo()
                                if r != 'exhausted':
                                    break
                        continue
                ketQua = yield from runStraightLocal(huongChon)
                if ketQua == 'exhausted':
                    while len(capQuayLuiCucBo) > 0:
                        r = yield from quayLuiCucBo()
                        if r != 'exhausted':
                            break
            elif len(capQuayLuiCucBo) > 0:
                ketQua = yield from quayLuiCucBo()
                if not ketQua:
                    break
                while ketQua == 'exhausted' and len(capQuayLuiCucBo) > 0:
                    ketQua = yield from quayLuiCucBo()
            else:
                break

        # ---- kiểm tra q_resume còn việc không ----
        conViec = False
        for huongTuongDoi in ('F', 'L', 'R'):
            huongThat = getAbsoluteDirection(huongQuayVe, huongTuongDoi)
            oKe = moveCell(viTriQuayVe, BUOC_DI[huongThat])
            if self.isFreeUncovered(oKe, viTriQuayVe):
                conViec = True
        if not conViec:
            self.soLanResumeCan += 1
            return True

        for _ in range(50):
            if self.pos == viTriQuayVe:
                break
            duongVe = self.thetaStar({viTriQuayVe}, obstacles=self._obstacles())
            if duongVe is None:
                return True
            yield from self.followPath(duongVe)
        if self.pos != viTriQuayVe:
            return True
        yield from self._turnTo(huongQuayVe)
        return False

    # =====================================================================
    # Vòng lặp chính (Thuật toán 1) — một generator cho mỗi robot
    # =====================================================================
    def _drain(self):
        """Sau khi q_resume cạn: while L != ∅ and globalBacktrack() == 'exhausted'."""
        while len(self.L) > 0:
            r = yield from self.globalBacktrack()
            if r != 'exhausted':
                break

    def behavior(self):
        # xử lý riêng ô sau lưng tại c0 (Thuật toán 1, dòng 8–15)
        doDai, danhSachHuong = self.getFeasibleDirections()
        oPhiaSau = self.getCellAt('B')
        if self.isFreeUncovered(oPhiaSau):
            if len(danhSachHuong) > 0:
                self.L.append((self.pos, oPhiaSau))
            else:
                yield from self._turnTo(turnRobot(self.heading, 'B'))
                doDai, danhSachHuong = self.getFeasibleDirections()
        self.findBacktrackPairs()

        while True:
            self.demVongLap += 1
            doDai, danhSachHuong = self.getFeasibleDirections()
            self.temp_avoid = set()                   # chỉ có hiệu lực cho 1 lần chọn hướng

            if len(danhSachHuong) > 0:
                self._setState('COVER')
                huongChon = self.selectDirection(doDai, danhSachHuong)
                if 'F' not in danhSachHuong and huongChon in ('L', 'R'):
                    hole = self.detectHole('turn', huongChon)
                    if hole is not None:
                        daCan = yield from self.coverHole(hole[0], hole[1], hole[2])
                        if daCan:
                            yield from self._drain()
                        continue
                ketQua = yield from self.runStraight(huongChon)
                if ketQua == 'exhausted':
                    yield from self._drain()
            else:
                # At = ∅: quay lui nếu còn cặp, ngược lại chờ (WAIT) — robot khác có thể mở cặp mới
                if len(self.L) > 0:
                    r = yield from self.globalBacktrack()
                    while r == 'exhausted' and len(self.L) > 0:
                        r = yield from self.globalBacktrack()
                    if r is True:
                        continue
                    if r == 'exhausted':
                        continue
                yield from self._waitStep()

    # =====================================================================
    def getResult(self):
        tong = 0.0
        for i in range(len(self.path) - 1):
            tong += math.hypot(self.path[i + 1][0] - self.path[i][0], self.path[i + 1][1] - self.path[i][1])
        soLanQuay = 0
        for i in range(len(self.path) - 1):
            if abs(normalizeAngle(self.path[i + 1][2] - self.path[i][2])) > 1e-9:
                soLanQuay += 1
        return {
            'id': self.id,
            'path': self.path,
            'trace': self.trace,
            'moves': self.moves,
            'covered': set(self.shared.covered_by[self.id]),
            'L(P)': tong,
            'NT(P)': soLanQuay,
            'global_backtracks': self.soLanQuayLuiToanCuc,
            'holes_covered': self.soHoleDaBaoPhu,
            'resume_exhausted': self.soLanResumeCan,
            'local_backtracks': self.soLanQuayLuiCucBo,
            'cond26': self.soLanDungDieuKien26,
            'wait_steps': self.totalWait,
            'cancelled_backtracks': self.cancelledBacktracks,
            'unreachable': self.unreachable,
        }