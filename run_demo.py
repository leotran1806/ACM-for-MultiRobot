# import matplotlib.pyplot as plt
# from ACM_LOS_FINAL import ACM_LOS, getFreeCells

# # 1. Khởi tạo và chạy thuật toán
# grid = [
#     [1,1,1,1,1,1,1,1,1,1],
#     [1,0,1,0,0,0,0,0,0,1],
#     [1,0,1,0,0,0,0,0,0,1],
#     [1,0,0,0,0,0,0,0,0,1],
#     [1,0,0,0,0,0,0,0,0,1],
#     [1,0,0,0,0,0,0,0,0,1],
#     [1,0,0,0,0,0,0,0,0,1],
#     [1,1,1,1,1,1,1,1,1,1],
# ]

# robot = ACM_LOS(grid, start=(1, 1), rho=3.0, huongBanDau='N')
# ketQua = robot.run()

# # 2. Xử lý đường đi và vẽ mũi tên hướng di chuyển
# path = ketQua['path']

# plt.figure(figsize=(8, 7))
# plt.imshow(grid, cmap='binary', origin='upper')

# da_tham = set()

# for i in range(len(path) - 1):
#     p1 = path[i]      # (dòng, cột)
#     p2 = path[i+1]    # (dòng, cột)
#     da_tham.add(p1)
    
#     # Tọa độ X=cột, Y=dòng
#     x1, y1 = p1[1], p1[0]
#     x2, y2 = p2[1], p2[0]
    
#     # Xác định loại bước đi để chọn màu sắc & kiểu nét
#     if p2 in da_tham:
#         if abs(y1 - y2) + abs(x1 - x2) > 1:
#             mau = 'tab:red'
#             kieu_net = 'dashed'
#             nhan = 'Chuyen Hole'
#         else:
#             mau = 'tab:orange'
#             kieu_net = 'solid'
#             nhan = 'Backtrack'
#     else:
#         mau = 'tab:blue'
#         kieu_net = 'solid'
#         nhan = 'Bao phu'

#     # Lấy nhãn cho Legend (tránh lặp lại)
#     label_to_use = nhan if nhan not in plt.gca().get_legend_handles_labels()[1] else ""

#     # Vẽ đường nối
#     plt.plot([x1, x2], [y1, y2], linestyle=kieu_net, color=mau, linewidth=1.5, label=label_to_use)

#     # Vẽ mũi tên hướng di chuyển ở giữa đoạn thẳng
#     x_mid = (x1 + x2) / 2
#     y_mid = (y1 + y2) / 2
    
#     plt.annotate(
#         '', 
#         xy=(x_mid + (x2 - x1) * 0.1, y_mid + (y2 - y1) * 0.1), # Điểm đầu mũi tên
#         xytext=(x_mid, y_mid),                              # Điểm gốc mũi tên
#         arrowprops=dict(
#             arrowstyle="-|>", 
#             color=mau, 
#             lw=1.5, 
#             mutation_scale=12  # Kích thước đầu mũi tên
#         )
#     )

# # Đánh dấu điểm Đầu & Kết thúc
# plt.plot(path[0][1], path[0][0], 'go', markersize=10, label='Bat dau')
# plt.plot(path[-1][1], path[-1][0], 'ro', markersize=10, label='Ket thuc')

# plt.title('ACM-LOS L(P)=%.2f  NT(P)=%d' % (ketQua['L(P)'], ketQua['NT(P)']))
# plt.legend(loc='upper right')
# plt.savefig('duong_di_acm_los_mui_ten.png', dpi=150)
# plt.show()

import matplotlib.pyplot as plt
from ACM_LOS_FINAL import ACM_LOS, ACM_OFS, getFreeCells

# 1. Khởi tạo bản đồ dùng chung cho cả 2 phương pháp cảm biến

## Map 1: Map demo -> để start position là (1,1)
# grid = [
#     [1,1,1,1,1,1,1,1,1,1],
#     [1,0,0,0,0,0,0,0,0,1],
#     [1,0,1,1,0,0,1,0,0,1],
#     [1,0,1,0,0,0,1,0,0,1],
#     [1,0,0,0,1,0,0,0,0,1],
#     [1,0,0,0,1,0,1,1,0,1],
#     [1,0,1,0,0,0,0,0,0,1],
#     [1,1,1,1,1,1,1,1,1,1],
# ]

# ## Map 2: so sánh cách hai phương pháp ACM-LOS và ACM-OFS phát hiện hole + bao phủ hole có điều kiện 26 kích hoạt
# grid = [
#     [1, 1, 1, 1, 1, 1, 1, 1, 1, 1], # Dòng 0: Biên trên
#     [1, 1, 0, 0, 0, 1, 1, 1, 1, 1], # Dòng 1: Phần bên trong của Hole (vùng chưa quét)
#     [1, 1, 0, 0, 0, 1, 1, 1, 1, 1], # Dòng 2: Cổng của Hole (có 3 ô trống, bị chặn ở ô thứ 4)
#     [1, 1, 0, 0, 0, 1, 1, 1, 1, 1], # Dòng 3: Tuyến đường chính. Robot ở vị trí (3,2)
#     [1, 1, 1, 1, 1, 1, 1, 1, 1, 1], # Dòng 4: Biên dưới
#     [1, 1, 1, 1, 1, 1, 1, 1, 1, 1], # Dòng 5
#     [1, 1, 1, 1, 1, 1, 1, 1, 1, 1], # Dòng 6
#     [1, 1, 1, 1, 1, 1, 1, 1, 1, 1]  # Dòng 7
# ]

## Map 3: cho thấy backtracking chọn đường đi tốt nhất
# grid = [
#     [1,1,1,1,1,1,1,1,1,1],
#     [1,0,1,0,0,0,1,1,1,1],
#     [1,0,1,0,0,0,1,1,1,1],
#     [1,0,1,0,0,0,1,1,1,1],
#     [1,0,0,0,0,0,1,1,1,1],
#     [1,1,0,1,1,1,1,1,1,1],
#     [1,1,0,0,0,0,0,0,0,1],
#     [1,1,1,1,1,1,1,1,1,1],
# ]
# Map 4: Hole trong hole
# grid = [
#     [1,1,1,1,1,1,1,1,1,1],
#     [1,0,1,0,0,0,1,1,1,1],
#     [1,0,1,0,0,0,1,1,1,1],
#     [1,0,1,0,1,1,0,1,1,1],
#     [1,0,0,0,0,0,0,1,1,1],
#     [1,0,1,1,1,1,1,1,1,1],
#     [1,0,0,0,0,0,0,0,0,1],
#     [1,1,1,1,1,1,1,1,1,1],
# ]
# Map 5: Map phức tạp
grid = [
    [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],  # dong 0
    [1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1],  # dong 1
    [1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1],  # dong 2
    [1,0,0,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,0,0,1],  # dong 3
    [1,0,0,1,1,1,1,1,1,1,0,1,0,0,0,1,0,0,1,0,0,1],  # dong 4
    [1,0,0,1,1,0,0,0,1,0,0,0,0,0,0,0,0,0,1,0,0,1],  # dong 5
    [1,0,0,1,1,0,0,1,1,1,0,1,0,0,0,1,0,0,1,0,0,1],  # dong 6
    [1,0,0,1,1,0,0,0,1,0,0,1,1,1,1,1,1,1,1,0,1,1],  # dong 7
    [1,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1],  # dong 8
    [1,0,0,1,1,0,1,1,1,1,1,1,1,1,0,1,1,1,1,1,0,1],  # dong 9
    [1,0,0,0,0,0,1,1,1,1,1,1,1,1,0,1,0,0,0,1,0,1],  # dong 10
    [1,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,1,0,1,0,1],  # dong 11
    [1,0,0,1,1,1,1,0,0,1,1,0,0,1,1,1,0,1,0,1,0,1],  # dong 12
    [1,0,0,1,0,0,0,0,0,1,1,0,0,1,1,1,0,0,0,1,0,1],  # dong 13
    [1,0,0,1,0,1,1,0,0,1,1,0,0,0,0,1,1,1,1,1,0,1],  # dong 14
    [1,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1],  # dong 15
    [1,0,0,1,1,1,1,0,1,0,1,0,1,0,1,0,1,0,0,1,0,1],  # dong 16
    [1,0,1,1,1,1,0,0,1,0,1,0,1,0,1,0,1,0,0,1,0,1],  # dong 17
    [1,0,1,0,0,1,0,0,1,1,1,1,1,1,1,1,1,1,0,1,0,1],  # dong 18
    [1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1],  # dong 19
    [1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1],  # dong 20
    [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],  # dong 21
]
START = (20, 1)   # góc dưới trái, giống chấm xanh trong hình
HUONG_BAN_DAU = 'N'
RHO = 3.0         # thử thêm 4.0 hoặc 5.0 để thấy hole lồng nhau rõ hơn

# 2. Chạy cả 2 phương pháp: ACM-LOS (line of sight) và ACM-OFS (occlusion-free sensing)
robotLOS = ACM_LOS(grid, start=START, rho=RHO, huongBanDau=HUONG_BAN_DAU)
ketQuaLOS = robotLOS.run()

robotOFS = ACM_OFS(grid, start=START, rho=RHO, huongBanDau=HUONG_BAN_DAU)
ketQuaOFS = robotOFS.run()


# 3. Hàm vẽ dùng chung cho 1 subplot: vẽ đường đi với mũi tên hướng + màu phân biệt
#    Bao phu (xanh dương, solid) / Backtrack 1 buoc (cam, solid) / Chuyen Hole - quay lui xa (do, dashed)
def plotPath(ax, ketQua, tieuDe):
    path = ketQua['path']
    
    rows = len(grid)
    cols = len(grid[0])

    # 1. Hiển thị ma trận với extent để căn chỉnh ranh giới ô từ -0.5 đến cols-0.5
    ax.imshow(grid, cmap='binary', origin='upper', extent=[-0.5, cols - 0.5, rows - 0.5, -0.5])

    # 2. Tạo đường kẻ caro ngang và dọc tại ranh giới các ô
    ax.set_xticks([x - 0.5 for x in range(cols + 1)])
    ax.set_yticks([y - 0.5 for y in range(rows + 1)])
    
    # Đặt nhãn trục tọa độ chính xác vào tâm của mỗi ô
    ax.set_xticks(range(cols), minor=True)
    ax.set_yticks(range(rows), minor=True)
    ax.xaxis.set_major_formatter(plt.NullFormatter())
    ax.yaxis.set_major_formatter(plt.NullFormatter())
    ax.xaxis.set_minor_formatter(plt.FormatStrFormatter('%d'))
    ax.yaxis.set_minor_formatter(plt.FormatStrFormatter('%d'))

    # Bật gridlines màu xám/đen mảnh bám theo các vạch chia chính (Major ticks)
    ax.grid(True, which='major', color='gray', linestyle='-', linewidth=0.8)
    ax.tick_params(which='both', length=0) # Ẩn gai vạch chia để giao diện gọn gàng

    da_tham = set()

    for i in range(len(path) - 1):
        p1 = path[i]      # (dòng, cột, góc)
        p2 = path[i + 1]  # (dòng, cột, góc)
        da_tham.add((p1[0], p1[1]))

        # Tọa độ X=cột, Y=dòng
        x1, y1 = p1[1], p1[0]
        x2, y2 = p2[1], p2[0]

        # Bỏ qua các bước chỉ quay (không đổi ô) khi vẽ đường/mũi tên di chuyển
        if (x1, y1) == (x2, y2):
            continue

        # Xác định loại bước đi để chọn màu sắc & kiểu nét
        if (p2[0], p2[1]) in da_tham:
            mau = 'tab:orange'
            kieu_net = 'solid'
            nhan = 'Backtrack'
        else:
            mau = 'tab:blue'
            kieu_net = 'solid'
            nhan = 'Bao phu'

        # Lấy nhãn cho Legend (tránh lặp lại)
        nhan_da_dung = ax.get_legend_handles_labels()[1]
        label_to_use = nhan if nhan not in nhan_da_dung else ""

        # Vẽ đường nối
        ax.plot([x1, x2], [y1, y2], linestyle=kieu_net, color=mau, linewidth=1.5, label=label_to_use)

        # Vẽ mũi tên hướng di chuyển ở giữa đoạn thẳng
        x_mid = (x1 + x2) / 2
        y_mid = (y1 + y2) / 2

        ax.annotate(
            '',
            xy=(x_mid + (x2 - x1) * 0.1, y_mid + (y2 - y1) * 0.1),
            xytext=(x_mid, y_mid),
            arrowprops=dict(
                arrowstyle="-|>",
                color=mau,
                lw=1.5,
                mutation_scale=12
            )
        )

    # Đánh dấu điểm Đầu & Kết thúc
    ax.plot(path[0][1], path[0][0], 'go', markersize=10, label='Bat dau')
    ax.plot(path[-1][1], path[-1][0], 'ro', markersize=10, label='Ket thuc')

    ax.set_title('%s\nL(P)=%.2f  NT(P)=%d' % (tieuDe, ketQua['L(P)'], ketQua['NT(P)']))
    ax.legend(loc='upper right', fontsize=8)

# 5. In bảng so sánh kết quả 2 phương pháp
soOTrong = len(getFreeCells(grid))
print("\n=== So sanh ACM-LOS vs ACM-OFS ===")
print(f"{'Chi so':<20}{'LOS':>12}{'OFS':>12}")
print(f"{'L(P) - quang duong':<20}{ketQuaLOS['L(P)']:>12.2f}{ketQuaOFS['L(P)']:>12.2f}")
print(f"{'NT(P) - so lan quay':<20}{ketQuaLOS['NT(P)']:>12d}{ketQuaOFS['NT(P)']:>12d}")
print(f"{'O da bao phu':<20}{len(ketQuaLOS['covered']):>10d}/{soOTrong}{len(ketQuaOFS['covered']):>10d}/{soOTrong}")
print(f"{'Quay lui toan cuc':<20}{ketQuaLOS['global_backtracks']:>12d}{ketQuaOFS['global_backtracks']:>12d}")
print(f"{'Quay lui cuc bo':<20}{ketQuaLOS['local_backtracks']:>12d}{ketQuaOFS['local_backtracks']:>12d}")
print(f"{'So hole da bao phu':<20}{ketQuaLOS['holes_covered']:>12d}{ketQuaOFS['holes_covered']:>12d}")
print(f"{'Resume can het viec':<20}{ketQuaLOS['resume_exhausted']:>12d}{ketQuaOFS['resume_exhausted']:>12d}")
print(f"So lan quay lui cuc bo cua ACM-LOS: {ketQuaLOS['local_backtracks']}")
print(f"So lan quay lui cuc bo cua ACM-OFS: {ketQuaOFS['local_backtracks']}")
print(f"So lan quay lui toan cuc cuar ACM-LOS: {ketQuaLOS['global_backtracks']}")
print(f"So lan quay lui toan cuc cuar ACM-OFS: {ketQuaOFS['global_backtracks']}")
# 4. Vẽ 2 subplot cạnh nhau: ACM-LOS và ACM-OFS
fig, (axLOS, axOFS) = plt.subplots(1, 2, figsize=(15, 7))
plotPath(axLOS, ketQuaLOS, 'ACM-LOS (Line of Sight)')
plotPath(axOFS, ketQuaOFS, 'ACM-OFS (Occlusion-Free Sensing)')

plt.tight_layout()
plt.savefig('duong_di_acm_los_vs_ofs.png', dpi=150)
plt.show()



