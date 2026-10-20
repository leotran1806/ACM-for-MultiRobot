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