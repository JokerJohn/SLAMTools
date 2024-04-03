import matplotlib.pyplot as plt
import pandas as pd
from matplotlib.ticker import AutoMinorLocator
import matplotlib.ticker as ticker

# 读取数据
file_path_1 = 'data_time.txt'
file_path_2 = 'data_time_o3d.txt'
data_1 = pd.read_csv(file_path_1, sep='\s+', header=None)
data_2 = pd.read_csv(file_path_2, sep='\s+', header=None)
# 处理数据
data_1['Total'] = data_1[1] + data_1[2] + data_1[3] + data_1[4]
data_2['Total'] = data_2[1] + data_2[2] + data_2[3] + data_2[4]

## 设置图表样式
plt.style.use('seaborn-whitegrid')
plt.rcParams['font.family'] = 'Times New Roman'
plt.rcParams['axes.edgecolor'] = 'black'
plt.rcParams['axes.linewidth'] = 1.2
plt.rcParams['grid.color'] = 'grey'
plt.rcParams['grid.linestyle'] = '--'
plt.rcParams['grid.linewidth'] = 0.6

# 准备数据
df_avg_1 = data_1[2].mean()
opt_avg_1 = data_1[4].mean()
df_avg_2 = data_2[2].mean()
opt_avg_2 = data_2[4].mean()

# 绘制第一个算法的图表
fig, ax = plt.subplots(figsize=(10, 8))
ax.fill_between(data_1[0], 0, data_1[2], label='DF Time', step='post', color='green', alpha=0.8)
ax.fill_between(data_1[0], data_1[2], data_1['Total'], label='Graph Optimization', step='post', color='red', alpha=0.8)
ax.axhline(y=df_avg_1, color='darkgreen', linestyle='--', label=f'Avg DF: {df_avg_1:.2f} ms')
ax.axhline(y=df_avg_1 + opt_avg_1, color='darkred', linestyle='--', label=f'Avg Graph Optimization: {opt_avg_1:.2f} ms')
ax.set_title('Time Analysis of PFL2')
ax.set_xlabel('Timestamp [s]')
ax.set_ylabel('Time [ms]')
ax.legend(edgecolor='black', facecolor='none', framealpha=1, markerscale=1, frameon=True)

# 添加平均值标记
ax.text(1.05, df_avg_1 / ax.get_ylim()[1], f'{df_avg_1:.2f}', transform=ax.transAxes, color='darkgreen')
ax.text(1.05, (df_avg_1 + opt_avg_1) / ax.get_ylim()[1], f'{df_avg_1 + opt_avg_1:.2f}', transform=ax.transAxes, color='darkred')

plt.savefig("algorithm_time_analysis1.pdf", format='pdf', bbox_inches='tight', dpi=300)


# 绘制第二个算法的图表
fig, ax = plt.subplots(figsize=(10, 8))
ax.fill_between(data_2[0], 0, data_2[2], label='DF Time', step='post', color='green', alpha=0.8)
ax.fill_between(data_2[0], data_2[2], data_2['Total'], label='Graph Optimization', step='post', color='red', alpha=0.8)
ax.axhline(y=df_avg_2, color='darkgreen', linestyle='--', label=f'Avg DF: {df_avg_2:.2f} ms')
ax.axhline(y=df_avg_2 + opt_avg_2, color='darkred', linestyle='--', label=f'Avg Graph Optimization: {opt_avg_2:.2f} ms')
ax.set_title('Time Analysis of ICP')
ax.set_xlabel('Timestamp [s]')
ax.set_ylabel('Time [ms]')
ax.legend(edgecolor='black', facecolor='none', framealpha=1, markerscale=1, frameon=True)

# 添加平均值标记
ax.text(1.05, df_avg_2 / ax.get_ylim()[1], f'{df_avg_2:.2f}', transform=ax.transAxes, color='green')
ax.text(1.05, (df_avg_2 + opt_avg_2) / ax.get_ylim()[1], f'{df_avg_2 + opt_avg_2:.2f}', transform=ax.transAxes, color='darkred')

plt.savefig("algorithm_time_analysis2.pdf", format='pdf', bbox_inches='tight', dpi=300)


# 显示图表
plt.show()




