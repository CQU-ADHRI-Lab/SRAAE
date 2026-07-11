import matplotlib.pyplot as plt
import numpy as np

# 读取数据文件并绘制曲线
def plot_data(filename, color, label):
    data = np.loadtxt(filename)
    x = data[:, 0]
    y = data[:, 1]
    plt.plot(x, y, color=color, label=label)

# 读取三个数据文件并绘制曲线
plot_data("/home/oseasy/SMMR_topo_ws/src/SMMR-Explore-topoexplore/src/TopoExplore/data_rrt_small.txt", "b", "rrt")
plot_data("/home/oseasy/SMMR_topo_ws/src/SMMR-Explore-topoexplore/src/TopoExplore/data_smmf_small.txt", "g", "smmf")
plot_data("/home/oseasy/SMMR_topo_ws/src/SMMR-Explore-topoexplore/src/TopoExplore/data_Topo_small.txt", "r", "Topo")

# 添加标题和标签，并显示图像
plt.xlabel('Path Length/m')
plt.ylabel('Explore Rate')
plt.title('Explore Rate vs Path Length')
plt.grid(True)
plt.legend()
plt.show()
