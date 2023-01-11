import matplotlib.pyplot as plt
import os
from datetime import datetime
import numpy as np

dirname = os.path.dirname
ABPATH = os.path.join(dirname(dirname(__file__))) + '/MITL_communication_centralized'
now = '2022-09-02'
fig_folder = ABPATH + '/figures/' + str(now)
if not (os.path.exists(fig_folder)):
    os.makedirs(fig_folder)

x_axis = ['3*2', '3*3', '4*3', '4*4', '5*5', '10*10']
label_e = ['2 cen solu', '2 de solu', '4 de solu']
label_s = ['2 cen syn', '2 de syn', '4 de syn']
color_list = ['red', 'blue', 'green']
text_loc_list_e = [10000, -4000, 6000]
text_loc_list_s = [50, -4, 6]
text_loc = ['right', 'left', 'left']
ave_e = [[] for i in range(3)]
ave_e[0] = [7.4, 26.9, 114.3, 612.3, np.nan, np.nan]
ave_e[1] = [12.0, 12.5, 12.7, 12.8, 12.1, 12.0]
ave_e[2] = [24.5, 25.6, 25.5, 26.2, 27.4, 30.1]
for i in range(3):
    ave_e[i] = [a*1000 for a in ave_e[i]]

ave_e = np.array(ave_e)

std_e = [[] for i in range(3)]
std_e[0] = [0.2, 3.1, 5.0, 17.5, np.nan, np.nan]
std_e[1] = [0.2, 0.2, 0.6, 1.3, 1.7, 1.6]
std_e[2] = [0.6, 1.2, 0.7, 1.3, 1.4, 1.3]
for i in range(3):
    std_e[i] = [a*1000 for a in std_e[i]]

std_e = np.array(std_e)


ave_s = [[] for i in range(3)]
ave_s[0] = [87.3, 495.8, 1834.4, 2888.2, np.nan, np.nan]
ave_s[1] = [5.2, 6.6, 6.7, 7.8, 9.2, 16.2]
ave_s[2] = [5.7, 8.62, 9.6, 14.5, 26.2, 46.5]

ave_s = np.array(ave_s)

std_s = [[] for i in range(3)]
std_s[0] = [63.2, 244.8, 949.2, 519.4, np.nan, np.nan]
std_s[1] = [0.7, 0.9, 1.3, 2.1, 3.1, 5.1]
std_s[2] = [0.8, 2.8, 4.6, 10.4, 16.2, 15.4]

std_s = np.array(std_s)

fig, ax = plt.subplots()
fig.set_figwidth(10)
fig.set_figheight(6)
for i in range(len(ave_s)):
    x_axis = ['3*2', '3*3', '4*3', '4*4', '5*5', '10*10']
    # x_axis = [6, 9, 12, 16, 25, 100]
    ax.set_ylim(1E0, 1E6)
    ax.set_title('Average Time over 50 simulations', fontsize=16)
    ax.set_xlabel('Work Region Size', fontsize=14)
    ax.set_ylabel('Time (ms)', fontsize=14)
    plt.yticks(fontsize=12)
    plt.xticks(fontsize=12)
    ax.errorbar(x_axis, ave_e[i], std_e[i], marker='.', color=color_list[i], label=label_e[i])
    ax.errorbar(x_axis, ave_s[i], std_s[i], marker='^', color=color_list[i], label=label_s[i])
    # ax.loglog()
    ax.set_yscale('log')

    li_e = ave_e[i].tolist()
    for x_axis, li_e in zip(x_axis, li_e):
        ax.annotate('{:.1e}'.format(li_e), xy=(x_axis, li_e+text_loc_list_e[i]), xytext=(0, 0), ha=text_loc[i],
                    color=color_list[i], textcoords='offset points', size=10)
    x_axis = ['3*2', '3*3', '4*3', '4*4', '5*5', '10*10']
    li_s = ave_s[i].tolist()
    for x_axis, li_s in zip(x_axis, li_s):
        ax.annotate('{}'.format(li_s), xy=(x_axis, li_s+text_loc_list_s[i]), xytext=(0, 0), ha=text_loc[i],
                    color=color_list[i], textcoords='offset points', size=10)

ax.legend(loc='best', fontsize=10)
plt.savefig(fig_folder + "/" + "Average execution time.png", dpi=300)
plt.show()


# x = [0, 1, 2, 3, 4, 5, 6]
# y_cen = [1, 1, 1, 1, 1, 1, 1]
# y_de = [1, 1, 1, 1, 1, 1, 1]
# fig, ax = plt.subplots()
# fig.set_figwidth(8)
# fig.set_figheight(5)
# ax.set_ylim(0, 1.1)
# ax.set_title(r'% of obtaining a solution ($3 \times 3$)', fontsize=16)
# # ax.set_xlabel('Work Region Size', fontsize=14)
# ax.set_xlabel(r'$\Delta = t_{start\_collab/meet} - t_{delivery}$', fontsize=14)
# ax.set_ylabel('%', fontsize=14)
# plt.yticks(fontsize=12)
# plt.xticks(fontsize=12)
# ax.plot(x, y_cen, '--', linewidth=4, color='grey', label='Centralized')
# ax.plot(x, y_de, 'o-', color='blue', label='Decentralized')
# ax.legend(loc='best', fontsize=12)
# plt.savefig(fig_folder + "/" + "Percentage.png", dpi=100)
# plt.show()

