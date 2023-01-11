from datetime import datetime
import os
import statistics

dirname = os.path.dirname
ABPATH = os.path.join(dirname(dirname(__file__))) + '/MITL_communication_centralized'

region_list = [[3, 2], [3, 3], [4, 3], [4, 4], [5, 5], [10, 10]]
region_row, region_col = region_list[5]

"""create the folder for the fig, file, output(video)"""
now = datetime.today().strftime('%Y-%m-%d')
fig_folder = ABPATH + '/figures/' + str(now)
file_folder = ABPATH + '/files/' + str(now)
output_folder = ABPATH + '/files/' + str(now) + '/output'

data = []
generate = []
synthesis = []
filename = file_folder + '/output_revise_paper/' + 'Time_de_' + str(region_row) + "*" + str(region_col)
with open(filename + ".txt") as f:
    lines = f.readlines()
    for i in range(len(lines)):
        data.append([int(n) for n in lines[i].split(',') if n != 'True' and n != 'False' and '\n' not in n])
        if 'de' in filename:
            if data[i][0] < 5000:
                generate.append(data[i][0])
            if data[i][1] < 5000:
                synthesis.append(data[i][1])
            if data[i][2] < 5000:
                generate.append(data[i][2])
            if data[i][3] < 5000:
                synthesis.append(data[i][3])
        else:
            generate.append(data[i][0])
            synthesis.append(data[i][1])
            generate.append(data[i][2])
            synthesis.append(data[i][3])
ave_generate = statistics.mean(generate)
std_generate = statistics.stdev(generate)
ave_synthesis = statistics.mean(synthesis)
std_synthesis = statistics.stdev(synthesis)

with open(filename + "_ave_std.txt", "w") as f:
    f.write(f"(gene)ave-std-(syn)ave-std\n")
    f.write(f"{ave_generate}\n")
    f.write(f"{std_generate}\n")
    f.write(f"{ave_synthesis}\n")
    f.write(f"{std_synthesis}\n")
    f.close()
