import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

df = pd.read_excel('../Path Planning Comparison.xlsx', skiprows=10, usecols='A:H')

# with pd.option_context('display.max_rows', None, 'display.max_columns', None):
#     print(df)

df_scn1 = df.iloc[0:4, 1:].reset_index(drop=True)
df_scn2 = df.iloc[4:8, 1:].reset_index(drop=True)
df_scn3 = df.iloc[8:12, 1:].reset_index(drop=True)


def plot_comparison(df_scn):
    x_label = []
    for i in range(df_scn.shape[0]):
        algorithm = df_scn.loc[i, 'Algorithm']
        success = df_scn.loc[i, 'Average Success (%)']
        label = f'{algorithm}\n{"{:.2f}".format(success)}%'
        x_label.append(label)

    x = df_scn['Algorithm']
    y1 = df_scn['Average Execution Time (ms)']
    y2 = df_scn['Average Path Length (px)']

    n_x = np.arange(len(x))

    bar_width = 0.75

    figure, axis1 = plt.subplots(figsize=(13, 8))

    color = '#1f968b'
    axis1.set_xlabel('Algorithms', fontsize=45, color='white', weight='bold')
    axis1.set_ylabel('Avg. Execution Time (ms)', color=color, fontsize=45, weight='bold')
    axis1.bar(n_x, y1, color=color, alpha=1.0, width=bar_width, label='Avg. Execution Time (ms)')
    axis1.tick_params(axis='y', labelcolor=color)
    axis1.tick_params(axis='both', which='major', labelsize=32, colors='white')
    axis1.set_ylim([min(y1) * 0.4, max(y1) * 1.14])

    for i, v in enumerate(y1):
        axis1.text(i, v + 10, str(round(v, 2)), color=color, weight='bold', ha='center', va='bottom', fontsize=32)

    axis2 = axis1.twinx()
    color = '#ffb703'
    axis2.set_ylabel('Avg. Path Length (px)', color=color, fontsize=45, weight='bold')
    axis2.plot(n_x, y2, color=color, marker='s', markersize=14, linewidth=12, label='Avg. Path Length (px)')
    axis2.tick_params(axis='y', labelcolor=color)
    axis2.tick_params(axis='both', which='major', labelsize=32, colors='white')
    axis2.set_ylim([min(y2) * 0.93, max(y2) * 1.05])

    for i, v in enumerate(y2):
        if i % 2 == 0:
            axis2.text(i, v + 5, str(round(v, 2)), color='white', weight='bold', ha='center', va='bottom', fontsize=32)
        else:
            axis2.text(i, v - 40, str(round(v, 2)), color='white', weight='bold', ha='center', va='bottom', fontsize=32)

    for axis, color in zip([axis1, axis2], ['white', 'white']):
        plt.setp(axis.spines.values(), linewidth=4, color=color)
        plt.setp([axis.get_xticklines(), axis.get_yticklines()], linewidth=4, color=color)

    figure.tight_layout()

    plt.xticks(n_x, x_label, color='white')
    plt.savefig('../plot/Comparison/SCN2.png', format='png', dpi=600, bbox_inches='tight', pad_inches=1.3, transparent=True)
    plt.show()


plot_comparison(df_scn2)
