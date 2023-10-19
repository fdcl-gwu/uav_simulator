import itertools
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import pdb

# plt.style.use('seaborn')

def plot_data():
    file_name = get_file_name()
    data = Data(file_name)

    t = data.t / 1000.0
    plot_31_2(t, data.x, t, data.xd, title='x', legend=['Estimated', 'Desired'])
    plot_33_2(t, data.R, t, data.Rd, title='R', legend=['Estimated', 'Desired'])

    print('Press Q to close each figure.')
    plt.show()


def get_file_name():
    with open('data_logs/last_log.txt', 'r') as f:
        return f.readline()


def plot_31_1(x, y, x_label='', y_label=['1', '2', '3'], title=''):

    fig, ax = plt.subplots(3, 1)
    fig.suptitle(r'${{{}}}$'.format(title), fontsize=12)

    for i in range(3):
        ax[i].plot(x[:], y[i][:], 'r')
        ax[i].set_ylabel(r'${{{}}}$'.format(y_label[i]))
        ax[i].grid('on')


def plot_31_2(x1, y1, x2, y2, x_label='', y_label=['', '', ''], title='',
              legend=['', '']):

    fig, ax = plt.subplots(3, 1)
    fig.suptitle(r'${{{}}}$'.format(title), fontsize=12)

    for i in range(3):
        ax[i].plot(x1[:], y1[i][:], 'r', label=legend[0])
        ax[i].plot(x2[:], y2[i][:], 'g', label=legend[1])
        ax[i].set_ylabel(r'${{{}}}$'.format(y_label[i]))
        ax[i].set_xlabel(r'${{{}}}$'.format(x_label))
        ax[i].grid('on')

    if not legend[0] == '':
        ax[0].legend()


def plot_33_1(x, y, x_label='', title=''):

    fig, ax = plt.subplots(3, 3)
    fig.suptitle(r'${{{}}}$'.format(title), fontsize=12)

    for i, j in list(itertools.product(range(0, 3), range(0, 3))):
        ax[i, j].plot(x[:], y[i, j, :], 'r')
        ax[i, j].set_ylim((-1.2, 1.2))
        ax[i, j].grid('on')


def plot_33_2(x1, y1, x2, y2, x_label='', title='', legend=['', '']):
    
    fig, ax = plt.subplots(3, 3)
    fig.suptitle(r'${{{}}}$'.format(title), fontsize=12)
    
    for i, j in list(itertools.product(range(0, 3), range(0, 3))):
        ax[i, j].plot(x1[:], y1[i, j, :], 'r', label=legend[0])
        ax[i, j].plot(x2[:], y2[i, j, :], 'g', label=legend[1])
        ax[i, j].set_ylim((-1.2, 1.2))
        ax[i, j].grid('on')
    
    if not legend[0] == '':
        ax[0, 0].legend()


class Data:
    def __init__(self, file_name):
        self.data = pd.read_csv(file_name, header=0, skipinitialspace=True)
        
        self.t_system = self.data['time']
        self.t = self.data['t']
        N = len(self.t)

        self.x = self.get_data_3x1('x')
        self.v = self.get_data_3x1('v')
        self.a = self.get_data_3x1('a')
        self.W = self.get_data_3x1('W')

        self.xd = self.get_data_3x1('xd')
        self.vd = self.get_data_3x1('xd_dot')
        self.b1d = self.get_data_3x1('b1d')

        self.R = self.rot_mat_from_df('R', N)
        self.Rd = self.rot_mat_from_df('Rd', N)

    def get_data_3x1(self, var_name):
        return [self.data['{}_0'.format(var_name)],
                self.data['{}_1'.format(var_name)],
                self.data['{}_2'.format(var_name)]]


    def rot_mat_from_df(self, mat, N):
        R = np.zeros((3, 3, N))

        for i in range(0, N):
            for j, k in list(itertools.product(range(0, 3), range(0, 3))):
                R[j, k, i] = self.data['{}_{}{}'.format(mat, j, k)][i]

        return R

if __name__ == '__main__':
    plot_data()
