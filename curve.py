# import libraries
from matplotlib import pyplot as plt
from matplotlib import style
import numpy as np
import csv
import sys

# constant for running average filter
filt = 10

def dependencies_for_freezing():
    import matplotlib.numerix
    import matplotlib.numerix.random_array
    import Tkinter
    import FileDialog

style.use('ggplot')

plt.rcParams.update({'font.size': 14})

reader = csv.reader(open(str(sys.argv[1]), "rb"), delimiter=',')
x=list(reader)
data=np.array(x)

tps = data[0,0:3].astype(str)
identify = data[1,0:3].astype(str)

label = data[2,0:3].astype(str)



VG1 = data[3:503, 0].astype(float)
VD1 = data[3:503, 1].astype(float)
ID1 = data[3:503, 2].astype(float)

VD1 = np.convolve(VD1,np.ones((filt,))/filt)
ID1 = np.convolve(ID1,np.ones((filt,))/filt)

VG2 = data[503:1003, 0].astype(float)
VD2 = data[503:1003, 1].astype(float)
ID2 = data[503:1003, 2].astype(float)

VD2 = np.convolve(VD2,np.ones((filt,))/filt)
ID2 = np.convolve(ID2,np.ones((filt,))/filt)

VG3 = data[1003:1503, 0].astype(float)
VD3 = data[1003:1503, 1].astype(float)
ID3 = data[1003:1503, 2].astype(float)

VD3 = np.convolve(VD3,np.ones((filt,))/filt)
ID3 = np.convolve(ID3,np.ones((filt,))/filt)

VG4 = data[1503:2003, 0].astype(float)
VD4 = data[1503:2003, 1].astype(float)
ID4 = data[1503:2003, 2].astype(float)

VD4 = np.convolve(VD4,np.ones((filt,))/filt)
ID4 = np.convolve(ID4,np.ones((filt,))/filt)

VG5 = data[2003:2503, 0].astype(float)
VD5 = data[2003:2503, 1].astype(float)
ID5 = data[2003:2503, 2].astype(float)

VD5 = np.convolve(VD5,np.ones((filt,))/filt)
ID5 = np.convolve(ID5,np.ones((filt,))/filt)

VG6 = data[2503:3003, 0].astype(float)
VD6 = data[2503:3003, 1].astype(float)
ID6 = data[2503:3003, 2].astype(float)

VD6 = np.convolve(VD6,np.ones((filt,))/filt)
ID6 = np.convolve(ID6,np.ones((filt,))/filt)

#identify = data[0, 0:3].astype(str);

p1, = plt.plot(VD1[10:500],ID1[10:500], linewidth=3.0)
p2, = plt.plot(VD2[10:500],ID2[10:500], linewidth=3.0)
p3, = plt.plot(VD3[10:500],ID3[10:500], linewidth=3.0)
p4, = plt.plot(VD4[10:500],ID4[10:500], linewidth=3.0)
p5, = plt.plot(VD5[10:500],ID5[10:500], linewidth=3.0)
p6, = plt.plot(VD6[10:500],ID6[10:500], linewidth=3.0)

plt.title('Curve Trace')
plt.suptitle(tps[0]+' '+tps[1]+' '+identify[0]+' '+identify[1]+' '+identify[2]);
plt.ylabel(label[2])
plt.xlabel(label[1])
plt.grid(True)

p6lab = "%s = %.1f" % (label[0], VG6[0])
p5lab = "%s = %.1f" % (label[0], VG5[0])
p4lab = "%s = %.1f" % (label[0], VG4[0])
p3lab = "%s = %.1f" % (label[0], VG3[0])
p2lab = "%s = %.1f" % (label[0], VG2[0])
p1lab = "%s = %.1f" % (label[0], VG1[0])


legend = plt.legend([p6, p5, p4, p3, p2, p1], [p6lab, p5lab, p4lab, p3lab, p2lab, p1lab], loc='upper right', shadow=True)

axes = plt.gca()
axes.set_xlim([0,4.85])

mng = plt.get_current_fig_manager()
mng.resize(*mng.window.maxsize())

plt.show()
