from matplotlib import pyplot as plt
from matplotlib.widgets import Button
import numpy as np

numplots = 4

fig = plt.figure()

# m_axes = [plt.subplot(201 + 10 * np.ceil(numplots/2.0) + x) for x in range(0, numplots)]

# axes = plt.subplot(221)
# axes2 = plt.subplot(224)
m_axes = [0,0,0,0]
# m_axes[0] = plt.subplot(221)
# m_axes[1] = plt.subplot(222)
# m_axes[2] = plt.subplot(223)
# m_axes[3] = plt.subplot(224)


# print 10 * np.ceil(numplots/2.0)
# m_axes = [plt.subplot(201 + 10 * np.ceil(numplots/2.0) + x) for x in range(0, numplots)]
# for x in range(0, numplots): print str(201 + 10 * np.ceil(numplots/2.0) + x)

for x in range(0, numplots):
	a = 201 + 10 * np.ceil(numplots/2.0) + x
	print a
	# m_axes[x] = plt.subplot(201 + 10 * np.ceil(numplots/2.0) + x)
	m_axes[x] = plt.subplot(int(a))


button_connect = Button(plt.axes([0.1, 0.05, 0.15, 0.075]), 'Connect')

plt.show()