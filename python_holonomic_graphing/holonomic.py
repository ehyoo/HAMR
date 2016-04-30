import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import pdb

fig = plt.figure(figsize=(15,8), facecolor='#E1E6E8')
fig.subplots_adjust(top=.85, wspace=.2, hspace=.4)

xdot = .7
ydot = .7
tdot = 0

a = 1.0
b = 1.0
r = 1.0

# t = np.arange(0, 2*np.pi, .01)
t = np.arange(0, np.pi / 2.0, .01)
# print t

sint = np.sin(t)
cost = np.cos(t)

b_s = b * sint
b_c = b * cost
a_s = a * sint
a_c = a * cost

m1 = (xdot*(-b_s - a_c) + ydot*(b_c - a_s)) / (r*b)
m2 = (xdot*(-b_s + a_c) + ydot*(b_c + a_s)) / (r*b)
# pdb.set_trace()

mt = (-xdot * cost - ydot * sint) / b - tdot

lines = [0,0,0,0]

ax1 = fig.add_subplot(221)
lines[0] = ax1.plot(t, m1)
ax1.set_title(r'$\dot{\theta}_{xr1}$', fontsize=20)
ax1.set_xlabel(r'$\theta_s$', fontsize=15)
ax1.set_ylim(-1.5, 1.5)

ax2 = fig.add_subplot(222)
lines[1] = ax2.plot(t, m2)
ax2.set_title(r'$\dot{\theta}_{xr2}$', fontsize=20) 
ax2.set_xlabel(r'$\theta_s$', fontsize=15)
ax2.set_ylim(0, 1.5)

ax3 = fig.add_subplot(223)
lines[2] = ax3.plot(t, mt)
ax3.set_title(r'$\dot{\theta}_{s}$', fontsize=20) 
ax3.set_xlabel(r'$\theta_s$', fontsize=15)
ax3.set_ylim(-1.5, 1.5)

ax4 = fig.add_subplot(224)
lines[3] = ax4.plot(m1, m2)
ax4.set_title(r'$\dot{\theta}_{xr1}$ vs $\dot{\theta}_{xr1}$', fontsize=20) 
ax4.set_xlabel(r'$\theta_s$', fontsize=15)
ax4.set_ylim(-1.5, 1.5)

fig_title = "Holonomic Actuator Velocities |  " + r'$\frac{a}{b} = $' + str(a/b)
title = plt.suptitle(fig_title, fontsize=25)
# plt.title(fig_title, y=1.08)
# plt.suptitle(fig_title, y=1.12)
# plt.figtext(.1,.1, r'$\frac{a}{b} = 1$', fontsize=15)
# title = plt.subplots_adjust(top=.2)
# title.set_position([.5, 1.05])

# ax2  = plt.subplot(1,2,2)

# plt.text(0.5, .5, fig_title,
#          horizontalalignment='center',
         # fontsize=20)

# plt.tight_layout()







def init_blit():
    for j in range(len(lines)):
        lines[j][0].set_ydata([])
        lines[j][0].set_xdata([])
        # m_axes[j].set_xlim(-10, 0)

    return tuple([line[0] for line in lines])

b = 0
def update_animation(data):
	global b
	b += 1
	b_s = b * sint
	b_c = b * cost
	m1 = (xdot*(-b_s - a_c) + ydot*(b_c - a_s)) / (r*b)
	m2 = (xdot*(-b_s + a_c) + ydot*(b_c + a_s)) / (r*b)
	mt = (-xdot * cost - ydot * sint) / b - tdot
	print len(m1)
	print len(m2)
	print len(mt)

	lines[0][0].set_ydata(m1)
	lines[1][0].set_ydata(m2)
	lines[2][0].set_ydata(mt)
	lines[3][0].set_xdata(m2)
	lines[3][0].set_ydata(m2)

	fig_title = "Holonomic Actuator Velocities |  " + r'$\frac{a}{b} = $' + str(a/b)
	title = plt.suptitle(fig_title, fontsize=25)
	
	if b > 5 : b = 0

	return tuple([line[0] for line in lines])


# anim = animation.FuncAnimation(fig, update_animation, interval=2000, init_func=init_blit, blit=True)

plt.show()
