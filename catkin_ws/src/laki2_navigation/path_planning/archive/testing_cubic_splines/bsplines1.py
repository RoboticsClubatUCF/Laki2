from scipy.interpolate import BSpline, make_interp_spline
import numpy as np
import matplotlib.pyplot as plt

# Parametric Variable
t = np.linspace(0, 2*np.pi, 5)
t_dense = np.linspace(0, 2.*np.pi, 100)

# We are creating a circle
x = np.cos(t)
y = np.sin(t)

dx = 0
dy = 0
while(1):
    # l and r are left and right boundary conditions
    # As far as I can tell it means something along the lines of this: 
    #   1st derivative, (dx/dt, dy/dt))
    # For a parametric euation in general: dy/dx = dy/dt / dx/dt. 
    l, r = [(1, (dx, dy))], 'natural'

    # Set a spline with the boundary conditions
    spl_with_bc = make_interp_spline(t, np.c_[x, y], bc_type=(l, r))

    # plot the new splined with controlled boundary conditions
    x_new, y_new = spl_with_bc(t_dense).T
    plt.figure(1)
    plt.plot(x, y, 'o')
    plt.plot(x_new, y_new, '-')
    plt.show(block=False)

    print "Enter dx: "
    dx = raw_input("")
    print ""

    print "Enter dy: "
    dy = raw_input("")
    print ""