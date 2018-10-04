from scipy.interpolate import BSpline, make_interp_spline
import numpy as np
import matplotlib.pyplot as plt

# Parametric Variable
t = np.linspace(0, 2*np.pi, 5)
t_dense = np.linspace(0, 2.*np.pi, 100)

# We are creating a circle
x = np.cos(t)
y = np.sin(t)

# Initialize slopes
dx = 0
dy = 0
# Bad practice, but just for a demo. ^C to quit progam
while(1):
    # l and r are left and right boundary conditions
    # As far as I can tell it means something along the lines of this: 
    #   1st derivative, (dx/dt, dy/dt))

    # General form for slope of a parametric equation is dy/dx = dy/dt / dx/dt.
    # Direction of spline at endpoint is determined by dy/dx
    # It would seem that the magnitude of how sensitive x and y are to t is 
    #   determined by the size of dx/dt and dy/dt respectively. 
    
    # This means that a spline with l = [(1, (1, 1))] and a spline with 
    #   l =[(1, (2, 2))] have the same slope at the left endpoint, but the second 
    #   spline will have a more pronounced arc
    # Try it below if this is unclear

    # Set the left endpoint to the user given value. Right point is always unclamped
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