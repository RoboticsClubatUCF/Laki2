from scipy.interpolate import BSpline, make_interp_spline
import numpy as np
import matplotlib.pyplot as plt

# Parametric Variable
t = np.linspace(0, 2*np.pi, 5)
t_dense = np.linspace(0, 2.*np.pi, 100)

# We are creating a circle
x = np.cos(t)
y = np.sin(t)

# Use this library function to make a spline: https://docs.scipy.org/doc/scipy/reference/generated/scipy.interpolate.make_interp_spline.html
# All the features to allow for parametric splines, and controling the slopes
#   are there, we just have to figure out how to use them

# Make a spline with no boundary conditions:
spl_no_bc = make_interp_spline(t, np.c_[x, y])

# Plot the spline and the original points for reference
x_new, y_new = spl_no_bc(t_dense).T
plt.figure(0)
plt.plot(x, y, 'o')
plt.plot(x_new, y_new, '-')
plt.show(block=False)

# l and r are left and right boundary conditions
# As far as I can tell it means something along the lines of this: 
#   1st derivative, (dx/dt, dy/dt))
# For a parametric euation in general: dy/dx = dy/dt / dx/dt. 
l, r = [(1, (0, 1))], [(1, (0, 1))]

# Set a spline with the boundary conditions
spl_with_bc = make_interp_spline(t, np.c_[x, y], bc_type=(l, r))

# spl_with_bc is a Bspline object: 
#   https://docs.scipy.org/doc/scipy/reference/generated/scipy.interpolate.BSpline.html
print "\nknots: ", spl_with_bc.t
print "coefficients: ", spl_with_bc.c
print "order: ", spl_with_bc.k

# plot the new splined with controlled boundary conditions
x_new, y_new = spl_with_bc(t_dense).T
plt.figure(1)
plt.plot(x, y, 'o')
plt.plot(x_new, y_new, '-')
plt.show(block=False)

# Wait for user to leave program
print "Press Enter to exit"
raw_input("")