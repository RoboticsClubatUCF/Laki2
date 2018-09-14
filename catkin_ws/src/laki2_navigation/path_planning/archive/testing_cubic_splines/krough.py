from scipy import interpolate
import numpy as np
import matplotlib.pyplot as plt

# Parametric Variable
t = np.linspace(0, 2*np.pi, 5)
t_dense = np.linspace(0, 2.*np.pi, 100)

# We are creating a circle
x = np.cos(t)
y = np.sin(t)

# Make a spline with no boundary conditions:
spl_no_bc = interpolate.KroghInterpolator(t, np.c_[x, y])

# Plot the spline and the original points for reference
x_new, y_new = spl_no_bc(t_dense).T
plt.figure(0)
plt.plot(x, y, 'o')
plt.plot(x_new, y_new, '-')
plt.show(block=False)

# Now I'm going to fuck with the lists to set the endpoint first and second derivatives
# (this can actually be done at anypoint)

# The way to set derivatives is as follows:
#   To create a new knot in the spline, t must increase, never decrease
#   The first time a point is in the t list it creates a knot
#   The second time a point is in the t list it sets the 1st derivative at that knot
#   The third time a point is in the t list it sets the 2nd derivative at that knot

# Convert np arrays to lists for easy manipulation
t = list(t)
x = list(x)
y = list(y)

# First derivative at first endpoint
t.insert(0, t[0])
x.insert(1, 0)
y.insert(1, 1)

# Second derivative at second endpoint
t.insert(0, t[0])
x.insert(2, -1)
y.insert(2, 0)

# First derivative at second endpoint
t.append(t[-1])
x.append(0)
y.append(1)

# Second derivative at second endpoint
t.append(t[-1])
x.append(-1)
y.append(0)

# Make a spline with no boundary conditions:
spl_bc = interpolate.KroghInterpolator(t, np.c_[x, y])

# Plot the spline and the original points for reference
x_new, y_new = spl_bc(t_dense).T
plt.figure(1)
plt.plot(x, y, 'o')
plt.plot(x_new, y_new, '-')
plt.show(block=False)
print "Press enter to exit"
raw_input("")

print dir(spl_bc)

# To get coefficients, use:     spl_bc.c
# This will be needed to find analytical equations in order to find intersection 
#   points with the circles, and the radius of curvature

# To get derivatives, use:      spl_bc.derivative(t, der=n)
# And it will return the nth derivative at point t

