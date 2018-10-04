import numpy as np

# W = m_dot * (omega * r)
#   m_dot = 0.5 * rho * 2 * pi * r * V_p

# W = rho * pi * V_p * omega * r**2

# F = integral(W)dr
# F = 1/3 * rho * pi * V_p * omega * r**3

# T = integral(F)dr
# T = 1/12 * rho * pi * V_p * omega * r**4

# V_p = V_e + sin(alpha) * V_ac
# V_e = 0.8 * omega * P
# T = 1/12 * rho * pi * (0.8 * omega * P + sin(alpha) * V_ac) * omega * r**4
# T = 1/12 * rho * pi * r**4 * ((0.8 * P * omega**2) + (omega * sin(alpha) * V_ac))