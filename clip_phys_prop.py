import numpy
import sys

rho = 7800.
h = 0.04
w = 0.015
r = 0.001

# 1 and 2: 
vol_12 = numpy.pi * r**2 * h
m_12 = rho * vol_12
Ix_12 = m_12 * (r**2 + 0.5 * w**2)
Iy_12 = m_12 / 6. * (3. * r**2 + h**2)
Iz_12 = 0.5* m_12 * (r**2 + w**2 + h**2 / 3.)

# 3 and 4:
vol_34 = numpy.pi * r**2 * w
m_34 = rho * vol_34
Ix_34 = m_34 / 6. * (3. * r**2 + w**2)
Iy_34 = m_34 * (r**2 + 0.5 * h**2)
Iz_34 = 0.5* m_34 * (r**2 + h**2 + w**2 / 3.)

mass_tot = 2. *(m_12 + m_34)

Ixx = Ix_12 + Ix_34
Iyy = Iy_12 + Iy_34
Izz = Iz_12 + Iz_34