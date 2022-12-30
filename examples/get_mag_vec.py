from datetime import datetime
import ppigrf

"""
this script returns the expected magnetometer given 
latitude longitude and altitude, use this in your
kalman filters.
"""

lon = 151.200043  # degrees east
lat = -33.896042  # degrees north
h = 0  # kilometers above sea level
date = datetime(2022, 12, 28)

Be, Bn, Bu = ppigrf.igrf(lon, lat, h, date)  # returns east, north, up

print("x: ", Bn)
print("y:", Be)
print("z:", Bu)
