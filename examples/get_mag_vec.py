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
date = datetime(2023, 1, 27)

Be, Bn, Bu = ppigrf.igrf(lon, lat, h, date)  # returns east, north, up

# 24.0457, 5.4353, 51.4607

print("x: ", Bn)
print("y:", Be)
print("z:", Bu)
