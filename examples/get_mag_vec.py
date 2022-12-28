from datetime import datetime
import ppigrf

lon = 151.2  # degrees east
lat = -33.2 # degrees north
h   = 0        # kilometers above sea level
date = datetime(2022, 12, 28)

Be, Bn, Bu = ppigrf.igrf(lon, lat, h, date) # returns east, north, up