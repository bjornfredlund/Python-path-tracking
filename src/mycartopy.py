import os
import numpy as np
import itertools
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.patheffects as pe
import cartopy
import utm
import cartopy.geodesic as cgeo
import cartopy.crs as ccrs
import requests
from dotenv import load_dotenv, find_dotenv
import cartopy.io.img_tiles as cimgt
import io
from urllib.request import urlopen, Request
from PIL import Image
import shapely

ZOOM = 115 # m

def get_path(start_lng, start_lat, end_lng, end_lat):
    
    load_dotenv(find_dotenv())
    MY_TOKEN = os.environ.get("TOKEN")
    url = f'https://api.openrouteservice.org/v2/directions/foot-walking?api_key={MY_TOKEN}&start={start_lng},{start_lat}&end={end_lng},{end_lat}'

    req = requests.get(url)

    j = req.json()
    coords = j['features'][0]['geometry']['coordinates']
    lng_dir = []
    lat_dir = []

    for lng,lat in coords:
        lng_dir.append(lng)
        lat_dir.append(lat)

    return lng_dir, lat_dir

def calc_extent(lon,lat,dist):

    dist_cnr = np.sqrt(2*dist**2)
    top_left = cgeo.Geodesic().direct(points=(lon,lat),azimuths=-45,distances=dist_cnr)[:,0:2][0]
    bot_right = cgeo.Geodesic().direct(points=(lon,lat),azimuths=135,distances=dist_cnr)[:,0:2][0]


    extent = [top_left[0], bot_right[0], bot_right[1], top_left[1]]

    return extent


def fix_background(lon, lat, ax):
    extent = calc_extent(lon, lat, ZOOM)
    ax.set_extent(extent) # set extents
    ax.add_image(img, 18) # add OSM with zoom specification
    gl = ax.gridlines(draw_labels=True, crs=ccrs.PlateCarree(),
                        color='k',lw=0.5)

    gl.top_labels = False
    gl.right_labels = False
    gl.xformatter = cartopy.mpl.gridliner.LONGITUDE_FORMATTER
    gl.yformatter = cartopy.mpl.gridliner.LATITUDE_FORMATTER


def osm_image(lon,lat,fig):
    global img_crs
    global img
    cimgt.OSM.get_image = image_spoof # reformat web request for street map spoofing
    img = cimgt.OSM() # spoofed, downloaded street map

    ax = fig.add_subplot(3,4,(5,11),projection=img.crs) # project using coordinate reference system (CRS) of street map

    fix_background(lon, lat, ax)


    img_crs = img.crs

    return ax

def to_geographic_coordinates(x,y):
    return utm.to_latlon(x, y, sector, letter)

def to_geographic_coordinates_cartopy(x, y):
    return list(itertools.chain(*ccrs.PlateCarree().transform_points(img_crs, x, y)))

def to_cartesian_coordinates(lng, lat):
    global sector, letter 
    x, y, sector, letter = utm.from_latlon(lat, lng)
    return x, y


def image_spoof(self, tile):

    url = self._image_url(tile)                # get the url of the street map API
    req = Request(url)                         # start request
    req.add_header('User-agent','Anaconda 3')  # add user agent to request
    fh = urlopen(req) 
    im_data = io.BytesIO(fh.read())            # get image
    fh.close()                                 # close url
    img = Image.open(im_data)                  # open image with PIL
    img = img.convert(self.desired_tile_form)  # set image format
    return img, self.tileextent(tile), 'lower' # reformat for cartopy
