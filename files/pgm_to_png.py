import argparse
import os
from PIL import Image, UnidentifiedImageError

"""
convert image from .pgm to .png
because map image is in .pgm format but Unity will receive .png image
"""

def convert_image(file_path, map_folder="map"):
    filename_ext = os.path.basename(file_path)
    filename_only, file_ext = os.path.splitext(filename_ext)
    try:
        Image.open(map_folder + "/" + filename_only + ".pgm").save( map_folder + "/" + filename_only + ".png")

    except (UnidentifiedImageError, PermissionError, OSError, ValueError) as e:
        pass

if __name__=="__main__":
    parser = argparse.ArgumentParser(description='input path of .pgm file to convert to .png file.')
    parser.add_argument('filepath', metavar='F', type=str, 
                        help='path of file to be converted.')

    args = parser.parse_args()
    convert_image(args.filepath)