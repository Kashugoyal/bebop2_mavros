#!/usr/bin/env python
import cv2
import numpy as np
from satellite import sat_image

def gen_path(image,xscale,yscale,lat2,lon2, width = 10):
    y_end, x_end, _ = image.shape
    x = width
    y = width
    flag = True
    x_list = []
    y_list = []
    x_list.append(x)
    y_list.append(y)
    while x <= x_end - width:
        while y <= y_end - width and y >= width:
            image[y, x] = [0,50,50]
            if flag:
                if not image[y+ width -1 , x][0] == 0:
                    y += 1
                else:
                    break
            else:
                if not image[y- width + 1, x][0] == 0:
                    y -= 1
                else:
                    break
        x_list.append(x)
        y_list.append(y)
        image[y ,x : x+width] = [0,50,50]
        x += width
        flag  = not flag
        if flag: y += 1
        else: y -= 1
        x_list.append(x)
        y_list.append(y)

    with open("path.txt", "w") as file:
        for row in zip(y_list, x_list):
            # file.write(str(row[0])+","+str(row[1]) + "\n")
            lat=row[0]*xscale+lat2;
            lon=row[1]*xscale+lon2;

            file.write(str(lat)+","+str(lon) + "\n")

    return image

if __name__ == '__main__' :

    # dummy values
    lat1 = 42.168019
    lon1 = -88.542951
    lat2 = 42.161040
    lon2 = -88.537836
    levelOfDetail = 17

    # At level 17 1 pixel corresponds to square of side 1.19 m

    # lat1 = input ('Enter latitude1: ')
    # lon1 = input ('Enter longitude1: ')
    # lat2 = input ('Enter latitude2: ')
    # lon2 = input ('Enter longitude2: ')
    # levelOfDetail = input('Enter level of detail: ')

    coord_array = [lat1,lon1,lat2,lon2,levelOfDetail]

    im = sat_image(coord_array)
    # cv2.resize(im, (0,0), fx = 0.1, fy = 0.1)
    r = cv2.selectROI(im, False, False)

    im[:,:] = [255,255,255]
    im[int(r[1]):int(r[1]+r[3]), int(r[0]):int(r[0]+r[2])] = [0,0,0]

    height, width = im.shape[:2]
    xscale=(lat1-lat2)/height
    yscale=(lon2-lon1)/width

    # Display cropped image
    image = gen_path(im,xscale,yscale,lat2,lon2)
    cv2.imshow("Image_out", image)
    cv2.imwrite('Path.png', image)

    cv2.waitKey(0)
