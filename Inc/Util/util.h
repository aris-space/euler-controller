//
// Created by Tun Kapgen on 04.10.20.
//


void interpolate(float y[2], float x[2], float xp, float *yp){
    *yp = y[0] + ((y[1]-y[0])/(x[1]-x[0])) * (xp - x[0]);
}