#pragma once

void bresenhamAlgorithm(int initX, int initY, int endX,  int endY, signed char * matrixMap, int w, int h, float res){
    int dx, dy, i, e;
    int incx, incy, inc1, inc2;
    int x, y;

    dx = endX - initX;
    dy = endY - initY;

    if(dx < 0)
        dx = -dx;
    if(dy < 0)
        dy = -dy;

    incx = 1;
    if(endX < initX)
        incx = -1;
    incy = 1;
    if(endY < initY)
        incy = -1;

    x = initX;
    y = initY;

    if(dx > dy){
        matrixMap[x + y * w] = 100;
        e = 2 * dy - dx;
        inc1 = 2 * (dy - dx);
        inc2 = 2 * dy;
        for(int i = 0; i < dx; i++){
            if(e >= 0){
                y += incy;
                e += inc1;
            }
            else
                e += inc2;
            x += incx;
            matrixMap[x + y * w] = 100;
        }
    }
    else{
        matrixMap[x + y * w] = 100;
        e = 2 * dx - dy;
        inc1 = 2 * (dx - dy);
        inc2 = 2 * dx;
        for(int i = 0; i < dy; i++){
            if(e >= 0){
                x += incx;
                e += inc1;
            }
            else
                e += inc2;
            y += incy;
            matrixMap[x + y * w] = 100;
        }
    }
    //matrixMap[initX + initY * w] = 100;
    //matrixMap[endX + endY * w] = 100;
}
