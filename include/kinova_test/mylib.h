#ifndef MYLIB_H
#define MYLIB_H

class Point
{
private:
    int _x;
    int _y;
    int _z;

public:
    Point();
    Point(int x, int y, int z);
    ~Point(){};
    double length();
};

#endif