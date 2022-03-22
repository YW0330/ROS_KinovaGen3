
#include <math.h>
#include "../include/kinova_test/mylib.h"

Point::Point():_x(0), _y(0), _z(0){}

Point::Point(int x,int y,int z):_x(x),_y(y),_z(z){}

double Point::length() { return sqrt(_x * _x + _y * _y + _z * _z); }
