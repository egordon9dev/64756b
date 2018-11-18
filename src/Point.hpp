#ifndef POINT_H
#define POINT_H
class Point {
   public:
    double x, y;
    Point();
    Point(double x, double y);
    friend Point operator+(const Point& p1, const Point& p2);
    friend Point operator-(const Point& p1, const Point& p2);
    friend double operator*(const Point& p1, const Point& p2);
    friend bool operator>(const Point& p1, const Point& p2);
    friend bool operator<(const Point& p1, const Point& p2);
    double mag() const;
    Point abs();
    Point rotate(int dir) const;
};
#endif