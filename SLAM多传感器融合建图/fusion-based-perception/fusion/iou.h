#pragma once

#include <cmath>
#include <assert.h>
#include <vector>

namespace kit {
namespace perception {
namespace fusion {

const double EPS = 1e-6;

enum WiseType {
    NoneWise,
    ClockWise,
    AntiClockWise
};
enum LocPosition {
    Outside,
    OnEdge,
    Inside
};


template <typename T>
struct Vec2 {
    // Members
    union {
        struct {
            T x;
            T y;
        };
        T D[2];
    };

    // Constructors
    Vec2() : x(0), y(0) {}
    Vec2(T _x, T _y) : x(_x), y(_y) {}

    // Access component
    inline T& operator[](unsigned int i) { assert(i < 2); return D[i]; }
    inline const T& operator[](unsigned int i) const { assert(i < 2); return D[i]; }

    // Operations
    inline bool operator==(const Vec2 &p) const {
        return (std::abs(x - p.x) <= EPS && std::abs(y - p.y) <= EPS);
    }

    template<typename TT>
    inline Vec2 operator*(TT t) const { return Vec2(x * t, y * t); }
    template<typename TT>
    inline Vec2 operator/(TT t) const { return Vec2(x / t, y / t); }
    template<typename TT>
    inline Vec2& operator*=(TT t) { x *= t; y *= t; return *this; }
    template<typename TT>
    inline Vec2& operator/=(TT t) { x /= t; y /= t; return *this; }

    inline Vec2 operator+(const Vec2 &p) const { return Vec2(x + p.x, y + p.y); }
    inline Vec2 operator-(const Vec2 &p) const { return Vec2(x - p.x, y - p.y); }
    inline Vec2& operator+=(const Vec2 &p) { x += p.x; y += p.y; return *this; }
    inline Vec2& operator-=(const Vec2 &p) { x -= p.x; y -= p.y; return *this; }

    inline bool isZero() { return (abs(x) <= EPS && abs(y) <= EPS); }

    inline Vec2 dmul(const Vec2 &p) const { return Vec2(x * p.x, y * p.y); }
    inline Vec2 ddiv(const Vec2 &p) const { return Vec2(x / p.x, y / p.y); }

    inline double dot(const Vec2 &p) const { return x * p.x + y * p.y; }
    inline double operator*(const Vec2 &p) const { return x * p.x + y * p.y; }

    inline double cmul(const Vec2 &p) const { return x * p.y - y * p.x; }
    inline double operator^(const Vec2 &p) const { return x * p.y - y * p.x; }

    inline double norm() const { return sqrt(x*x + y*y); }
    inline double normSquared() const { return x*x + y*y; }

    void normalize() { *this /= norm(); }
    Vec2 normalized() const { return *this / norm(); }

    double distance(const Vec2 &p) const {
        return (*this - p).norm();
    }
    double squareDistance(const Vec2 &p) const {
        return (*this - p).normSquared();
    }
    double angle(const Vec2 &r) const {
        return acos( dot(r) / (norm() * r.norm()) ); }
    double theta() const {
        return atan2(y, x);
    }
};

template <typename T>
double norm(const Vec2<T> &p) { return p.norm(); }
template <typename T>
double normSquared(const Vec2<T> &p) { return p.normSquared(); }
template <typename T>
void normalize(Vec2<T> &p) { p.normalize(); }
template <typename T>
Vec2<T> normalized(const Vec2<T> &p) { return p.normalized(); }
template <typename T, typename TT>
inline Vec2<T> operator*(TT t, const Vec2<T>& v) { return Vec2<T>(v.x * t, v.y * t); }
template <typename T>
inline double distance(const Vec2<T> &p1, const Vec2<T> &p2) { return p1.distance(p2); }
template <typename T>
inline double squareDistance(const Vec2<T> &p1, const Vec2<T> &p2) { return p1.squareDistance(p2); }
template <typename T>
inline double angle(const Vec2<T> &p1, const Vec2<T> &p2) { return p1.angle(p2); }
template <typename T>
inline double theta(const Vec2<T> &p) { return p.theta(); }

typedef Vec2<int> Vec2i;
typedef Vec2<float> Vec2f;
typedef Vec2<double> Vec2d;

typedef Vec2d Point;
typedef std::vector<Point> Vertexes;

class Line {
 public:
    // Members
    Point p1;
    Point p2;

    // Constructors
    Line() : p1(Point()), p2(Point()) {}
    Line(const Point &_p1, const Point &_p2) : p1(_p1), p2(_p2) {}
    Line(const Point _vert[2]) : p1(_vert[0]), p2(_vert[1]) {}

    // Methods
    double length() const {return p1.distance(p2); }
    bool isOnEdge(const Point &p) const;
    Point intersection(const Line &line, bool *bOnEdge = 0) const;
};
inline bool isOnEdge(const Line &line, const Point &p) {
    return line.isOnEdge(p); }
inline bool isOnEdge(const Point &p, const Line &line) {
    return line.isOnEdge(p); }
inline Point intersection(const Line &line1, const Line &line2, bool *bOnEdge = 0) {
    return line1.intersection(line2,bOnEdge); }

class Quad {
 public:
    // Members [in clockwise]
    Point p1;
    Point p2;
    Point p3;
    Point p4;

    // Constructors
    Quad() : p1(Point()), p2(Point()), p3(Point()), p4(Point()) {}
    Quad(const Point &_p1, const Point &_p2, const Point &_p3, const Point &_p4)
        : p1(_p1), p2(_p2), p3(_p3), p4(_p4) {}
    Quad(const Point _vert[4])
        : p1(_vert[0]), p2(_vert[1]), p3(_vert[2]), p4(_vert[3]) {}

    // Methods
    void flip() {
        Point tmp = p2;
        p2 = p4;
        p4 = tmp; }
    void getVertList(Vertexes &_vert) const;

    double area() const;
    WiseType whichWise() const;
    bool isInClockWise() const { return ClockWise == whichWise(); }
    bool isInAntiClockWise() const { return AntiClockWise == whichWise(); }
    void beInSomeWise(const WiseType wiseType);
    void beInClockWise() { beInSomeWise(ClockWise); }
    void beInAntiClockWise() { beInSomeWise(AntiClockWise); }

    LocPosition location(const Point &p) const;
    int interPts(const Line &line, Vertexes &pts) const;
};

inline LocPosition location(const Quad &quad, const Point &p) {
    return quad.location(p); }
inline int interPts(const Quad &quad, const Line &line, Vertexes &pts) {
    return quad.interPts(line, pts); }

// For any convex polygon
double areaEx(const Vertexes &C);
WiseType whichWiseEx(const Vertexes &C);
void beInSomeWiseEx(Vertexes &C, const WiseType wiseType);
LocPosition locationEx(const Vertexes &C, const Point &p);
int interPtsEx(const Vertexes &C, const Line &line, Vertexes &pts);

// For any convex polygon
int findInterPointsEx(const Vertexes &C1, const Vertexes &C2, Vertexes &vert);
int findInnerPointsEx(const Vertexes &C1, const Vertexes &C2, Vertexes &vert);
double areaIntersectionEx(const Vertexes &C1, const Vertexes &C2);
double areaUnionEx(const Vertexes &C1, const Vertexes &C2);
double iouEx(const Vertexes &C1, const Vertexes &C2);

// For convex quadrilateral
int findInterPoints(const Quad &Q1, const Quad &Q2, Vertexes &vert);
int findInnerPoints(const Quad &Q1, const Quad &Q2, Vertexes &vert);
double areaIntersection(const Quad&Q1, const Quad &Q2);
double areaUnion(const Quad &Q1, const Quad &Q2);
double iou(const Quad &Q1, const Quad &Q2);

}  // namespace fusion
}  // namespace perception
}  // namespace kit
