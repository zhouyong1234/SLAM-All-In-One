#include "iou.h"
#include <algorithm>
#include <iostream>

namespace kit {
namespace perception {
namespace fusion {

bool Line::isOnEdge(const Point &p) const {
    if (p1 == p2)
        return (p == (p1 + p2) / 2.0);

    Point pp1 = p - p1;
    Point pp2 = p - p2;

    if (abs(pp1^pp2) < EPS &&
        pp1*pp2 < EPS)
        return true;
    else
        return false;
}

Point Line::intersection(const Line &line, bool *bOnEdge) const {
    Point pInter(0,0);
    bool bOn = false;

    if (p1 == p2 && line.p1 == line.p2){
        // Both lines are actually points.
        bOn =((p1 + p2) / 2.0 == (line.p1 + line.p2) / 2.0);
        if (bOn)
            pInter = (p1 + p2 + line.p1 + line.p2) / 4.0;
    } else if (p1 == p2) {
        // This line is actually a point.
        bOn = line.isOnEdge((p1 + p2) / 2.0);
        if (bOn)
            pInter = (p1 + p2) / 2.0;
    } else if (line.p1 == line.p2) {
        // The input line is actually a point.
        bOn = isOnEdge((line.p1 + line.p2) / 2.0);
        if (bOn)
            pInter = (line.p1 + line.p2) / 2.0;
    } else {
        // Normal cases.
        Point a12 = p2 - p1;
        Point b12 = line.p2 - line.p1;
        double ang = angle(a12, b12);
        if (ang < EPS || abs(3.141592653 - ang) < EPS)
            bOn = false; // Collinear!!
        else {
            // a1_x + m*a12_x = b1_x + n*b12_x
            // a1_y + m*a12_y = b1_y + n*b12_y
            // n = ( (a1_y-b1_y)*a12_x - (a1_x-b1_x)*a12_y ) / (a12_x*b12_y - b12_x*a12_y)
            // m = ( (a1_y-b1_y)*b12_x - (a1_x-b1_x)*b12_y ) / (a12_x*b12_y - b12_x*a12_y)
            // 0 < m < 1
            // 0 < n < 1
            double abx = p1.x - line.p1.x;
            double aby = p1.y - line.p1.y;
            double ab = a12.x*b12.y - b12.x*a12.y;
            assert(abs(ab)>EPS);
            double n = (aby*a12.x - abx*a12.y) / ab;
            double m = (aby*b12.x - abx*b12.y) / ab;

            if (n >= -EPS && n-1.0 <= EPS &&
                m >= -EPS && m-1.0 <= EPS) {
                Point ip1 = p1 + m*a12;
                Point ip2 = line.p1 + n*b12;
                pInter = (ip1 + ip2) / 2.0;
                bOn = true;
            } else {
                bOn = false;
            }
        }
    }
    if (bOnEdge != 0)
        *bOnEdge = bOn;
    return pInter;
}

void Quad::getVertList(Vertexes &_vert) const {
    Vertexes vertTemp;
    vertTemp.reserve(4);
    vertTemp.push_back(p1);
    vertTemp.push_back(p2);
    vertTemp.push_back(p3);
    vertTemp.push_back(p4);
    _vert.swap(vertTemp);
}

double Quad::area() const {
    Vertexes vertTemp;
    getVertList(vertTemp);

    return areaEx(vertTemp);
}

WiseType Quad::whichWise() const {
    Vertexes vertTemp;
    getVertList(vertTemp);
    return whichWiseEx(vertTemp);
}
void Quad::beInSomeWise(const WiseType wiseType) {
    if (wiseType != NoneWise) {
        Vertexes vertTemp;
        getVertList(vertTemp);
        beInSomeWiseEx(vertTemp,wiseType);
        p1 = vertTemp[0];
        p2 = vertTemp[1];
        p3 = vertTemp[2];
        p4 = vertTemp[3];
    }
}

LocPosition Quad::location(const Point &p) const {
    Vertexes vertTemp;
    getVertList(vertTemp);
    return locationEx(vertTemp, p);
}
int Quad::interPts(const Line &line, Vertexes &pts) const {
    Vertexes vertTemp;
    getVertList(vertTemp);
    return interPtsEx(vertTemp, line, pts);
}

double areaEx(const Vertexes &C) {
    if (whichWiseEx(C) == NoneWise)
        return -1.0;

    double sArea = 0.0;
    const int N = C.size();
    if (N > 2) {
        const Point &p0 = C.at(0);
        for (int i = 1; i < N-1; ++i) {
            const Point &p1 = C.at(i);
            const Point &p2 = C.at(i + 1);
            Point p01 = p1 - p0;
            Point p02 = p2 - p0;
            sArea += abs(p01^p02)*0.5;
        }
    }
    return sArea;
}

WiseType whichWiseEx(const Vertexes &C) {
    WiseType wiseType = NoneWise;
    const int N = C.size();

    if (N > 2) {
        Point p0 = C.at(N - 1);
        Point p1 = C.at(0);
        Point p2 = C.at(1);
        Point p01 = p1 - p0;
        Point p12 = p2 - p1;
        if ((abs(p01^p12) <= EPS) && p01*p12 < 0.0)
            return NoneWise;
        else
            wiseType = (p01^p12) > 0.0 ? AntiClockWise : ClockWise;

        const double flip = (wiseType == ClockWise) ? 1.0 : -1.0;
        for (int i = 1; i < N ; ++i) {
            p0 = C.at((i-1)%N);
            p1 = C.at(i%N);
            p2 = C.at((i+1)%N);
            p01 = p1 - p0;
            p12 = p2 - p1;
            if ((p01^p12)*flip > 0.0 ||
                ((abs(p01^p12) <= EPS) && p01*p12 < 0.0)) {
                return NoneWise;
            }
        }
    }
    return wiseType;
}

typedef std::pair<double, Point> AngPoint;
bool angIncrease(const AngPoint &p1, const AngPoint &p2) {
    return p1.first < p2.first;
}
bool angDecrease(const AngPoint &p1, const AngPoint &p2) {
    return p1.first > p2.first;
}
void beInSomeWiseEx(Vertexes &C, const WiseType wiseType) {
    if (wiseType != NoneWise) {
        const int N = C.size();
        if (N>2) {
            Point pO(0.0,0.0);
            for (int i = 0; i < N; ++i)
                pO += C.at(i);
            pO /= N;
            std::vector<AngPoint> APList;
            APList.reserve(N);
            for (int i = 0; i < N; ++i) {
                Point op = C.at(i) - pO;
                double ang = op.theta();
                APList.push_back(AngPoint(ang, C.at(i)));
            }
            if (wiseType == AntiClockWise)
                std::sort(APList.begin(), APList.end(), angIncrease);
            else
                std::sort(APList.begin(), APList.end(), angDecrease);
            Vertexes vertTemp;
            for (int i = 0; i < N; ++i)
                vertTemp.push_back(APList.at(i).second);
            C.swap(vertTemp);
        }
    }
}

LocPosition locationEx(const Vertexes &C, const Point &p) {
    const int N = C.size();
    // Special cases.
    if (N == 0)
        return Outside;
    if (N == 1) {
        if (C[0] == p)
            return Inside;
        else
            return Outside;
    }
    if (N == 2) {
        if (isOnEdge(Line(C[0],C[1]),p))
            return OnEdge;
        else
            return Outside;
    }

    // Normal cases.
    // Check OnEdge.
    for (int i=0; i<N; ++i) {
        if (isOnEdge(Line(C[i%N],C[(i+1)%N]),p))
            return OnEdge;
    }
    // Check Outside.
    Point pO(0.0,0.0);
    for (int i=0; i<N; ++i) {
        pO += C[i];
    }
    pO /= N;
    Line op(pO,p);
    bool bIntersection = true;
    for (int i=0; i<N; ++i) {
        intersection(Line(C[i%N],C[(i+1)%N]),op,&bIntersection);
        if (bIntersection)
            return Outside;
    }

    return Inside;
}

int interPtsEx(const Vertexes &C, const Line &line, Vertexes &pts) {
    Vertexes vertTemp;
    const int N = C.size();
    bool bIntersection = false;
    for (int i=0; i<N; ++i) {
        Point p = intersection(Line(C[i%N],C[(i+1)%N]),line,&bIntersection);
        if (bIntersection)
            vertTemp.push_back(p);
    }
    pts.swap(vertTemp);

    return pts.size();
}

int findInterPointsEx(const Vertexes &C1, const Vertexes &C2, Vertexes &vert) {
    Vertexes _vert;
    const int N = C2.size();
    for (int i=0; i<N; ++i) {
        Vertexes pts;
        interPtsEx(C1,Line(C2[i%N],C2[(i+1)%N]),pts);
        for (int i = 0; i < pts.size(); ++i)
            _vert.push_back(pts.at(i));
    }
    vert.swap(_vert);
    return vert.size();
}

int findInnerPointsEx(const Vertexes &C1, const Vertexes &C2, Vertexes &vert) {
    Vertexes _vert;
    for (int i=0; i<C2.size(); ++i) {
        if (locationEx(C1,C2[i]) != Outside)
            _vert.push_back(C2[i]);
    }
    vert.swap(_vert);
    return vert.size();
}

double areaIntersectionEx(const Vertexes &C1, const Vertexes &C2) {
    if (whichWiseEx(C1) == NoneWise ||
        whichWiseEx(C2) == NoneWise )
        return -1.0;

    Vertexes interVert;
    Vertexes innerVert12;
    Vertexes innerVert21;
    Vertexes allVerts;
    //---------------
    findInterPointsEx(C1, C2, interVert);
    findInnerPointsEx(C1, C2, innerVert12);
    findInnerPointsEx(C2, C1, innerVert21);
    //---------------
    // TODO : Check conditions
    for (int i = 0; i < interVert.size(); ++i)
        allVerts.push_back(interVert.at(i));
    for (int i = 0; i < innerVert12.size(); ++i)
        allVerts.push_back(innerVert12.at(i));
    for (int i = 0; i < innerVert21.size(); ++i)
        allVerts.push_back(innerVert21.at(i));

    if (allVerts.empty())
        return 0.0;
    else {
        assert(allVerts.size() >= 3);
        beInSomeWiseEx(allVerts, ClockWise);
        if (whichWiseEx(allVerts) == NoneWise)
            return -1.0;
        else
            return areaEx(allVerts);
    }
    return -1.0;
}

double areaUnionEx(const Vertexes &C1, const Vertexes &C2) {
    return areaEx(C1) + areaEx(C2) - areaIntersectionEx(C1, C2);
}

double iouEx(const Vertexes &C1, const Vertexes &C2) {
    return areaIntersectionEx(C1,C2)/areaUnionEx(C1,C2);
}

int findInterPoints(const Quad &Q1, const Quad &Q2, Vertexes &vert) {
    Vertexes V1, V2;
    Q1.getVertList(V1);
    Q2.getVertList(V2);
    return findInterPointsEx(V1,V2,vert);
}

int findInnerPoints(const Quad &Q1, const Quad &Q2, Vertexes &vert) {
    Vertexes V1, V2;
    Q1.getVertList(V1);
    Q2.getVertList(V2);
    return findInnerPointsEx(V1,V2,vert);
}

double areaIntersection(const Quad&Q1, const Quad &Q2) {
    if (Q1.whichWise() == NoneWise ||
        Q2.whichWise() == NoneWise )
        return -1.0;

    Vertexes interVert;
    Vertexes innerVert12;
    Vertexes innerVert21;
    Vertexes allVerts;
    //---------------
    findInterPoints(Q1, Q2, interVert);
    findInnerPoints(Q1, Q2, innerVert12);
    findInnerPoints(Q2, Q1, innerVert21);
    //---------------
    // TODO : Check conditions
    for (int i = 0; i < interVert.size(); ++i)
        allVerts.push_back(interVert.at(i));
    for (int i = 0; i < innerVert12.size(); ++i)
        allVerts.push_back(innerVert12.at(i));
    for (int i = 0; i < innerVert21.size(); ++i)
        allVerts.push_back(innerVert21.at(i));

    if (allVerts.empty())
        return 0.0;
    else {
        assert(allVerts.size() >= 3);
        beInSomeWiseEx(allVerts, ClockWise);
        if (whichWiseEx(allVerts) == NoneWise)
            return -1.0;
        else
            return areaEx(allVerts);
    }
    return -1.0;
}

double areaUnion(const Quad &Q1, const Quad &Q2){
    return Q1.area()+Q2.area()-areaIntersection(Q1,Q2);
}

double iou(const Quad &Q1, const Quad &Q2) {
    return areaIntersection(Q1,Q2)/areaUnion(Q1,Q2);
}

}  // namespace fusion
}  // namespace perception
}  // namespace kit
