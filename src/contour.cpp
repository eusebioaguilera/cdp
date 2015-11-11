/*
 * contour.cpp
 *
 * Copyright 2014 Eusebio Aguilera <eusebio.aguilera@gmai.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 *
 *
 */

#include "contour.hpp"
#include "rect.hpp"
#include <fstream>

namespace libcontour {

ContourStats::ContourStats(const std::string& file, bool closed) {
    // Open file and read it!!
    std::string line;
    std::fstream fp;
    // First values to check
    double x0, y0;
    bool first_value = false;
    // Contour contour;
    // Contour c;
    double x, y;

    fp.open(file.c_str(), std::ios_base::in);

    if (fp) {
        while (!fp.eof()) {
            fp >> x >> y;

            if (closed && !first_value) {
                x0 = x;
                y0 = y;
            }

            if (!fp.eof()) {
                // std::cout << x << " " << y << std::endl;
                cv::Point2d p;
                p.x = x;
                p.y = y;
                this->_contour.push_back(p);
            }
        }
    }

    fp.close();

    // Check if closed
    if (closed) {
        int csize = this->_contour.size();

        Point p0 = this->_contour[0];
        Point pLast = this->_contour[csize - 1];

        if (p0 != pLast) {
            // We must replicate the last point
            this->_contour.push_back(p0);
        }
    }

    // this->_contour = c;
    this->initialize();
}

ContourStats::ContourStats(void) {}

ContourStats::ContourStats(const Contour& cnt) {
    // Copy the new contour
    this->_contour = cnt;

    // Init the object
    this->initialize();
}

void ContourStats::initialize(void) {
    assert(this->_contour.size() > 2);

    _x.resize(this->_contour.size());
    _y.resize(this->_contour.size());
    _xx.resize(this->_contour.size());
    _yy.resize(this->_contour.size());
    _xy.resize(this->_contour.size());

    double x = this->_contour[0].x;
    double y = this->_contour[0].y;
    _x[0] = x;
    _y[0] = y;
    _xx[0] = x * x;
    _yy[0] = y * y;
    _xy[0] = x * y;

    for (unsigned p = 1; p < this->_contour.size(); ++p) {

        x = this->_contour[p].x;
        y = this->_contour[p].y;
        _x[p] = x + _x[p - 1];
        _y[p] = y + _y[p - 1];
        _xx[p] = x * x + _xx[p - 1];
        _yy[p] = y * y + _yy[p - 1];
        _xy[p] = x * y + _xy[p - 1];
    }

    //_contour = contour;
}

int ContourStats::distance(int i, int j) const throw() {
    int s = _x.size();
    return (j - i + s) % s;
}

/*
 * This function computes ISE value using the method described in
 * Perez, J.-C., Vidal, E., 1994. Optimum polygonal approximation of digitized curves. Pattern Recognition Letters 15,
 *743–750. doi:10.1016/0167-8655(94)90002-7
 *
 * */

double ContourStats::computeISEForSegment(const int i, const int j) const throw() {
    int numPoints = this->_x.size();
    assert(i >= 0 and i < (int)numPoints);
    assert(j >= 0 and j < (int)numPoints);
    double ISE = std::numeric_limits<double>::infinity();

    const int npoints = distance(i, j) - 1;
    if (npoints > 0) {
        Rect l;
        if (this->_contour[i] != this->_contour[j]) {
            // Rect l(this->_contour[i], this->_contour[j]);
            l = Rect(this->_contour[i], this->_contour[j]);
        } else {
            // If the two points are equals, we no compute a line. so
            // our proposal is find the most distant point and set a set the line
            // paralell.
            double maxDistance = 0;
            int maxDistP = i;
            int csize = this->_contour.size();
            int k = i;

            while (k != j) {
                // for (unsigned k=i; k<j; k=(k+1)%csize) {
                const double dist = cv::norm(cv::Mat(this->_contour[i]), cv::Mat(this->_contour[k]));
                // const double dist = point(i).distance(point(k));
                if (dist > maxDistance) {
                    maxDistance = dist;
                    maxDistP = k;
                }
                // std::cout << "i= " << i << " k=" << k << " maxDistP=" << maxDistP << " dist=" << maxDistance << " j="
                // << j << std::endl;
                //}
                k = (k + 1) % csize;
            }
            if (maxDistance > 0.0) {
                // ava::Point2d v = maxDistP-point(i);
                // l = ava::Line2d (point(i), v.y, -v.x);
                Point maxDistPoint = this->_contour[maxDistP];
                Point iPoint = this->_contour[i];
                Rect tmpr = Rect(iPoint, maxDistPoint);
                double m1 = -(tmpr.a() / tmpr.b());
                double m2 = -1 / m1;
                // If the rect is vertical, parallel to the y axis, then the perpendicular one is a zero slope rect
                // (parallel to the x axis)
                if (std::numeric_limits<double>::infinity() == m1)
                    m2 = 0; // std::cout << "(INFINITY) m1=" << m1 << " " << std::endl;

                double _a = -m2;
                double _b = 1;
                double _c = (m2 * iPoint.x) - iPoint.y;

                // Point p = Point(maxDistPoint.x-iPoint.x, maxDistPoint.y-iPoint.y);
                // l = Rect(this->_contour[i], p);

                l = Rect();
                l.create(_a, _b, _c);
            }
        }

        int jMinus1 = (j - 1 + numPoints) % numPoints;
        double x, y, xx, yy, xy;
        if (i < jMinus1) {
            x = _x[jMinus1] - _x[i];
            y = _y[jMinus1] - _y[i];
            xx = _xx[jMinus1] - _xx[i];
            yy = _yy[jMinus1] - _yy[i];
            xy = _xy[jMinus1] - _xy[i];
        } else {
            x = _x[numPoints - 1] - _x[i] + _x[jMinus1];
            y = _y[numPoints - 1] - _y[i] + _y[jMinus1];
            xx = _xx[numPoints - 1] - _xx[i] + _xx[jMinus1];
            yy = _yy[numPoints - 1] - _yy[i] + _yy[jMinus1];
            xy = _xy[numPoints - 1] - _xy[i] + _xy[jMinus1];
        }

        // std::cout << "x " << x << " y " << y << std::endl;
        // std::cout << "a " << l.a() << " b " << l.b() << " c " << l.c() << std::endl;

        ISE = l.a() * l.a() * xx + l.b() * l.b() * yy + (npoints) * l.c() * l.c() + 2.0 * l.b() * l.c() * y +
              2.0 * l.a() * l.c() * x + 2.0 * l.a() * l.b() * xy;
    } else
        ISE = 0.0;

    return ISE;
}

double ContourStats::computeISE(std::vector<int>& dpoints) const throw() {
    int s = dpoints.size();
    double ise = 0.0;

    for (int i = 0; i < s; i++) {
        int current = dpoints[i];
        int next = dpoints[(i + 1) % s];
        double temp = this->computeISEForSegment(current, next);
        ise += temp;
    }

    return ise;
}

double ContourStats::computeMaxDeviation(const std::vector<int>& dpoints) const throw() {
    double cdev, maxdev;

    int dsize = dpoints.size();

    for (int i = 0; i < dsize; i++) {
        cdev = this->computeMaxDeviationForSegment(dpoints[i], dpoints[(i + 1) % dsize]);

        if (cdev > maxdev) {
            maxdev = cdev;
        }
    }

    return maxdev;
}

double ContourStats::computeMaxDeviationForSegment(const int i, const int j) const throw() {
    double maxDeviation = 0.0;
    int s = _x.size();

    if (_contour[i] != _contour[j]) {
        int n = (i + 1) % s;
        while (n != j) {
            Rect l = Rect(_contour[i], _contour[j]);
            double temp = std::abs(l.distance(_contour[n]));
            if (temp > maxDeviation) {
                maxDeviation = temp;
            }
            n = (n + 1) % s;
        }
    }

    return maxDeviation;
}

#ifdef USE_PYTHON
// This function is only used in Python Wrapper
double ContourStats::computeISE(boost::python::list& dpoints) const throw() {
    int s = boost::python::len(dpoints);
    double ise = 0.0;

    for (int i = 0; i < s; i++) {
        int current = boost::python::extract<int>(dpoints[i]);
        int next = boost::python::extract<int>(dpoints[(i + 1) % s]);
        double temp = this->computeISEForSegment(current, next);
        ise += temp;
    }

    return ise;
}

double ContourStats::computeMaxDeviation(const boost::python::list& dpoints) const throw() {
    double cdev, maxdev;

    int dsize = boost::python::len(dpoints);

    for (int i = 0; i < dsize; i++) {
        int current = boost::python::extract<int>(dpoints[i]);
        int next = boost::python::extract<int>(dpoints[(i + 1) % dsize]);

        cdev = this->computeMaxDeviationForSegment(current, next);

        if (cdev > maxdev) {
            maxdev = cdev;
        }
    }

    return maxdev;
}

boost::python::list ContourStats::getContour_(void) const throw() {
    boost::python::list cnt;

    for (Contour::const_iterator it = this->_contour.begin(); it != this->_contour.end(); it++) {
        boost::python::list point;

        point.append(it->x);
        point.append(it->y);

        cnt.append(point);
    }

    return cnt;
}

#endif
};
