/*
 * tools.cpp
 *
 * Copyright 2014 Eusebio Aguilera <eusebio.aguilera@gmail.com>
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

#include "tools.hpp"

namespace libcontour {
namespace tools {
std::vector<int> breakpoints(const Contour& contour) {
    // int nPoints = contour.size();
    int current, next;

    std::vector<int> result; // Breakpoints
    std::vector<int> chain;  // Chain codes
    // int  ccc    = 0;         // Current Chain Code
    // bool second = false;

    // Compute Freeman chain codes
    chain = freemanChainCodes(contour);
    current = 0;
    int csize = chain.size();
    int cchain = chain[csize - 1];

    for (int i = 0; i < csize; i++) {
        // int next = (i+1+csize) % csize;
        // int previous = (i-1+csize) % csize;
        // std::cout << previous << "," << i << "," << next <<", " <<
        // chain[previous] << " " << chain[i] << " " << chain[next] << std::endl;
        if (chain[i] != cchain) {
            result.push_back(i);
            cchain = chain[i];
        }
    }

    return result;
}

std::vector<int> freemanChainCodes(const Contour& contour) {
    int nPoints = contour.size();
    double dx, dy, angle;
    Point current, next;

    std::vector<int> result; // Chain codes
    int ccc = 0;             // Current Chain Code

    for (int i = 0; i < nPoints; i++) {
        current = contour[i];
        next = contour[(i + 1) % nPoints];
        dx = next.x - current.x;
        dy = next.y - current.y;
        angle = atan2(dx, dy);
        angle = (180 / M_PI) * angle;

        if (angle < 0.0) {
            do {
                angle += 360.0;
            } while (angle < 0.0);
        }

        if (angle > 337.5)
            ccc = 2;
        else if (angle > 292.5)
            ccc = 3;
        else if (angle > 247.5)
            ccc = 4;
        else if (angle > 202.5)
            ccc = 5;
        else if (angle > 157.5)
            ccc = 6;
        else if (angle > 112.5)
            ccc = 7;
        else if (angle > 67.5)
            ccc = 0;
        else if (angle > 22.5)
            ccc = 1;
        else
            ccc = 2;

        // Add to the vector
        result.push_back(ccc);
    }
    return result;
}

std::vector<cv::Point> getPolygon(const Contour& contour, const std::vector<int>& dpoints) {
    std::vector<cv::Point> result;

    for (std::vector<int>::const_iterator it = dpoints.begin(); it != dpoints.end(); it++) {
        result.push_back(contour[*it]);
    }

    return result;
}

#ifdef USE_PYTHON
boost::python::list breakpoints(const boost::python::list& contour) {
    int nPoints = boost::python::len(contour);
    int current, next;
    boost::python::list result; // Breakpoints
    boost::python::list chain;  // Chain codes
    int ccc = 0;                // Current Chain Code
    bool second = false;

    // Compute Freeman chain codes
    chain = freemanChainCodes(contour);
    current = 0;
    next = current;

    while (not second) {
        while (chain[current] == chain[next]) {
            next = (next + 1) % nPoints;

            if (next == 0) {
                second = true;
            }
        }
        result.append(next);
        current = next;
    }
    return result;
}

boost::python::list freemanChainCodes(const boost::python::list& contour) {
    int nPoints = boost::python::len(contour);
    double dx, dy, angle;
    boost::python::list current, next;
    boost::python::list result; // Chain codes
    double cx, cy, nx, ny;
    int ccc = 0; // Current Chain Code

    for (int i = 0; i < nPoints; i++) {
        current = boost::python::extract<boost::python::list>(contour[i]);
        next = boost::python::extract<boost::python::list>(contour[(i + 1) % nPoints]);

        cx = boost::python::extract<double>(current[0]);
        cy = boost::python::extract<double>(current[1]);
        nx = boost::python::extract<double>(next[0]);
        ny = boost::python::extract<double>(next[1]);

        dx = nx - cx;
        dy = ny - cy;

        angle = atan2(dx, dy);
        angle = (180 / M_PI) * angle;

        if (angle < 0.0) {
            do {
                angle += 360.0;
            } while (angle < 0.0);
        }

        // std::cout << "(" << cx << ", " << cy << ") (" << nx << ", " << ny << ")
        // --> " << dx << " " << dy << " angle " << angle << std::endl;

        if (angle > 337.5)
            ccc = 2;
        else if (angle > 292.5)
            ccc = 3;
        else if (angle > 247.5)
            ccc = 4;
        else if (angle > 202.5)
            ccc = 5;
        else if (angle > 157.5)
            ccc = 6;
        else if (angle > 112.5)
            ccc = 7;
        else if (angle > 67.5)
            ccc = 0;
        else if (angle > 22.5)
            ccc = 1;
        else
            ccc = 2;

        // Add to the vector
        // std::cout << i << " " << ( i + 1 ) % nPoints << " " << ccc << std::endl;
        result.append(ccc);
    }
    return result;
}

#endif // USE_PYTHON
};
};
