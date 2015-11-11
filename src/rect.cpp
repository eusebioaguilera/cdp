/*
 * rect.cpp
 *
 * Copyright 2013 Eusebio Aguilera <eusebio.aguilera@gmail.com>
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

#include "rect.hpp"

namespace libcontour {
Rect::Rect(const cv::Point2d& p1, const cv::Point2d& p2) {
    // Init values

    a(p2.y - p1.y);
    b(-(p2.x - p1.x));
    c(-(b() * p2.y + a() * p2.x));
    double mod = sqrt(a() * a() + b() * b());
    a(a() / mod);
    b(b() / mod);
    c(c() / mod);
}

void Rect::create(const cv::Point2d& p1, const cv::Point2d& p2) {
    // Init values
    a(p2.y - p1.y);
    b(-(p2.x - p1.x));
    c(-(b() * p2.y + a() * p2.x));
    double mod = sqrt(a() * a() + b() * b());
    a(a() / mod);
    b(b() / mod);
    c(c() / mod);
}

#ifdef USE_PYTHON

// This constructor is only used for Python
Rect::Rect(const boost::python::tuple& p1, const boost::python::tuple& p2) {
    double p1_x, p1_y;
    double p2_x, p2_y;

    p1_x = boost::python::extract<double>(p1[0]);
    p1_y = boost::python::extract<double>(p1[1]);
    p2_x = boost::python::extract<double>(p2[0]);
    p2_y = boost::python::extract<double>(p2[1]);

    a(p2_y - p1_y);
    b(-(p2_x - p1_x));
    c(-(b() * p2_y + a() * p2_x));
    double mod = sqrt(a() * a() + b() * b());
    a(a() / mod);
    b(b() / mod);
    c(c() / mod);
}

#endif // ifdef USE_PYTHON

double Rect::distance(const cv::Point2d& p) {
    // return fabs( a() * p.x + b() * p.y + c() ) / sqrt( a() * a() + b() * b() );
    return a() * p.x + b() * p.y + c();
}
};
