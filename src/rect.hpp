/*
 * rect.cpp
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

/**
 * @file rect.hpp
 * This class implements the rect object and calculates rect values a, b y c.
 * @brief This class implements the rect object and calculates rect values a, b
 *y c.
 * @author Eusebio Aguilera
 * @version 1.0
 */

#ifndef RECT_HPP
#define RECT_HPP

#include <cmath>
#include "cv.hpp"

#ifdef USE_PYTHON

// This constructor is only used for Python
#include <boost/python.hpp>
#include <boost/python/tuple.hpp>
#endif // ifdef USE_PYTHON

namespace libcontour {
class Rect {
    double _a;
    double _b;
    double _c;

  protected:
    // Setters are protected
    void a(double value) { this->_a = value; }

    void b(double value) { this->_b = value; }

    void c(double value) { this->_c = value; }

  public:
    // Getters and setters
    double a(void) { return this->_a; }

    double b(void) { return this->_b; }

    double c(void) { return this->_c; }

    Rect(const cv::Point2d& p1, const cv::Point2d& p2);

    // Empty constructor
    Rect(void) {}

    // Build rect
    void create(const cv::Point2d& p1, const cv::Point2d& p2);

    // Build using A, B, C values
    void create(double A, double B, double C) {
        this->_a = A;
        this->_b = B;
        this->_c = C;
    }

#ifdef USE_PYTHON

    // This constructor is only used for Python
    Rect(const boost::python::tuple& p1, const boost::python::tuple& p2);
#endif // ifdef USE_PYTHON

    // / Returns the distance between the rect and the point p
    double distance(const cv::Point2d& p);

    // TODO: Add distance for python
};
};

#endif // RECT_HPP
