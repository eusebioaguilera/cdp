/*
 * contour.hpp
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

#ifndef CONTOUR_HPP
#define CONTOUR_HPP

#include "cv.h" // OpenCV
#include <vector>
#include <string>
#include <valarray> // valarray
#include <limits>   // std::numeric_limits

#ifdef USE_PYTHON
#include <boost/python/list.hpp>
#endif

namespace libcontour {

/**
 *
 * @file contour.hpp
 * Definition of the contour object. Contor object represent a contour that
 * can be draw, modified, etc.
 * @author Eusebio J. Aguilera
 * @version 1.0
 *
 * */

typedef cv::Point2d Point;
typedef std::vector<Point> Contour;

// Contour readContourFromTxt(const std::string& file);

/*
 * This class computes the statitics of the contour
 * @author Eusebio J. Aguilera
 * @version 1.0
 * */
class ContourStats {
  private:
    /*
     * This variables contains the increments in x and y
     * */
    std::valarray<double> _x, _y, _xx, _yy, _xy;
    Contour _contour;

    void initialize(void);

  public:
    ContourStats(void);
    ContourStats(const std::string& file, bool closed = false);
    ContourStats(const Contour& cnt);
    //~ContourStats();

    int distance(int i, int j) const throw();

    double computeISE(std::vector<int>& dpoints) const throw();

    double computeMaxDeviation(const std::vector<int>& dpoints) const throw();

#ifdef USE_PYTHON
    // This constructor is only used for Python
    double computeISE(boost::python::list& dpoints) const throw();
    double computeMaxDeviation(const boost::python::list& dpoints) const throw();
#endif

    double computeISEForSegment(const int i, const int j) const throw();

    double computeMaxDeviationForSegment(const int i, const int j) const throw();

    Contour getContour(void) const throw() { return this->_contour; }

#ifdef USE_PYTHON
    // This constructor is only used for Python
    // The name getContour_ is due to C++ does not allow overload by return type
    // http://stackoverflow.com/questions/9568852/overloading-by-return-type
    boost::python::list getContour_(void) const throw();
#endif
};
};
#endif /* CONTOUR_HPP */
