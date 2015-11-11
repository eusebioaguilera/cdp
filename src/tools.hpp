/*
 * tools.hpp
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
 * @file tools.hpp
 * This file constains the tools used for obtain the breakpoints, Freeman chain
 *codes, etc
 * @brief This file constains the tools used for obtain the breakpoints, Freeman
 *chain codes, etc
 * @author Eusebio Aguilera
 * @version 1.0
 */

#ifndef __TOOLS_HPP__
#define __TOOLS_HPP__

#include <vector>
#include "contour.hpp"

#ifdef USE_PYTHON
#include <boost/python.hpp>
#include <boost/python/list.hpp>
#endif // ifdef USE_PYTHON

namespace libcontour {
namespace tools {
std::vector<int> breakpoints(const Contour& contour);
std::vector<int> freemanChainCodes(const Contour& contour);
std::vector<cv::Point> getPolygon(const Contour& contour, const std::vector<int>& dpoints);

#ifdef USE_PYTHON
boost::python::list breakpoints(const boost::python::list& contour);
boost::python::list freemanChainCodes(const boost::python::list& contour);
#endif // USE_PYTHON
};
};

#endif // __TOOLS_HPP__
