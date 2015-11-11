/*
 * contour.cpp
 *
 * Copyright 2015 Eusebio Aguilera <eusebio.aguilera@gmai.com>
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

#ifndef CDP_HPP
#define CDP_HPP

#include <contour.hpp> // Contour & ContourStats
#include <set>         // std::set
#include <vector>      // std::vector

namespace libcontour {
class CDPAlgortihm {
  private:
    bool _log;
    ContourStats _stats;
    std::set<int> _cdp;

    std::vector<std::vector<int> > initQueue(void);

    bool isBetter(const std::vector<int>& indexes);

    std::vector<int> getIndexesBetween(int i, int j);

  public:
    CDPAlgortihm();
    CDPAlgortihm(const ContourStats& stats);
    void apply(void);
    inline std::set<int> cdp(void) { return this->_cdp; }

    void log(bool value) { this->_log = value; }
    inline bool log(void) { return this->_log; }
};
};

#endif // CDP_HPP