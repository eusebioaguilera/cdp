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

#include <cdp.hpp>                     // CDPAlgorithm definition
#include <iostream>                    // cout && endl
#include "opencv2/imgproc/imgproc.hpp" // Convex hull
#include <limits>                      // std::numeric_limits

namespace libcontour {
CDPAlgortihm::CDPAlgortihm(const ContourStats& stats) { this->_stats = stats; }

/**
 * @brief This method executes the main CDP algorithm steps. This algorithm works as
 * follows. For each subset A of points in the queue we apply the convex hull and test
 * if this subset A of points is better for approximating the whole subset than the segment
 * formed by the initial and ending points of this subset. If the test is positive for each
 * two consecutive points i and j of this subset A we add the points between i and j (both
 * inclusive) to the queue job. If the test is negative we do not add anything. We repeat
 * this process until no vector of points is contained in queue.
 */

void CDPAlgortihm::apply(void) {
    Contour cnt = this->_stats.getContour(); // Contour
    int _csize = cnt.size();                 // Contour size

    if (this->log())
        std::cout << "Number of points of the contour " << this->_stats.getContour().size() << std::endl << std::endl;

    std::vector<std::vector<int> > queue;

    // Get initial queue of jobs
    queue = this->initQueue();

    // for (unsigned int i = 0; i < queue.size(); i++) {
    while (!queue.empty()) {

        std::vector<int> _aux = queue[0]; // current job that we need to analyze. Contains indexes
        queue.erase(queue.begin());       // Erase first item
        std::vector<Point> _subset;       // current job subset of points
        std::vector<cv::Point> _points;   // vector points for converting to point2i
        std::vector<int> _hull;           // indexes that form the convex hull
        std::vector<int> _cover_hull;     // Contains the cover candidate

        // std::vector<Point2f> to std::vector<Point2i>
        for (unsigned int i = 0; i < _aux.size(); i++) {
            _subset.push_back(cnt[_aux[i]]);
        }
        cv::Mat(_subset).copyTo(_points);

        // Apply the conver hull
        cv::convexHull(_points, _hull, false, false);

        if (this->log())
            std::cout << std::endl << "Original segment indexes are ";

        for (unsigned int i = 0; i < _aux.size(); i++) {
            if (this->log())
                std::cout << _aux[i] << " ";

            if (std::find(_hull.begin(), _hull.end(), i) != _hull.end())
                _cover_hull.push_back(_aux[i]);
        }

        if (this->log())
            std::cout << std::endl;

        if (this->log()) {
            std::cout << "Convex hull obtained indexes are ";
            for (unsigned int i = 0; i < _cover_hull.size(); i++) {
                std::cout << _cover_hull[i] << " ";
            }
            std::cout << std::endl;
        }

        // If this cover is better than the segment formed by the init and end points
        // Add the cover to the queue and to the CDP points
        if (this->isBetter(_cover_hull)) {
            if (this->log())
                std::cout << "Adding convex hull indexes to the CDP" << std::endl;
            // Add to CDP
            for (unsigned int i = 0; i < _cover_hull.size(); i++) {
                // We add all points of the cover to the CDP
                this->_cdp.insert(_cover_hull[i]);

                // We add subsets of points to queue except the last point of the cover
                if (i < _cover_hull.size() - 1) {
                    std::vector<int> _indexes =
                        getIndexesBetween(_cover_hull[i], _cover_hull[(i + 1) % _cover_hull.size()]);

                    // Add indexes to the queue
                    queue.push_back(_indexes);

                    if (this->log()) {
                        std::cout << "Adding indexes ";
                        for (unsigned int j = 0; j < _indexes.size(); j++) {
                            std::cout << _indexes[j] << " ";
                        }
                        std::cout << " to the queue to explore nodes" << std::endl;
                    }
                }
            }
        }
    }
}

/**
 * @brief This function inits the queue of jobs for the CDP algorithm. This is done
 * by apply the convex hull and for each two consecutive points i and j of this convex hull
 * insert the points between i and j in the queue, both inclusive.
 * @return a vector of std::vector<Point> that contains these points
 */

std::vector<std::vector<int> > CDPAlgortihm::initQueue(void) {
    Contour cnt = this->_stats.getContour(); // Original contour
    unsigned int _csize = cnt.size();        // Contour size
    std::vector<int> hull;                   // First convex hull
    std::vector<cv::Point> points;           // Copy of the original point
    std::vector<std::vector<int> > queue;    // Result queue

    // Convert Contour (std::vector<cv::Point2f>) to std::vector<cv::Point2i>
    cv::Mat(cnt).copyTo(points);

    // Initial convex hull
    cv::convexHull(points, hull, false, false);

    // Sort the indexes
    std::sort(hull.begin(), hull.end());

    // Add all the slices to the queue
    for (unsigned int i = 0; i < hull.size(); i++) {
        bool _exit = false;                     // Exit condition
        int _init = hull[i];                    // init point of the subset of points defined by the hull
        int _end = hull[(i + 1) % hull.size()]; // end point of the subset
        int _counter = _init;                   // counter to add each point between _init and _end
        std::vector<int> _aux;                  // auxiliar var to add points

        // Add _init and _end points to the result (CDP)
        this->_cdp.insert(_init);
        this->_cdp.insert(_end);

        do {
            // If _counter == _end this is the last item to add
            if (_counter == _end)
                _exit = true;

            _aux.push_back(_counter);

            _counter = (_counter + 1) % _csize;
        } while (!_exit);

        // Add _aux to the queue job
        queue.push_back(_aux);
    }

    return queue;
}

/**
 * @brief This function tests if convex hull of the subset of points is better
 * than the init and end points of this subset.
 * @param indexes that form the subset of points
 * @return true if the convex hull is better than initial and end points
 */

bool CDPAlgortihm::isBetter(const std::vector<int>& indexes) {
    Contour _cnt = this->_stats.getContour(); // Original contour
    int _csize = _cnt.size();                 // Contour size
    double _local_ise_cr = 0.0;               // Local F1 = ISE / CR of the init and end points
    double _candidate_ise_cr = 0.0;           // Candidate fom of the convex hull contained in indexes
    int N = ((indexes[indexes.size() - 1] - indexes[0]) + _csize + 1) % _csize; // Number of points of the convex hull
    bool condition; // This represents the condition that tests if the convex hull is better than the segment

    // Compute local F1 for the test
    _local_ise_cr = this->_stats.computeISEForSegment(indexes[0], indexes[indexes.size() - 1]) / (N / 2.0);

    // Compute candidate F1 for the test
    for (unsigned int i = 0; i < indexes.size() - 1; i++) {
        int _current = indexes[i];                     // current point for computing the ISE
        int _next = indexes[(i + 1) % indexes.size()]; // next point for computing the ISE

        // Compute ISE
        _candidate_ise_cr += this->_stats.computeISEForSegment(_current, _next);
    }

    // Compute F1 = ISE / CR
    _candidate_ise_cr = _candidate_ise_cr / ((double)N / (double)indexes.size());

    // We check if the _candidate_ise_cr is lower than _local_ise_cr and then we return that
    // the candidate (convex hull) is better than the segment that joins the first and last
    // point of the hull
    condition = (_local_ise_cr - _candidate_ise_cr) > std::numeric_limits<double>::round_error();

    if (this->log()) {
        std::cout << "local ISE/CR = " << _local_ise_cr << " candidate ISE/CR = " << _candidate_ise_cr << " condition "
                  << std::boolalpha << condition << std::endl;
    }

    return condition;
}

/**
 * @brief This function returns the indexes between point i and point j, that is, returns a
 * vector that contains indexes [i, i+1, ..., j-1, j]
 * @param i init index
 * @param j end index
 * @return A vector that contains indexes between i and j
 */

std::vector<int> CDPAlgortihm::getIndexesBetween(int i, int j) {
    std::vector<int> _indexes;                     // Vector of indexes
    bool _exit = false;                            // Exit condition
    int _init = i;                                 // Init index
    int _end = j;                                  // End index
    int _counter = i;                              // Counter
    int _csize = this->_stats.getContour().size(); // Contour size

    do {
        if (_counter == _end)
            _exit = true;

        // Add index
        _indexes.push_back(_counter);

        // Increment
        _counter = (_counter + 1) % _csize;
    } while (!_exit);

    return _indexes;
}
};
