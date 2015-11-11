#include <iostream>    //cout & endl
#include <unistd.h>    //getopt
#include <cstdlib>     //atoi
#include <contour.hpp> //Contour & ContourStats
#include <cdp.hpp>     // CDPAlgorithm

using namespace std;
using namespace libcontour;

void help(void) {
    cout << "This program obtains the CDP candidate points from a contour file." << endl;
    cout << "This software implements the paper http://dx.doi.org/10.1016/j.jvcir.2014.09.012" << endl;
    cout << "The contour file must contain the coordinates of each point that define the shape" << endl << endl;
    cout << "Usage:" << endl << endl;
    cout << "\t-h:\t\t\tPrints this help." << endl;
    cout << "\t-v:\t\t\tPrints version of the program." << endl;
    cout << "\t-f <contour_file>:\tContour file used for calculating the CDP candidate points." << endl;
    cout << "\t-t <output_type>:\tThis parameter set the output type for the CDP points. " << endl;
    cout << "\t\t\t\t0 for the indexes of the points, 1 for the coordinates, 2 for a mask. Default value is 0." << endl;
    cout << "\t-V:\t\t\tPrints verbose messages of the algorithm" << endl;
}

enum OUTPUT_TYPE {
    INDEXES = 0,
    COORDINATES = 1,
    MASK = 2
};

int main(int argc, char **argv) {
    int c;
    int mayor = 1, minor = 0, rev = 0;
    int output_type = INDEXES;
    bool verbosity = false;

    std::string contourFile;

    // / Checks for options
    while ((c = getopt(argc, argv, "hvf:t:V")) != -1)
        switch (c) {
        case 'h':
            help();
            exit(0);
            break;

        case 'v':
            cout << "Version " << mayor << "." << minor << "." << rev << endl;
            break;

        case 'f':
            contourFile = optarg;
            break;

        case 't':
            output_type = atoi(optarg);
            break;

        case 'V':
            verbosity = true;
            break;

        default:
            help();
            exit(-1);
        }

    if (contourFile.empty()) {
        std::cout << "Error: No contour file name set" << std::endl;
        help();
        exit(1);
    }

    // Contour Statistics
    ContourStats stats = ContourStats(contourFile);

    // Contour
    Contour contour = stats.getContour();

    if (output_type != INDEXES && output_type != COORDINATES && output_type != MASK) {
        cout << "Error: invalid value " << output_type << " for output type" << endl;
        help();
        exit(1);
    }

    CDPAlgortihm alg = CDPAlgortihm(stats);

    // Set verbosity
    alg.log(verbosity);

    alg.apply();

    std::set<int> result = alg.cdp();
    // Output type
    if (output_type == INDEXES) {
        for (std::set<int>::iterator it = result.begin(); it != result.end(); it++) {
            std::cout << *it << " ";
        }
        std::cout << std::endl;
    } else if (output_type == COORDINATES) {
        for (unsigned int i = 0; i < contour.size(); i++) {
            if (result.find(i) != result.end()) {
                Point p = contour[i];
                std::cout << "[" << p.x << ", " << p.y << "] ";
            }
        }
        std::cout << std::endl;
    } else {
        std::string ss;
        for (unsigned int i = 0; i < stats.getContour().size(); i++) {
            if (result.find(i) != result.end()) {
                ss += "1";
            } else {
                ss += "0";
            }
        }
        std::cout << ss << std::endl;
    }

    return 0;
}
