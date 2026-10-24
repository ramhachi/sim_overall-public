#include "cpp/include/fsae/track/track_model.hpp"
#include <iostream>
#include <iomanip>
using namespace fsae;

int main() {
    auto track = track::create_figure_eight_track(25.0, 3.0);
    std::cout << "Track length: " << track.length() << " m\n";

    auto start = track.get_point(0.0);
    auto end = track.get_point(track.length());

    std::cout << std::fixed << std::setprecision(6);
    std::cout << "Start: x=" << start.x << ", y=" << start.y << ", theta=" << start.theta << "\n";
    std::cout << "End:   x=" << end.x << ", y=" << end.y << ", theta=" << end.theta << "\n";

    double dx = end.x - start.x;
    double dy = end.y - start.y;
    std::cout << "Distance between start/end: " << std::sqrt(dx*dx + dy*dy) << " m\n";
    std::cout << "Theta difference: " << (end.theta - start.theta) * 180.0 / M_PI << " deg\n";

    // Check some intermediate points
    std::cout << "\nIntermediate points:\n";
    for (double s = 0; s <= track.length(); s += track.length() / 10.0) {
        auto pt = track.get_point(s);
        std::cout << "s=" << s << ": x=" << pt.x << ", y=" << pt.y << ", kappa=" << pt.kappa << "\n";
    }

    return 0;
}
