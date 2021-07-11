

#include "../include/path_plotter.h"
#include "../include/survey_path_generator.h"


int main() {

    double cruise_height = 10;
    double camera_horizontal_fov = 1.0855947947;
    double overlap = 0.5;

    geometry_msgs::Point vehicle_local, vehicle_global;

    vehicle_local.x = 0;
    vehicle_local.y = 0;
    vehicle_local.z = 0;
    vehicle_global.x = 41.0857468;
    vehicle_global.y = 29.0401726;

    survey_path_generator survey;

    survey.d = (cruise_height * sin(camera_horizontal_fov / 2.0)) * (1.0 - overlap);
    //survey.d=5;

    survey.read_from_file("/home/ahmet/int/src/new_rpi/fly/fly_control/config/coordinates.txt", vehicle_local,
                          vehicle_global);
    std::cout << survey.Polygon.size();

    survey.standardize();
    survey.generate_path();
    survey.add_arcs(survey.choose_side(vehicle_local));


    std::cout << "Polygone size:" << survey.Polygon.size() << std::endl;
    for (int i = 0; i < survey.Polygon.size(); i++) {
        std::cout << survey.Polygon[i].x << " " << survey.Polygon[i].y << std::endl;
    }


    path_plotter::plot(survey.survey_points, survey.Polygon);

}

