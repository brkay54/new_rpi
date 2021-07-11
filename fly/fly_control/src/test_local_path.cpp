

#include "../include/path_plotter.h"
#include "../include/survey_path_generator.h"




int main () {

    double cruise_height = 10;
    double camera_horizontal_fov = 1.0855947947;
    double overlap = 0.5;

    survey_path_generator survey;

    survey.d = (cruise_height * sin(camera_horizontal_fov / 2.0)) * (1.0 - overlap);
    //survey.d=5;

    survey.read_from_file("/home/ahmet/int/src/new_rpi/fly/fly_control/config/local_coordinates.txt");
    std::cout<<survey.Polygon.size();

    survey.standardize();
    survey.generate_path();
    survey.add_arcs(0);

    std::cout<<"Polygone size:"<<survey.Polygon.size()<<std::endl;
    for(int i=0; i<survey.Polygon.size(); i++){
        std::cout<<survey.Polygon[i].x<<" "<<survey.Polygon[i].y<<std::endl;
    }


    path_plotter::plot(survey.survey_points, survey.Polygon);

}

