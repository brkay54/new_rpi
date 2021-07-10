

#include "../include/path_plotter.h"
#include "../include/survey_path_generator.h"




int main () {
    geometry_msgs::Point global, local, target;

    survey_path_generator survey;
    survey.d=5;
    survey.read_from_file("/home/ahmet/int/src/new_rpi/fly/fly_control/config/local_coordinates.txt");
    std::cout<<survey.Polygon.size();

    survey.standardize();
    survey.generate_path();
    survey.add_arcs(0);
    std::cout<<survey.Polygon.size();
    for(int i=0; i<survey.Polygon.size(); i++){
        std::cout<<survey.Polygon[i].x<<" "<<survey.Polygon[i].y<<std::endl;
    }


    path_plotter::plot(survey.survey_points, survey.Polygon);

}

/*
   survey_path_generator survey;
   survey.d=10;
    std::cout<<"1";
   survey.read_from_file();
    std::cout<<survey.Polygon.size();
    survey.standardize();
    std::cout<<"3";
    survey.generate_path();
    std::cout<<"4";
    survey.add_arcs(1);
    std::cout<<"5";


    path_plotter::plot(survey.survey_points, survey.Polygon);
*/