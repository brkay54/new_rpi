//
// Created by ahmet on 6.01.2021.
//

#include "survey_path_generator.h"

const double pi = 3.14159265358979323846;

void survey_path_generator::generate_path() {

    std::vector <PathSegment> farthest_edge(Polygon.size());
    std::vector <double> max_distance(Polygon.size(),0);

    for(int i=0; i<Polygon.size(); i++){

        double distance;
        PathSegment edge;
        for(int j=0; j<Polygon.size(); j++){

            if(j!=i && j!=i-1 && j!=Polygon.size()-1){

                edge.point_begin=Polygon[j];
                edge.point_end=Polygon[j+1];

                distance=commons::distance_to_line(edge, Polygon[i]);
            }
            else if (j!=i && j!=i-1 && j==Polygon.size()-1){
                edge.point_begin=Polygon[j];
                edge.point_end=Polygon[0];

                distance=commons::distance_to_line(edge, Polygon[i]);

            }
            if(distance>max_distance[i])  {

                max_distance[i]=distance;
                farthest_edge[i]=edge;
                }


        }
    }


    int sweep_point;
    double max_dist=0.0;
    for (int i=0; i<Polygon.size(); i++){
        if(max_distance[i]>max_dist){
            sweep_point=i;
            max_dist=max_distance[i];
        }
    }


    int edge1, edge2;
    for (int i=0; i<Polygon.size(); i++){
        if(Polygon[i].x ==farthest_edge[sweep_point].point_begin.x && Polygon[i].y ==farthest_edge[sweep_point].point_begin.y){  //******************************
            edge1=i;
        }
        else if(Polygon[i].x ==farthest_edge[sweep_point].point_end.x && Polygon[i].y ==farthest_edge[sweep_point].point_end.y ){
            edge2=i;
        }
    }


    std::vector <geometry_msgs::Point> PPlefts;
    std::vector <geometry_msgs::Point> PPrights;
    for(int i=0; i<Polygon.size(); i++){

        if(sweep_point> edge2){
            if(i>=edge2 && i<sweep_point){
                PPrights.push_back(Polygon[i]);

            }
            else if(i != sweep_point){
                PPlefts.push_back(Polygon[i]);
            }
        }
        else{
            if(i>sweep_point && i<= edge1){
                PPlefts.push_back(Polygon[i]);
            }
            else if(i !=sweep_point){
                PPrights.push_back(Polygon[i]);
            }
        }
    }
    PPlefts.push_back(Polygon[sweep_point]);
    PPrights.push_back(Polygon[sweep_point]);


    PathSegment sweep_line;

    sweep_line.point_begin=commons::projection_on_line(farthest_edge[sweep_point], Polygon[sweep_point]);

    sweep_line.point_end=Polygon[sweep_point];



    std::vector<std::array<std::vector<geometry_msgs::Point>,4>> LSD;

    int LSD_index=0;
    double sweep_line_lenght = commons::distance_between_points(sweep_line.point_begin, sweep_line.point_end);
    while(double(LSD_index)*d < sweep_line_lenght){
        LSD.push_back(std::array<std::vector<geometry_msgs::Point>,4>());
        geometry_msgs::Point LSpoint;
        LSpoint.x=sweep_line.point_begin.x +commons::line_direction(sweep_line).x*(double(LSD_index)+0.5)*d;
        LSpoint.y=sweep_line.point_begin.y +commons::line_direction(sweep_line).y*(double(LSD_index)+0.5)*d;

        for(int i=0; i<PPlefts.size(); i++){
            if(commons::distance_between_points(LSpoint, commons::projection_on_line(sweep_line, PPlefts[i]))<=d/2+0.1){
                LSD[LSD_index][0].push_back(PPlefts[i]);
            }
        }
        for(int i=0; i<PPrights.size(); i++){
            if(commons::distance_between_points(LSpoint, commons::projection_on_line(sweep_line, PPrights[i]))<=d/2+0.1){
                LSD[LSD_index][1].push_back(PPrights[i]);
            }
        }
        LSD_index++;
    }
    for(int i=0; i<LSD.size(); i++){

        if(LSD[i][0].size()!=0){
            geometry_msgs::Point left_border;
            double farthest=0;
            for(int j=0; j<LSD[i][0].size(); j++){
                if(commons::distance_to_line(sweep_line, LSD[i][0][j])>= farthest){
                    farthest=commons::distance_to_line(sweep_line, LSD[i][0][j]);
                    left_border = LSD[i][0][j];
                }
            }
            LSD[i][2].push_back(left_border);
        }
        else{
            PathSegment left_border_line;
            int n=1;
            while(LSD[i-n][0].size()==0){n++;}

            geometry_msgs::Point LSpoint;
            LSpoint.x=sweep_line.point_begin.x +commons::line_direction(sweep_line).x*(double(LSD_index)+0.5)*d;
            LSpoint.y=sweep_line.point_begin.y +commons::line_direction(sweep_line).y*(double(LSD_index)+0.5)*d;

            double min_dist_to_lspoint = 0;
            for(int j=0; j<LSD[i-n][0].size(); j++){
                if(commons::distance_between_points(commons::projection_on_line(sweep_line, LSD[i-n][0][j]), LSpoint)<min_dist_to_lspoint || min_dist_to_lspoint ==0){
                    left_border_line.point_begin = LSD[i-n][0][j];
                    min_dist_to_lspoint = commons::distance_between_points(commons::projection_on_line(sweep_line, LSD[i-n][0][j]), LSpoint);
                }
            }
            n=1;
            while(LSD[i+n][0].size()==0){n++;}

            min_dist_to_lspoint = 0;
            for(int j=0; j<LSD[i+n][0].size(); j++){
                if(commons::distance_between_points(commons::projection_on_line(sweep_line, LSD[i+n][0][j]), LSpoint)<min_dist_to_lspoint || min_dist_to_lspoint ==0){
                    left_border_line.point_end = LSD[i+n][0][j];
                    min_dist_to_lspoint = commons::distance_between_points(commons::projection_on_line(sweep_line, LSD[i+n][0][j]), LSpoint);
                }
            }

            if(commons::distance_to_line(sweep_line, left_border_line.point_begin)<commons::distance_to_line(sweep_line, left_border_line.point_end)){
                PathSegment cross_line;
                cross_line.point_begin.x=sweep_line.point_begin.x +commons::line_direction(sweep_line).x*(double(i)+1.0)*d;
                cross_line.point_begin.y=sweep_line.point_begin.y +commons::line_direction(sweep_line).y*(double(i)+1.0)*d;

                cross_line.point_end.x = cross_line.point_begin.x - commons::line_direction(sweep_line).y;
                cross_line.point_end.y = cross_line.point_begin.y + commons::line_direction(sweep_line).x;

                LSD[i][2].push_back(commons::intersect_lines(cross_line, left_border_line));
            }
            else{
                PathSegment cross_line;
                cross_line.point_begin.x=sweep_line.point_begin.x +commons::line_direction(sweep_line).x*(double(i))*d;
                cross_line.point_begin.y=sweep_line.point_begin.y +commons::line_direction(sweep_line).y*(double(i))*d;

                cross_line.point_end.x = cross_line.point_begin.x - commons::line_direction(sweep_line).y;
                cross_line.point_end.y = cross_line.point_begin.y + commons::line_direction(sweep_line).x;

                LSD[i][2].push_back(commons::intersect_lines(cross_line, left_border_line));
            }
        }

        if(LSD[i][1].size()!=0){
            geometry_msgs::Point right_border;
            double farthest=0;
            for(int j=0; j<LSD[i][1].size(); j++){
                if(commons::distance_to_line(sweep_line, LSD[i][1][j])>= farthest){
                    farthest=commons::distance_to_line(sweep_line, LSD[i][1][j]);
                    right_border = LSD[i][1][j];
                }
            }
            LSD[i][3].push_back(right_border);
        }
        else{
            PathSegment right_border_line;
            int n=1;
            while(LSD[i-n][1].size()==0){n++;}

            geometry_msgs::Point LSpoint;
            LSpoint.x=sweep_line.point_begin.x +commons::line_direction(sweep_line).x*(double(LSD_index)+0.5)*d;
            LSpoint.y=sweep_line.point_begin.y +commons::line_direction(sweep_line).y*(double(LSD_index)+0.5)*d;

            double min_dist_to_lspoint = 0;
            for(int j=0; j<LSD[i-n][1].size(); j++){
                if(commons::distance_between_points(commons::projection_on_line(sweep_line, LSD[i-n][1][j]), LSpoint)<min_dist_to_lspoint || min_dist_to_lspoint ==0){
                    right_border_line.point_begin = LSD[i-n][1][j];
                    min_dist_to_lspoint = commons::distance_between_points(commons::projection_on_line(sweep_line, LSD[i-n][1][j]), LSpoint);
                }
            }
            n=1;
            while(LSD[i+n][1].size()==0){n++;}
            min_dist_to_lspoint = 0;

            for(int j=0; j<LSD[i+n][1].size(); j++){

                if(commons::distance_between_points(commons::projection_on_line(sweep_line, LSD[i+n][1][j]), LSpoint)<min_dist_to_lspoint || min_dist_to_lspoint ==0){
                    right_border_line.point_end = LSD[i+n][1][j];
                    min_dist_to_lspoint = commons::distance_between_points(commons::projection_on_line(sweep_line, LSD[i+n][1][j]), LSpoint);
                }
            }

            if(commons::distance_to_line(sweep_line, right_border_line.point_begin)<commons::distance_to_line(sweep_line, right_border_line.point_end)){
                PathSegment cross_line;
                cross_line.point_begin.x=sweep_line.point_begin.x +commons::line_direction(sweep_line).x*(double(i)+1.0)*d;
                cross_line.point_begin.y=sweep_line.point_begin.y +commons::line_direction(sweep_line).y*(double(i)+1.0)*d;

                cross_line.point_end.x = cross_line.point_begin.x + commons::line_direction(sweep_line).y;
                cross_line.point_end.y = cross_line.point_begin.y - commons::line_direction(sweep_line).x;

                LSD[i][3].push_back(commons::intersect_lines(cross_line, right_border_line));
            }
            else{
                PathSegment cross_line;
                cross_line.point_begin.x=sweep_line.point_begin.x +commons::line_direction(sweep_line).x*(double(i))*d;
                cross_line.point_begin.y=sweep_line.point_begin.y +commons::line_direction(sweep_line).y*(double(i))*d;

                cross_line.point_end.x = cross_line.point_begin.x + commons::line_direction(sweep_line).y;
                cross_line.point_end.y = cross_line.point_begin.y - commons::line_direction(sweep_line).x;

                LSD[i][3].push_back(commons::intersect_lines(cross_line, right_border_line));
            }
        }
    }


    geometry_msgs::Point left_direction, right_direction,point_on_sweep_line;
    sweep_line_direction.x=commons::line_direction(sweep_line).x;
    sweep_line_direction.y=commons::line_direction(sweep_line).y;
    left_direction.x=-sweep_line_direction.y;
    left_direction.y=sweep_line_direction.x;
    right_direction.x=sweep_line_direction.y;
    right_direction.y=-sweep_line_direction.x;
    PathSegment from_sweep_begin_to_LSD_point;
    from_sweep_begin_to_LSD_point.point_begin= sweep_line.point_begin;
    double distance_left, distance_right;
    for(int i=0; i<LSD.size(); i++){
        point_on_sweep_line.x=sweep_line.point_begin.x+(double(i)+0.5)*d*sweep_line_direction.x;
        point_on_sweep_line.y=sweep_line.point_begin.y+(double(i)+0.5)*d*sweep_line_direction.y;
        distance_left=commons::distance_to_line(sweep_line, LSD[i][2][0]);
        distance_right=commons::distance_to_line(sweep_line, LSD[i][3][0]);

        from_sweep_begin_to_LSD_point.point_end=LSD[i][2][0];
        if(commons::angle_between_lines(sweep_line, from_sweep_begin_to_LSD_point)<0){
            distance_left= -distance_left;
        }
        from_sweep_begin_to_LSD_point.point_end=LSD[i][3][0];
        if(commons::angle_between_lines(sweep_line, from_sweep_begin_to_LSD_point)>0){
            distance_right= -distance_right;
        }
        PathSegment lines;
        lines.point_begin.x=point_on_sweep_line.x+distance_left*left_direction.x;
        lines.point_begin.y=point_on_sweep_line.y+distance_left*left_direction.y;

        lines.point_end.x=point_on_sweep_line.x+distance_right*right_direction.x;
        lines.point_end.y=point_on_sweep_line.y+distance_right*right_direction.y;

        survey_points.push_back(lines);

    }

    for(int i=0; i<survey_points.size(); i++){
        survey_points[i].type=1;
    }

}

void survey_path_generator::read_from_file() {

    std::fstream file("/home/ahmet/jws/src/drive/jaguar_control/config/coordinates.txt", std::ios_base::in);

    geometry_msgs::Point point;

    while(file >>point.x >> point.y){
        this->Polygon.push_back(point);
    }

    file.close();

}

void survey_path_generator::read_from_file(geometry_msgs::Point vehicle_local, geometry_msgs::Point vehicle_global) {


    std::fstream file("/home/ahmet/integrated/src/integrated/fly/fly_control/config/coordinates.txt", std::ios_base::in);

    geometry_msgs::Point point;
    while(file >>point.x >> point.y){
        point.z=vehicle_global.z;
        this->Polygon.push_back(commons::global_to_local(point, vehicle_global, vehicle_local));

    }

    file.close();


}

void survey_path_generator::standardize() {
    PathSegment l1, l2;
    l1.point_begin=Polygon[0];
    l1.point_end=Polygon[1];
    l2.point_begin=Polygon[1];
    l2.point_end=Polygon[2];
    double angle = commons::angle_between_lines(l1, l2);
    angle = angle<0 ? angle +2.0*pi : angle;

    if(angle>pi){
        std::reverse(Polygon.begin(), Polygon.end());
        }


}

void survey_path_generator::add_arcs(int side) {

    if(side==0 || side==2){
        int line_index =0;
        PathSegment arc;
        for(int i=0; i<survey_points.size()-1; i=i+2 ){
            if(line_index%2==0){
                arc.center.x= (survey_points[i].point_end.x + survey_points[i+1].point_end.x) / 2.0;
                arc.center.y= (survey_points[i].point_end.y + survey_points[i+1].point_end.y) / 2.0;

                arc.point_begin.x = arc.center.x - sweep_line_direction.x*d/2;
                arc.point_begin.y = arc.center.y  - sweep_line_direction.y*d/2;

                arc.point_end.x = arc.center.x + sweep_line_direction.x*d/2;
                arc.point_end.y = arc.center.y + sweep_line_direction.y*d/2;

                survey_points[i].point_end=arc.point_begin;
                survey_points[i+1].point_end=arc.point_end;

            }
            else{
                arc.center.x= (survey_points[i].point_begin.x + survey_points[i+1].point_begin.x) / 2.0;
                arc.center.y= (survey_points[i].point_begin.y + survey_points[i+1].point_begin.y) / 2.0;

                arc.point_begin.x = arc.center.x - sweep_line_direction.x*d/2;
                arc.point_begin.y = arc.center.y  - sweep_line_direction.y*d/2;

                arc.point_end.x = arc.center.x + sweep_line_direction.x*d/2;
                arc.point_end.y = arc.center.y + sweep_line_direction.y*d/2;

                survey_points[i].point_begin=arc.point_begin;
                survey_points[i+1].point_begin=arc.point_end;
            }
            if(line_index%2 ==0){
                arc.turn_direction=0;
            }
            else{ arc.turn_direction =1;}

            arc.type=2;

            survey_points.insert(survey_points.begin()+ i + 1, arc);
            line_index++;

        }
        geometry_msgs::Point temp;
        for(int i=0; i<survey_points.size(); i++){
            if(i%4==2){
                temp=survey_points[i].point_end;
                survey_points[i].point_end=survey_points[i].point_begin;
                survey_points[i].point_begin=temp;
            }
        }
        geometry_msgs::Point temp1;
        if(side==2){
            for(int i=0; i<survey_points.size(); i++){
                temp1=survey_points[i].point_end;
                survey_points[i].point_end=survey_points[i].point_begin;
                survey_points[i].point_begin=temp1;
                if(survey_points[i].type == 2){
                    survey_points[i].turn_direction = !survey_points[i].turn_direction;
                }
            }
            reverse(survey_points.begin(), survey_points.end());
        }

    }
    else if(side==1 || side==3){
        int line_index =0;
        PathSegment arc;
        for(int i=0; i<survey_points.size()-1; i=i+2 ){
            if(line_index%2==0){
                arc.center.x= (survey_points[i].point_begin.x + survey_points[i+1].point_begin.x) / 2.0;
                arc.center.y= (survey_points[i].point_begin.y + survey_points[i+1].point_begin.y) / 2.0;

                arc.point_begin.x = arc.center.x - sweep_line_direction.x*d/2;
                arc.point_begin.y = arc.center.y  - sweep_line_direction.y*d/2;

                arc.point_end.x = arc.center.x + sweep_line_direction.x*d/2;
                arc.point_end.y = arc.center.y + sweep_line_direction.y*d/2;

                survey_points[i].point_begin=arc.point_begin;
                survey_points[i+1].point_begin=arc.point_end;

            }
            else{
                arc.center.x= (survey_points[i].point_end.x + survey_points[i+1].point_end.x) / 2.0;
                arc.center.y= (survey_points[i].point_end.y + survey_points[i+1].point_end.y) / 2.0;

                arc.point_begin.x = arc.center.x - sweep_line_direction.x*d/2;
                arc.point_begin.y = arc.center.y  - sweep_line_direction.y*d/2;

                arc.point_end.x = arc.center.x + sweep_line_direction.x*d/2;
                arc.point_end.y = arc.center.y + sweep_line_direction.y*d/2;

                survey_points[i].point_end=arc.point_begin;
                survey_points[i+1].point_end=arc.point_end;
            }
            if(line_index%2 ==0){
                arc.turn_direction=1;
            }
            else{ arc.turn_direction =0;}

            arc.type=2;


            survey_points.insert(survey_points.begin()+ i + 1, arc);
            line_index++;

        }
        geometry_msgs::Point temp;
        for(int i=0; i<survey_points.size(); i++){
            if(i%4==0){
                temp=survey_points[i].point_end;
                survey_points[i].point_end=survey_points[i].point_begin;
                survey_points[i].point_begin=temp;
            }
        }
        geometry_msgs::Point temp1;
        if(side==3){
            for(int i=0; i<survey_points.size(); i++){
                temp1=survey_points[i].point_end;
                survey_points[i].point_end=survey_points[i].point_begin;
                survey_points[i].point_begin=temp1;
                if(survey_points[i].type == 2){
                    survey_points[i].turn_direction = !survey_points[i].turn_direction;
                }
            }
            reverse(survey_points.begin(), survey_points.end());
        }
    }


}

int survey_path_generator::choose_side(geometry_msgs::Point vehicle_pose){
    int which_side;
    double distance_to_begin;

    distance_to_begin = commons::distance_between_points(vehicle_pose, this->survey_points[0].point_begin);
    which_side = 0;

    std::cout<<commons::distance_between_points(vehicle_pose, this->survey_points[0].point_begin)<<" s0"<<std::endl;
    std::cout<<commons::distance_between_points(vehicle_pose, this->survey_points[0].point_end)<<" s1"<<std::endl;
    std::cout<< commons::distance_between_points(vehicle_pose, this->survey_points[this->survey_points.size()-1].point_begin)<<" s2"<<std::endl;
    std::cout<<commons::distance_between_points(vehicle_pose, this->survey_points[this->survey_points.size()-1].point_end)<<" s3"<<std::endl;

    if(commons::distance_between_points(vehicle_pose, this->survey_points[0].point_end)<distance_to_begin){
        distance_to_begin = commons::distance_between_points(vehicle_pose, this->survey_points[0].point_end);
        which_side = 1;
    }
    if(commons::distance_between_points(vehicle_pose, this->survey_points[this->survey_points.size()-1].point_begin)<distance_to_begin){
        distance_to_begin = commons::distance_between_points(vehicle_pose, this->survey_points[this->survey_points.size()-1].point_begin);
        which_side = 2;
    }
    if(commons::distance_between_points(vehicle_pose, this->survey_points[this->survey_points.size()-1].point_end)<distance_to_begin){
        distance_to_begin = commons::distance_between_points(vehicle_pose, this->survey_points[this->survey_points.size()-1].point_end);
        which_side = 3;
    }

    std::cout<<"side "<<which_side<<std::endl;
    return which_side;
};

