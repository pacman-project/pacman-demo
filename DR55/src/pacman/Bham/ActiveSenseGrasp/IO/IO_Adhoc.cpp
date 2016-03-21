
/** @file IO_Adhoc.cpp
 *
 * PaCMan IO_Adhoc definitions
 *
 */



#include "pacman/Bham/ActiveSenseGrasp/IO/IO_Adhoc.h"
#include <octomap/math/Utils.h>
#include <map>

namespace pacman {

namespace io_adhoc {

void add_testData(oct::AngleOcTree& tree){
    oct::point3d origin (0.01f, 0.01f, 0.02f);
    oct::point3d point_on_surface (0.5f, -0.5f, 0.01f);



    oct::Pointcloud cloud;

    for (int i=-100; i<101; i++) {
        for (int j=-100; j<101; j++) {
            oct::point3d rotated = point_on_surface;
            rotated.rotate_IP(0, DEG2RAD(i*0.5), DEG2RAD(j*0.5));
            cloud.push_back(rotated);
        }
    }

    // insert in global coordinates:
    tree.insertPointCloud(cloud, origin);
}


std::string toString(int i)
{
//    S_RANDOM,
//    S_CONTACT_BASED,
//    S_CONTACT_BASED2,
//    S_CONTACT_BASED3,
//    S_INFORMATION_GAIN,
//    S_SEQUENTIAL,
//    S_NONE
    std::map<std::string, int> retMap;

    retMap["random"] = 0;
    retMap["contact_based"] = 1;
    retMap["contact_based_v2"] = 2;
    retMap["contact_based_v3"] = 3;
    retMap["information_gain"] = 4;
    retMap["sequential"] = 5;


    for (auto j = retMap.begin(); j != retMap.end(); ++j)
        if (j->second == i) return j->first;
    return std::string("none");
}


void log_out(FILE* out, double landmark, const std::vector<double>& collisionProfile){
    if(!out){
        printf("log_out: no file descriptor to log!\n");
        return;
    }

    fprintf(out,"%lf ", landmark);
    for(double c : collisionProfile)
        fprintf(out,"%lf ",c);
    fprintf(out,"\n");
}

void log_out(FILE* out, const std::string& annotation, int experiment_id, int trial, int selection_method, bool has_contacts,
             int view_id, double landmark, double view_value, double entropy, double inf_gain, double collision_prob){


    if(!out){
        printf("log_out: no file descriptor to log!\n");
        return;
    }

    std::string selection_method_str = toString(selection_method);

    if(entropy != entropy) entropy = 0;
    if(collision_prob != collision_prob) collision_prob = 0;
    if(inf_gain != inf_gain) inf_gain = 0;

    fprintf(out,"%s %d %d %s %d %d %lf %lf %lf %lf %lf\n",annotation.c_str(), experiment_id, trial, selection_method_str.c_str(), has_contacts,
                                                      view_id, landmark, view_value, entropy, inf_gain, collision_prob);
    fflush(out);

}




}


}


