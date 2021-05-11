#include <iostream>
#include "obj_reader.h"
#include "point_sets_fitting.h"

using namespace std;

int main(int argc, char** argv)
{
        objReader template_obj;
        objReader target_obj;

        template_obj.objLoadFile("");
        target_obj.objLoadFile("");

        template_obj.objLoadModel();
        target_obj.objLoadModel();

        point_sets_fitting pfit;
        auto result = pfit.Fitting(template_obj, target_obj);

        auto transformation = get<0>(result);

        return 0;
}
