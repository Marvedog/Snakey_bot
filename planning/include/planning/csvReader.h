#ifndef GROUND_TRUTH_DATA_H
#define GROUND_TRUTH_DATA_H

#include <planning/csv.h>
#include <pkg_utils/point3d.h>
#include <vector>

namespace csv
{

	void readConesFromFile(std::vector<utils::Point3d> &wps, std::string filename);

} /* End namespace csv */
#endif
