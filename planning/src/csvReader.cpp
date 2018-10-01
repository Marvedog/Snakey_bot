#include <planning/csvReader.h>
#include <ros/package.h>

namespace csv
{

	void readConesFromFile(std::vector<utils::Point3d> &cones, std::string filename) {
  	io::CSVReader<3> in(ros::package::getPath("planning") +"/test_data/" + filename);
  	in.read_header(io::ignore_extra_column, "x", "y", "z");
  	float x; 
		float y;
		float z;
 	 	while(in.read_row(x, y, z)){
    	cones.push_back(utils::Point3d(x, y, z));
  	}
	}

} /* End namespace csv */
