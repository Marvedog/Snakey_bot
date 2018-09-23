#ifndef STATISTICS_H
#define STATISTICS_H

#include <time.h>
#include <fstream>
#include <vector>

#include <pkg_utils/point2d.h>

namespace Test
{

	/* Compute multivariate statistics for testing */
	class Statistics
	{
		public:
			Statistics();

			/* Sample from a univariate normal distribution */
			double sampleNormal(double mean, double var);

			/* Compute rmse between two arrays of data */
			double rmse(std::vector<Utils::Point2d> &data_1, const std::vector<Utils::Point2d> &data_2);

			/* Compute statistics and store to file  */
      void computeStatistics(std::vector<double> data, std::string log_name, std::string log_type, double &mean, double &var, double &median, double &min, double &max);

    private:

			std::string date_str;

      /* Utility data structure for sorting */
      struct Compare
  		{
  			bool operator() (double a, double b) {return (a<b);}
  		} CompareObject;

			/* 1D mean */
			double mean(std::vector<double> data);

			/* 1D variance */
			double variance(std::vector<double> data, double mean);

			/* 1D median  */
			double median(std::vector<double> data);

      /* Positive jitter if it takes longer than reference */
      std::vector<double> absoluteJitter(std::vector<double> data, double jitter_ref);
	};
} // Test

#endif
