#include <random>
#include <cmath>
#include <algorithm>

#include <pkg_utils/testing_statistics.h>

namespace Test
{

	Statistics::Statistics()
	{
		/* Create date-time character */
		time_t rawtime;
		struct tm* timeinfo;
		char date_ch[10];

		time(&rawtime);
		timeinfo = localtime(&rawtime);
		strftime(date_ch, 10, "%d_%m_%y", timeinfo);

		/* Convert to string */
		for (auto& ch:date_ch) 
			date_str.push_back(ch);
	}

	double
	Statistics::sampleNormal(double mean, double var)
	{
		std::random_device random_device{};
		std::mt19937 gen{random_device()};
		std::normal_distribution<double> d{mean,var};
		return d(gen);
	}

	double
	Statistics::rmse(std::vector<Utils::Point2d> &data_1, const std::vector<Utils::Point2d> &data_2)
	{
		double rmse = 0.0;
		unsigned int it = 0;
		for (auto& p: data_1)
		{
			rmse += p.distanceTo(data_2[it]);
			it++;
		}
		return sqrt(rmse/(double)it);
	}

  void
  Statistics::computeStatistics(
  	std::vector<double> data,
    std::string log_name,
    std::string log_type,
    double &mean,
    double &var,
    double &median,
    double &min,
    double &max)
  {
		/* Data storage for post processing */
		{
			std::ofstream log_file;
 			log_file.open(log_name + "_" + log_type + "_" + date_str);
			for (auto& i: data)
			{
				log_file << i << "\n";
			}
			log_file.close();
		}

		/* Statistics */
		{
			std::ofstream log_file;
 			log_file.open(date_str + log_name + "_" + log_type + "_statistics");
			log_file << log_type << " statistics \n";
   		log_file << "--------------------------------------\n";

   		/* Sort timing data */
			std::sort(data.begin(), data.end(), CompareObject);

   		/* Logging of timing statistics */
 			mean =  this->mean(data);
 			var =  this->variance(data, mean);
 			median =  this->median(data);

 			std::vector<double>::iterator min_it = std::min_element(data.begin(), data.end());
 			std::vector<double>::iterator max_it = std::max_element(data.begin(), data.end());

   		min = data[std::distance(data.begin(), min_it)];
   		max = data[std::distance(data.begin(), max_it)];

 			log_file << "--------------------------------------\n";
 			log_file << "Mean " <<  mean << "\n";
 			log_file << "Standard deviation " << sqrt(var) << "\n";
 			log_file << "Variance " << var << "\n";
 			log_file << "Median " <<  median << "\n";
 			log_file << "Min: " << min << "\n";
 			log_file << "Max " <<  max << "\n";
 			log_file << "--------------------------------------\n";
			log_file.close();
		}
  }

	double
	Statistics::mean(std::vector<double> data)
	{
		double mean = 0;
		for (auto& i: data)
			mean += i;
		return mean / (double)data.size();
	}

	double
	Statistics::variance(std::vector<double> data, double mean)
	{
		double var = 0;
		for (auto &i: data)
			var += (i - mean)*(i - mean);
		return var / (double)data.size();
	}

	double
	Statistics::median(std::vector<double> data)
	{
		int size = data.size();
		if (size % 2 == 0) {
			return (data[(size / 2) - 1] + data[(size / 2)]) / 2.0;
		}
		return data[std::ceil((double)size / 2.0) + 1];
	}

  std::vector<double>
  Statistics::absoluteJitter(std::vector<double> data, double jitter_ref)
  {
  	std::vector<double> jitter;
    for (auto& i: data)
    	jitter.push_back(jitter_ref - i);
    return jitter;
  }
} // Test
