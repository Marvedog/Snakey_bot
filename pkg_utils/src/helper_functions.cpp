#include "pkg_utils/helper_functions.h"
#include <math.h>

double normalize_angle_plusmin_pi(double angle){
	while(angle > M_PI) {
		angle -= 2*M_PI;
	}
	while(angle < -M_PI) {
		angle += 2*M_PI;
	}
	return angle;
}

double sign(double input){
	if (input > 0) return 1;
	if (input < 0) return -1;
	return 0;
}

double saturate(double value, double min, double max){
	if(value > max){
		return max;
	} else if(value < min){
		return min;
	}
	return value;
}

int partialFactorial(int x, std::size_t n){
	int y = 1;
	for (; n > 0; --n, --x) {
		y *= x;
	}
	return y;
}

// template<typename T>
// void f(T s)
// {
//     std::cout << s << '\n';
// }


// int main()
// {
//     f<double>(1); // instantiates and calls f<double>(double)
//     f<>('a'); // instantiates and calls f<char>(char)
//     f(7); // instantiates and calls f<int>(int)
//     void (*ptr)(std::string) = f; // instantiates f<string>(string)
// }


// template<typename type>
// void f(type s)
// {
//     std::cout << s << '\n';
// }

