/* ------------ Base class for Revolve driverless test framework ---------
	 -----------------------------------------------------------------------
	 Author: Marcus Engebretsen / marcusengebretsen@gmail.com
	 
	 This class should be inherited in your test to provide syncronization
	 between the test threads and the callback threads.

	 How to use: 
	 ------------> int main(int argc, char **argc) 
	 ------------> {
	 ------------> 	 int number_of_callbacks = N;
	 ------------> 	 SnakeTest::TestingBase<number_of_callbacks> base_class; 
	 ------------> 	 return 0; 
	 ------------> } 
*/

#include <vector>
#include <mutex>
#include <cmath>

namespace Test
{

	/* N: number of callbacks */
	template<int N>
	class TestingBase
	{
		public:
			
			/* Default constructor */
			TestingBase()
			: got_topic_(std::vector<bool>(N))
			, mutex_(std::vector<std::mutex>(N))
			{;}

			/* Default destructor */
			~TestingBase() {;}
    	
			/* Set function for shared variable mutex */
    	void setNewMessageReceived(bool val, int n);

			/* Get function for shared variable mutex */	
    	bool getNewMessageReceived(int n);
		
		private: 
			
			/* Keeping track of whether a new message is received on each topic */
			std::vector<bool> got_topic_;
			
			/* Mutex for each of the above topics */
			std::vector<std::mutex> mutex_;
	};
	
	template<int N> void
	TestingBase<N>::setNewMessageReceived(bool val, int n)
	{
		/* Thread safe set-value */
		mutex_[n].lock(); 
		got_topic_[n] = val; 
		mutex_[n].unlock(); 
	}
	
	template<int N> bool
	TestingBase<N>::getNewMessageReceived(int n)
	{
		/* Thread safe get-value */
		mutex_[n].lock(); 
		bool val = got_topic_[n]; 
		mutex_[n].unlock(); 
		return val;
	}
} /* End namespace Test */
