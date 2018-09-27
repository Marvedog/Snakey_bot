#include "pkg_utils/track.h"
#include "pkg_utils/eigen_helpers.h"
#include <gtest/gtest.h>


TEST(TrackTest, EmptyCtor) {
    Track track;
    ASSERT_TRUE(true);
}


int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
