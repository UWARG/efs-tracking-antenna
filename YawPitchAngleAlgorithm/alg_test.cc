#include <gtest/gtest.h>
#include <iostream>

int n = 2; //number of data points you want to check
// Test data array
double lat_1[2] = {54.9967051, 54.9967051};
double long_1[2] = {-61.415482, -61.415482};
float alt_1[2] = {0, 0};

double lat_2[2] = {54.7477657, 55.0530773};
double long_2[2] = {-61.149705, -61.706192};
float alt_2[2] = {0.12, 0.12};

//Data to compare
/*arr[n][y] where n is number of data points, 
y = 0 is true value
y = 1 is old value
y = 2 is new value
new and old value should be updated with every algorithm improvement 
*/
double lat_seg[3][2] = {{27.61988823, -6.2545322}, //negatives are important!
                       {27.6807, -6.2683},
                       {27.6807, -6.2683}};
double long_seg[3][2] = {{17.01982789, 18.47586596},
                        {17.0573, 18.5165},
                        {17.0573, 18.5165}};
double vector[3][2] = {{32.41522194, 19.51812144},
                      {32.4805, 19.5638},
                      {32.4805, 19.5638}};

double pitch_rot[3][2] = {{0.0660224, 0.2642947},
                          {0.070640, 0.199218},
                          {0.070640, 0.199218}};
double yaw_rot[3][2] = {{211.67199328, 71.1909919}, //true
                       {211.678561, 71.167397}, //old
                       {211.678561, 71.167397}}; //new

// Demonstrate some basic assertions.
TEST(LatLongSides, BasicAssertions) //checks if lat and long segments are closer to the true value than their old values were
{ 
  for(int i = 0; i < n; i++)
  {
    EXPECT_TRUE(abs(lat_seg[0][i] - lat_seg[1][i]) >= abs(lat_seg[0][i] - lat_seg[2][i]));
    EXPECT_TRUE(abs(long_seg[0][i] - long_seg[1][i]) >= abs(long_seg[0][i] - long_seg[2][i]));
  }
}

TEST(Vector, BasicAssertions) //checks if vector length is closer to the true value than its old value was
{ 
  for(int j = 0; j < n; j++)
  {
    EXPECT_TRUE(abs(vector[0][j] - vector[1][j]) >= abs(vector[0][j] - vector[2][j]));
  }
}

TEST(YawRotation, BasicAssertions) //checks if yaw rotation angle is closer to the true value than its old value was
{ 
  // Expect equality.
  for(int k = 0; k < n; k++)
  {
    EXPECT_TRUE(abs(yaw_rot[0][k] - yaw_rot[1][k]) >= abs(yaw_rot[0][k] - yaw_rot[2][k]));
  }
  //std::cout << "Yaw: True: " << yaw_rot[0][0] << "Old: " << yaw_rot[1][0] << "New: " << yaw_rot[2][0];
}

TEST(PitchRotation, BasicAssertions) //checks if pitch rotation angle is closer to the true value than its old value was
{ 
  // Expect equality.
  for(int l = 0; l < n; l++)
  {
    EXPECT_TRUE(abs(pitch_rot[0][l] - pitch_rot[1][l]) >= abs(pitch_rot[0][l] - pitch_rot[2][l]));
  }
  //std::cout << "Pitch: True: " << pitch_rot[0][0] << "Old: " << pitch_rot[1][0] << "New: " << pitch_rot[2][0];
}