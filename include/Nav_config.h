// Covariace matrix where it is 6x6 (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
const double pose_covariance[36] = {0.0, 0.0, 0.0, 	0.0, 	0.0, 	0.0, 
									0.0, 0.0, 0.0, 	0.0, 	0.0, 	0.0, 
									0.0, 0.0, 0.0, 	0.0, 	0.0, 	0.0, 
									0.0, 0.0, 0.0, 	0.001, 	0.0,	0.0, 
									0.0, 0.0, 0.0, 	0.0, 	0.001, 	0.0, 
									0.0, 0.0, 0.0, 	0.0, 	0.0, 	0.001};

									/*
									{0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 
									0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 
									0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 
									0.0, 0.0, 0.0, 0.0001, 0.0, 0.0, 
									0.0, 0.0, 0.0, 0.0, 0.0001, 0.0, 
									0.0, 0.0, 0.0, 0.0, 0.0, 0.0001 }
									*/

									/*
									{1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
									0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 
									0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 
									0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 
									0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 
									0.0, 0.0, 0.0, 0.0, 0.0, 1.0};
									*/