//
// Created by tolga on 31.05.21.
//

#include "jacobian.h"

MatrixXd calculateJacobian(const VectorXd theta){

MatrixXd jacobian(12,7);

jacobian <<
sin(theta[6])*(cos(theta[5])*(1.0*sin(theta[3])*(1.0*cos(theta[0])*sin(theta[2]) -
                                           1.0*cos(theta[1])*cos(theta[2])*sin(theta[0])) - 1.0*cos(theta[3])*sin(theta[0])*sin(theta[1])) -
             1.0*sin(theta[5])*(cos(theta[4])*(cos(theta[3])*(1.0*cos(theta[0])*sin(theta[2]) -
                                                        1.0*cos(theta[1])*cos(theta[2])*sin(theta[0])) + 1.0*sin(theta[0])*sin(theta[1])*sin(theta[3])) +
                              sin(theta[4])*(1.0*cos(theta[0])*cos(theta[2]) + 1.0*cos(theta[1])*sin(theta[0])*sin(theta[2])))) +
cos(theta[6])*(sin(theta[5])*(1.0*sin(theta[3])*(1.0*cos(theta[0])*sin(theta[2]) -
                                           1.0*cos(theta[1])*cos(theta[2])*sin(theta[0])) - 1.0*cos(theta[3])*sin(theta[0])*sin(theta[1])) +
             cos(theta[5])*(cos(theta[4])*(cos(theta[3])*(1.0*cos(theta[0])*sin(theta[2]) -
                                                    1.0*cos(theta[1])*cos(theta[2])*sin(theta[0])) + 1.0*sin(theta[0])*sin(theta[1])*sin(theta[3])) +
                          sin(theta[4])*(1.0*cos(theta[0])*cos(theta[2]) + 1.0*cos(theta[1])*sin(theta[0])*sin(theta[2])))),
        sin(theta[6])*(cos(theta[0])*sin(theta[5])*(cos(theta[1])*cos(theta[4])*sin(theta[3]) -
1.0*sin(theta[1])*sin(theta[2])*sin(theta[4]) + cos(theta[2])*cos(theta[3])*cos(theta[4])*sin(theta[1])) +
cos(theta[0])*cos(theta[5])*(cos(theta[1])*cos(theta[3]) - 1.0*cos(theta[2])*sin(theta[1])*sin(theta[3]))) -
cos(theta[6])*(cos(theta[0])*cos(theta[5])*(cos(theta[1])*cos(theta[4])*sin(theta[3]) -
1.0*sin(theta[1])*sin(theta[2])*sin(theta[4]) + cos(theta[2])*cos(theta[3])*cos(theta[4])*sin(theta[1])) -
cos(theta[0])*sin(theta[5])*(cos(theta[1])*cos(theta[3]) - 1.0*cos(theta[2])*sin(theta[1])*sin(theta[3]))),
sin(theta[6])*(sin(theta[5])*(sin(theta[4])*(sin(theta[0])*sin(theta[2]) + cos(theta[0])*cos(theta[1])*cos(theta[2]))
- cos(theta[3])*cos(theta[4])*(cos(theta[2])*sin(theta[0]) - cos(theta[0])*cos(theta[1])*sin(theta[2]))) +
cos(theta[5])*sin(theta[3])*(cos(theta[2])*sin(theta[0]) - cos(theta[0])*cos(theta[1])*sin(theta[2]))) -
cos(theta[6])*(cos(theta[5])*(sin(theta[4])*(sin(theta[0])*sin(theta[2]) +
cos(theta[0])*cos(theta[1])*cos(theta[2])) - cos(theta[3])*cos(theta[4])*(cos(theta[2])*sin(theta[0]) -
cos(theta[0])*cos(theta[1])*sin(theta[2]))) - 1.0*sin(theta[3])*sin(theta[5])*(cos(theta[2])*sin(theta[0]) -
cos(theta[0])*cos(theta[1])*sin(theta[2]))),
sin(theta[6])*(cos(theta[5])*(1.0*cos(theta[3])*(1.0*sin(theta[0])*sin(theta[2]) +
1.0*cos(theta[0])*cos(theta[1])*cos(theta[2])) - 1.0*cos(theta[0])*sin(theta[1])*sin(theta[3])) +
1.0*cos(theta[4])*sin(theta[5])*(sin(theta[3])*(1.0*sin(theta[0])*sin(theta[2]) +
1.0*cos(theta[0])*cos(theta[1])*cos(theta[2])) + 1.0*cos(theta[0])*cos(theta[3])*sin(theta[1]))) +
cos(theta[6])*(sin(theta[5])*(1.0*cos(theta[3])*(1.0*sin(theta[0])*sin(theta[2]) +
1.0*cos(theta[0])*cos(theta[1])*cos(theta[2])) - 1.0*cos(theta[0])*sin(theta[1])*sin(theta[3])) -
cos(theta[4])*cos(theta[5])*(sin(theta[3])*(1.0*sin(theta[0])*sin(theta[2]) +
1.0*cos(theta[0])*cos(theta[1])*cos(theta[2])) + 1.0*cos(theta[0])*cos(theta[3])*sin(theta[1]))),
-cos(theta[5] + theta[6])*(cos(theta[0])*cos(theta[1])*cos(theta[4])*sin(theta[2]) -
1.0*cos(theta[2])*cos(theta[4])*sin(theta[0]) - 1.0*cos(theta[0])*sin(theta[1])*sin(theta[3])*sin(theta[4]) +
cos(theta[3])*sin(theta[0])*sin(theta[2])*sin(theta[4]) +
cos(theta[0])*cos(theta[1])*cos(theta[2])*cos(theta[3])*sin(theta[4])),
cos(theta[6])*(cos(theta[5])*(1.0*sin(theta[3])*(1.0*sin(theta[0])*sin(theta[2]) +
1.0*cos(theta[0])*cos(theta[1])*cos(theta[2])) + 1.0*cos(theta[0])*cos(theta[3])*sin(theta[1])) -
sin(theta[5])*(cos(theta[4])*(cos(theta[3])*(1.0*sin(theta[0])*sin(theta[2]) +
1.0*cos(theta[0])*cos(theta[1])*cos(theta[2])) - 1.0*cos(theta[0])*sin(theta[1])*sin(theta[3])) +
sin(theta[4])*(1.0*cos(theta[2])*sin(theta[0]) - 1.0*cos(theta[0])*cos(theta[1])*sin(theta[2])))) -
sin(theta[6])*(sin(theta[5])*(1.0*sin(theta[3])*(1.0*sin(theta[0])*sin(theta[2]) +
1.0*cos(theta[0])*cos(theta[1])*cos(theta[2])) + 1.0*cos(theta[0])*cos(theta[3])*sin(theta[1])) +
1.0*cos(theta[5])*(cos(theta[4])*(cos(theta[3])*(1.0*sin(theta[0])*sin(theta[2]) +
1.0*cos(theta[0])*cos(theta[1])*cos(theta[2])) - 1.0*cos(theta[0])*sin(theta[1])*sin(theta[3])) +
sin(theta[4])*(1.0*cos(theta[2])*sin(theta[0]) - 1.0*cos(theta[0])*cos(theta[1])*sin(theta[2])))),
cos(theta[6])*(cos(theta[5])*(1.0*sin(theta[3])*(1.0*sin(theta[0])*sin(theta[2]) +1.0*cos(theta[0])*cos(theta[1])*cos(theta[2])) + 1.0*cos(theta[0])*cos(theta[3])*sin(theta[1])) -
1.0*sin(theta[5])*(cos(theta[4])*(cos(theta[3])*(1.0*sin(theta[0])*sin(theta[2]) +
1.0*cos(theta[0])*cos(theta[1])*cos(theta[2])) - 1.0*cos(theta[0])*sin(theta[1])*sin(theta[3])) +
sin(theta[4])*(1.0*cos(theta[2])*sin(theta[0]) - 1.0*cos(theta[0])*cos(theta[1])*sin(theta[2])))) -
sin(theta[6])*(sin(theta[5])*(1.0*sin(theta[3])*(1.0*sin(theta[0])*sin(theta[2]) +
1.0*cos(theta[0])*cos(theta[1])*cos(theta[2])) + 1.0*cos(theta[0])*cos(theta[3])*sin(theta[1])) +
cos(theta[5])*(cos(theta[4])*(cos(theta[3])*(1.0*sin(theta[0])*sin(theta[2]) +
1.0*cos(theta[0])*cos(theta[1])*cos(theta[2])) - 1.0*cos(theta[0])*sin(theta[1])*sin(theta[3])) +
sin(theta[4])*(1.0*cos(theta[2])*sin(theta[0]) - 1.0*cos(theta[0])*cos(theta[1])*sin(theta[2]))))
,
0,
sin(theta[6])*(1.0*sin(theta[5])*(cos(theta[4])*(1.0*sin(theta[1])*sin(theta[3]) -
1.0*cos(theta[1])*cos(theta[2])*cos(theta[3])) + 1.0*cos(theta[1])*sin(theta[2])*sin(theta[4])) +
cos(theta[5])*(1.0*cos(theta[3])*sin(theta[1]) + 1.0*cos(theta[1])*cos(theta[2])*sin(theta[3]))) -
cos(theta[6])*(cos(theta[5])*(cos(theta[4])*(1.0*sin(theta[1])*sin(theta[3]) -
1.0*cos(theta[1])*cos(theta[2])*cos(theta[3])) + 1.0*cos(theta[1])*sin(theta[2])*sin(theta[4])) -
sin(theta[5])*(1.0*cos(theta[3])*sin(theta[1]) + 1.0*cos(theta[1])*cos(theta[2])*sin(theta[3]))),
sin(theta[1])*sin(theta[6])*(cos(theta[2])*sin(theta[4])*sin(theta[5]) -
1.0*cos(theta[5])*sin(theta[2])*sin(theta[3]) + cos(theta[3])*cos(theta[4])*sin(theta[2])*sin(theta[5])) -
cos(theta[6])*sin(theta[1])*(sin(theta[2])*sin(theta[3])*sin(theta[5]) + cos(theta[2])*cos(theta[5])*sin(theta[4]) +
cos(theta[3])*cos(theta[4])*cos(theta[5])*sin(theta[2])),
cos(theta[6])*(sin(theta[5])*(cos(theta[1])*sin(theta[3]) + cos(theta[2])*cos(theta[3])*sin(theta[1])) +
cos(theta[4])*cos(theta[5])*(cos(theta[1])*cos(theta[3]) - 1.0*cos(theta[2])*sin(theta[1])*sin(theta[3]))) +
sin(theta[6])*(cos(theta[5])*(cos(theta[1])*sin(theta[3]) + cos(theta[2])*cos(theta[3])*sin(theta[1])) -
1.0*cos(theta[4])*sin(theta[5])*(cos(theta[1])*cos(theta[3]) - 1.0*cos(theta[2])*sin(theta[1])*sin(theta[3]))),
-cos(theta[5] + theta[6])*(cos(theta[4])*sin(theta[1])*sin(theta[2]) + cos(theta[1])*sin(theta[3])*sin(theta[4]) +
cos(theta[2])*cos(theta[3])*sin(theta[1])*sin(theta[4])),
- sin(theta[6])*(1.0*cos(theta[5])*(cos(theta[4])*(1.0*cos(theta[1])*sin(theta[3]) +
1.0*cos(theta[2])*cos(theta[3])*sin(theta[1])) - 1.0*sin(theta[1])*sin(theta[2])*sin(theta[4])) -
sin(theta[5])*(1.0*cos(theta[1])*cos(theta[3]) - 1.0*cos(theta[2])*sin(theta[1])*sin(theta[3]))) -
cos(theta[6])*(sin(theta[5])*(cos(theta[4])*(1.0*cos(theta[1])*sin(theta[3]) +
1.0*cos(theta[2])*cos(theta[3])*sin(theta[1])) - 1.0*sin(theta[1])*sin(theta[2])*sin(theta[4])) +
cos(theta[5])*(1.0*cos(theta[1])*cos(theta[3]) - 1.0*cos(theta[2])*sin(theta[1])*sin(theta[3]))),
- cos(theta[6])*(1.0*sin(theta[5])*(cos(theta[4])*(1.0*cos(theta[1])*sin(theta[3]) +
1.0*cos(theta[2])*cos(theta[3])*sin(theta[1])) - 1.0*sin(theta[1])*sin(theta[2])*sin(theta[4])) +
cos(theta[5])*(1.0*cos(theta[1])*cos(theta[3]) - 1.0*cos(theta[2])*sin(theta[1])*sin(theta[3]))) -
sin(theta[6])*(cos(theta[5])*(cos(theta[4])*(1.0*cos(theta[1])*sin(theta[3]) +
1.0*cos(theta[2])*cos(theta[3])*sin(theta[1])) - 1.0*sin(theta[1])*sin(theta[2])*sin(theta[4])) -
sin(theta[5])*(1.0*cos(theta[1])*cos(theta[3]) - 1.0*cos(theta[2])*sin(theta[1])*sin(theta[3])))
, - sin(theta[6])*(cos(theta[5])*(1.0*sin(theta[3])*(1.0*sin(theta[0])*sin(theta[2]) +
1.0*cos(theta[0])*cos(theta[1])*cos(theta[2])) + 1.0*cos(theta[0])*cos(theta[3])*sin(theta[1])) -
1.0*sin(theta[5])*(cos(theta[4])*(cos(theta[3])*(1.0*sin(theta[0])*sin(theta[2]) +
1.0*cos(theta[0])*cos(theta[1])*cos(theta[2])) - 1.0*cos(theta[0])*sin(theta[1])*sin(theta[3])) +
sin(theta[4])*(1.0*cos(theta[2])*sin(theta[0]) - 1.0*cos(theta[0])*cos(theta[1])*sin(theta[2])))) -
cos(theta[6])*(sin(theta[5])*(1.0*sin(theta[3])*(1.0*sin(theta[0])*sin(theta[2]) +
1.0*cos(theta[0])*cos(theta[1])*cos(theta[2])) + 1.0*cos(theta[0])*cos(theta[3])*sin(theta[1])) +
cos(theta[5])*(cos(theta[4])*(cos(theta[3])*(1.0*sin(theta[0])*sin(theta[2]) +
1.0*cos(theta[0])*cos(theta[1])*cos(theta[2])) - 1.0*cos(theta[0])*sin(theta[1])*sin(theta[3])) +
sin(theta[4])*(1.0*cos(theta[2])*sin(theta[0]) - 1.0*cos(theta[0])*cos(theta[1])*sin(theta[2])))),cos(theta[6])*(cos(theta[5])*sin(theta[0])*(cos(theta[1])*cos(theta[4])*sin(theta[3]) -
1.0*sin(theta[1])*sin(theta[2])*sin(theta[4]) + cos(theta[2])*cos(theta[3])*cos(theta[4])*sin(theta[1])) -
sin(theta[0])*sin(theta[5])*(cos(theta[1])*cos(theta[3]) - 1.0*cos(theta[2])*sin(theta[1])*sin(theta[3]))) -
sin(theta[6])*(sin(theta[0])*sin(theta[5])*(cos(theta[1])*cos(theta[4])*sin(theta[3]) -
1.0*sin(theta[1])*sin(theta[2])*sin(theta[4]) + cos(theta[2])*cos(theta[3])*cos(theta[4])*sin(theta[1])) +
cos(theta[5])*sin(theta[0])*(cos(theta[1])*cos(theta[3]) - 1.0*cos(theta[2])*sin(theta[1])*sin(theta[3]))),
sin(theta[6])*(sin(theta[5])*(sin(theta[4])*(cos(theta[0])*sin(theta[2]) -
1.0*cos(theta[1])*cos(theta[2])*sin(theta[0])) - cos(theta[3])*cos(theta[4])*(cos(theta[0])*cos(theta[2]) +
cos(theta[1])*sin(theta[0])*sin(theta[2]))) + cos(theta[5])*sin(theta[3])*(cos(theta[0])*cos(theta[2]) +
cos(theta[1])*sin(theta[0])*sin(theta[2]))) -
cos(theta[6])*(cos(theta[5])*(sin(theta[4])*(cos(theta[0])*sin(theta[2]) -
1.0*cos(theta[1])*cos(theta[2])*sin(theta[0])) - cos(theta[3])*cos(theta[4])*(cos(theta[0])*cos(theta[2]) +
cos(theta[1])*sin(theta[0])*sin(theta[2]))) - 1.0*sin(theta[3])*sin(theta[5])*(cos(theta[0])*cos(theta[2]) +
cos(theta[1])*sin(theta[0])*sin(theta[2]))),
sin(theta[6])*(cos(theta[5])*(1.0*cos(theta[3])*(1.0*cos(theta[0])*sin(theta[2]) -
1.0*cos(theta[1])*cos(theta[2])*sin(theta[0])) + 1.0*sin(theta[0])*sin(theta[1])*sin(theta[3])) +
1.0*cos(theta[4])*sin(theta[5])*(sin(theta[3])*(1.0*cos(theta[0])*sin(theta[2]) -
1.0*cos(theta[1])*cos(theta[2])*sin(theta[0])) - 1.0*cos(theta[3])*sin(theta[0])*sin(theta[1]))) +
cos(theta[6])*(sin(theta[5])*(1.0*cos(theta[3])*(1.0*cos(theta[0])*sin(theta[2]) -
1.0*cos(theta[1])*cos(theta[2])*sin(theta[0])) + 1.0*sin(theta[0])*sin(theta[1])*sin(theta[3])) -
cos(theta[4])*cos(theta[5])*(sin(theta[3])*(1.0*cos(theta[0])*sin(theta[2]) -
1.0*cos(theta[1])*cos(theta[2])*sin(theta[0])) - 1.0*cos(theta[3])*sin(theta[0])*sin(theta[1]))),
cos(theta[5] + theta[6])*(cos(theta[0])*cos(theta[2])*cos(theta[4]) +
cos(theta[1])*cos(theta[4])*sin(theta[0])*sin(theta[2]) -
1.0*cos(theta[0])*cos(theta[3])*sin(theta[2])*sin(theta[4]) -
1.0*sin(theta[0])*sin(theta[1])*sin(theta[3])*sin(theta[4]) +
cos(theta[1])*cos(theta[2])*cos(theta[3])*sin(theta[0])*sin(theta[4])),
cos(theta[6])*(cos(theta[5])*(1.0*sin(theta[3])*(1.0*cos(theta[0])*sin(theta[2]) -
1.0*cos(theta[1])*cos(theta[2])*sin(theta[0])) - 1.0*cos(theta[3])*sin(theta[0])*sin(theta[1])) -
sin(theta[5])*(cos(theta[4])*(cos(theta[3])*(1.0*cos(theta[0])*sin(theta[2]) -
1.0*cos(theta[1])*cos(theta[2])*sin(theta[0])) + 1.0*sin(theta[0])*sin(theta[1])*sin(theta[3])) +
sin(theta[4])*(1.0*cos(theta[0])*cos(theta[2]) + 1.0*cos(theta[1])*sin(theta[0])*sin(theta[2])))) -
sin(theta[6])*(sin(theta[5])*(1.0*sin(theta[3])*(1.0*cos(theta[0])*sin(theta[2]) -
1.0*cos(theta[1])*cos(theta[2])*sin(theta[0])) - 1.0*cos(theta[3])*sin(theta[0])*sin(theta[1])) +
1.0*cos(theta[5])*(cos(theta[4])*(cos(theta[3])*(1.0*cos(theta[0])*sin(theta[2]) -
1.0*cos(theta[1])*cos(theta[2])*sin(theta[0])) + 1.0*sin(theta[0])*sin(theta[1])*sin(theta[3])) +
sin(theta[4])*(1.0*cos(theta[0])*cos(theta[2]) + 1.0*cos(theta[1])*sin(theta[0])*sin(theta[2])))),
cos(theta[6])*(cos(theta[5])*(1.0*sin(theta[3])*(1.0*cos(theta[0])*sin(theta[2]) -
1.0*cos(theta[1])*cos(theta[2])*sin(theta[0])) - 1.0*cos(theta[3])*sin(theta[0])*sin(theta[1])) -
1.0*sin(theta[5])*(cos(theta[4])*(cos(theta[3])*(1.0*cos(theta[0])*sin(theta[2]) -
1.0*cos(theta[1])*cos(theta[2])*sin(theta[0])) + 1.0*sin(theta[0])*sin(theta[1])*sin(theta[3])) +
sin(theta[4])*(1.0*cos(theta[0])*cos(theta[2]) + 1.0*cos(theta[1])*sin(theta[0])*sin(theta[2])))) -
sin(theta[6])*(sin(theta[5])*(1.0*sin(theta[3])*(1.0*cos(theta[0])*sin(theta[2]) -
1.0*cos(theta[1])*cos(theta[2])*sin(theta[0])) - 1.0*cos(theta[3])*sin(theta[0])*sin(theta[1])) +
cos(theta[5])*(cos(theta[4])*(cos(theta[3])*(1.0*cos(theta[0])*sin(theta[2]) -
1.0*cos(theta[1])*cos(theta[2])*sin(theta[0])) + 1.0*sin(theta[0])*sin(theta[1])*sin(theta[3])) +
sin(theta[4])*(1.0*cos(theta[0])*cos(theta[2]) + 1.0*cos(theta[1])*sin(theta[0])*sin(theta[2])))),cos(theta[6])*(cos(theta[5])*(1.0*sin(theta[3])*(1.0*cos(theta[0])*sin(theta[2]) -
1.0*cos(theta[1])*cos(theta[2])*sin(theta[0])) - 1.0*cos(theta[3])*sin(theta[0])*sin(theta[1])) -
1.0*sin(theta[5])*(cos(theta[4])*(cos(theta[3])*(1.0*cos(theta[0])*sin(theta[2]) -
1.0*cos(theta[1])*cos(theta[2])*sin(theta[0])) + 1.0*sin(theta[0])*sin(theta[1])*sin(theta[3])) +
sin(theta[4])*(1.0*cos(theta[0])*cos(theta[2]) + 1.0*cos(theta[1])*sin(theta[0])*sin(theta[2])))) -
1.0*sin(theta[6])*(sin(theta[5])*(1.0*sin(theta[3])*(1.0*cos(theta[0])*sin(theta[2]) -
1.0*cos(theta[1])*cos(theta[2])*sin(theta[0])) - 1.0*cos(theta[3])*sin(theta[0])*sin(theta[1])) +
cos(theta[5])*(cos(theta[4])*(cos(theta[3])*(1.0*cos(theta[0])*sin(theta[2]) -
1.0*cos(theta[1])*cos(theta[2])*sin(theta[0])) + 1.0*sin(theta[0])*sin(theta[1])*sin(theta[3])) +
sin(theta[4])*(1.0*cos(theta[0])*cos(theta[2]) + 1.0*cos(theta[1])*sin(theta[0])*sin(theta[2])))),
sin(theta[6])*(cos(theta[0])*cos(theta[5])*(cos(theta[1])*cos(theta[4])*sin(theta[3]) -
1.0*sin(theta[1])*sin(theta[2])*sin(theta[4]) + cos(theta[2])*cos(theta[3])*cos(theta[4])*sin(theta[1])) -
cos(theta[0])*sin(theta[5])*(cos(theta[1])*cos(theta[3]) - 1.0*cos(theta[2])*sin(theta[1])*sin(theta[3]))) +
cos(theta[6])*(cos(theta[0])*sin(theta[5])*(cos(theta[1])*cos(theta[4])*sin(theta[3]) -
1.0*sin(theta[1])*sin(theta[2])*sin(theta[4]) + cos(theta[2])*cos(theta[3])*cos(theta[4])*sin(theta[1])) +
cos(theta[0])*cos(theta[5])*(cos(theta[1])*cos(theta[3]) - 1.0*cos(theta[2])*sin(theta[1])*sin(theta[3]))),
cos(theta[6])*(sin(theta[5])*(sin(theta[4])*(sin(theta[0])*sin(theta[2]) +
cos(theta[0])*cos(theta[1])*cos(theta[2])) - cos(theta[3])*cos(theta[4])*(cos(theta[2])*sin(theta[0]) -
cos(theta[0])*cos(theta[1])*sin(theta[2]))) + cos(theta[5])*sin(theta[3])*(cos(theta[2])*sin(theta[0]) -
cos(theta[0])*cos(theta[1])*sin(theta[2]))) +
sin(theta[6])*(cos(theta[5])*(sin(theta[4])*(sin(theta[0])*sin(theta[2]) +
cos(theta[0])*cos(theta[1])*cos(theta[2])) - cos(theta[3])*cos(theta[4])*(cos(theta[2])*sin(theta[0]) -
cos(theta[0])*cos(theta[1])*sin(theta[2]))) - 1.0*sin(theta[3])*sin(theta[5])*(cos(theta[2])*sin(theta[0]) -
cos(theta[0])*cos(theta[1])*sin(theta[2]))),
cos(theta[6])*(cos(theta[5])*(1.0*cos(theta[3])*(1.0*sin(theta[0])*sin(theta[2]) +
1.0*cos(theta[0])*cos(theta[1])*cos(theta[2])) - 1.0*cos(theta[0])*sin(theta[1])*sin(theta[3])) +
1.0*cos(theta[4])*sin(theta[5])*(sin(theta[3])*(1.0*sin(theta[0])*sin(theta[2]) +
1.0*cos(theta[0])*cos(theta[1])*cos(theta[2])) + 1.0*cos(theta[0])*cos(theta[3])*sin(theta[1]))) -
1.0*sin(theta[6])*(sin(theta[5])*(1.0*cos(theta[3])*(1.0*sin(theta[0])*sin(theta[2]) +
1.0*cos(theta[0])*cos(theta[1])*cos(theta[2])) - 1.0*cos(theta[0])*sin(theta[1])*sin(theta[3])) -
cos(theta[4])*cos(theta[5])*(sin(theta[3])*(1.0*sin(theta[0])*sin(theta[2]) +
1.0*cos(theta[0])*cos(theta[1])*cos(theta[2])) + 1.0*cos(theta[0])*cos(theta[3])*sin(theta[1]))),
sin(theta[5] + theta[6])*(cos(theta[0])*cos(theta[1])*cos(theta[4])*sin(theta[2]) -
1.0*cos(theta[2])*cos(theta[4])*sin(theta[0]) - 1.0*cos(theta[0])*sin(theta[1])*sin(theta[3])*sin(theta[4]) +
cos(theta[3])*sin(theta[0])*sin(theta[2])*sin(theta[4]) +
cos(theta[0])*cos(theta[1])*cos(theta[2])*cos(theta[3])*sin(theta[4])), -
cos(theta[6])*(sin(theta[5])*(1.0*sin(theta[3])*(1.0*sin(theta[0])*sin(theta[2]) +
1.0*cos(theta[0])*cos(theta[1])*cos(theta[2])) + 1.0*cos(theta[0])*cos(theta[3])*sin(theta[1])) +
1.0*cos(theta[5])*(cos(theta[4])*(cos(theta[3])*(1.0*sin(theta[0])*sin(theta[2]) +
1.0*cos(theta[0])*cos(theta[1])*cos(theta[2])) - 1.0*cos(theta[0])*sin(theta[1])*sin(theta[3])) +
sin(theta[4])*(1.0*cos(theta[2])*sin(theta[0]) - 1.0*cos(theta[0])*cos(theta[1])*sin(theta[2])))) -
1.0*sin(theta[6])*(cos(theta[5])*(1.0*sin(theta[3])*(1.0*sin(theta[0])*sin(theta[2]) +
1.0*cos(theta[0])*cos(theta[1])*cos(theta[2])) + 1.0*cos(theta[0])*cos(theta[3])*sin(theta[1])) -
sin(theta[5])*(cos(theta[4])*(cos(theta[3])*(1.0*sin(theta[0])*sin(theta[2]) +
1.0*cos(theta[0])*cos(theta[1])*cos(theta[2])) - 1.0*cos(theta[0])*sin(theta[1])*sin(theta[3])) +
sin(theta[4])*(1.0*cos(theta[2])*sin(theta[0]) - 1.0*cos(theta[0])*cos(theta[1])*sin(theta[2])))), -
sin(theta[6])*(cos(theta[5])*(1.0*sin(theta[3])*(1.0*sin(theta[0])*sin(theta[2]) +
1.0*cos(theta[0])*cos(theta[1])*cos(theta[2])) + 1.0*cos(theta[0])*cos(theta[3])*sin(theta[1])) -
1.0*sin(theta[5])*(cos(theta[4])*(cos(theta[3])*(1.0*sin(theta[0])*sin(theta[2]) +1.0*cos(theta[0])*cos(theta[1])*cos(theta[2])) - 1.0*cos(theta[0])*sin(theta[1])*sin(theta[3])) +
sin(theta[4])*(1.0*cos(theta[2])*sin(theta[0]) - 1.0*cos(theta[0])*cos(theta[1])*sin(theta[2])))) -
1.0*cos(theta[6])*(sin(theta[5])*(1.0*sin(theta[3])*(1.0*sin(theta[0])*sin(theta[2]) +
1.0*cos(theta[0])*cos(theta[1])*cos(theta[2])) + 1.0*cos(theta[0])*cos(theta[3])*sin(theta[1])) +
cos(theta[5])*(cos(theta[4])*(cos(theta[3])*(1.0*sin(theta[0])*sin(theta[2]) +
1.0*cos(theta[0])*cos(theta[1])*cos(theta[2])) - 1.0*cos(theta[0])*sin(theta[1])*sin(theta[3])) +
sin(theta[4])*(1.0*cos(theta[2])*sin(theta[0]) - 1.0*cos(theta[0])*cos(theta[1])*sin(theta[2]))))
,
0,
cos(theta[6])*(1.0*sin(theta[5])*(cos(theta[4])*(1.0*sin(theta[1])*sin(theta[3]) -
1.0*cos(theta[1])*cos(theta[2])*cos(theta[3])) + 1.0*cos(theta[1])*sin(theta[2])*sin(theta[4])) +
cos(theta[5])*(1.0*cos(theta[3])*sin(theta[1]) + 1.0*cos(theta[1])*cos(theta[2])*sin(theta[3]))) +
1.0*sin(theta[6])*(cos(theta[5])*(cos(theta[4])*(1.0*sin(theta[1])*sin(theta[3]) -
1.0*cos(theta[1])*cos(theta[2])*cos(theta[3])) + 1.0*cos(theta[1])*sin(theta[2])*sin(theta[4])) -
sin(theta[5])*(1.0*cos(theta[3])*sin(theta[1]) + 1.0*cos(theta[1])*cos(theta[2])*sin(theta[3]))),
sin(theta[1])*sin(theta[6])*(sin(theta[2])*sin(theta[3])*sin(theta[5]) + cos(theta[2])*cos(theta[5])*sin(theta[4]) +
cos(theta[3])*cos(theta[4])*cos(theta[5])*sin(theta[2])) +
cos(theta[6])*sin(theta[1])*(cos(theta[2])*sin(theta[4])*sin(theta[5]) -
1.0*cos(theta[5])*sin(theta[2])*sin(theta[3]) + cos(theta[3])*cos(theta[4])*sin(theta[2])*sin(theta[5])),
cos(theta[6])*(cos(theta[5])*(cos(theta[1])*sin(theta[3]) + cos(theta[2])*cos(theta[3])*sin(theta[1])) -
1.0*cos(theta[4])*sin(theta[5])*(cos(theta[1])*cos(theta[3]) - 1.0*cos(theta[2])*sin(theta[1])*sin(theta[3]))) -
1.0*sin(theta[6])*(sin(theta[5])*(cos(theta[1])*sin(theta[3]) + cos(theta[2])*cos(theta[3])*sin(theta[1])) +
cos(theta[4])*cos(theta[5])*(cos(theta[1])*cos(theta[3]) - 1.0*cos(theta[2])*sin(theta[1])*sin(theta[3]))),
sin(theta[5] + theta[6])*(cos(theta[4])*sin(theta[1])*sin(theta[2]) + cos(theta[1])*sin(theta[3])*sin(theta[4]) +
cos(theta[2])*cos(theta[3])*sin(theta[1])*sin(theta[4])),
1.0*sin(theta[6])*(sin(theta[5])*(cos(theta[4])*(1.0*cos(theta[1])*sin(theta[3]) +
1.0*cos(theta[2])*cos(theta[3])*sin(theta[1])) - 1.0*sin(theta[1])*sin(theta[2])*sin(theta[4])) +
cos(theta[5])*(1.0*cos(theta[1])*cos(theta[3]) - 1.0*cos(theta[2])*sin(theta[1])*sin(theta[3]))) -
cos(theta[6])*(1.0*cos(theta[5])*(cos(theta[4])*(1.0*cos(theta[1])*sin(theta[3]) +
1.0*cos(theta[2])*cos(theta[3])*sin(theta[1])) - 1.0*sin(theta[1])*sin(theta[2])*sin(theta[4])) -
sin(theta[5])*(1.0*cos(theta[1])*cos(theta[3]) - 1.0*cos(theta[2])*sin(theta[1])*sin(theta[3]))),
sin(theta[6])*(1.0*sin(theta[5])*(cos(theta[4])*(1.0*cos(theta[1])*sin(theta[3]) +
1.0*cos(theta[2])*cos(theta[3])*sin(theta[1])) - 1.0*sin(theta[1])*sin(theta[2])*sin(theta[4])) +
cos(theta[5])*(1.0*cos(theta[1])*cos(theta[3]) - 1.0*cos(theta[2])*sin(theta[1])*sin(theta[3]))) -
1.0*cos(theta[6])*(cos(theta[5])*(cos(theta[4])*(1.0*cos(theta[1])*sin(theta[3]) +
1.0*cos(theta[2])*cos(theta[3])*sin(theta[1])) - 1.0*sin(theta[1])*sin(theta[2])*sin(theta[4])) -
sin(theta[5])*(1.0*cos(theta[1])*cos(theta[3]) - 1.0*cos(theta[2])*sin(theta[1])*sin(theta[3])))
,1.0*sin(theta[6])*(sin(theta[5])*(1.0*sin(theta[3])*(1.0*sin(theta[0])*sin(theta[2]) +
1.0*cos(theta[0])*cos(theta[1])*cos(theta[2])) + 1.0*cos(theta[0])*cos(theta[3])*sin(theta[1])) +
cos(theta[5])*(cos(theta[4])*(cos(theta[3])*(1.0*sin(theta[0])*sin(theta[2]) +
1.0*cos(theta[0])*cos(theta[1])*cos(theta[2])) - 1.0*cos(theta[0])*sin(theta[1])*sin(theta[3])) +
sin(theta[4])*(1.0*cos(theta[2])*sin(theta[0]) - 1.0*cos(theta[0])*cos(theta[1])*sin(theta[2])))) -
cos(theta[6])*(cos(theta[5])*(1.0*sin(theta[3])*(1.0*sin(theta[0])*sin(theta[2]) +
1.0*cos(theta[0])*cos(theta[1])*cos(theta[2])) + 1.0*cos(theta[0])*cos(theta[3])*sin(theta[1])) -
1.0*sin(theta[5])*(cos(theta[4])*(cos(theta[3])*(1.0*sin(theta[0])*sin(theta[2]) +
1.0*cos(theta[0])*cos(theta[1])*cos(theta[2])) - 1.0*cos(theta[0])*sin(theta[1])*sin(theta[3])) +
sin(theta[4])*(1.0*cos(theta[2])*sin(theta[0]) - 1.0*cos(theta[0])*cos(theta[1])*sin(theta[2])))), -
cos(theta[6])*(sin(theta[0])*sin(theta[5])*(cos(theta[1])*cos(theta[4])*sin(theta[3]) -
1.0*sin(theta[1])*sin(theta[2])*sin(theta[4]) + cos(theta[2])*cos(theta[3])*cos(theta[4])*sin(theta[1])) +cos(theta[5])*sin(theta[0])*(cos(theta[1])*cos(theta[3]) - 1.0*cos(theta[2])*sin(theta[1])*sin(theta[3]))) -
sin(theta[6])*(cos(theta[5])*sin(theta[0])*(cos(theta[1])*cos(theta[4])*sin(theta[3]) -
1.0*sin(theta[1])*sin(theta[2])*sin(theta[4]) + cos(theta[2])*cos(theta[3])*cos(theta[4])*sin(theta[1])) -
sin(theta[0])*sin(theta[5])*(cos(theta[1])*cos(theta[3]) - 1.0*cos(theta[2])*sin(theta[1])*sin(theta[3]))),
cos(theta[6])*(sin(theta[5])*(sin(theta[4])*(cos(theta[0])*sin(theta[2]) -
1.0*cos(theta[1])*cos(theta[2])*sin(theta[0])) - cos(theta[3])*cos(theta[4])*(cos(theta[0])*cos(theta[2]) +
cos(theta[1])*sin(theta[0])*sin(theta[2]))) + cos(theta[5])*sin(theta[3])*(cos(theta[0])*cos(theta[2]) +
cos(theta[1])*sin(theta[0])*sin(theta[2]))) +
sin(theta[6])*(cos(theta[5])*(sin(theta[4])*(cos(theta[0])*sin(theta[2]) -
1.0*cos(theta[1])*cos(theta[2])*sin(theta[0])) - cos(theta[3])*cos(theta[4])*(cos(theta[0])*cos(theta[2]) +
cos(theta[1])*sin(theta[0])*sin(theta[2]))) - 1.0*sin(theta[3])*sin(theta[5])*(cos(theta[0])*cos(theta[2]) +
cos(theta[1])*sin(theta[0])*sin(theta[2]))),
cos(theta[6])*(cos(theta[5])*(1.0*cos(theta[3])*(1.0*cos(theta[0])*sin(theta[2]) -
1.0*cos(theta[1])*cos(theta[2])*sin(theta[0])) + 1.0*sin(theta[0])*sin(theta[1])*sin(theta[3])) +
1.0*cos(theta[4])*sin(theta[5])*(sin(theta[3])*(1.0*cos(theta[0])*sin(theta[2]) -
1.0*cos(theta[1])*cos(theta[2])*sin(theta[0])) - 1.0*cos(theta[3])*sin(theta[0])*sin(theta[1]))) -
1.0*sin(theta[6])*(sin(theta[5])*(1.0*cos(theta[3])*(1.0*cos(theta[0])*sin(theta[2]) -
1.0*cos(theta[1])*cos(theta[2])*sin(theta[0])) + 1.0*sin(theta[0])*sin(theta[1])*sin(theta[3])) -
cos(theta[4])*cos(theta[5])*(sin(theta[3])*(1.0*cos(theta[0])*sin(theta[2]) -
1.0*cos(theta[1])*cos(theta[2])*sin(theta[0])) - 1.0*cos(theta[3])*sin(theta[0])*sin(theta[1]))),
-sin(theta[5] + theta[6])*(cos(theta[0])*cos(theta[2])*cos(theta[4]) +
cos(theta[1])*cos(theta[4])*sin(theta[0])*sin(theta[2]) -
1.0*cos(theta[0])*cos(theta[3])*sin(theta[2])*sin(theta[4]) -
1.0*sin(theta[0])*sin(theta[1])*sin(theta[3])*sin(theta[4]) +
cos(theta[1])*cos(theta[2])*cos(theta[3])*sin(theta[0])*sin(theta[4])), -
cos(theta[6])*(sin(theta[5])*(1.0*sin(theta[3])*(1.0*cos(theta[0])*sin(theta[2]) -
1.0*cos(theta[1])*cos(theta[2])*sin(theta[0])) - 1.0*cos(theta[3])*sin(theta[0])*sin(theta[1])) +
1.0*cos(theta[5])*(cos(theta[4])*(cos(theta[3])*(1.0*cos(theta[0])*sin(theta[2]) -
1.0*cos(theta[1])*cos(theta[2])*sin(theta[0])) + 1.0*sin(theta[0])*sin(theta[1])*sin(theta[3])) +
sin(theta[4])*(1.0*cos(theta[0])*cos(theta[2]) + 1.0*cos(theta[1])*sin(theta[0])*sin(theta[2])))) -
1.0*sin(theta[6])*(cos(theta[5])*(1.0*sin(theta[3])*(1.0*cos(theta[0])*sin(theta[2]) -
1.0*cos(theta[1])*cos(theta[2])*sin(theta[0])) - 1.0*cos(theta[3])*sin(theta[0])*sin(theta[1])) -
sin(theta[5])*(cos(theta[4])*(cos(theta[3])*(1.0*cos(theta[0])*sin(theta[2]) -
1.0*cos(theta[1])*cos(theta[2])*sin(theta[0])) + 1.0*sin(theta[0])*sin(theta[1])*sin(theta[3])) +
sin(theta[4])*(1.0*cos(theta[0])*cos(theta[2]) + 1.0*cos(theta[1])*sin(theta[0])*sin(theta[2])))), -
sin(theta[6])*(cos(theta[5])*(1.0*sin(theta[3])*(1.0*cos(theta[0])*sin(theta[2]) -
1.0*cos(theta[1])*cos(theta[2])*sin(theta[0])) - 1.0*cos(theta[3])*sin(theta[0])*sin(theta[1])) -
1.0*sin(theta[5])*(cos(theta[4])*(cos(theta[3])*(1.0*cos(theta[0])*sin(theta[2]) -
1.0*cos(theta[1])*cos(theta[2])*sin(theta[0])) + 1.0*sin(theta[0])*sin(theta[1])*sin(theta[3])) +
sin(theta[4])*(1.0*cos(theta[0])*cos(theta[2]) + 1.0*cos(theta[1])*sin(theta[0])*sin(theta[2])))) -
1.0*cos(theta[6])*(sin(theta[5])*(1.0*sin(theta[3])*(1.0*cos(theta[0])*sin(theta[2]) -
1.0*cos(theta[1])*cos(theta[2])*sin(theta[0])) - 1.0*cos(theta[3])*sin(theta[0])*sin(theta[1])) +
cos(theta[5])*(cos(theta[4])*(cos(theta[3])*(1.0*cos(theta[0])*sin(theta[2]) -
1.0*cos(theta[1])*cos(theta[2])*sin(theta[0])) + 1.0*sin(theta[0])*sin(theta[1])*sin(theta[3])) +
sin(theta[4])*(1.0*cos(theta[0])*cos(theta[2]) + 1.0*cos(theta[1])*sin(theta[0])*sin(theta[2]))))
,
1.0*sin(theta[4])*(cos(theta[3])*(1.0*cos(theta[0])*sin(theta[2]) -
1.0*cos(theta[1])*cos(theta[2])*sin(theta[0])) + 1.0*sin(theta[0])*sin(theta[1])*sin(theta[3])) -1.0*cos(theta[4])*(1.0*cos(theta[0])*cos(theta[2]) + 1.0*cos(theta[1])*sin(theta[0])*sin(theta[2])),
- 1.0*sin(theta[4])*(1.0*cos(theta[0])*cos(theta[1])*sin(theta[3]) +
1.0*cos(theta[0])*cos(theta[2])*cos(theta[3])*sin(theta[1])) -
1.0*cos(theta[0])*cos(theta[4])*sin(theta[1])*sin(theta[2]),
        cos(theta[4])*(sin(theta[0])*sin(theta[2]) + cos(theta[0])*cos(theta[1])*cos(theta[2])) +
cos(theta[3])*sin(theta[4])*(cos(theta[2])*sin(theta[0]) - cos(theta[0])*cos(theta[1])*sin(theta[2])),
-1.0*sin(theta[4])*(sin(theta[3])*(1.0*sin(theta[0])*sin(theta[2]) +
1.0*cos(theta[0])*cos(theta[1])*cos(theta[2])) + 1.0*cos(theta[0])*cos(theta[3])*sin(theta[1])),
1.0*cos(theta[4])*(cos(theta[3])*(1.0*sin(theta[0])*sin(theta[2]) +
1.0*cos(theta[0])*cos(theta[1])*cos(theta[2])) - 1.0*cos(theta[0])*sin(theta[1])*sin(theta[3])) +
1.0*sin(theta[4])*(1.0*cos(theta[2])*sin(theta[0]) - 1.0*cos(theta[0])*cos(theta[1])*sin(theta[2])),
0,
0
,
0,
1.0*cos(theta[1])*cos(theta[4])*sin(theta[2]) - 1.0*sin(theta[4])*(1.0*sin(theta[1])*sin(theta[3]) -
1.0*cos(theta[1])*cos(theta[2])*cos(theta[3])),
sin(theta[1])*(cos(theta[2])*cos(theta[4]) - 1.0*cos(theta[3])*sin(theta[2])*sin(theta[4])),
sin(theta[4])*(cos(theta[1])*cos(theta[3]) - 1.0*cos(theta[2])*sin(theta[1])*sin(theta[3])),
1.0*cos(theta[4])*(1.0*cos(theta[1])*sin(theta[3]) + 1.0*cos(theta[2])*cos(theta[3])*sin(theta[1])) -
1.0*sin(theta[1])*sin(theta[2])*sin(theta[4]),
0,
0
,
1.0*cos(theta[4])*(1.0*cos(theta[2])*sin(theta[0]) - 1.0*cos(theta[0])*cos(theta[1])*sin(theta[2])) -
1.0*sin(theta[4])*(cos(theta[3])*(1.0*sin(theta[0])*sin(theta[2]) +
1.0*cos(theta[0])*cos(theta[1])*cos(theta[2])) - 1.0*cos(theta[0])*sin(theta[1])*sin(theta[3])),
1.0*sin(theta[4])*(1.0*cos(theta[1])*sin(theta[0])*sin(theta[3]) +
1.0*cos(theta[2])*cos(theta[3])*sin(theta[0])*sin(theta[1])) +
1.0*cos(theta[4])*sin(theta[0])*sin(theta[1])*sin(theta[2]),
        cos(theta[4])*(cos(theta[0])*sin(theta[2]) - 1.0*cos(theta[1])*cos(theta[2])*sin(theta[0])) +
cos(theta[3])*sin(theta[4])*(cos(theta[0])*cos(theta[2]) + cos(theta[1])*sin(theta[0])*sin(theta[2])),
-1.0*sin(theta[4])*(sin(theta[3])*(1.0*cos(theta[0])*sin(theta[2]) -
1.0*cos(theta[1])*cos(theta[2])*sin(theta[0])) - 1.0*cos(theta[3])*sin(theta[0])*sin(theta[1])),
1.0*cos(theta[4])*(cos(theta[3])*(1.0*cos(theta[0])*sin(theta[2]) -
1.0*cos(theta[1])*cos(theta[2])*sin(theta[0])) + 1.0*sin(theta[0])*sin(theta[1])*sin(theta[3])) +
1.0*sin(theta[4])*(1.0*cos(theta[0])*cos(theta[2]) + 1.0*cos(theta[1])*sin(theta[0])*sin(theta[2])),
0,
0
,
0.316*cos(theta[0]) +
0.384*cos(theta[0])*cos(theta[2]) + 0.088*cos(theta[4])*(cos(theta[3])*(1.0*cos(theta[0])*sin(theta[2]) -
1.0*cos(theta[1])*cos(theta[2])*sin(theta[0])) + 1.0*sin(theta[0])*sin(theta[1])*sin(theta[3])) -
0.0825*cos(theta[1])*sin(theta[0]) - 0.0825*cos(theta[0])*sin(theta[2]) +
0.107*sin(theta[4])*(cos(theta[3])*(1.0*cos(theta[0])*sin(theta[2]) -
1.0*cos(theta[1])*cos(theta[2])*sin(theta[0])) + 1.0*sin(theta[0])*sin(theta[1])*sin(theta[3])) -
0.107*cos(theta[4])*(1.0*cos(theta[0])*cos(theta[2]) + 1.0*cos(theta[1])*sin(theta[0])*sin(theta[2])) +
0.088*sin(theta[4])*(1.0*cos(theta[0])*cos(theta[2]) + 1.0*cos(theta[1])*sin(theta[0])*sin(theta[2])) +0.384*cos(theta[1])*sin(theta[0])*sin(theta[2]) + 0.0825*cos(theta[1])*cos(theta[2])*sin(theta[0]),
-(cos(theta[0])*(165.0*sin(theta[1]) - 165.0*cos(theta[2])*sin(theta[1]) - 768.0*sin(theta[1])*sin(theta[2]) +
214.0*cos(theta[4])*sin(theta[1])*sin(theta[2]) + 214.0*cos(theta[1])*sin(theta[3])*sin(theta[4]) -
176.0*sin(theta[1])*sin(theta[2])*sin(theta[4]) + 176.0*cos(theta[1])*cos(theta[4])*sin(theta[3]) +
176.0*cos(theta[2])*cos(theta[3])*cos(theta[4])*sin(theta[1]) +
214.0*cos(theta[2])*cos(theta[3])*sin(theta[1])*sin(theta[4])))/2000.0,
0.107*cos(theta[4])*(sin(theta[0])*sin(theta[2]) + cos(theta[0])*cos(theta[1])*cos(theta[2])) -
0.384*sin(theta[0])*sin(theta[2]) - 0.0825*cos(theta[2])*sin(theta[0]) -
0.088*sin(theta[4])*(sin(theta[0])*sin(theta[2]) + cos(theta[0])*cos(theta[1])*cos(theta[2])) +
0.088*cos(theta[3])*cos(theta[4])*(cos(theta[2])*sin(theta[0]) - cos(theta[0])*cos(theta[1])*sin(theta[2])) +
0.107*cos(theta[3])*sin(theta[4])*(cos(theta[2])*sin(theta[0]) - cos(theta[0])*cos(theta[1])*sin(theta[2])) -
0.384*cos(theta[0])*cos(theta[1])*cos(theta[2]) + 0.0825*cos(theta[0])*cos(theta[1])*sin(theta[2]),
-0.1385*cos(theta[4] -
0.8825)*(sin(theta[0])*sin(theta[2])*sin(theta[3]) +
cos(theta[0])*cos(theta[3])*sin(theta[1]) + cos(theta[0])*cos(theta[1])*cos(theta[2])*sin(theta[3])),
0.107*cos(theta[4])*(cos(theta[3])*(1.0*sin(theta[0])*sin(theta[2]) +
1.0*cos(theta[0])*cos(theta[1])*cos(theta[2])) - 1.0*cos(theta[0])*sin(theta[1])*sin(theta[3])) -
0.088*sin(theta[4])*(cos(theta[3])*(1.0*sin(theta[0])*sin(theta[2]) +
1.0*cos(theta[0])*cos(theta[1])*cos(theta[2])) - 1.0*cos(theta[0])*sin(theta[1])*sin(theta[3])) +
0.088*cos(theta[4])*(1.0*cos(theta[2])*sin(theta[0]) - 1.0*cos(theta[0])*cos(theta[1])*sin(theta[2])) +
0.107*sin(theta[4])*(1.0*cos(theta[2])*sin(theta[0]) - 1.0*cos(theta[0])*cos(theta[1])*sin(theta[2])),
0,
0
,
0,
0.0825*cos(theta[1]) - 0.0825*cos(theta[1])*cos(theta[2]) - 0.384*cos(theta[1])*sin(theta[2]) -
0.088*cos(theta[4])*(1.0*sin(theta[1])*sin(theta[3]) - 1.0*cos(theta[1])*cos(theta[2])*cos(theta[3])) -
0.107*sin(theta[4])*(1.0*sin(theta[1])*sin(theta[3]) - 1.0*cos(theta[1])*cos(theta[2])*cos(theta[3])) -
0.088*cos(theta[1])*sin(theta[2])*sin(theta[4]) + 0.107*cos(theta[1])*cos(theta[4])*sin(theta[2]),
-0.0005*sin(theta[1])*(768.0*cos(theta[2]) - 165.0*sin(theta[2]) - 214.0*cos(theta[2])*cos(theta[4]) +
176.0*cos(theta[2])*sin(theta[4]) + 214.0*cos(theta[3])*sin(theta[2])*sin(theta[4]) +
176.0*cos(theta[3])*cos(theta[4])*sin(theta[2])),
0.1385*cos(theta[4] -
0.8825)*(cos(theta[1])*cos(theta[3]) -
1.0*cos(theta[2])*sin(theta[1])*sin(theta[3])),
0.107*cos(theta[4])*(1.0*cos(theta[1])*sin(theta[3]) + 1.0*cos(theta[2])*cos(theta[3])*sin(theta[1])) -
0.088*sin(theta[4])*(1.0*cos(theta[1])*sin(theta[3]) + 1.0*cos(theta[2])*cos(theta[3])*sin(theta[1])) -
0.088*cos(theta[4])*sin(theta[1])*sin(theta[2]) - 0.107*sin(theta[1])*sin(theta[2])*sin(theta[4]),
0,
0
,
0.107*cos(theta[4])*(1.0*cos(theta[2])*sin(theta[0]) - 1.0*cos(theta[0])*cos(theta[1])*sin(theta[2])) -
0.088*cos(theta[4])*(cos(theta[3])*(1.0*sin(theta[0])*sin(theta[2]) +
1.0*cos(theta[0])*cos(theta[1])*cos(theta[2])) - 1.0*cos(theta[0])*sin(theta[1])*sin(theta[3])) -
0.0825*cos(theta[0])*cos(theta[1]) - 0.107*sin(theta[4])*(cos(theta[3])*(1.0*sin(theta[0])*sin(theta[2]) +
1.0*cos(theta[0])*cos(theta[1])*cos(theta[2])) - 1.0*cos(theta[0])*sin(theta[1])*sin(theta[3])) -
0.384*cos(theta[2])*sin(theta[0]) - 0.316*sin(theta[0]) + 0.0825*sin(theta[0])*sin(theta[2]) -
0.088*sin(theta[4])*(1.0*cos(theta[2])*sin(theta[0]) - 1.0*cos(theta[0])*cos(theta[1])*sin(theta[2])) +0.0825*cos(theta[0])*cos(theta[1])*cos(theta[2]) + 0.384*cos(theta[0])*cos(theta[1])*sin(theta[2]),
(sin(theta[0])*(165.0*sin(theta[1]) - 165.0*cos(theta[2])*sin(theta[1]) - 768.0*sin(theta[1])*sin(theta[2]) +
214.0*cos(theta[4])*sin(theta[1])*sin(theta[2]) + 214.0*cos(theta[1])*sin(theta[3])*sin(theta[4]) -
176.0*sin(theta[1])*sin(theta[2])*sin(theta[4]) + 176.0*cos(theta[1])*cos(theta[4])*sin(theta[3]) +
176.0*cos(theta[2])*cos(theta[3])*cos(theta[4])*sin(theta[1]) +
214.0*cos(theta[2])*cos(theta[3])*sin(theta[1])*sin(theta[4])))/2000.0,
0.107*cos(theta[4])*(cos(theta[0])*sin(theta[2]) - 1.0*cos(theta[1])*cos(theta[2])*sin(theta[0])) -
0.384*cos(theta[0])*sin(theta[2]) - 0.0825*cos(theta[0])*cos(theta[2]) -
0.088*sin(theta[4])*(cos(theta[0])*sin(theta[2]) - 1.0*cos(theta[1])*cos(theta[2])*sin(theta[0])) -
0.0825*cos(theta[1])*sin(theta[0])*sin(theta[2]) +
0.088*cos(theta[3])*cos(theta[4])*(cos(theta[0])*cos(theta[2]) + cos(theta[1])*sin(theta[0])*sin(theta[2])) +
0.107*cos(theta[3])*sin(theta[4])*(cos(theta[0])*cos(theta[2]) + cos(theta[1])*sin(theta[0])*sin(theta[2])) +
0.384*cos(theta[1])*cos(theta[2])*sin(theta[0]),
0.1385*cos(theta[4] -
0.8825)*(cos(theta[3])*sin(theta[0])*sin(theta[1]) -
cos(theta[0])*sin(theta[2])*sin(theta[3]) + cos(theta[1])*cos(theta[2])*sin(theta[0])*sin(theta[3])),
0.107*cos(theta[4])*(cos(theta[3])*(1.0*cos(theta[0])*sin(theta[2]) -
1.0*cos(theta[1])*cos(theta[2])*sin(theta[0])) + 1.0*sin(theta[0])*sin(theta[1])*sin(theta[3])) -
0.088*sin(theta[4])*(cos(theta[3])*(1.0*cos(theta[0])*sin(theta[2]) -
1.0*cos(theta[1])*cos(theta[2])*sin(theta[0])) + 1.0*sin(theta[0])*sin(theta[1])*sin(theta[3])) +
0.088*cos(theta[4])*(1.0*cos(theta[0])*cos(theta[2]) + 1.0*cos(theta[1])*sin(theta[0])*sin(theta[2])) +
0.107*sin(theta[4])*(1.0*cos(theta[0])*cos(theta[2]) + 1.0*cos(theta[1])*sin(theta[0])*sin(theta[2])),
0,
0;



return jacobian;
}

