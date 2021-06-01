//
// Created by tolga on 31.05.21.
//

#include "jacobian.h"

MatrixXd calculateJacobian(const sensor_msgs::JointState& msg);{
theta1= msg.position[0];
theta2= msg.position[1];
theta3= msg.position[2];
theta4= msg.position[3];
theta5= msg.position[4];
theta6= msg.position[5];
theta7= msg.position[6];
MatrixXd jacobian(12,7);

jacobian <<
J =
sin(theta7)*(cos(theta6)*(1.0*sin(theta4)*(1.0*cos(theta1)*sin(theta3) -
                                           1.0*cos(theta2)*cos(theta3)*sin(theta1)) - 1.0*cos(theta4)*sin(theta1)*sin(theta2)) -
             1.0*sin(theta6)*(cos(theta5)*(cos(theta4)*(1.0*cos(theta1)*sin(theta3) -
                                                        1.0*cos(theta2)*cos(theta3)*sin(theta1)) + 1.0*sin(theta1)*sin(theta2)*sin(theta4)) +
                              sin(theta5)*(1.0*cos(theta1)*cos(theta3) + 1.0*cos(theta2)*sin(theta1)*sin(theta3)))) +
cos(theta7)*(sin(theta6)*(1.0*sin(theta4)*(1.0*cos(theta1)*sin(theta3) -
                                           1.0*cos(theta2)*cos(theta3)*sin(theta1)) - 1.0*cos(theta4)*sin(theta1)*sin(theta2)) +
             cos(theta6)*(cos(theta5)*(cos(theta4)*(1.0*cos(theta1)*sin(theta3) -
                                                    1.0*cos(theta2)*cos(theta3)*sin(theta1)) + 1.0*sin(theta1)*sin(theta2)*sin(theta4)) +
                          sin(theta5)*(1.0*cos(theta1)*cos(theta3) + 1.0*cos(theta2)*sin(theta1)*sin(theta3)))),
        sin(theta7)*(cos(theta1)*sin(theta6)*(cos(theta2)*cos(theta5)*sin(theta4) -
1.0*sin(theta2)*sin(theta3)*sin(theta5) + cos(theta3)*cos(theta4)*cos(theta5)*sin(theta2)) +
cos(theta1)*cos(theta6)*(cos(theta2)*cos(theta4) - 1.0*cos(theta3)*sin(theta2)*sin(theta4))) -
cos(theta7)*(cos(theta1)*cos(theta6)*(cos(theta2)*cos(theta5)*sin(theta4) -
1.0*sin(theta2)*sin(theta3)*sin(theta5) + cos(theta3)*cos(theta4)*cos(theta5)*sin(theta2)) -
cos(theta1)*sin(theta6)*(cos(theta2)*cos(theta4) - 1.0*cos(theta3)*sin(theta2)*sin(theta4))),
sin(theta7)*(sin(theta6)*(sin(theta5)*(sin(theta1)*sin(theta3) + cos(theta1)*cos(theta2)*cos(theta3))
- cos(theta4)*cos(theta5)*(cos(theta3)*sin(theta1) - cos(theta1)*cos(theta2)*sin(theta3))) +
cos(theta6)*sin(theta4)*(cos(theta3)*sin(theta1) - cos(theta1)*cos(theta2)*sin(theta3))) -
cos(theta7)*(cos(theta6)*(sin(theta5)*(sin(theta1)*sin(theta3) +
cos(theta1)*cos(theta2)*cos(theta3)) - cos(theta4)*cos(theta5)*(cos(theta3)*sin(theta1) -
cos(theta1)*cos(theta2)*sin(theta3))) - 1.0*sin(theta4)*sin(theta6)*(cos(theta3)*sin(theta1) -
cos(theta1)*cos(theta2)*sin(theta3))),
sin(theta7)*(cos(theta6)*(1.0*cos(theta4)*(1.0*sin(theta1)*sin(theta3) +
1.0*cos(theta1)*cos(theta2)*cos(theta3)) - 1.0*cos(theta1)*sin(theta2)*sin(theta4)) +
1.0*cos(theta5)*sin(theta6)*(sin(theta4)*(1.0*sin(theta1)*sin(theta3) +
1.0*cos(theta1)*cos(theta2)*cos(theta3)) + 1.0*cos(theta1)*cos(theta4)*sin(theta2))) +
cos(theta7)*(sin(theta6)*(1.0*cos(theta4)*(1.0*sin(theta1)*sin(theta3) +
1.0*cos(theta1)*cos(theta2)*cos(theta3)) - 1.0*cos(theta1)*sin(theta2)*sin(theta4)) -
cos(theta5)*cos(theta6)*(sin(theta4)*(1.0*sin(theta1)*sin(theta3) +
1.0*cos(theta1)*cos(theta2)*cos(theta3)) + 1.0*cos(theta1)*cos(theta4)*sin(theta2))),
-cos(theta6 + theta7)*(cos(theta1)*cos(theta2)*cos(theta5)*sin(theta3) -
1.0*cos(theta3)*cos(theta5)*sin(theta1) - 1.0*cos(theta1)*sin(theta2)*sin(theta4)*sin(theta5) +
cos(theta4)*sin(theta1)*sin(theta3)*sin(theta5) +
cos(theta1)*cos(theta2)*cos(theta3)*cos(theta4)*sin(theta5)),
cos(theta7)*(cos(theta6)*(1.0*sin(theta4)*(1.0*sin(theta1)*sin(theta3) +
1.0*cos(theta1)*cos(theta2)*cos(theta3)) + 1.0*cos(theta1)*cos(theta4)*sin(theta2)) -
sin(theta6)*(cos(theta5)*(cos(theta4)*(1.0*sin(theta1)*sin(theta3) +
1.0*cos(theta1)*cos(theta2)*cos(theta3)) - 1.0*cos(theta1)*sin(theta2)*sin(theta4)) +
sin(theta5)*(1.0*cos(theta3)*sin(theta1) - 1.0*cos(theta1)*cos(theta2)*sin(theta3)))) -
sin(theta7)*(sin(theta6)*(1.0*sin(theta4)*(1.0*sin(theta1)*sin(theta3) +
1.0*cos(theta1)*cos(theta2)*cos(theta3)) + 1.0*cos(theta1)*cos(theta4)*sin(theta2)) +
1.0*cos(theta6)*(cos(theta5)*(cos(theta4)*(1.0*sin(theta1)*sin(theta3) +
1.0*cos(theta1)*cos(theta2)*cos(theta3)) - 1.0*cos(theta1)*sin(theta2)*sin(theta4)) +
sin(theta5)*(1.0*cos(theta3)*sin(theta1) - 1.0*cos(theta1)*cos(theta2)*sin(theta3)))),
cos(theta7)*(cos(theta6)*(1.0*sin(theta4)*(1.0*sin(theta1)*sin(theta3) +1.0*cos(theta1)*cos(theta2)*cos(theta3)) + 1.0*cos(theta1)*cos(theta4)*sin(theta2)) -
1.0*sin(theta6)*(cos(theta5)*(cos(theta4)*(1.0*sin(theta1)*sin(theta3) +
1.0*cos(theta1)*cos(theta2)*cos(theta3)) - 1.0*cos(theta1)*sin(theta2)*sin(theta4)) +
sin(theta5)*(1.0*cos(theta3)*sin(theta1) - 1.0*cos(theta1)*cos(theta2)*sin(theta3)))) -
sin(theta7)*(sin(theta6)*(1.0*sin(theta4)*(1.0*sin(theta1)*sin(theta3) +
1.0*cos(theta1)*cos(theta2)*cos(theta3)) + 1.0*cos(theta1)*cos(theta4)*sin(theta2)) +
cos(theta6)*(cos(theta5)*(cos(theta4)*(1.0*sin(theta1)*sin(theta3) +
1.0*cos(theta1)*cos(theta2)*cos(theta3)) - 1.0*cos(theta1)*sin(theta2)*sin(theta4)) +
sin(theta5)*(1.0*cos(theta3)*sin(theta1) - 1.0*cos(theta1)*cos(theta2)*sin(theta3))))
,
0,
sin(theta7)*(1.0*sin(theta6)*(cos(theta5)*(1.0*sin(theta2)*sin(theta4) -
1.0*cos(theta2)*cos(theta3)*cos(theta4)) + 1.0*cos(theta2)*sin(theta3)*sin(theta5)) +
cos(theta6)*(1.0*cos(theta4)*sin(theta2) + 1.0*cos(theta2)*cos(theta3)*sin(theta4))) -
cos(theta7)*(cos(theta6)*(cos(theta5)*(1.0*sin(theta2)*sin(theta4) -
1.0*cos(theta2)*cos(theta3)*cos(theta4)) + 1.0*cos(theta2)*sin(theta3)*sin(theta5)) -
sin(theta6)*(1.0*cos(theta4)*sin(theta2) + 1.0*cos(theta2)*cos(theta3)*sin(theta4))),
sin(theta2)*sin(theta7)*(cos(theta3)*sin(theta5)*sin(theta6) -
1.0*cos(theta6)*sin(theta3)*sin(theta4) + cos(theta4)*cos(theta5)*sin(theta3)*sin(theta6)) -
cos(theta7)*sin(theta2)*(sin(theta3)*sin(theta4)*sin(theta6) + cos(theta3)*cos(theta6)*sin(theta5) +
cos(theta4)*cos(theta5)*cos(theta6)*sin(theta3)),
cos(theta7)*(sin(theta6)*(cos(theta2)*sin(theta4) + cos(theta3)*cos(theta4)*sin(theta2)) +
cos(theta5)*cos(theta6)*(cos(theta2)*cos(theta4) - 1.0*cos(theta3)*sin(theta2)*sin(theta4))) +
sin(theta7)*(cos(theta6)*(cos(theta2)*sin(theta4) + cos(theta3)*cos(theta4)*sin(theta2)) -
1.0*cos(theta5)*sin(theta6)*(cos(theta2)*cos(theta4) - 1.0*cos(theta3)*sin(theta2)*sin(theta4))),
-cos(theta6 + theta7)*(cos(theta5)*sin(theta2)*sin(theta3) + cos(theta2)*sin(theta4)*sin(theta5) +
cos(theta3)*cos(theta4)*sin(theta2)*sin(theta5)),
- sin(theta7)*(1.0*cos(theta6)*(cos(theta5)*(1.0*cos(theta2)*sin(theta4) +
1.0*cos(theta3)*cos(theta4)*sin(theta2)) - 1.0*sin(theta2)*sin(theta3)*sin(theta5)) -
sin(theta6)*(1.0*cos(theta2)*cos(theta4) - 1.0*cos(theta3)*sin(theta2)*sin(theta4))) -
cos(theta7)*(sin(theta6)*(cos(theta5)*(1.0*cos(theta2)*sin(theta4) +
1.0*cos(theta3)*cos(theta4)*sin(theta2)) - 1.0*sin(theta2)*sin(theta3)*sin(theta5)) +
cos(theta6)*(1.0*cos(theta2)*cos(theta4) - 1.0*cos(theta3)*sin(theta2)*sin(theta4))),
- cos(theta7)*(1.0*sin(theta6)*(cos(theta5)*(1.0*cos(theta2)*sin(theta4) +
1.0*cos(theta3)*cos(theta4)*sin(theta2)) - 1.0*sin(theta2)*sin(theta3)*sin(theta5)) +
cos(theta6)*(1.0*cos(theta2)*cos(theta4) - 1.0*cos(theta3)*sin(theta2)*sin(theta4))) -
sin(theta7)*(cos(theta6)*(cos(theta5)*(1.0*cos(theta2)*sin(theta4) +
1.0*cos(theta3)*cos(theta4)*sin(theta2)) - 1.0*sin(theta2)*sin(theta3)*sin(theta5)) -
sin(theta6)*(1.0*cos(theta2)*cos(theta4) - 1.0*cos(theta3)*sin(theta2)*sin(theta4)))
, - sin(theta7)*(cos(theta6)*(1.0*sin(theta4)*(1.0*sin(theta1)*sin(theta3) +
1.0*cos(theta1)*cos(theta2)*cos(theta3)) + 1.0*cos(theta1)*cos(theta4)*sin(theta2)) -
1.0*sin(theta6)*(cos(theta5)*(cos(theta4)*(1.0*sin(theta1)*sin(theta3) +
1.0*cos(theta1)*cos(theta2)*cos(theta3)) - 1.0*cos(theta1)*sin(theta2)*sin(theta4)) +
sin(theta5)*(1.0*cos(theta3)*sin(theta1) - 1.0*cos(theta1)*cos(theta2)*sin(theta3)))) -
cos(theta7)*(sin(theta6)*(1.0*sin(theta4)*(1.0*sin(theta1)*sin(theta3) +
1.0*cos(theta1)*cos(theta2)*cos(theta3)) + 1.0*cos(theta1)*cos(theta4)*sin(theta2)) +
cos(theta6)*(cos(theta5)*(cos(theta4)*(1.0*sin(theta1)*sin(theta3) +
1.0*cos(theta1)*cos(theta2)*cos(theta3)) - 1.0*cos(theta1)*sin(theta2)*sin(theta4)) +
sin(theta5)*(1.0*cos(theta3)*sin(theta1) - 1.0*cos(theta1)*cos(theta2)*sin(theta3)))),cos(theta7)*(cos(theta6)*sin(theta1)*(cos(theta2)*cos(theta5)*sin(theta4) -
1.0*sin(theta2)*sin(theta3)*sin(theta5) + cos(theta3)*cos(theta4)*cos(theta5)*sin(theta2)) -
sin(theta1)*sin(theta6)*(cos(theta2)*cos(theta4) - 1.0*cos(theta3)*sin(theta2)*sin(theta4))) -
sin(theta7)*(sin(theta1)*sin(theta6)*(cos(theta2)*cos(theta5)*sin(theta4) -
1.0*sin(theta2)*sin(theta3)*sin(theta5) + cos(theta3)*cos(theta4)*cos(theta5)*sin(theta2)) +
cos(theta6)*sin(theta1)*(cos(theta2)*cos(theta4) - 1.0*cos(theta3)*sin(theta2)*sin(theta4))),
sin(theta7)*(sin(theta6)*(sin(theta5)*(cos(theta1)*sin(theta3) -
1.0*cos(theta2)*cos(theta3)*sin(theta1)) - cos(theta4)*cos(theta5)*(cos(theta1)*cos(theta3) +
cos(theta2)*sin(theta1)*sin(theta3))) + cos(theta6)*sin(theta4)*(cos(theta1)*cos(theta3) +
cos(theta2)*sin(theta1)*sin(theta3))) -
cos(theta7)*(cos(theta6)*(sin(theta5)*(cos(theta1)*sin(theta3) -
1.0*cos(theta2)*cos(theta3)*sin(theta1)) - cos(theta4)*cos(theta5)*(cos(theta1)*cos(theta3) +
cos(theta2)*sin(theta1)*sin(theta3))) - 1.0*sin(theta4)*sin(theta6)*(cos(theta1)*cos(theta3) +
cos(theta2)*sin(theta1)*sin(theta3))),
sin(theta7)*(cos(theta6)*(1.0*cos(theta4)*(1.0*cos(theta1)*sin(theta3) -
1.0*cos(theta2)*cos(theta3)*sin(theta1)) + 1.0*sin(theta1)*sin(theta2)*sin(theta4)) +
1.0*cos(theta5)*sin(theta6)*(sin(theta4)*(1.0*cos(theta1)*sin(theta3) -
1.0*cos(theta2)*cos(theta3)*sin(theta1)) - 1.0*cos(theta4)*sin(theta1)*sin(theta2))) +
cos(theta7)*(sin(theta6)*(1.0*cos(theta4)*(1.0*cos(theta1)*sin(theta3) -
1.0*cos(theta2)*cos(theta3)*sin(theta1)) + 1.0*sin(theta1)*sin(theta2)*sin(theta4)) -
cos(theta5)*cos(theta6)*(sin(theta4)*(1.0*cos(theta1)*sin(theta3) -
1.0*cos(theta2)*cos(theta3)*sin(theta1)) - 1.0*cos(theta4)*sin(theta1)*sin(theta2))),
cos(theta6 + theta7)*(cos(theta1)*cos(theta3)*cos(theta5) +
cos(theta2)*cos(theta5)*sin(theta1)*sin(theta3) -
1.0*cos(theta1)*cos(theta4)*sin(theta3)*sin(theta5) -
1.0*sin(theta1)*sin(theta2)*sin(theta4)*sin(theta5) +
cos(theta2)*cos(theta3)*cos(theta4)*sin(theta1)*sin(theta5)),
cos(theta7)*(cos(theta6)*(1.0*sin(theta4)*(1.0*cos(theta1)*sin(theta3) -
1.0*cos(theta2)*cos(theta3)*sin(theta1)) - 1.0*cos(theta4)*sin(theta1)*sin(theta2)) -
sin(theta6)*(cos(theta5)*(cos(theta4)*(1.0*cos(theta1)*sin(theta3) -
1.0*cos(theta2)*cos(theta3)*sin(theta1)) + 1.0*sin(theta1)*sin(theta2)*sin(theta4)) +
sin(theta5)*(1.0*cos(theta1)*cos(theta3) + 1.0*cos(theta2)*sin(theta1)*sin(theta3)))) -
sin(theta7)*(sin(theta6)*(1.0*sin(theta4)*(1.0*cos(theta1)*sin(theta3) -
1.0*cos(theta2)*cos(theta3)*sin(theta1)) - 1.0*cos(theta4)*sin(theta1)*sin(theta2)) +
1.0*cos(theta6)*(cos(theta5)*(cos(theta4)*(1.0*cos(theta1)*sin(theta3) -
1.0*cos(theta2)*cos(theta3)*sin(theta1)) + 1.0*sin(theta1)*sin(theta2)*sin(theta4)) +
sin(theta5)*(1.0*cos(theta1)*cos(theta3) + 1.0*cos(theta2)*sin(theta1)*sin(theta3)))),
cos(theta7)*(cos(theta6)*(1.0*sin(theta4)*(1.0*cos(theta1)*sin(theta3) -
1.0*cos(theta2)*cos(theta3)*sin(theta1)) - 1.0*cos(theta4)*sin(theta1)*sin(theta2)) -
1.0*sin(theta6)*(cos(theta5)*(cos(theta4)*(1.0*cos(theta1)*sin(theta3) -
1.0*cos(theta2)*cos(theta3)*sin(theta1)) + 1.0*sin(theta1)*sin(theta2)*sin(theta4)) +
sin(theta5)*(1.0*cos(theta1)*cos(theta3) + 1.0*cos(theta2)*sin(theta1)*sin(theta3)))) -
sin(theta7)*(sin(theta6)*(1.0*sin(theta4)*(1.0*cos(theta1)*sin(theta3) -
1.0*cos(theta2)*cos(theta3)*sin(theta1)) - 1.0*cos(theta4)*sin(theta1)*sin(theta2)) +
cos(theta6)*(cos(theta5)*(cos(theta4)*(1.0*cos(theta1)*sin(theta3) -
1.0*cos(theta2)*cos(theta3)*sin(theta1)) + 1.0*sin(theta1)*sin(theta2)*sin(theta4)) +
sin(theta5)*(1.0*cos(theta1)*cos(theta3) + 1.0*cos(theta2)*sin(theta1)*sin(theta3)))),cos(theta7)*(cos(theta6)*(1.0*sin(theta4)*(1.0*cos(theta1)*sin(theta3) -
1.0*cos(theta2)*cos(theta3)*sin(theta1)) - 1.0*cos(theta4)*sin(theta1)*sin(theta2)) -
1.0*sin(theta6)*(cos(theta5)*(cos(theta4)*(1.0*cos(theta1)*sin(theta3) -
1.0*cos(theta2)*cos(theta3)*sin(theta1)) + 1.0*sin(theta1)*sin(theta2)*sin(theta4)) +
sin(theta5)*(1.0*cos(theta1)*cos(theta3) + 1.0*cos(theta2)*sin(theta1)*sin(theta3)))) -
1.0*sin(theta7)*(sin(theta6)*(1.0*sin(theta4)*(1.0*cos(theta1)*sin(theta3) -
1.0*cos(theta2)*cos(theta3)*sin(theta1)) - 1.0*cos(theta4)*sin(theta1)*sin(theta2)) +
cos(theta6)*(cos(theta5)*(cos(theta4)*(1.0*cos(theta1)*sin(theta3) -
1.0*cos(theta2)*cos(theta3)*sin(theta1)) + 1.0*sin(theta1)*sin(theta2)*sin(theta4)) +
sin(theta5)*(1.0*cos(theta1)*cos(theta3) + 1.0*cos(theta2)*sin(theta1)*sin(theta3)))),
sin(theta7)*(cos(theta1)*cos(theta6)*(cos(theta2)*cos(theta5)*sin(theta4) -
1.0*sin(theta2)*sin(theta3)*sin(theta5) + cos(theta3)*cos(theta4)*cos(theta5)*sin(theta2)) -
cos(theta1)*sin(theta6)*(cos(theta2)*cos(theta4) - 1.0*cos(theta3)*sin(theta2)*sin(theta4))) +
cos(theta7)*(cos(theta1)*sin(theta6)*(cos(theta2)*cos(theta5)*sin(theta4) -
1.0*sin(theta2)*sin(theta3)*sin(theta5) + cos(theta3)*cos(theta4)*cos(theta5)*sin(theta2)) +
cos(theta1)*cos(theta6)*(cos(theta2)*cos(theta4) - 1.0*cos(theta3)*sin(theta2)*sin(theta4))),
cos(theta7)*(sin(theta6)*(sin(theta5)*(sin(theta1)*sin(theta3) +
cos(theta1)*cos(theta2)*cos(theta3)) - cos(theta4)*cos(theta5)*(cos(theta3)*sin(theta1) -
cos(theta1)*cos(theta2)*sin(theta3))) + cos(theta6)*sin(theta4)*(cos(theta3)*sin(theta1) -
cos(theta1)*cos(theta2)*sin(theta3))) +
sin(theta7)*(cos(theta6)*(sin(theta5)*(sin(theta1)*sin(theta3) +
cos(theta1)*cos(theta2)*cos(theta3)) - cos(theta4)*cos(theta5)*(cos(theta3)*sin(theta1) -
cos(theta1)*cos(theta2)*sin(theta3))) - 1.0*sin(theta4)*sin(theta6)*(cos(theta3)*sin(theta1) -
cos(theta1)*cos(theta2)*sin(theta3))),
cos(theta7)*(cos(theta6)*(1.0*cos(theta4)*(1.0*sin(theta1)*sin(theta3) +
1.0*cos(theta1)*cos(theta2)*cos(theta3)) - 1.0*cos(theta1)*sin(theta2)*sin(theta4)) +
1.0*cos(theta5)*sin(theta6)*(sin(theta4)*(1.0*sin(theta1)*sin(theta3) +
1.0*cos(theta1)*cos(theta2)*cos(theta3)) + 1.0*cos(theta1)*cos(theta4)*sin(theta2))) -
1.0*sin(theta7)*(sin(theta6)*(1.0*cos(theta4)*(1.0*sin(theta1)*sin(theta3) +
1.0*cos(theta1)*cos(theta2)*cos(theta3)) - 1.0*cos(theta1)*sin(theta2)*sin(theta4)) -
cos(theta5)*cos(theta6)*(sin(theta4)*(1.0*sin(theta1)*sin(theta3) +
1.0*cos(theta1)*cos(theta2)*cos(theta3)) + 1.0*cos(theta1)*cos(theta4)*sin(theta2))),
sin(theta6 + theta7)*(cos(theta1)*cos(theta2)*cos(theta5)*sin(theta3) -
1.0*cos(theta3)*cos(theta5)*sin(theta1) - 1.0*cos(theta1)*sin(theta2)*sin(theta4)*sin(theta5) +
cos(theta4)*sin(theta1)*sin(theta3)*sin(theta5) +
cos(theta1)*cos(theta2)*cos(theta3)*cos(theta4)*sin(theta5)), -
cos(theta7)*(sin(theta6)*(1.0*sin(theta4)*(1.0*sin(theta1)*sin(theta3) +
1.0*cos(theta1)*cos(theta2)*cos(theta3)) + 1.0*cos(theta1)*cos(theta4)*sin(theta2)) +
1.0*cos(theta6)*(cos(theta5)*(cos(theta4)*(1.0*sin(theta1)*sin(theta3) +
1.0*cos(theta1)*cos(theta2)*cos(theta3)) - 1.0*cos(theta1)*sin(theta2)*sin(theta4)) +
sin(theta5)*(1.0*cos(theta3)*sin(theta1) - 1.0*cos(theta1)*cos(theta2)*sin(theta3)))) -
1.0*sin(theta7)*(cos(theta6)*(1.0*sin(theta4)*(1.0*sin(theta1)*sin(theta3) +
1.0*cos(theta1)*cos(theta2)*cos(theta3)) + 1.0*cos(theta1)*cos(theta4)*sin(theta2)) -
sin(theta6)*(cos(theta5)*(cos(theta4)*(1.0*sin(theta1)*sin(theta3) +
1.0*cos(theta1)*cos(theta2)*cos(theta3)) - 1.0*cos(theta1)*sin(theta2)*sin(theta4)) +
sin(theta5)*(1.0*cos(theta3)*sin(theta1) - 1.0*cos(theta1)*cos(theta2)*sin(theta3)))), -
sin(theta7)*(cos(theta6)*(1.0*sin(theta4)*(1.0*sin(theta1)*sin(theta3) +
1.0*cos(theta1)*cos(theta2)*cos(theta3)) + 1.0*cos(theta1)*cos(theta4)*sin(theta2)) -
1.0*sin(theta6)*(cos(theta5)*(cos(theta4)*(1.0*sin(theta1)*sin(theta3) +1.0*cos(theta1)*cos(theta2)*cos(theta3)) - 1.0*cos(theta1)*sin(theta2)*sin(theta4)) +
sin(theta5)*(1.0*cos(theta3)*sin(theta1) - 1.0*cos(theta1)*cos(theta2)*sin(theta3)))) -
1.0*cos(theta7)*(sin(theta6)*(1.0*sin(theta4)*(1.0*sin(theta1)*sin(theta3) +
1.0*cos(theta1)*cos(theta2)*cos(theta3)) + 1.0*cos(theta1)*cos(theta4)*sin(theta2)) +
cos(theta6)*(cos(theta5)*(cos(theta4)*(1.0*sin(theta1)*sin(theta3) +
1.0*cos(theta1)*cos(theta2)*cos(theta3)) - 1.0*cos(theta1)*sin(theta2)*sin(theta4)) +
sin(theta5)*(1.0*cos(theta3)*sin(theta1) - 1.0*cos(theta1)*cos(theta2)*sin(theta3))))
,
0,
cos(theta7)*(1.0*sin(theta6)*(cos(theta5)*(1.0*sin(theta2)*sin(theta4) -
1.0*cos(theta2)*cos(theta3)*cos(theta4)) + 1.0*cos(theta2)*sin(theta3)*sin(theta5)) +
cos(theta6)*(1.0*cos(theta4)*sin(theta2) + 1.0*cos(theta2)*cos(theta3)*sin(theta4))) +
1.0*sin(theta7)*(cos(theta6)*(cos(theta5)*(1.0*sin(theta2)*sin(theta4) -
1.0*cos(theta2)*cos(theta3)*cos(theta4)) + 1.0*cos(theta2)*sin(theta3)*sin(theta5)) -
sin(theta6)*(1.0*cos(theta4)*sin(theta2) + 1.0*cos(theta2)*cos(theta3)*sin(theta4))),
sin(theta2)*sin(theta7)*(sin(theta3)*sin(theta4)*sin(theta6) + cos(theta3)*cos(theta6)*sin(theta5) +
cos(theta4)*cos(theta5)*cos(theta6)*sin(theta3)) +
cos(theta7)*sin(theta2)*(cos(theta3)*sin(theta5)*sin(theta6) -
1.0*cos(theta6)*sin(theta3)*sin(theta4) + cos(theta4)*cos(theta5)*sin(theta3)*sin(theta6)),
cos(theta7)*(cos(theta6)*(cos(theta2)*sin(theta4) + cos(theta3)*cos(theta4)*sin(theta2)) -
1.0*cos(theta5)*sin(theta6)*(cos(theta2)*cos(theta4) - 1.0*cos(theta3)*sin(theta2)*sin(theta4))) -
1.0*sin(theta7)*(sin(theta6)*(cos(theta2)*sin(theta4) + cos(theta3)*cos(theta4)*sin(theta2)) +
cos(theta5)*cos(theta6)*(cos(theta2)*cos(theta4) - 1.0*cos(theta3)*sin(theta2)*sin(theta4))),
sin(theta6 + theta7)*(cos(theta5)*sin(theta2)*sin(theta3) + cos(theta2)*sin(theta4)*sin(theta5) +
cos(theta3)*cos(theta4)*sin(theta2)*sin(theta5)),
1.0*sin(theta7)*(sin(theta6)*(cos(theta5)*(1.0*cos(theta2)*sin(theta4) +
1.0*cos(theta3)*cos(theta4)*sin(theta2)) - 1.0*sin(theta2)*sin(theta3)*sin(theta5)) +
cos(theta6)*(1.0*cos(theta2)*cos(theta4) - 1.0*cos(theta3)*sin(theta2)*sin(theta4))) -
cos(theta7)*(1.0*cos(theta6)*(cos(theta5)*(1.0*cos(theta2)*sin(theta4) +
1.0*cos(theta3)*cos(theta4)*sin(theta2)) - 1.0*sin(theta2)*sin(theta3)*sin(theta5)) -
sin(theta6)*(1.0*cos(theta2)*cos(theta4) - 1.0*cos(theta3)*sin(theta2)*sin(theta4))),
sin(theta7)*(1.0*sin(theta6)*(cos(theta5)*(1.0*cos(theta2)*sin(theta4) +
1.0*cos(theta3)*cos(theta4)*sin(theta2)) - 1.0*sin(theta2)*sin(theta3)*sin(theta5)) +
cos(theta6)*(1.0*cos(theta2)*cos(theta4) - 1.0*cos(theta3)*sin(theta2)*sin(theta4))) -
1.0*cos(theta7)*(cos(theta6)*(cos(theta5)*(1.0*cos(theta2)*sin(theta4) +
1.0*cos(theta3)*cos(theta4)*sin(theta2)) - 1.0*sin(theta2)*sin(theta3)*sin(theta5)) -
sin(theta6)*(1.0*cos(theta2)*cos(theta4) - 1.0*cos(theta3)*sin(theta2)*sin(theta4)))
,1.0*sin(theta7)*(sin(theta6)*(1.0*sin(theta4)*(1.0*sin(theta1)*sin(theta3) +
1.0*cos(theta1)*cos(theta2)*cos(theta3)) + 1.0*cos(theta1)*cos(theta4)*sin(theta2)) +
cos(theta6)*(cos(theta5)*(cos(theta4)*(1.0*sin(theta1)*sin(theta3) +
1.0*cos(theta1)*cos(theta2)*cos(theta3)) - 1.0*cos(theta1)*sin(theta2)*sin(theta4)) +
sin(theta5)*(1.0*cos(theta3)*sin(theta1) - 1.0*cos(theta1)*cos(theta2)*sin(theta3)))) -
cos(theta7)*(cos(theta6)*(1.0*sin(theta4)*(1.0*sin(theta1)*sin(theta3) +
1.0*cos(theta1)*cos(theta2)*cos(theta3)) + 1.0*cos(theta1)*cos(theta4)*sin(theta2)) -
1.0*sin(theta6)*(cos(theta5)*(cos(theta4)*(1.0*sin(theta1)*sin(theta3) +
1.0*cos(theta1)*cos(theta2)*cos(theta3)) - 1.0*cos(theta1)*sin(theta2)*sin(theta4)) +
sin(theta5)*(1.0*cos(theta3)*sin(theta1) - 1.0*cos(theta1)*cos(theta2)*sin(theta3)))), -
cos(theta7)*(sin(theta1)*sin(theta6)*(cos(theta2)*cos(theta5)*sin(theta4) -
1.0*sin(theta2)*sin(theta3)*sin(theta5) + cos(theta3)*cos(theta4)*cos(theta5)*sin(theta2)) +cos(theta6)*sin(theta1)*(cos(theta2)*cos(theta4) - 1.0*cos(theta3)*sin(theta2)*sin(theta4))) -
sin(theta7)*(cos(theta6)*sin(theta1)*(cos(theta2)*cos(theta5)*sin(theta4) -
1.0*sin(theta2)*sin(theta3)*sin(theta5) + cos(theta3)*cos(theta4)*cos(theta5)*sin(theta2)) -
sin(theta1)*sin(theta6)*(cos(theta2)*cos(theta4) - 1.0*cos(theta3)*sin(theta2)*sin(theta4))),
cos(theta7)*(sin(theta6)*(sin(theta5)*(cos(theta1)*sin(theta3) -
1.0*cos(theta2)*cos(theta3)*sin(theta1)) - cos(theta4)*cos(theta5)*(cos(theta1)*cos(theta3) +
cos(theta2)*sin(theta1)*sin(theta3))) + cos(theta6)*sin(theta4)*(cos(theta1)*cos(theta3) +
cos(theta2)*sin(theta1)*sin(theta3))) +
sin(theta7)*(cos(theta6)*(sin(theta5)*(cos(theta1)*sin(theta3) -
1.0*cos(theta2)*cos(theta3)*sin(theta1)) - cos(theta4)*cos(theta5)*(cos(theta1)*cos(theta3) +
cos(theta2)*sin(theta1)*sin(theta3))) - 1.0*sin(theta4)*sin(theta6)*(cos(theta1)*cos(theta3) +
cos(theta2)*sin(theta1)*sin(theta3))),
cos(theta7)*(cos(theta6)*(1.0*cos(theta4)*(1.0*cos(theta1)*sin(theta3) -
1.0*cos(theta2)*cos(theta3)*sin(theta1)) + 1.0*sin(theta1)*sin(theta2)*sin(theta4)) +
1.0*cos(theta5)*sin(theta6)*(sin(theta4)*(1.0*cos(theta1)*sin(theta3) -
1.0*cos(theta2)*cos(theta3)*sin(theta1)) - 1.0*cos(theta4)*sin(theta1)*sin(theta2))) -
1.0*sin(theta7)*(sin(theta6)*(1.0*cos(theta4)*(1.0*cos(theta1)*sin(theta3) -
1.0*cos(theta2)*cos(theta3)*sin(theta1)) + 1.0*sin(theta1)*sin(theta2)*sin(theta4)) -
cos(theta5)*cos(theta6)*(sin(theta4)*(1.0*cos(theta1)*sin(theta3) -
1.0*cos(theta2)*cos(theta3)*sin(theta1)) - 1.0*cos(theta4)*sin(theta1)*sin(theta2))),
-sin(theta6 + theta7)*(cos(theta1)*cos(theta3)*cos(theta5) +
cos(theta2)*cos(theta5)*sin(theta1)*sin(theta3) -
1.0*cos(theta1)*cos(theta4)*sin(theta3)*sin(theta5) -
1.0*sin(theta1)*sin(theta2)*sin(theta4)*sin(theta5) +
cos(theta2)*cos(theta3)*cos(theta4)*sin(theta1)*sin(theta5)), -
cos(theta7)*(sin(theta6)*(1.0*sin(theta4)*(1.0*cos(theta1)*sin(theta3) -
1.0*cos(theta2)*cos(theta3)*sin(theta1)) - 1.0*cos(theta4)*sin(theta1)*sin(theta2)) +
1.0*cos(theta6)*(cos(theta5)*(cos(theta4)*(1.0*cos(theta1)*sin(theta3) -
1.0*cos(theta2)*cos(theta3)*sin(theta1)) + 1.0*sin(theta1)*sin(theta2)*sin(theta4)) +
sin(theta5)*(1.0*cos(theta1)*cos(theta3) + 1.0*cos(theta2)*sin(theta1)*sin(theta3)))) -
1.0*sin(theta7)*(cos(theta6)*(1.0*sin(theta4)*(1.0*cos(theta1)*sin(theta3) -
1.0*cos(theta2)*cos(theta3)*sin(theta1)) - 1.0*cos(theta4)*sin(theta1)*sin(theta2)) -
sin(theta6)*(cos(theta5)*(cos(theta4)*(1.0*cos(theta1)*sin(theta3) -
1.0*cos(theta2)*cos(theta3)*sin(theta1)) + 1.0*sin(theta1)*sin(theta2)*sin(theta4)) +
sin(theta5)*(1.0*cos(theta1)*cos(theta3) + 1.0*cos(theta2)*sin(theta1)*sin(theta3)))), -
sin(theta7)*(cos(theta6)*(1.0*sin(theta4)*(1.0*cos(theta1)*sin(theta3) -
1.0*cos(theta2)*cos(theta3)*sin(theta1)) - 1.0*cos(theta4)*sin(theta1)*sin(theta2)) -
1.0*sin(theta6)*(cos(theta5)*(cos(theta4)*(1.0*cos(theta1)*sin(theta3) -
1.0*cos(theta2)*cos(theta3)*sin(theta1)) + 1.0*sin(theta1)*sin(theta2)*sin(theta4)) +
sin(theta5)*(1.0*cos(theta1)*cos(theta3) + 1.0*cos(theta2)*sin(theta1)*sin(theta3)))) -
1.0*cos(theta7)*(sin(theta6)*(1.0*sin(theta4)*(1.0*cos(theta1)*sin(theta3) -
1.0*cos(theta2)*cos(theta3)*sin(theta1)) - 1.0*cos(theta4)*sin(theta1)*sin(theta2)) +
cos(theta6)*(cos(theta5)*(cos(theta4)*(1.0*cos(theta1)*sin(theta3) -
1.0*cos(theta2)*cos(theta3)*sin(theta1)) + 1.0*sin(theta1)*sin(theta2)*sin(theta4)) +
sin(theta5)*(1.0*cos(theta1)*cos(theta3) + 1.0*cos(theta2)*sin(theta1)*sin(theta3))))
,
1.0*sin(theta5)*(cos(theta4)*(1.0*cos(theta1)*sin(theta3) -
1.0*cos(theta2)*cos(theta3)*sin(theta1)) + 1.0*sin(theta1)*sin(theta2)*sin(theta4)) -1.0*cos(theta5)*(1.0*cos(theta1)*cos(theta3) + 1.0*cos(theta2)*sin(theta1)*sin(theta3)),
- 1.0*sin(theta5)*(1.0*cos(theta1)*cos(theta2)*sin(theta4) +
1.0*cos(theta1)*cos(theta3)*cos(theta4)*sin(theta2)) -
1.0*cos(theta1)*cos(theta5)*sin(theta2)*sin(theta3),
        cos(theta5)*(sin(theta1)*sin(theta3) + cos(theta1)*cos(theta2)*cos(theta3)) +
cos(theta4)*sin(theta5)*(cos(theta3)*sin(theta1) - cos(theta1)*cos(theta2)*sin(theta3)),
-1.0*sin(theta5)*(sin(theta4)*(1.0*sin(theta1)*sin(theta3) +
1.0*cos(theta1)*cos(theta2)*cos(theta3)) + 1.0*cos(theta1)*cos(theta4)*sin(theta2)),
1.0*cos(theta5)*(cos(theta4)*(1.0*sin(theta1)*sin(theta3) +
1.0*cos(theta1)*cos(theta2)*cos(theta3)) - 1.0*cos(theta1)*sin(theta2)*sin(theta4)) +
1.0*sin(theta5)*(1.0*cos(theta3)*sin(theta1) - 1.0*cos(theta1)*cos(theta2)*sin(theta3)),
0,
0
,
0,
1.0*cos(theta2)*cos(theta5)*sin(theta3) - 1.0*sin(theta5)*(1.0*sin(theta2)*sin(theta4) -
1.0*cos(theta2)*cos(theta3)*cos(theta4)),
sin(theta2)*(cos(theta3)*cos(theta5) - 1.0*cos(theta4)*sin(theta3)*sin(theta5)),
sin(theta5)*(cos(theta2)*cos(theta4) - 1.0*cos(theta3)*sin(theta2)*sin(theta4)),
1.0*cos(theta5)*(1.0*cos(theta2)*sin(theta4) + 1.0*cos(theta3)*cos(theta4)*sin(theta2)) -
1.0*sin(theta2)*sin(theta3)*sin(theta5),
0,
0
,
1.0*cos(theta5)*(1.0*cos(theta3)*sin(theta1) - 1.0*cos(theta1)*cos(theta2)*sin(theta3)) -
1.0*sin(theta5)*(cos(theta4)*(1.0*sin(theta1)*sin(theta3) +
1.0*cos(theta1)*cos(theta2)*cos(theta3)) - 1.0*cos(theta1)*sin(theta2)*sin(theta4)),
1.0*sin(theta5)*(1.0*cos(theta2)*sin(theta1)*sin(theta4) +
1.0*cos(theta3)*cos(theta4)*sin(theta1)*sin(theta2)) +
1.0*cos(theta5)*sin(theta1)*sin(theta2)*sin(theta3),
        cos(theta5)*(cos(theta1)*sin(theta3) - 1.0*cos(theta2)*cos(theta3)*sin(theta1)) +
cos(theta4)*sin(theta5)*(cos(theta1)*cos(theta3) + cos(theta2)*sin(theta1)*sin(theta3)),
-1.0*sin(theta5)*(sin(theta4)*(1.0*cos(theta1)*sin(theta3) -
1.0*cos(theta2)*cos(theta3)*sin(theta1)) - 1.0*cos(theta4)*sin(theta1)*sin(theta2)),
1.0*cos(theta5)*(cos(theta4)*(1.0*cos(theta1)*sin(theta3) -
1.0*cos(theta2)*cos(theta3)*sin(theta1)) + 1.0*sin(theta1)*sin(theta2)*sin(theta4)) +
1.0*sin(theta5)*(1.0*cos(theta1)*cos(theta3) + 1.0*cos(theta2)*sin(theta1)*sin(theta3)),
0,
0
,
0.316*cos(theta1) +
0.384*cos(theta1)*cos(theta3) + 0.088*cos(theta5)*(cos(theta4)*(1.0*cos(theta1)*sin(theta3) -
1.0*cos(theta2)*cos(theta3)*sin(theta1)) + 1.0*sin(theta1)*sin(theta2)*sin(theta4)) -
0.0825*cos(theta2)*sin(theta1) - 0.0825*cos(theta1)*sin(theta3) +
0.107*sin(theta5)*(cos(theta4)*(1.0*cos(theta1)*sin(theta3) -
1.0*cos(theta2)*cos(theta3)*sin(theta1)) + 1.0*sin(theta1)*sin(theta2)*sin(theta4)) -
0.107*cos(theta5)*(1.0*cos(theta1)*cos(theta3) + 1.0*cos(theta2)*sin(theta1)*sin(theta3)) +
0.088*sin(theta5)*(1.0*cos(theta1)*cos(theta3) + 1.0*cos(theta2)*sin(theta1)*sin(theta3)) +0.384*cos(theta2)*sin(theta1)*sin(theta3) + 0.0825*cos(theta2)*cos(theta3)*sin(theta1),
-(cos(theta1)*(165.0*sin(theta2) - 165.0*cos(theta3)*sin(theta2) - 768.0*sin(theta2)*sin(theta3) +
214.0*cos(theta5)*sin(theta2)*sin(theta3) + 214.0*cos(theta2)*sin(theta4)*sin(theta5) -
176.0*sin(theta2)*sin(theta3)*sin(theta5) + 176.0*cos(theta2)*cos(theta5)*sin(theta4) +
176.0*cos(theta3)*cos(theta4)*cos(theta5)*sin(theta2) +
214.0*cos(theta3)*cos(theta4)*sin(theta2)*sin(theta5)))/2000,
0.107*cos(theta5)*(sin(theta1)*sin(theta3) + cos(theta1)*cos(theta2)*cos(theta3)) -
0.384*sin(theta1)*sin(theta3) - 0.0825*cos(theta3)*sin(theta1) -
0.088*sin(theta5)*(sin(theta1)*sin(theta3) + cos(theta1)*cos(theta2)*cos(theta3)) +
0.088*cos(theta4)*cos(theta5)*(cos(theta3)*sin(theta1) - cos(theta1)*cos(theta2)*sin(theta3)) +
0.107*cos(theta4)*sin(theta5)*(cos(theta3)*sin(theta1) - cos(theta1)*cos(theta2)*sin(theta3)) -
0.384*cos(theta1)*cos(theta2)*cos(theta3) + 0.0825*cos(theta1)*cos(theta2)*sin(theta3),
-0.13853880322855398479140561087412*cos(theta5 -
0.88252746126555690350974162213837)*(sin(theta1)*sin(theta3)*sin(theta4) +
cos(theta1)*cos(theta4)*sin(theta2) + cos(theta1)*cos(theta2)*cos(theta3)*sin(theta4)),
0.107*cos(theta5)*(cos(theta4)*(1.0*sin(theta1)*sin(theta3) +
1.0*cos(theta1)*cos(theta2)*cos(theta3)) - 1.0*cos(theta1)*sin(theta2)*sin(theta4)) -
0.088*sin(theta5)*(cos(theta4)*(1.0*sin(theta1)*sin(theta3) +
1.0*cos(theta1)*cos(theta2)*cos(theta3)) - 1.0*cos(theta1)*sin(theta2)*sin(theta4)) +
0.088*cos(theta5)*(1.0*cos(theta3)*sin(theta1) - 1.0*cos(theta1)*cos(theta2)*sin(theta3)) +
0.107*sin(theta5)*(1.0*cos(theta3)*sin(theta1) - 1.0*cos(theta1)*cos(theta2)*sin(theta3)),
0,
0
,
0,
0.0825*cos(theta2) - 0.0825*cos(theta2)*cos(theta3) - 0.384*cos(theta2)*sin(theta3) -
0.088*cos(theta5)*(1.0*sin(theta2)*sin(theta4) - 1.0*cos(theta2)*cos(theta3)*cos(theta4)) -
0.107*sin(theta5)*(1.0*sin(theta2)*sin(theta4) - 1.0*cos(theta2)*cos(theta3)*cos(theta4)) -
0.088*cos(theta2)*sin(theta3)*sin(theta5) + 0.107*cos(theta2)*cos(theta5)*sin(theta3),
-0.0005*sin(theta2)*(768.0*cos(theta3) - 165.0*sin(theta3) - 214.0*cos(theta3)*cos(theta5) +
176.0*cos(theta3)*sin(theta5) + 214.0*cos(theta4)*sin(theta3)*sin(theta5) +
176.0*cos(theta4)*cos(theta5)*sin(theta3)),
0.13853880322855398479140561087412*cos(theta5 -
0.88252746126555690350974162213837)*(cos(theta2)*cos(theta4) -
1.0*cos(theta3)*sin(theta2)*sin(theta4)),
0.107*cos(theta5)*(1.0*cos(theta2)*sin(theta4) + 1.0*cos(theta3)*cos(theta4)*sin(theta2)) -
0.088*sin(theta5)*(1.0*cos(theta2)*sin(theta4) + 1.0*cos(theta3)*cos(theta4)*sin(theta2)) -
0.088*cos(theta5)*sin(theta2)*sin(theta3) - 0.107*sin(theta2)*sin(theta3)*sin(theta5),
0,
0
,
0.107*cos(theta5)*(1.0*cos(theta3)*sin(theta1) - 1.0*cos(theta1)*cos(theta2)*sin(theta3)) -
0.088*cos(theta5)*(cos(theta4)*(1.0*sin(theta1)*sin(theta3) +
1.0*cos(theta1)*cos(theta2)*cos(theta3)) - 1.0*cos(theta1)*sin(theta2)*sin(theta4)) -
0.0825*cos(theta1)*cos(theta2) - 0.107*sin(theta5)*(cos(theta4)*(1.0*sin(theta1)*sin(theta3) +
1.0*cos(theta1)*cos(theta2)*cos(theta3)) - 1.0*cos(theta1)*sin(theta2)*sin(theta4)) -
0.384*cos(theta3)*sin(theta1) - 0.316*sin(theta1) + 0.0825*sin(theta1)*sin(theta3) -
0.088*sin(theta5)*(1.0*cos(theta3)*sin(theta1) - 1.0*cos(theta1)*cos(theta2)*sin(theta3)) +0.0825*cos(theta1)*cos(theta2)*cos(theta3) + 0.384*cos(theta1)*cos(theta2)*sin(theta3),
(sin(theta1)*(165.0*sin(theta2) - 165.0*cos(theta3)*sin(theta2) - 768.0*sin(theta2)*sin(theta3) +
214.0*cos(theta5)*sin(theta2)*sin(theta3) + 214.0*cos(theta2)*sin(theta4)*sin(theta5) -
176.0*sin(theta2)*sin(theta3)*sin(theta5) + 176.0*cos(theta2)*cos(theta5)*sin(theta4) +
176.0*cos(theta3)*cos(theta4)*cos(theta5)*sin(theta2) +
214.0*cos(theta3)*cos(theta4)*sin(theta2)*sin(theta5)))/2000,
0.107*cos(theta5)*(cos(theta1)*sin(theta3) - 1.0*cos(theta2)*cos(theta3)*sin(theta1)) -
0.384*cos(theta1)*sin(theta3) - 0.0825*cos(theta1)*cos(theta3) -
0.088*sin(theta5)*(cos(theta1)*sin(theta3) - 1.0*cos(theta2)*cos(theta3)*sin(theta1)) -
0.0825*cos(theta2)*sin(theta1)*sin(theta3) +
0.088*cos(theta4)*cos(theta5)*(cos(theta1)*cos(theta3) + cos(theta2)*sin(theta1)*sin(theta3)) +
0.107*cos(theta4)*sin(theta5)*(cos(theta1)*cos(theta3) + cos(theta2)*sin(theta1)*sin(theta3)) +
0.384*cos(theta2)*cos(theta3)*sin(theta1),
0.13853880322855398479140561087412*cos(theta5 -
0.88252746126555690350974162213837)*(cos(theta4)*sin(theta1)*sin(theta2) -
cos(theta1)*sin(theta3)*sin(theta4) + cos(theta2)*cos(theta3)*sin(theta1)*sin(theta4)),
0.107*cos(theta5)*(cos(theta4)*(1.0*cos(theta1)*sin(theta3) -
1.0*cos(theta2)*cos(theta3)*sin(theta1)) + 1.0*sin(theta1)*sin(theta2)*sin(theta4)) -
0.088*sin(theta5)*(cos(theta4)*(1.0*cos(theta1)*sin(theta3) -
1.0*cos(theta2)*cos(theta3)*sin(theta1)) + 1.0*sin(theta1)*sin(theta2)*sin(theta4)) +
0.088*cos(theta5)*(1.0*cos(theta1)*cos(theta3) + 1.0*cos(theta2)*sin(theta1)*sin(theta3)) +
0.107*sin(theta5)*(1.0*cos(theta1)*cos(theta3) + 1.0*cos(theta2)*sin(theta1)*sin(theta3)),
0,
0;



return jacobian;
}

