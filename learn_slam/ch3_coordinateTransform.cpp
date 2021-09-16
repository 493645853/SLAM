#include<iostream>
#include<vector>
#include<algorithm>
#include<Eigen/Core>
#include<Eigen/Geometry>

int main()
{
    // little carrot 1
    Eigen::Quaterniond q1(0.35,0.2,0.3,0.1);
    q1.normalize();
    Eigen::Vector3d t1(0.3,0.1,0.1);

    // little carrot 2
    Eigen::Quaterniond q2(-0.5,0.4,-0.1,0.2);
    q2.normalize();
    Eigen::Vector3d t2(-0.1,0.5,0.3);

    Eigen::Isometry3d T1(q1), T2(q2);
    T1.pretranslate(t1);
    T2.pretranslate(t2);

    // a point in carrot 1's coordinate
    Eigen::Vector3d p(0.5,0.0,0.2);

    // i.e., from p1 to world coordinate to p2
    std::cout<<"P in 2's coordinate\n"<<(T2*T1.inverse()*p).transpose()<<std::endl;

    return 0;
}