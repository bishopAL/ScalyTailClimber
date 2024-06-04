#ifndef leg_H
#define leg_H

#include <iostream>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <Eigen/Sparse>
#include <Eigen/SVD>
#include <time.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <unistd.h>
#include <map>
#include <string>
#include <string.h>
#include <stdio.h>
#include <fstream>
#include <sstream>
#include <bitset>

using namespace Eigen;
enum enum_LEGNAME{LF,RF,LH,RH};
enum enum_LEGSTATUS{detach=0, swingUp, swingDown, attach, recover, stance};

class CLeg{

private:
    enum_LEGNAME m_sName;
    float m_fL1,m_fL2,m_fL3;   //length of three linkages;
    float m_fTheta1,m_fTheta2,m_fTheta3; //theta of three linkages;theta1--Z,theta2--X,theta3--Y; //represent present
    Matrix<float,3,3> m_mfJacobian;//present jacobian
    enum_LEGSTATUS m_eLegStatus;
    
public:
    CLeg();
    CLeg(enum_LEGNAME name,float L1,float L2,float L3);
    Matrix<float,3,3> GetJacobian();
    void SetJointPos(Matrix<float,3,1> jointPos);
    void UpdateJacobian();
    Matrix<float,3,1> ForwardKinematic();
    Matrix<float,3,1> InverseKinematic(Matrix<float, 1, 3> cmdpos);   // standing state
    void ChangeStatus(enum_LEGSTATUS legStatus);
    enum_LEGSTATUS GetLegStatus();
};

#endif