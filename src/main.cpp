#include "leg.h"

using namespace std;
using namespace Eigen;

int main()
{
    CLeg* m_glLeg[4];
    m_glLeg[0] = new CLeg(LF,60.0,60.0,30.0);  // mm
    m_glLeg[1] = new CLeg(RF,60.0,60.0,30.0);
    m_glLeg[2] = new CLeg(LH,60.0,60.0,30.0);
    m_glLeg[3] = new CLeg(RH,60.0,60.0,30.0);
    Matrix<float, 1, 3> cmdpos;
    cmdpos<<0.060, 0.06, -0.03;
    cout<<"LF ik: "<<m_glLeg[0]->InverseKinematic(cmdpos)<<endl;
}