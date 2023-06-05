#include "ControllerPrototype.hpp"
#include <Eigen/Geometry> 

double upperError = 10e3;
double lowError = 7e3;
double smallAdjustment = 0.1f;
int main(int argc, char *argv[])
{
    int jointEff = 5;
    int jointNo = jointEff + 1;

    // timesteps are equal to joint no
    int timesteps = jointEff;
    Vector3d reconciliationAngles = Vector3d{0,90,180};
    double EMulitplier = 20;
    /* * * * * * * * * * * * * * * * * * * * * * * * *
     * PRECOMPUTATION FOR EACH TIMESTEP BEGINS HERE  *
     *                                               *
     *                                               *
     * * * * * * * * * * * * * * * * * * * * * * * * */
    std::vector<Vector3d> AppliedFields;

        std::vector<double> DesiredAngles(jointNo);
    if (argc == 6)
    {
        DesiredAngles[0] = std::stod(argv[1]);
        DesiredAngles[1] = std::stod(argv[2]);
        DesiredAngles[2] = std::stod(argv[3]);
        DesiredAngles[3] = std::stod(argv[4]);
        DesiredAngles[4] = std::stod(argv[5]);
        DesiredAngles[jointEff] = 0;
    } else {
        DesiredAngles[0] = 45;
        DesiredAngles[1] = 0;
        DesiredAngles[2] = 0;
        DesiredAngles[3] = 0;
        DesiredAngles[4] = 0;
        DesiredAngles[jointEff] = 0;
    }

    if( argc == 2 || argc == 7) {
        EMulitplier = std::stod(argv[argc-1]);
    }

    std::vector<Vector3d> Magnetisations(jointNo);
    Magnetisations[0] = Vector3d(-0.0011, 0, -0.0028);
    Magnetisations[1] = Vector3d(-0.0028, 0, -0.001);
    Magnetisations[2] = Vector3d(0, 0, -0.003);
    Magnetisations[3] = Vector3d(-0.003, 0, 0);
    Magnetisations[4] = Vector3d(0, 0, -0.003);
    Magnetisations[jointEff] = Vector3d(0, 0, 0);

    // Magnetisations[0] = Vector3d(0, 0, -0.003);
    // Magnetisations[1] = Vector3d(0, 0, -0.003);
    // Magnetisations[2] = Vector3d(0, 0, -0.003);
    // Magnetisations[3] = Vector3d(0, 0, -0.003);
    // Magnetisations[4] = Vector3d(0, 0, -0.003);
    // Magnetisations[jointEff] = Vector3d(0, 0, 0);

    std::vector<PosOrientation> iPosVec(jointNo);
    std::vector<Joint> iJoints(jointNo);
    for (int i = 0; i < jointNo; i++)
    {
        iJoints[i].assignPosOri(iPosVec[i]);
    }

    for (int i = 0; i < jointNo; i++)
    {
        iJoints[i].q = Vector3d(0, DesiredAngles[i] * M_PI / 180, 0);
        iJoints[i].LocMag = Magnetisations[i];
    }

    // create vector of links for properties
    std::vector<Link> iLinks(jointEff);
    adjustStiffness(iLinks, EMulitplier);

    Vector3d field = CalculateField(iLinks, iJoints, iPosVec);
    field = RotateField(field, reconciliationAngles);
    field(1) = 0;
    std::cout << "Initial answer:\n" << field << "\n";

return 0;
}