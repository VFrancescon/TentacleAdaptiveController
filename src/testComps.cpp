#include "ControllerPrototype.hpp"
#include <Eigen/Geometry> 

double upperError = 10e3;
double lowError = 7e3;
double smallAdjustment = 0.1f;
int main(int argc, char *argv[])
{
    int jointMultiplier = 1;
    int jointEff = 5;
    int jointNo = jointEff + 1;

    // timesteps are equal to joint no
    int timesteps = jointEff;
    Vector3d reconciliationAngles = Vector3d{0,0,0};
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
        DesiredAngles[0] = 10;
        DesiredAngles[1] = 15;
        DesiredAngles[2] = 15;
        DesiredAngles[3] = 10;
        DesiredAngles[4] = 20;
        DesiredAngles[jointEff] = 0;
    }

    if( argc == 2 || argc == 7) {
        jointMultiplier = std::stoi(argv[argc-1]);
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
    std::vector<double> DesiredAnglesSPLIT(jointEff*jointMultiplier);
    std::vector<Vector3d> MagnetisationsSPLIT(jointEff*jointMultiplier);
    if(jointMultiplier > 1){
    //convert sub5 joint numbers
        for(int i = 0; i < jointEff * jointMultiplier; i++) {
            DesiredAnglesSPLIT[i] = DesiredAngles[i / jointMultiplier] / jointMultiplier;
            MagnetisationsSPLIT[i] = Magnetisations[i / jointMultiplier];
        }
        DesiredAnglesSPLIT.push_back(0);
        MagnetisationsSPLIT.push_back(Vector3d(0, 0, 0));
    } else {
        std::cout << "Using defaults\n";
        DesiredAnglesSPLIT = DesiredAngles;
        MagnetisationsSPLIT = Magnetisations;
    }

    std::vector<PosOrientation> iPosVec(jointNo);
    std::vector<Joint> iJoints(jointNo);
    for (int i = 0; i < jointNo; i++)
    {
        iJoints[i].assignPosOri(iPosVec[i]);
    }

    for (int i = 0; i < jointNo; i++)
    {
        iJoints[i].q = Vector3d(0, DesiredAnglesSPLIT[i] * M_PI / 180, 0);
        iJoints[i].LocMag = MagnetisationsSPLIT[i];
    }

    // create vector of links for properties
    std::vector<Link> iLinks(jointEff);
    adjustStiffness(iLinks, EMulitplier, jointMultiplier);

    Vector3d field = CalculateField(iLinks, iJoints, iPosVec);
    field = RotateField(field, reconciliationAngles);
    field(1) = 0;
    std::cout << "Initial answer:\n" << field << "\n";

return 0;
}