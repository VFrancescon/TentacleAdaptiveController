#include "ControllerPrototype.hpp"

std::vector<Point2d> CalcPoints(MatrixXd angles)
{
    double linkL = 10e-3;

    std::vector<double> anglesVect;
    std::vector<Point2d> pointsVect;
    pointsVect.push_back(Point(0, 0));
    for (int i = 0; i < angles.size(); i++)
    {
        if (i % 2 == 1)
            anglesVect.push_back(angles(i));
    }
    for (int i = 1; i < anglesVect.size(); i++)
    {
        double angle = 0;
        for (int k = 0; k < i; k++)
            angle += anglesVect[k];
        double xdiff = (linkL)*sin(angle);
        double ydiff = (linkL)*cos(angle);
        Point2d pn = Point2d{(pointsVect[i - 1].x + xdiff), (pointsVect[i - 1].y + ydiff)};
        pointsVect.push_back(pn);
    }
    return pointsVect;
}

std::vector<Point2d> CalcPoints(std::vector<double> angles)
{
    double linkL = 10e-3;

    std::vector<Point2d> pointsVect;
    pointsVect.push_back(Point(0, 0));
    for (int i = 1; i < angles.size(); i++)
    {
        double angle = 0;
        for (int k = 0; k < i; k++)
            angle += angles[k];
        angle = angle * M_PI / 180;
        double xdiff = (linkL)*sin(angle);
        double ydiff = (linkL)*cos(angle);
        Point2d pn = Point2d{(pointsVect[i - 1].x + xdiff), (pointsVect[i - 1].y + ydiff)};
        pointsVect.push_back(pn);
    }
    return pointsVect;
}

int main(int argc, char *argv[])
{
    /**
     * Get today's date
     */
    time_t curr_time;
    tm *curr_tm;
    time(&curr_time);
    curr_tm = localtime(&curr_time);
    char date_string[100];
    strftime(date_string, 50, "%d_%m_%y_%H%M%S", curr_tm);
    std::string date(date_string);
    std::ofstream recordPerformance;
    recordPerformance.open("../Validation.csv", std::ios_base::app);
    recordPerformance << date << "\n";
    // recordPerformance << "Step, Error(t), Error(t-1), E_Multiplier, Bx, By, Bz\n";
    recordPerformance << "Joint, PxT, PyT, PzT, PxP, PyP, PzP, Mx, My, Mz, Bx, By, Bz\n";

    int jointEff = 5;
    int jointNo = jointEff + 1;

    // timesteps are equal to joint no
    int timesteps = jointEff;
    Vector3d reconciliationAngles = Vector3d{0, 0, 180};
    double EMulitplier = 30;
    
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
        DesiredAngles[4] = 10;
        DesiredAngles[jointEff] = 0;
    }
    /* * * * * * * * * * * * * * * * * * * * * * * * *
     * PRECOMPUTATION FOR EACH TIMESTEP BEGINS HERE  *
     *                                               *
     *                                               *
     * * * * * * * * * * * * * * * * * * * * * * * * */
    std::vector<Vector3d> AppliedFields;

    // std::vector<double> DesiredAngles(jointNo);
    // DesiredAngles[0] = 45;
    // DesiredAngles[1] = 0;
    // DesiredAngles[2] = 0;
    // DesiredAngles[3] = 0;
    // DesiredAngles[4] = 10;
    // DesiredAngles[jointEff] = 0;

    std::vector<Vector3d> Magnetisations(jointNo);
    Magnetisations[0] = Vector3d(-0.0011, 0, -0.0028);
    Magnetisations[1] = Vector3d(-0.0028, 0, 0.001);
    Magnetisations[2] = Vector3d(0, 0, -0.003);
    Magnetisations[3] = Vector3d(-0.003, 0, 0);
    Magnetisations[4] = Vector3d(0, 0, -0.003);
    Magnetisations[jointEff] = Vector3d(0, 0, 0);

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
    MatrixXd possibleAngles = backwardsQ(iLinks, iJoints, iPosVec);
    // field = RotateField(field, reconciliationAngles);
    // field(1) = 0;
    std::cout << "Initial answer:\n"
              << field << "\n";

    std::cout << "backwards Angles. Size: " << possibleAngles.size() << ":\n";
    for(int i = 0; i < possibleAngles.size(); i++) std::cout << possibleAngles(i) << "\n";

    std::vector<double> CalculatedAngles;
    for(int i = 0; i < jointNo - 1; i++){
        CalculatedAngles.push_back(possibleAngles(1+3*i));
    }
    CalculatedAngles.push_back(0);
    std::cout << "calced angles:\n";
    for(auto i: CalculatedAngles) std::cout << i << " ";
    std::cout << "\n";
    std::vector<Point2d> theoreticalPoints = CalcPoints(DesiredAngles);
    std::vector<Point2d> calculatedPoints = CalcPoints(possibleAngles);
    // std::vector<Point2d> calculatedPoints = {Point2d(0,0), Point2d(0,0), Point2d(0,0), Point2d(0,0), Point2d(0,0), Point2d(0,0)};
    std::vector<Vector3d> MagnetisationsOutput = Magnetisations;
    MagnetisationsOutput.insert(MagnetisationsOutput.begin(), Vector3d(0,0,0));
    for (int i = 0; i < theoreticalPoints.size(); i++)
    {
        std::cout << "PxT, PyT " << theoreticalPoints[i].x << "," << theoreticalPoints[i].y << " vs ";
        std::cout << "PxP, PyP " << calculatedPoints[i].x << "," << calculatedPoints[i].y << "\n";
        // recordPerformance << "Joint, PxT, PyT, PzT, PxP, PyP, PzP, Mx, My, Mz, Bx, By, Bz\n";
        recordPerformance << i << "," << 
            theoreticalPoints[i].x << "," << 0 << "," << theoreticalPoints[i].y << "," <<
            calculatedPoints[i].x << "," << 0 << "," << calculatedPoints[i].y << "," <<
            MagnetisationsOutput[i](0) << "," << MagnetisationsOutput[i](1) << "," << MagnetisationsOutput[i](2) << "," <<
            field(0) << "," << field(1) << "," << field(2) << "\n";
    }
    for (int i = 0; i < possibleAngles.size(); i++)
    {
        if (possibleAngles(i) < 1e-6)
            possibleAngles(i) = 0;
        else
        {
            possibleAngles(i) = possibleAngles(i) * 180 / M_PI;
        }
    }
    std::cout << "Backwards angles:\n"
              << possibleAngles << "\n";

    return 0;
}
