#include "ControllerPrototype.hpp"
#include <matplotlibcpp.h>

namespace plt = matplotlibcpp;

std::vector<Vector3d> CalcPoints(MatrixXd angles)
{
    double linkL = 10;

    std::vector<double> anglesVect;
    std::vector<Vector3d> pointsVect;
    pointsVect.push_back(Vector3d(0, 0, 0));
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
        Vector3d pn = Vector3d{(pointsVect[i - 1](0) + xdiff), 0, (pointsVect[i - 1](2) + ydiff)};
        
        pointsVect.push_back(pn);
    }

    for(int i = 0; i < pointsVect.size(); i++){
        pointsVect[i] = AngleAxisd(M_PI, Vector3d::UnitX()) *  pointsVect[i];
    }
    
    return pointsVect;
}

std::vector<Vector3d> CalcPoints(std::vector<double> angles)
{
    double linkL = 10;

    
    std::vector<Vector3d> pointsVect;    
    pointsVect.push_back(Vector3d(0, 0, 0));

    for (int i = 1; i < angles.size(); i++)
    {
        double angle = 0;
        for (int k = 0; k < i; k++)
            angle += angles[k];
        angle = angle * M_PI / 180;
        double xdiff = (linkL)*sin(angle);
        double ydiff = (linkL)*cos(angle);        
        Vector3d pn = Vector3d{(pointsVect[i - 1](0) + xdiff), 0, (pointsVect[i - 1](2) + ydiff)};
        pointsVect.push_back(pn);
    }
    for(int i = 0; i < pointsVect.size(); i++){
        pointsVect[i] = AngleAxisd(M_PI, Vector3d::UnitX()) *  pointsVect[i];
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
    double EMulitplier = 1;
    
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
    Vector3d backupField = field / 1000;
    field = RotateField(field, Vector3d(0,0,0));
    // Vector3d field = Vector3d(0,0,0);
    Vector3d fedbackField = Vector3d( field(0)/1000, field(1)/1000, field(2)/1000);
    MatrixXd possibleAngles = backwardsQwithField(iLinks, iJoints, iPosVec, fedbackField );
    std::cout << "Initial answer:\n"
              << field << "\n";


    std::vector<double> CalculatedAngles;
    for(int i = 0; i < jointNo - 1; i++){
        CalculatedAngles.push_back(possibleAngles(1+3*i));
    }
    CalculatedAngles.push_back(0);
    std::vector<Vector3d> theoreticalPoints = CalcPoints(DesiredAngles);
    std::vector<Vector3d> calculatedPoints = CalcPoints(possibleAngles);
    std::vector<Vector3d> MagnetisationsOutput = Magnetisations;
    MagnetisationsOutput.insert(MagnetisationsOutput.begin(), Vector3d(0,0,0));
    // for (int i = 0; i < theoreticalPoints.size(); i++)
    // {
    //     std::cout << "PxT, PyT " << theoreticalPoints[i].x << "," << theoreticalPoints[i].y << " vs ";
    //     std::cout << "PxP, PyP " << calculatedPoints[i].x << "," << calculatedPoints[i].y << "\n";
    //     recordPerformance << i << "," << 
    //         theoreticalPoints[i].x << "," << 0 << "," << theoreticalPoints[i].y << "," <<
    //         calculatedPoints[i].x << "," << 0 << "," << calculatedPoints[i].y << "," <<
    //         MagnetisationsOutput[i](0) << "," << MagnetisationsOutput[i](1) << "," << MagnetisationsOutput[i](2) << "," <<
    //         field(0) << "," << field(1) << "," << field(2) << "\n";
    // }
    for (int i = 0; i < possibleAngles.size(); i++)
    {
        if (possibleAngles(i) < 1e-6)
            possibleAngles(i) = 0;
        else
        {
            possibleAngles(i) = possibleAngles(i) * 180 / M_PI;
        }
    }
    std::vector<double> PxT, PyT, PzT;
    for(auto i: theoreticalPoints){
        PxT.push_back(i(0));
        PyT.push_back(i(1));
        PzT.push_back(i(2));
    }
    calculatedPoints.pop_back();
    std::vector<double> PxC, PyC, PzC;
    for(auto i: calculatedPoints){
        PxC.push_back(i(0));
        PyC.push_back(i(1));
        PzC.push_back(i(2));
    }

    std::vector<double> mx, my, mz;
    MagnetisationsOutput.pop_back();
    for(auto i: MagnetisationsOutput){
        Vector3d RotatedMag = AngleAxisd(M_PI, Vector3d::UnitX()) * i;
        mx.push_back(RotatedMag(0));
        my.push_back(RotatedMag(1));
        mz.push_back(RotatedMag(2));
    }


    std::map<std::string, std::string> keywordsQuiverMag;
    keywordsQuiverMag["color"] = "green";
    keywordsQuiverMag["label"] = "magnetisation";

    std::map<std::string, std::string> keywordsPT;
    keywordsPT["color"] = "blue";
    keywordsPT["label"] = "Desired Points";
    keywordsPT["linestyle"]  = "--";
    keywordsPT["marker"] = "o";
    keywordsPT["linewidth"] = "1";
    keywordsPT["markersize"] = "12";

    std::map<std::string, std::string> keywordsPC;
    keywordsPC["color"] = "orange";
    keywordsPC["label"] = "Calculated Points";
    keywordsPC["linestyle"]  = "--";
    keywordsPC["marker"] = "o";
    keywordsPC["linewidth"] = "1";
    keywordsPC["markersize"] = "12";

    std::map<std::string, std::string> keywordsQuiverFeild;
    keywordsQuiverFeild["color"] = "red";
    keywordsQuiverFeild["label"] = "field";

    //out-of-loop plot
    // plt::plot(PxT, PzT, keywordsPT);
    // plt::plot(PxC, PzC, keywordsPC);
    // plt::quiver(PxC, PzC, mx, mz, keywordsQuiverMag);
    // plt::grid(true);
    // plt::xlabel("X Axis");
    // plt::ylabel("Z Axis");
    // plt::legend();
    // plt::show();
    
    int jointsCached = 0;
    int error = 0, prev_error = 0;
    int d_error = 0;
    int step_count = 0;
    bool firstRun = true;
    bool finished = false;
    int baseline_error;
    int zero_count;
    double adjuster = 1;

    // for(int k = 0; k < 40; k++){
    while(true){
        plt::clf();
        
        
        // int Kd = derivativeAdjustment(abs(d_error), error); //send abs because we took care of signs a few lines above.
        if(firstRun){
            baseline_error = positionWiseError(theoreticalPoints, calculatedPoints);
            error = baseline_error;
            firstRun = false;
            prev_error = 0;

            continue;
        }
        error = positionWiseError(theoreticalPoints, calculatedPoints);
        d_error = prev_error - error;
        std::cout << "prev_error(" << prev_error << ") - error(" << error << 
                ") = d_error(" << d_error << 
                "\nBaseline Error( " << baseline_error <<  
                ") - error(" << error << 
                ") = " << baseline_error-error <<")\n";
        
        prev_error = error;

        double Kp = (double) error / (double) baseline_error;  // a decimal of the error wrt the baseline
        
        double KpPercent = Kp * 100; // a Percentage of the error wrt the baseline
        // int signFlag = (error < 0 | d_error < 0) ? -1 : 1;
        double Kd;
        switch( (int) KpPercent) {
            case 0 ... 10:
                Kd = 8; 
            break;

            case 11 ... 20:
                Kd = 4;
            break;

            case 21 ... 40:
                Kd = 2;
            break;

            default:
                Kd = 1;
            break;
        }

        int signFlag;

        if(d_error < 0) {
            signFlag = -1;
        } else signFlag = 1;
        
        std::cout <<
                " KpPercent: " << KpPercent  << 
                " Kp " << Kp << 
                " Kd " << Kd << 
                " signFlag " << signFlag << "\n";
        if( abs(error) < 5 || abs(KpPercent) < 20){
            std::cout << "Error( " << error <<  ") - baseline(" << baseline_error << ")\n";
            std::cout << "Victory\n";
            finished = true;
            plt::show();
            break;
        } 
        // else if( KpPercent < 30){
        //     std::cout << "Small adj\n";
        //     adjuster += Kp*signFlag*Kd;
        // } 
        // else {
        //     std::cout << "Big adj\n";
        //     adjuster += 1*signFlag;
        // }
        if( error - baseline_error > 0 ){
            adjuster += 0.01;
        } else {
            adjuster -= 0.01;
        }
        adjuster = 1;
        field(1) = 0;
        std::cout << "End of iter. Adjuster: "<< adjuster << " field: " <<"\n" << 
                field(0) << "    " << (field(0) *= adjuster) << "\n" <<
                "0.000000" << " -> " << "0.000000" << "\n" <<
                field(2) << "    " << (field(2) *= adjuster) << "\n";
        
        if (field(0) < 1e-4  && field(2) < 1e-4) {
            field = backupField;
            continue;
        }
        
        std::cout << "\n---------------------------------------------------------\n";
        field += field * 0.2 * signFlag;

        std::vector<double> fqX = {PxC[1]}; 
        std::vector<double> fqY ={PzC[3]}; 
        std::vector<double> fqU ={(field(0))}; 
        std::vector<double> fqV ={(field(2))};

        fedbackField = Vector3d( field(0)/1000, field(1)/1000, field(2)/1000);
        
        calculatedPoints.clear();
        possibleAngles = backwardsQwithField(iLinks, iJoints, iPosVec, fedbackField);
        calculatedPoints = CalcPoints(possibleAngles);
        calculatedPoints.pop_back();
        PxC.clear(); 
        PyC.clear(); 
        PzC.clear();
        for(auto i: calculatedPoints){
            PxC.push_back(i(0));
            PyC.push_back(i(1));
            PzC.push_back(i(2));
        }


        plt::quiver( fqX, fqY, fqU, fqV, keywordsQuiverFeild);
        plt::plot(PxT, PzT, keywordsPT);
        plt::plot(PxC, PzC, keywordsPC);
        plt::quiver(PxC, PzC, mx, mz, keywordsQuiverMag);
        plt::xlabel("X Axis");
        plt::ylabel("Y Axis");
        // plt::set_zlabel("z label"); // set_zlabel rather than just zlabel, in accordance with the Axes3D method
        plt::title("Validation");
        plt::grid(true);
        plt::legend();
        // plt::show();
        // plt::clf();
        plt::pause(2);
    }
    // std::cout << "Backwards angles:\n"
    //           << possibleAngles << "\n";
    plt::close();
    return 0;
}
