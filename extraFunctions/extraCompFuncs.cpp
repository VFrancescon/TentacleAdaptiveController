#include "ExtraComputationFuncs.hpp"

/**************************************************************
 * 
 * 
 * PRECOMPUTATION FUNCTIONS DOWN HERE
 * 
 * 
*****************************************************************/

/**
 * @brief Precomputation Function. Evaluates the stiffness Matrix of the system given the links
 * 
 * @param iLinks Vector of Link structs containing all Link mechanical properties
 * @return MatrixXd Diagonally Stacked 3.n x 3.n stiffness matrix
 */
MatrixXd EvaluateK(std::vector<Link> &iLinks){
	std::vector<Matrix3d> K_vec;

    for(int i = 0; i < iLinks.size(); i++){
		double lRadius = iLinks[i].d / 2;
		double I = M_PI_4 * lRadius * lRadius * lRadius * lRadius;
		double G = iLinks[i].E / (2* (iLinks[i].v + 1) );
		double J = M_PI_2 * lRadius * lRadius * lRadius * lRadius;
		double Kb = iLinks[i].E*I/iLinks[i].dL;
		double Kt = G*J/iLinks[i].dL;
		Matrix3d K = Matrix3d::Zero();
		K(0,0) = Kb;
		K(1,1) = Kb;
		K(2,2) = Kt;
		K_vec.push_back(K);
        // std::cout << "I " << i << "\nK\n" << K << "\n";
	}
	
	MatrixXd KDiagonal;
	KDiagonal = StackDiagonals(K_vec);
    return KDiagonal;
}

/**
 * @brief Precomputation Function. Precalculates axis unit vectors and positional vectors for Jacobian computation
 * 
 * @param iPosVec vector containing joint positions and axis orientation
 * @param iJoints vector containing the joint's Transform matrix and pointers to iPosVec
 * @param iLinks vector containing the links and their mechanical properties
 */
void DirectKinematics(std::vector<PosOrientation> &iPosVec, std::vector<Joint> &iJoints, std::vector<Link> &iLinks){
	iJoints[0].Rotation = Matrix3d::Identity();
	iJoints[0].pLocal = Vector3d::Zero();

    int jointEff = (int) iJoints.size();

	for(int i = 1; i < jointEff; i++){
		iJoints[i].Rotation = RotationZYX(iJoints[i-1].Rotation, iJoints[i-1].q);
		iJoints[i].pLocal = iJoints[i-1].pLocal + iJoints[i].Rotation * Vector3d(0,0, -iLinks[i-1].dL);
    }

    for(int i = 0; i < jointEff; i++){
		
		iPosVec[i].p = iJoints[i].pLocal;
		
		iPosVec[i].z(placeholders::all,0) = iJoints[i].Rotation * Vector3d::UnitX();
		
		iPosVec[i].z(placeholders::all,1) = AngleAxisd( iJoints[i].q(0) * M_PI / 180, Vector3d::UnitX() ) *  
							iJoints[i].Rotation * Vector3d::UnitY();
		
		iPosVec[i].z(placeholders::all,2) = AngleAxisd( iJoints[i].q(1) * M_PI / 180, Vector3d::UnitY() ) * 
							AngleAxisd( iJoints[i].q(0) * M_PI / 180, Vector3d::UnitX() ) * 
							iJoints[i].Rotation * Vector3d::UnitZ();	

    }
    return;
}

/**
 * @brief Precomputation Function. Evaluates the jacobian using orientations and positions containing in a vector of points
 * 
 * @param iPosVec vector contains Positions and Orientations
 * @param jointEff Number of effective joints (discount end effector basically)
 * @return MatrixXd size (joint)*6 x (jointEff)*3 containing the full Jacobian computed
 */
MatrixXd EvaluateJacobian(std::vector<PosOrientation> &iPosVec){
	/**
	 * @note
	 * 
	 * Given J = [ 	J00 J01
	 * 				J10 J11 	]
	 * Where J_xy = [Jp_xy
	 * 				Jo_xy]
	 * 
	 * In the loops below, 	i tracks y
	 * 						k tracks x
	 * 
	 * Also the 'stacking' of the full jacobian is actually done by 
	 * initialising an empty Mat of the correct size and filling in the blocks
	 * stacking in the Matrix algebra library we use is possible, but
	 * a pain, so filling is good enough, probably.
	 */
    
    int jointEff = (int) iPosVec.size() -1;
	Matrix3d Jp, Jo;
	MatrixXd Jacobian(jointEff*6, jointEff*3);
	for(int i = 0; i < jointEff; i++){
		//i goes vertically
		for(int k = 0; k < jointEff; k++){
			//k goes horizontally
			if( k > i ) {
				Jp = Matrix3d::Zero();
				Jo = Matrix3d::Zero();
			} else{
                try{
                    Vector3d pDiff = iPosVec[i+1].p - iPosVec[k].p;
                    std::vector<Vector3d> z1{iPosVec[k].z(placeholders::all,0), iPosVec[k].z(placeholders::all,1), iPosVec[k].z(placeholders::all,2)};
                    std::vector<Vector3d> ZcrossP{z1[0].cross(pDiff), z1[1].cross(pDiff), z1[2].cross(pDiff)};
                    Jp << ZcrossP[0] , ZcrossP[1], ZcrossP[2];
                    Jo << z1[0], z1[1], z1[2];
                    
                    // std::cout << "i: " << i << " k: " << k << "\n";
                    // std::cout << "Jp\n" << Jp << "\n";
                    // std::cout << "Jo\n" << Jo << "\n";	
                    
                    }
                    
                catch(std::exception &e){
                    std::cout << "caught error at e: " << e.what() << "\n";
                    throw;
                }
			}
			MatrixXd Jn( Jp.rows() + Jo.rows(), Jp.cols());	
			Jn << Jp, 
				Jo;
			Jacobian(seq(0+i*6, 5+i*6), seq(0+k*3,2+k*3)) = Jn;
		}
	}
	return Jacobian;
}

/**
 * @brief Precomputation function. Calculates a Vertically stacked Map from Magnetisation to Field
 * 
 * @param iJoints ordered vector of joints, each containing the set magnetisation
 * @return MatrixXd 6*n x 3 matrix of vertically stacked 0(3x3) and maps. See Llyod 2020 and Salmanipour/Diller 2018
 */
MatrixXd MagtoFieldMap(std::vector<Joint> &iJoints){
    MatrixXd Map(6*(iJoints.size()-1), 3);
    Map = MatrixXd::Zero(6*(iJoints.size()-1), 3);
    Matrix3d Zeros = Matrix3d::Zero();
    for(int i = 0; i < iJoints.size()-1; i++){
        iJoints[i].GlobMag = iJoints[i+1].Rotation * iJoints[i].LocMag;
        Matrix3d Skewd = SkewMatrix(iJoints[i].GlobMag);
        Map( seqN(3+6*i, 3), seqN(0, 3)  ) = -Skewd;         
    }
    // std::cout << "Magtofield map\n" << Map << "\n";
    // std::cout << "Sizing: " << Map.rows() << "x" << Map.cols() << "\n";
    // std::cout << "Size of ijoints " << iJoints.size() << "\n";
    return Map;
}


Matrix3d SkewMatrix(Vector3d src){
    Matrix3d Skew;
    Skew = Matrix3d::Zero();
    Skew << 0, -src[2], src[1],
            src[2], 0, -src[0],
            -src[1], src[0], 0;
    return Skew;
}

VectorXd StackAngles(std::vector<Joint>& iJoints){
    int jointEff = iJoints.size();
    VectorXd stacked;

    if(jointEff == 2){
        stacked = iJoints[0].q;
    }
    else{
        stacked = VerticalStack(iJoints[0].q, iJoints[1].q);
        for(int i = 2; i < jointEff-1; i++){
            stacked = VerticalStack(stacked, iJoints[i].q);
        }
    }

    // stacked = stacked * M_PI / 180;

    return stacked;
}

/**
 * @brief Utility Function. Vertically Stacks a 3x3 Matrix onto an existing Xx3 Matrix
 * 
 * @param M1 Matrix of any number of rows, but 3 columns
 * @param M2 3x3 Matrix to stack below
 * @return MatrixXd resultatant of stack
 */
MatrixXd VerticalStack(MatrixXd M1, MatrixXd M2){
    MatrixXd Stack(M1.rows() + M2.rows(), M1.cols());
    Stack << M1, M2;
    return Stack;
}

/**
 * @brief Utility Function. Creates a diagonal matrix by stacking 3x3 matrices contained in vector matrices
 * 
 * @param matrices vector containing n 3x3 matrices
 * @return MatrixXd - 3*n, 3*n Matrix containing the diagonal stack
 */
MatrixXd StackDiagonals(std::vector<Matrix3d> matrices){
	MatrixXd diagonal(matrices.size()*3, matrices.size()*3);
    diagonal = MatrixXd::Zero(diagonal.rows(), diagonal.cols());
	for(size_t i = 0; i < matrices.size(); i++){
		
		diagonal( seq(i*3, 2+i*3), seq(i*3, 2+i*3)) = matrices[i];
	
	}
	// std::cout << "Diagonal evaluated\n" << diagonal << "\n";
    return diagonal;
}

/**
 * @brief Explicit reimplementation or RotationZYX. Applies the same principle to rotate the applied field.
 * Rotates field by Z and then X
 * 
 * @param field 3d vector containing applied field in all directions.
 * @param rotationAngles Angles required to rotate, leave Y blank
 * @return Vector3d 
 */
Vector3d RotateField(Vector3d field, Vector3d rotationAngles){
    double AngleZ = rotationAngles(2) * M_PI / 180;
    double AngleX = rotationAngles(0) * M_PI / 180;

    return AngleAxisd(AngleZ, Vector3d::UnitZ()) 
                * AngleAxisd(AngleX, Vector3d::UnitX()) * field;
}

/**
 * @brief Utility Function. Rotates matrix src by angles in vector jointAngles in the ZYX order.
 * 
 * @param src matrix containing original position
 * @param jointAngles column vector containing rotations. (X,Y,Z) 
 * @return Matrix3d, Rotated matrix after being multiplied by angles jointAngles 
 */
Matrix3d RotationZYX(Matrix3d src, Vector3d jointAngles){
	double AngleZ = jointAngles(2) * M_PI / 180;
	double AngleY = jointAngles(1) * M_PI / 180;
	double AngleX = jointAngles(0) * M_PI / 180;
	
	return src * AngleAxisd(AngleZ, Vector3d::UnitZ())
		* AngleAxisd(AngleY, Vector3d::UnitY())
		* AngleAxisd(AngleX, Vector3d::UnitX());
}

/***********************************************
 * Real time adjustment functions below
 * 
 * 
************************************************/

void adjustStiffness(std::vector<Link> &iLinks, double EMulitplier){
    dfltValues MechPpts;

    for(int i = 0; i < iLinks.size(); i++){
        iLinks[i].dL = MechPpts.len;
        iLinks[i].d = MechPpts.d;
        iLinks[i].E = MechPpts.E * EMulitplier;
        iLinks[i].v = MechPpts.v;

    }

}


Vector3d CalculateField(std::vector<Link> &iLinks, std::vector<Joint> &iJoints, 
    std::vector<PosOrientation> &iPosVec){
    MatrixXd KStacked;
    KStacked = EvaluateK(iLinks);
    int jointNo = iLinks.size();
    VectorXd AnglesStacked;
    AnglesStacked = StackAngles(iJoints);
    

    DirectKinematics(iPosVec, iJoints, iLinks);
    MatrixXd Jacobian, Jt;
    Jacobian = EvaluateJacobian(iPosVec);
    Jt = Jacobian.transpose();

    MatrixXd FieldMap;
    FieldMap = MagtoFieldMap(iJoints);

    MatrixXd RHS = Jt*FieldMap;
    AnglesStacked = AnglesStacked * M_PI / 180;
    MatrixXd LHS = KStacked * AnglesStacked;
    
    MatrixXd solution = RHS.completeOrthogonalDecomposition().solve(LHS);
    return solution * 1000;
}