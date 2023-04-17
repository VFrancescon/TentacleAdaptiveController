#ifndef COMPFUNCS
#define COMPFUNCS

#include <iostream>
#include <math.h>
#include <vector>
#include <cassert>

#include <eigen3/Eigen/Core>
#include <DataTypes/DataTypes.hpp>

template <typename T>
void pop_front(std::vector<T> &vec)
{
    assert(!vec.empty());
    vec.erase(vec.begin());
};

template <typename T>
void pop_2ndback(std::vector<T> &vec)
{
    assert(!vec.empty());
    vec.erase(vec.end() - 2);
};

struct dfltValues
{
    float len = 10e-3;
    float d = 2e-3;
    float E = 100e3;
    float v = 0.43;
};

// Evaluation functions

/**
 * @brief Precomputation Function. Evaluates the stiffness Matrix of the system given the links
 *
 * @param iLinks Vector of Link structs containing all Link mechanical properties
 * @return MatrixXd Diagonally Stacked 3.n x 3.n stiffness matrix
 */
MatrixXd EvaluateK(std::vector<Link> &iLinks);

/**
 * @brief Precomputation Function. Precalculates axis unit vectors and positional vectors for Jacobian computation
 *
 * @param iPosVec vector containing joint positions and axis orientation
 * @param iJoints vector containing the joint's Transform matrix and pointers to iPosVec
 * @param iLinks vector containing the links and their mechanical properties
 */
void DirectKinematics(std::vector<PosOrientation> &iPosVec, std::vector<Joint> &iJoints, std::vector<Link> &iLinks);

/**
 * @brief Precomputation Function. Evaluates the jacobian using orientations and positions containing in a vector of points
 *
 * @param iPosVec vector contains Positions and Orientations
 * @param jointEff Number of effective joints (discount end effector basically)
 * @return MatrixXd size (joint)*6 x (jointEff)*3 containing the full Jacobian computed
 */
MatrixXd EvaluateJacobian(std::vector<PosOrientation> &iPosVec);

/**
 * @brief Precomputation function. Calculates a Vertically stacked Map from Magnetisation to Field
 *
 * @param iJoints ordered vector of joints, each containing the set magnetisation
 * @return MatrixXd 6*n x 3 matrix of vertically stacked 0(3x3) and maps. See Llyod 2020 and Salmanipour/Diller 2018
 */
MatrixXd MagtoFieldMap(std::vector<Joint> &iJoints);

// Utility functions

/**
 * @brief Utility Function. Creates a diagonal matrix by stacking 3x3 matrices contained in vector matrices
 *
 * @param matrices vector containing n 3x3 matrices
 * @return MatrixXd - 3*n, 3*n Matrix containing the diagonal stack
 */
MatrixXd StackDiagonals(std::vector<Matrix3d> matrices);

/**
 * @brief Utility Function. Rotates matrix src by angles in vector jointAngles in the ZYX order.
 *
 * @param src matrix containing original position
 * @param jointAngles column vector containing rotations. (X,Y,Z)
 * @return Matrix3d, Rotated matrix after being multiplied by angles jointAngles
 */
Matrix3d RotationZYX(Matrix3d src, Vector3d jointAngles);

/**
 * @brief Builds skew-symmetric matrix out of 3d vector src
 *
 * @param src 3d vector of doubles
 * @return Matrix3d skew-symmetrix matrix built from src
 */
Matrix3d SkewMatrix(Vector3d src);

/**
 * @brief Utility Function. Vertically Stacks a 3x3 Matrix onto an existing Xx3 Matrix
 *
 * @param M1 Matrix of any number of rows, but 3 columns
 * @param M2 3x3 Matrix to stack below
 * @return MatrixXd resultatant of stack
 */
MatrixXd VerticalStack(MatrixXd M1, MatrixXd M2);

/**
 * @brief Stacks all joint angles into one x by 1 vector
 *
 * @param iJoints vector containing all Joint data
 * @return VectorXd column vector containing all angles stacked
 */
VectorXd StackAngles(std::vector<Joint> &iJoints);

/**
 * @brief Explicit reimplementation or RotationZYX. Applies the same principle to rotate the applied field.
 * Rotates field by Z and then X
 *
 * @param field 3d vector containing applied field in all directions.
 * @param rotationAngles Angles required to rotate, leave Y blank
 * @return Vector3d
 */
Vector3d RotateField(Vector3d field, Vector3d rotationAngles);

/**
 * @brief Adjusts stiffness Matrix by changing the multiplier applied to the Young's Modulus
 *
 * @param iLinks Vector containing all the link data
 * @param EMulitplier multiplier applied to the Young's Modulus
 */
void adjustStiffness(std::vector<Link> &iLinks, double EMulitplier);

/**
 * @brief Calculates Field B required to satisfy Q = K^-1.J^T.S.B
 *
 * @param iLinks Vector containing link-related data (stiffness)
 * @param iJoints Vector containing joint-related data (angular displacement)
 * @param iPosVec Vector containing position-related data (linear displacement)
 * @return Vector3d Recalculates K, J and S, then feeds forward to solve for field B
 */
Vector3d CalculateField(std::vector<Link> &iLinks, std::vector<Joint> &iJoints, std::vector<PosOrientation> &iPosVec);

/**
 * @brief Solves backwards with field to find obtainable joint angles. also factors in field multiplier.
 *
 * @param iLinks Vector containing link-related data (stiffness)
 * @param iJoints Vector containing joint-related data (angular displacement)
 * @param iPosVec Vector containing position-related data (linear displacement)
 * @param fieldMultiplier Postproc multiplier to field. (Default 1)
 * @return Vector3d Recalculates K, J and S, then feeds forward to solve for field B
 */
MatrixXd backwardsQ(std::vector<Link> &iLinks, std::vector<Joint> &iJoints,
                    std::vector<PosOrientation> &iPosVec, double fieldMultiplier=1);

#endif