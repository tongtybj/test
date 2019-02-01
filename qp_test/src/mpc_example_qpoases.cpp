// osqp-eigen
#include <qpOASES.hpp>
USING_NAMESPACE_QPOASES

// eigen
#include <Eigen/Dense>
#include <Eigen/Sparse>

#include <chrono>

#include <iostream>

void setDynamicsMatrices(Eigen::Matrix<double, 12, 12> &a, Eigen::Matrix<double, 12, 4> &b)
{
    a << 1.,      0.,     0., 0., 0., 0., 0.1,     0.,     0.,  0.,     0.,     0.    ,
        0.,      1.,     0., 0., 0., 0., 0.,      0.1,    0.,  0.,     0.,     0.    ,
        0.,      0.,     1., 0., 0., 0., 0.,      0.,     0.1, 0.,     0.,     0.    ,
        0.0488,  0.,     0., 1., 0., 0., 0.0016,  0.,     0.,  0.0992, 0.,     0.    ,
        0.,     -0.0488, 0., 0., 1., 0., 0.,     -0.0016, 0.,  0.,     0.0992, 0.    ,
        0.,      0.,     0., 0., 0., 1., 0.,      0.,     0.,  0.,     0.,     0.0992,
        0.,      0.,     0., 0., 0., 0., 1.,      0.,     0.,  0.,     0.,     0.    ,
        0.,      0.,     0., 0., 0., 0., 0.,      1.,     0.,  0.,     0.,     0.    ,
        0.,      0.,     0., 0., 0., 0., 0.,      0.,     1.,  0.,     0.,     0.    ,
        0.9734,  0.,     0., 0., 0., 0., 0.0488,  0.,     0.,  0.9846, 0.,     0.    ,
        0.,     -0.9734, 0., 0., 0., 0., 0.,     -0.0488, 0.,  0.,     0.9846, 0.    ,
        0.,      0.,     0., 0., 0., 0., 0.,      0.,     0.,  0.,     0.,     0.9846;

    b << 0.,      -0.0726,  0.,     0.0726,
        -0.0726,  0.,      0.0726, 0.    ,
        -0.0152,  0.0152, -0.0152, 0.0152,
        -0.,     -0.0006, -0.,     0.0006,
        0.0006,   0.,     -0.0006, 0.0000,
        0.0106,   0.0106,  0.0106, 0.0106,
        0,       -1.4512,  0.,     1.4512,
        -1.4512,  0.,      1.4512, 0.    ,
        -0.3049,  0.3049, -0.3049, 0.3049,
        -0.,     -0.0236,  0.,     0.0236,
        0.0236,   0.,     -0.0236, 0.    ,
        0.2107,   0.2107,  0.2107, 0.2107;
}


void setInequalityConstraints(Eigen::Matrix<double, 12, 1> &xMax, Eigen::Matrix<double, 12, 1> &xMin,
                              Eigen::Matrix<double, 4, 1> &uMax, Eigen::Matrix<double, 4, 1> &uMin)
{
    double u0 = 10.5916;

    // input inequality constraints
    uMin << 9.6 - u0,
        9.6 - u0,
        9.6 - u0,
        9.6 - u0;

    uMax << 13 - u0,
        13 - u0,
        13 - u0,
        13 - u0;

    // state inequality constraints
    xMin << -M_PI/6,-M_PI/6,-INFTY,-INFTY,-INFTY,-1.,
        -INFTY, -INFTY,-INFTY,-INFTY,
        -INFTY,-INFTY;

    xMax << M_PI/6,M_PI/6, INFTY,INFTY,INFTY,
        INFTY, INFTY,INFTY,INFTY,
        INFTY,INFTY,INFTY;
}

void setWeightMatrices(Eigen::DiagonalMatrix<double, 12> &Q, Eigen::DiagonalMatrix<double, 4> &R)
{
    Q.diagonal() << 0, 0, 10., 10., 10., 10., 0, 0, 0, 5., 5., 5.;
    R.diagonal() << 0.1, 0.1, 0.1, 0.1;
}

void castMPCToQPHessian(const Eigen::DiagonalMatrix<double, 12> &Q, const Eigen::DiagonalMatrix<double, 4> &R, int mpcWindow,
                        Eigen::MatrixXd &hessianMatrix)
{

    hessianMatrix = Eigen::MatrixXd::Zero(12*(mpcWindow+1) + 4 * mpcWindow, 12*(mpcWindow+1) + 4 * mpcWindow);

    //populate hessian matrix
    for(int i = 0; i<12*(mpcWindow+1) + 4 * mpcWindow; i++){
        if(i < 12*(mpcWindow+1)){
            int posQ=i%12;
            float value = Q.diagonal()[posQ];
            if(value != 0)
                hessianMatrix(i,i) = value;
        }
        else{
            int posR=i%4;
            float value = R.diagonal()[posR];
            if(value != 0)
                hessianMatrix(i,i) = value;
        }
    }
}

void castMPCToQPGradient(const Eigen::DiagonalMatrix<double, 12> &Q, const Eigen::Matrix<double, 12, 1> &xRef, int mpcWindow,
                         Eigen::VectorXd &gradient)
{

    Eigen::Matrix<double,12,1> Qx_ref;
    Qx_ref = Q * (-xRef);

    // populate the gradient vector
    gradient = Eigen::VectorXd::Zero(12*(mpcWindow+1) +  4*mpcWindow, 1);
    for(int i = 0; i<12*(mpcWindow+1); i++){
        int posQ=i%12;
        float value = Qx_ref(posQ,0);
        gradient(i,0) = value;
    }
}

void castMPCToQPConstraintMatrix(const Eigen::Matrix<double, 12, 12> &dynamicMatrix, const Eigen::Matrix<double, 12, 4> &controlMatrix,
                                 int mpcWindow, Eigen::MatrixXd &constraintMatrix)
{
  constraintMatrix = Eigen::MatrixXd::Zero(12*(mpcWindow+1), 12*(mpcWindow+1) + 4 * mpcWindow);

    // populate linear constraint matrix
    for(int i = 0; i<12*(mpcWindow+1); i++){
        constraintMatrix(i,i) = -1;
    }

    for(int i = 0; i < mpcWindow; i++)
        for(int j = 0; j<12; j++)
            for(int k = 0; k<12; k++){
                float value = dynamicMatrix(j,k);
                if(value != 0){
                    constraintMatrix(12 * (i+1) + j, 12 * i + k) = value;
                }
            }

    for(int i = 0; i < mpcWindow; i++)
        for(int j = 0; j < 12; j++)
            for(int k = 0; k < 4; k++){
                float value = controlMatrix(j,k);
                if(value != 0){
                    constraintMatrix(12*(i+1)+j, 4*i+k+12*(mpcWindow + 1)) = value;
                }
            }
}

void castMPCToQPConstraintVectors(const Eigen::Matrix<double, 12, 1> &xMax, const Eigen::Matrix<double, 12, 1> &xMin,
                                   const Eigen::Matrix<double, 4, 1> &uMax, const Eigen::Matrix<double, 4, 1> &uMin,
                                   const Eigen::Matrix<double, 12, 1> &x0,
                                  int mpcWindow, Eigen::VectorXd &lowerBound_A, Eigen::VectorXd &upperBound_A, Eigen::VectorXd &lowerBound_b, Eigen::VectorXd &upperBound_b)
{
    // evaluate the lower and the upper inequality vectors
    lowerBound_b = Eigen::MatrixXd::Zero(12*(mpcWindow+1) +  4 * mpcWindow, 1);
    upperBound_b = Eigen::MatrixXd::Zero(12*(mpcWindow+1) +  4 * mpcWindow, 1);
    for(int i=0; i<mpcWindow+1; i++){
      lowerBound_b.block(12*i,0,12,1) = xMin;
      upperBound_b.block(12*i,0,12,1) = xMax;
    }

    for(int i=0; i<mpcWindow; i++){
      lowerBound_b.block(4 * i + 12 * (mpcWindow + 1), 0, 4, 1) = uMin;
      upperBound_b.block(4 * i + 12 * (mpcWindow + 1), 0, 4, 1) = uMax;
    }

    // evaluate the lower and the upper equality vectors
    lowerBound_A = Eigen::MatrixXd::Zero(12*(mpcWindow+1),1 );
    lowerBound_A.block(0,0,12,1) = -x0;
    upperBound_A = lowerBound_A;
}


void updateConstraintVectors(const Eigen::Matrix<double, 12, 1> &x0,
                             Eigen::VectorXd &lowerBound_A, Eigen::VectorXd &upperBound_A)
{
    lowerBound_A.block(0,0,12,1) = -x0;
    upperBound_A.block(0,0,12,1) = -x0;
}


double getErrorNorm(const Eigen::Matrix<double, 12, 1> &x,
                    const Eigen::Matrix<double, 12, 1> &xRef)
{
    // evaluate the error
    Eigen::Matrix<double, 12, 1> error = x - xRef;

    // return the norm
    return error.norm();
}


int main()
{
    // set the preview window
    int mpcWindow = 20;

    // allocate the dynamics matrices
    Eigen::Matrix<double, 12, 12> a;
    Eigen::Matrix<double, 12, 4> b;

    // allocate the constraints vector
    Eigen::Matrix<double, 12, 1> xMax;
    Eigen::Matrix<double, 12, 1> xMin;
    Eigen::Matrix<double, 4, 1> uMax;
    Eigen::Matrix<double, 4, 1> uMin;

    // allocate the weight matrices
    Eigen::DiagonalMatrix<double, 12> Q;
    Eigen::DiagonalMatrix<double, 4> R;

    // allocate the initial and the reference state space
    Eigen::Matrix<double, 12, 1> x0;
    Eigen::Matrix<double, 12, 1> xRef;

    // allocate QP problem matrices and vectores
    Eigen::MatrixXd hessian;
    Eigen::VectorXd gradient;
    Eigen::MatrixXd linearMatrix;
    Eigen::MatrixXd linearMatrix_t;
    Eigen::VectorXd lowerBound_A, lowerBound_b;
    Eigen::VectorXd upperBound_A, upperBound_b;

    // set the initial and the desired states
    x0 << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ;
    xRef <<  0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0;

    // set MPC problem quantities
    setDynamicsMatrices(a, b);
    setInequalityConstraints(xMax, xMin, uMax, uMin);
    setWeightMatrices(Q, R);

    // cast the MPC problem as QP problem
    castMPCToQPHessian(Q, R, mpcWindow, hessian);
    castMPCToQPGradient(Q, xRef, mpcWindow, gradient);
    castMPCToQPConstraintMatrix(a, b, mpcWindow, linearMatrix);
    castMPCToQPConstraintVectors(xMax, xMin, uMax, uMin, x0, mpcWindow, lowerBound_A, upperBound_A, lowerBound_b, upperBound_b);
    linearMatrix_t = linearMatrix.transpose();

    // instantiate the solver
    SQProblem qp_solver(12 * (mpcWindow + 1) + 4 * mpcWindow,  12 * (mpcWindow + 1));
    int n_wsr;
    Options qp_options;
    qp_options.enableEqualities = BT_TRUE;
    qp_options.printLevel = PL_LOW;
    qp_solver.setOptions(qp_options);

    // controller input and QPSolution vector
    Eigen::Vector4d ctr;
    Eigen::VectorXd QPSolution;

    // number of iteration steps
    int numberOfSteps = 50;

    auto t_start = std::chrono::high_resolution_clock::now();

    for (int i = 0; i < numberOfSteps; i++)
      {
        n_wsr = 200;
        if(i == 0)
          {
            qp_solver.init(hessian.data(), gradient.data(),
                           linearMatrix_t.data(),
                           lowerBound_b.data(), upperBound_b.data(),
                           lowerBound_A.data(), upperBound_A.data(), n_wsr);
          }
        else
          {
            /* full update
            qp_solver.hotstart(hessian.data(), gradient.data(),
                               linearMatrix_t.data(),
                               lowerBound_b.data(), upperBound_b.data(),
                               lowerBound_A.data(), upperBound_A.data(), n_wsr);
            */

            qp_solver.hotstart(gradient.data(),
                               lowerBound_b.data(), upperBound_b.data(),
                               lowerBound_A.data(), upperBound_A.data(), n_wsr);

          }

        Eigen::VectorXd result_vector = Eigen::VectorXd::Zero(12 * (mpcWindow + 1) + 4 * mpcWindow);
        qp_solver.getPrimalSolution(result_vector.data());
        ctr = result_vector.block(12 * (mpcWindow + 1), 0, 4, 1);

        std::cout << x0.transpose() << std::endl;

        // propagate the model
        x0 = a * x0 + b * ctr;

        // update the constraint bound
        updateConstraintVectors(x0, lowerBound_A, upperBound_A);
      }

    auto t_end = std::chrono::high_resolution_clock::now();

    std::cout << "Wall clock time passed: "
              << std::chrono::duration<double, std::milli>(t_end-t_start).count()
              << " ms\n";

    return 0;
}
