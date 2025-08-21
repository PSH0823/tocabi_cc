#include "mpc.h"

//TODO 2
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Set the path for data.txt file to your directory.
//ex. ofstream dataMPC_zmp_com ("/home/dyros/ubuntu-20-04/dataMPC_zmp_com.txt");
//The absolute path to the directory can be check by typing "pwd" command to the terminal.
ofstream dataMPC_zmp_com ("/home/dyros/ubuntu-20-04/dataMPC_zmp_com.txt");
ofstream dataMPC_time    ("/home/dyros/ubuntu-20-04/dataMPC_time.txt");
ofstream dataMPC_data1   ("/home/dyros/ubuntu-20-04/dataMPC_data1.txt");
ofstream dataMPC_data2   ("/home/dyros/ubuntu-20-04/dataMPC_data2.txt");
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

MPC::MPC(double initMpcFreq, double initN) : mpc_freq(initMpcFreq), N(initN)
{
    std::cout << "MPC FREQ: " << mpc_freq << "Hz" << std::endl;
    std::cout << "MPC HORIZON: " << N / mpc_freq << "s" << std::endl;
    std::cout << "MPC CLASS IS SUCCESSFULLY CONSTRUCTED" << std::endl;
}

void MPC::cartTableModel(double T, double h)
{
    /*
    Variables in walking controller
    ----------
    A_mpc   : global eigen matrix variable for A matrix of the Cart Table Model.
    B_mpc   : global eigen vector variable for B matrix of the Cart Table Model.
    C_mpc   : global eigen matrix variable for C matrix of the Cart Table Model.

    T       : function arguments for the time intervals of one MPC control tick.
    h       : function arguments for the CoM height of Cart Table Model.
    GRAVITY : global variable that contains the gravity acceleration.


    Useful functions for Eigen variable.
    ----------
    SetZero        : Matrix.setZero(n,m).         SetZero function can be utilized to initialize the variables with given size.
                                                  Resize the matrix to nxm size and set all element of matrix to zero.
                     Matrix.setZero().            You can use setZero function without resizing.
    resize         : Matrix.resize(n,m).          Resize the matrix to nxm size. All element could be zero but not guaranteed.
    block          : Matrix.block(p, q, n, m).    Get access to the part of matrix, size nxm starting at (p,q).
    segment        : Vector.segment(p,n).         Get access to the part of vector, size n starting at p.
    col            : Matrix.col(n).               Get nth column of matrix.
    row            : Matrix.row(m).               Get mth row of matrix.
    inverse        : Matrix.inverse().            Get inverse matrix of square matrix.

    element access : Matrix(n,m)                  Get access to element of Matrix.
                   : Vector(n)                    Get access to element of Vector.
    
    Assignment     : Matrix                       = SameSizeMatrix or part of it as AnotherMatrix.block(p, q, n, m);
                   : Matrix.block(p, q, n, m)     = SameSizeMatrix or AnotherMatrix.block(p, q, n, m);
                   : Matrix.block(p, q, n, 1)     = SameSizeVector or Vector.segment(p, n);
                   : Matrix.col(n) or row(m)      = SameSizeVector or Matrix or part of them as above;
                   
                   : Vector                       = SameSizeVector or Matrix or part of them as above;
                   : Vector.segment(p,n)          = SameSizeVector or Matrix or part of them as above;
    */

    //TODO 3
    //Define the matrix size and element for the Cart Table Model, A, B, C and save them in the global variable.
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    A_mpc.setZero(3, 3);
    B_mpc.setZero(3, 1);
    C_mpc.setZero(1, 3);

    A_mpc(0, 0) = 1.0;
    A_mpc(0, 1) = T;
    A_mpc(0, 2) = T * T / 2.0;
    A_mpc(1, 0) = 0.0;
    A_mpc(1, 1) = 1.0;
    A_mpc(1, 2) = T;
    A_mpc(2, 0) = 0.0;
    A_mpc(2, 1) = 0.0;
    A_mpc(2, 2) = 1.0;

    B_mpc(0, 0) = T * T * T / 6.0;
    B_mpc(1, 0) = T * T / 2.0;
    B_mpc(2, 0) = T;

    C_mpc(0, 0) = 1.0;
    C_mpc(0, 1) = 0.0;
    C_mpc(0, 2) = -h / GRAVITY;
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
}

void MPC::cartTableModelMPC(double T, double h)
{
    /*
    Variables in walking controller
    ----------
    P_zs_mpc : global eigen matrix variable for Pzs matrix of the Model Predictive Control to stabilize ZMP.
    P_ps_mpc : global eigen matrix variable for Pps matrix of the Model Predictive Control to stabilize ZMP.
    P_zu_mpc : global eigen matrix variable for Pzu matrix of the Model Predictive Control to stabilize ZMP.
    P_pu_mpc : global eigen matrix variable for Ppu matrix of the Model Predictive Control to stabilize ZMP.

    T        : function arguments for the time intervals of one MPC control tick.
    h        : function arguments for the CoM height of Cart Table Model.
    GRAVITY  : global variable that contains the gravity acceleration.


    Useful functions for Eigen variable.
    ----------
    SetZero        : Matrix.setZero(n,m).         SetZero function can be utilized to initialize the variables with given size.
                                                     Resize the matrix to nxm size and set all element of matrix to zero.
                     Matrix.setZero().            You can use setZero function without resizing.
    resize         : Matrix.resize(n,m).          Resize the matrix to nxm size. All element could be zero but not guaranteed.
    block          : Matrix.block(p, q, n, m).    Get access to the part of matrix, size nxm starting at (p,q).
    segment        : Vector.segment(p,n).         Get access to the part of vector, size n starting at p.
    col            : Matrix.col(n).               Get nth column of matrix.
    row            : Matrix.row(m).               Get mth row of matrix.
    inverse        : Matrix.inverse().            Get inverse matrix of square matrix.

    element access : Matrix(n,m)                  Get access to element of Matrix.
                   : Vector(n)                    Get access to element of Vector.
    
    Assignment     : Matrix                       = SameSizeMatrix or part of it as AnotherMatrix.block(p, q, n, m);
                   : Matrix.block(p, q, n, m)     = SameSizeMatrix or AnotherMatrix.block(p, q, n, m);
                   : Matrix.block(p, q, n, 1)     = SameSizeVector or Vector.segment(p, n);
                   : Matrix.col(n) or row(m)      = SameSizeVector or Matrix or part of them as above;
                   
                   : Vector                       = SameSizeVector or Matrix or part of them as above;
                   : Vector.segment(p,n)          = SameSizeVector or Matrix or part of them as above;
    */

    //Function to define matrix A, B, C for the cart table model.
    cartTableModel(T, h);

    //TODO 4
    //Define the matrix size and element for the Model Predictive Control P_zs_mpc, P_ps_mpc, P_zu_mpc, P_pu_mpc to stabilize ZMP
    //with A, B, C matrix in Cart Table Model and save them in the global variable.
    //'if' and 'for' statement can be utilized.
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    P_ps_mpc.setZero(N, 3);
    P_pu_mpc.setZero(N, N);
    P_zs_mpc.setZero(N, 3);
    P_zu_mpc.setZero(N, N);

    for (int i = 0; i < N; i++)
    {
        P_ps_mpc(i, 0) = 1.0;
        P_ps_mpc(i, 1) = (i + 1) * T;
        P_ps_mpc(i, 2) = ((i + 1) * (i + 1) * T * T) / 2.0;

        P_zs_mpc(i, 0) = 1.0;
        P_zs_mpc(i, 1) = (i + 1) * T;
        P_zs_mpc(i, 2) = ((i + 1) * (i + 1) * T * T) / 2.0 - h / GRAVITY;

        for (int j = 0; j < N; j++)
        {
            if (j > i)
            {
                P_pu_mpc(i, j) = 0.0;
                P_zu_mpc(i, j) = 0.0;
            }
            else
            {
                P_pu_mpc(i, j) = (1 + 3 * (i - j) + 3 * (i - j) * (i - j)) * (T * T * T) / 6.0;
                P_zu_mpc(i, j) = (1 + 3 * (i - j) + 3 * (i - j) * (i - j)) * (T * T * T) / 6.0 - (T * h / GRAVITY);
            }
        }
    }
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
}

void MPC::ComTrajectoryGenerator(Eigen::VectorXd &zx_ref, Eigen::VectorXd &zy_ref, Eigen::VectorXd &zx_ref_wo_offset, Eigen::VectorXd &zy_ref_wo_offset, Eigen::Vector3d &x_hat, Eigen::Vector3d &y_hat, double comHeight)
{
    //variable to check the time taken for the Model Predictive Control.
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

    //variable for the the time intervals of one MPC control tick.
    double dT = 1.0 / mpc_freq;

    //variables fo the calculation of Q, R matrix.
    double w1 = 1e-0;
    double w2 = 1e-6;

    //initialization
    if(is_mpc_init_ == true)
    {
        //Function to define matrix for the Model Predictive Control.
        cartTableModelMPC(dT, comHeight);

        //Q, R matrix for the cost function
        Q_mpc = w1*MatrixXd::Identity(N, N);
        R_mpc = w2*MatrixXd::Identity(N, N);
        
        //TODO 5
        //Calculate the Hessian Matrix with matrix above.
        //Note that it does not vary according to the control tick.
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        Hess_.setZero(N, N);
        Hess_ = P_zu_mpc.transpose() * Q_mpc * P_zu_mpc + R_mpc;
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        //Gradient Matrix for the Model Predictive Control.
        grad_x.setZero(N);
        grad_y.setZero(N);

        //Vector variables for the constraint of the Model Predictive Control.
        lb_x.setZero(N);
        ub_x.setZero(N);
        lb_y.setZero(N);
        ub_y.setZero(N);

        //Vector variables for the input of the Model Predictive Control.
        U_x.setZero(N);
        U_y.setZero(N);
        
        //Initialize the Problem Size for the QPoases variables.
        QP_mpc_x_.InitializeProblemSize(N, N);
        QP_mpc_y_.InitializeProblemSize(N, N);

        is_mpc_init_ = false;

        std::cout << "SUCCESS TO INITIALIZE MPC PARAMETER" << std::endl;
    }

    /*
    Variables in walking controller
    ----------
    zx_ref : local eigen vector variable contains x-direction reference ZMP calculated from getZMPtrajectory() and sended to MPC thread.
    zy_ref : local eigen vector variable contains y-direction reference ZMP calculated from getZMPtrajectory() and sended to MPC thread.
    */
    //TODO 6
    //Calculate the gradient matrix
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    grad_x = P_zu_mpc.transpose() * Q_mpc * (P_zs_mpc * x_hat - zx_ref);
    grad_y = P_zu_mpc.transpose() * Q_mpc * (P_zs_mpc * y_hat - zy_ref);
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    //eigen vector variables contain limit of ZMP
    //Since the reference ZMP trajectory without any offset connects the center of the each supporting foot,
    //the limit for the ZMP trajectory can be calculated by utilizing it.
    Eigen::VectorXd zmp_x_max, zmp_x_min;
    Eigen::VectorXd zmp_y_max, zmp_y_min;

    zmp_x_max = zx_ref_wo_offset + 0.17*Eigen::VectorXd::Ones(N);
    zmp_x_min = zx_ref_wo_offset - 0.13*Eigen::VectorXd::Ones(N);

    zmp_y_max = zy_ref_wo_offset + 0.08*Eigen::VectorXd::Ones(N);
    zmp_y_min = zy_ref_wo_offset - 0.08*Eigen::VectorXd::Ones(N);

    /*
    Variables in walking controller
    ----------
    x_hat : global eigen vector variable contains the x-direction state in the MPC thread.
    y_hat : global eigen vector variable contains the y-direction state in the MPC thread.

    zmp_x_max : local eigen vector variable contains maximum limit of x-direction ZMP.
    zmp_x_min : local eigen vector variable contains minimum limit of x-direction ZMP.

    zmp_y_max : local eigen vector variable contains maximum limit of y-direction ZMP.
    zmp_y_min : local eigen vector variable contains minimum limit of y-direction ZMP.
    */
    //TODO 7
    //Calculate the vectors for the constraint
    //Utilizing the ZMP reference trajectory wo offset, the constraint for the MPC can be calculated since the reference ZMP trajectory 
    //the bound for the x-direction (-0.13, 0.17);
    //the bound for the y-direction (-0.08, 0.08);
    //You can utilize 'for' statement or Eigen::VectorXd::Ones(N), which will return the vector variable with size N and all the element are 1.
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    lb_x = zmp_x_min - P_zs_mpc * x_hat;
    ub_x = zmp_x_max - P_zs_mpc * x_hat;
    lb_y = zmp_y_min - P_zs_mpc * y_hat;
    ub_y = zmp_y_max - P_zs_mpc * y_hat;
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    //variables for the calculation of QP library.
    Eigen::VectorXd U_x_temp;

    //Settings for the QP with QPoases variables.
    //X direction
    //Set the equality criteria for the termination condition of QP.
    QP_mpc_x_.EnableEqualityCondition(1e-8);
    //Set the Hessian and Gradient Matrix for the QP.
    QP_mpc_x_.UpdateMinProblem(Hess_, grad_x);
    //Reset the Constraint for the QP.
    QP_mpc_x_.DeleteSubjectToAx();
    //Add the Constraint for the QP.
    QP_mpc_x_.UpdateSubjectToAx(P_zu_mpc, lb_x, ub_x);

    //Solve the QP.
    if(QP_mpc_x_.SolveQPoases(500, U_x_temp))
    {
        U_x = U_x_temp.segment(0, N);

        //Update the X direction state for MPC.
        x_hat = A_mpc * x_hat + B_mpc * U_x(0);

        //Calculate the ZMP for the state of MPC.
        zmp_calc_mpc.segment(0, 1) = C_mpc*x_hat;
    }
    else
    {
        //Alert for the failure of solving QP.
        std::cout << "COM MPC X-AXIS SolveQPoases ERROR: Unable to find a valid solution." << std::endl;
    }

    //variables for the calculation of QP library.
    Eigen::VectorXd U_y_temp;

    //Settings for the QP with QPoases variables.
    //X direction
    //Set the equality criteria for the termination condition of QP.
    QP_mpc_y_.EnableEqualityCondition(1e-8);
    //Set the Hessian and Gradient Matrix for the QP.
    QP_mpc_y_.UpdateMinProblem(Hess_, grad_y);
    //Reset the Constraint for the QP.
    QP_mpc_y_.DeleteSubjectToAx();
    //Add the Constraint for the QP.
    QP_mpc_y_.UpdateSubjectToAx(P_zu_mpc, lb_y, ub_y);

    //Solve the QP.
    if(QP_mpc_y_.SolveQPoases(500, U_y_temp))
    {
        U_y = U_y_temp.segment(0, N);

        //Update the X direction state for MPC.
        y_hat = A_mpc * y_hat + B_mpc * U_y(0);
        
        //Calculate the ZMP for the state of MPC.
        zmp_calc_mpc.segment(1, 1) = C_mpc*y_hat;
    }
    else
    {
        //Alert for the failure of solving QP.
        std::cout << "COM MPC Y-AXIS SolveQPoases ERROR: Unable to find a valid solution." << std::endl;
    }

    //variable to check the time taken for the Model Predictive Control.
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

    //Calculate the time taken to conduct the Model Predictive Control.
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1);

    //Save the data.
    dataMPC_zmp_com << zx_ref(0)       << "," << zy_ref(0)       << ","
                    << zmp_calc_mpc(0) << "," << zmp_calc_mpc(1) << ","
                    << x_hat(0)        << "," << y_hat(0)        << ","
                    << x_hat(1)        << "," << y_hat(1)        << ","
                    << x_hat(1)        << "," << y_hat(1)        << ","
                    << endl;

    //Save the data to check the time taken.
    //Mind that it should be less than the 0.025s = 1/50.
    dataMPC_time    << duration.count() << std::endl;

    //You can check the predicted variables utilizing the matrix above.
    //Predicted_X_ZMP = P_zs_mpc*x_hat + P_zu_mpc*U_x;
    //Predicted_X_CoM = P_ps_mpc*x_hat + P_pu_mpc*U_x;
    //Predicted_Y_ZMP = P_zs_mpc*y_hat + P_zu_mpc*U_y;
    //Predicted_Y_CoM = P_ps_mpc*y_hat + P_pu_mpc*U_y;

    //Similarly, you can check the predicted CoM velocity and accleration by defining the matrix P_vs_mpc, P_vu_mpc, P_as_mpc, P_au_mpc.
    //Predicted_X_CoM_Vel = P_vs_mpc*x_hat + P_vu_mpc*U_x;
    //Predicted_X_CoM_Acc = P_as_mpc*x_hat + P_au_mpc*U_x;
    //Predicted_Y_CoM_Vel = P_vs_mpc*y_hat + P_vu_mpc*U_y;
    //Predicted_Y_CoM_Acc = P_as_mpc*y_hat + P_au_mpc*U_y;
}
