namespace cv
{

KalmanFilter::KalmanFilter() {}
KalmanFilter::KalmanFilter(int dynamParams, int measureParams, int controlParams, int type)
/*Parameters:	

    dynamParams – Dimensionality of the state.
    measureParams – Dimensionality of the measurement.
    controlParams – Dimensionality of the control vector.
    type – Type of the created matrices that should be CV_32F or CV_64F.
*/
{
    init(dynamParams, measureParams, controlParams, type);
}

void KalmanFilter::init(int DP, int MP, int CP, int type)
{
    CV_Assert( DP > 0 && MP > 0 ); // comprueba qeu se cumpla esa condicion
    CV_Assert( type == CV_32F || type == CV_64F );
    CP = std::max(CP, 0);

    statePre = Mat::zeros(DP, 1, type);
    statePost = Mat::zeros(DP, 1, type);
    transitionMatrix = Mat::eye(DP, DP, type);

    processNoiseCov = Mat::eye(DP, DP, type);
    measurementMatrix = Mat::zeros(MP, DP, type);
    measurementNoiseCov = Mat::eye(MP, MP, type);

    errorCovPre = Mat::zeros(DP, DP, type);
    errorCovPost = Mat::zeros(DP, DP, type);
    gain = Mat::zeros(DP, MP, type);// K DE KALMAN

    if( CP > 0 )
        controlMatrix = Mat::zeros(DP, CP, type);
    else
        controlMatrix.release();

    temp1.create(DP, DP, type);
    temp2.create(MP, DP, type);
    temp3.create(MP, MP, type);
    temp4.create(MP, DP, type);
    temp5.create(MP, 1, type);
}

const Mat& KalmanFilter::predict(const Mat& control)
{
    // update the state: x'(k) = A*x(k)
    statePre = transitionMatrix*statePost;

    if( !control.empty() )
        // x'(k) = x'(k) + B*u(k)
        statePre += controlMatrix*control;

    // update error covariance matrices: temp1 = A*P(k)
    temp1 = transitionMatrix*errorCovPost;

    // P'(k) = temp1*At + Q
    gemm(temp1, transitionMatrix, 1, processNoiseCov, 1, errorCovPre, GEMM_2_T);

    // handle the case when there will be measurement before the next predict.
    statePre.copyTo(statePost);
    errorCovPre.copyTo(errorCovPost);

    return statePre;
}
