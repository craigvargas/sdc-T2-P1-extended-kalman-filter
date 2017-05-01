#include "kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
  
//  cout<<endl<<"Prediction"<<endl<<"==================="<<endl;
//  cout<<"Prediction P:"<<endl<<P_<<endl<<endl;
//  cout<<"Prediction x:"<<endl<<x_<<endl<<endl;
//  cout<<"Prediction F:"<<endl<<F_<<endl<<endl;
//  cout<<"Prediction Q:"<<endl<<Q_<<endl<<endl;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;
  
  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  //Radar
  
  //Convert prediction into polar form
  VectorXd z_pred = VectorXd(3);
  z_pred(0) = sqrt(x_(0)*x_(0) + x_(1)*x_(1));
  z_pred(1) = atan2(x_(1), x_(0));
  z_pred(2) = (x_(0)*x_(2) + x_(1)*x_(3)) / z_pred(0);
//  cout<<endl<<"***z_pred:***"<<endl<<z_pred<<endl;
  
  //Calculate differece between reading and prediction
  VectorXd y = z - z_pred;
  
  //Normalize bearing
  if(y(1) < -M_PI){
    y(1) = y(1) + 2*M_PI;
  }
  if(y(1) > M_PI){
    y(1) = y(1) - 2*M_PI;
  }
//  cout<<endl<<"***y:***"<<endl<<y<<endl;
  
  
  MatrixXd Ht = H_.transpose();
//  cout<<endl<<"***Ht:***"<<endl<<Ht<<endl;
  MatrixXd S = H_ * P_ * Ht + R_;
//  cout<<endl<<"***S:***"<<endl<<S<<endl;
  MatrixXd Si = S.inverse();
//  cout<<endl<<"***Si:***"<<endl<<Si<<endl;
  MatrixXd PHt = P_ * Ht;
//  cout<<endl<<"***PHt:***"<<endl<<PHt<<endl;
  MatrixXd K = PHt * Si;
//  cout<<endl<<"***K:***"<<endl<<K<<endl;
  
  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}
