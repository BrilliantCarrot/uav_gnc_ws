#include "uav_gnc/ekf.h"

// Error-State EKF (ES-EKF)의 핵심: 모든 상태를 직접 수정하기보다, 오차 dx를 먼저 계산하고 그 오차를 명목 상태(Nominal State)에 반영하는 방식
// 이 방식은 특히 자세(Attitude)를 다룰 때 수치적으로 매우 안정적이라서 실제 드론이나 자율주행 기기에서 많이 쓰임
// 직관적으로는, "내가 지금까지 추정한 위치/속도/자세에서 실제로 얼마나 틀렸는지"를 오차 상태로 계산하고,
// 그 오차를 명목 상태에 반영하여 최종적으로 실제 위치/속도/자세를 업데이트하는 구조
// 자율 비행을 위해선 내가 지금 어디, 어떤 자세로 있는지를 정확히 알아야하며 보통 IMU, GPS를 이용하나 오차 문제 존제
// 이런 센서 데이터의 불확실성을 모델링하고, 실제 측정값과 모델 예측값의 차이를 이용해 오차를 계산하여 상태를 보정하는 것이 EKF의 핵심

EKF::EKF() {
    // 상태 벡터 15차원 초기화
    x_ = VectorXd::Zero(15);
    P_ = MatrixXd::Identity(15, 15);
    q_ = Eigen::Quaterniond(1, 0, 0, 0); // Identity quaternion

    // 초기 오차 공분산 행렬(P), 초기 공분산 설정 (초기값에 대한 불확실성, 자신감)
    // 내 초기값이 얼마나 정확한지 스스로 얼마나 믿는가
    // 값이 클수록 초기 추정이 틀릴 수 있다고 가정하는 것 (센서 데이터를 더 많이 반영하라)
    P_.block<3, 3>(0, 0) *= 1.0;   // Pos
    // Eigen 에서 제공하는 블록 연산(Block Operations)으로 행렬의 전체 데이터 중 특정 부분(서브 행렬)만 추출하거나 조작
    // <3,3>: 컴파일 타임에 결정된 블록의 크기, 3행3열 크기의 서브 행렬
    // (0,0): 블록이 시작되는 시작 좌표(인덱스), 0행0열부터 시작함
    P_.block<3, 3>(3, 3) *= 0.1;   // Vel
    P_.block<3, 3>(6, 6) *= 0.01;  // Att 자세는 0.1rad(약 5도) 정도로 정확하다고 가정
    P_.block<3, 3>(9, 9) *= 0.01;  // Acc Bias
    P_.block<3, 3>(12, 12) *= 0.001; // Gyro Bias 자이로는 상대적으로 더 정확함
    // Process Noise 행렬(모델의 한계, Q), 튜닝 파라미터
    // 시스템 모델이 시간이 지나며 얼마나 흔들리는지, 모델이 얼마나 불확실한지, 즉
    // 시간이 흐를 때 수학 모델이 실제 물리 현상과 얼마나 달라질 수 있는가 불확실성을 정의
    // 실제 센서를 다룰 때에는 대각 행렬 형태로 간단히 초기화한 후 튜닝하는 경우가 많음
    // Q가 크면 모델(예측값)을 믿지 않고 센서 측정값(z)에 더 의존, 결과가 빠르지만 노이즈가 심해짐
    // Q가 작으면 모델을 더 믿고 센서 측정값에 덜 의존, 결과가 부드럽지만 실제 움직임을 따라가는 반응 속도가 느림
    Q_ = MatrixXd::Identity(15, 15);
    Q_.block<3, 3>(0, 0) *= 0.0;     // Pos (직접 적분 안함)
    Q_.block<3, 3>(3, 3) *= 0.01;    // Vel noise (from accel noise), 가속도계 적분 시 발생하는 속도 불확실성
    Q_.block<3, 3>(6, 6) *= 0.001;   // Att noise (from gyro noise), 자이로 적분 시 발생하는 자세 불확실성
    Q_.block<3, 3>(9, 9) *= 1e-5;    // Accel Bias Random Walk,센서 바이어스 오차는 시간이 흐르며 조금씩 변하므로
    // 아주 작은 값을 더해줌으로써 EKF가 바이어스 추정치를 조금씩 변화시킬 수 있도록 허용 (너무 작으면 바이어스가 고정되어 버릴 수 있음)
    Q_.block<3, 3>(12, 12) *= 1e-6;  // Gyro Bias Random Walk
    // 튜닝 팁: 드론이 너무 격하게 움직여서 모델이 못 따라가면 Q를 높이고, 값이 너무 지저분하게 나도면 낮춤

    // Measurement Noise(측정 노이즈 행렬, R), 센서가 내주는 값을 얼마나 신뢰하는지 결정, 여기선 GPS 위치 측정값에 대한 노이즈 공분산 행렬
    // 크게 설정하면 EKF가 현재 사용중인 GPS가 상태가 안 좋음을 인지하게하끔
    R_gps_ = MatrixXd::Identity(3, 3) * 50.0; // 튜닝 전 GPS 오차 약 0.5m -> 공분산 행렬에 들어갈 값은 제곱인 0.25 
    // 튜닝으로 0.25->2.0->5.0->10.0
    // 외란 추가할 시 더 크게 10->30->50
    // P(나의 불확실성)에 비해 R(센서 불확실성)이 작으면: 센서가 정확하므로 센서 측정값(z)에 더 의존, 결과가 빠르지만 노이즈가 심해짐
    // P에 비해 R이 크면: 센서가 부정확하므로 모델 예측값에 더 의존, 결과가 부드럽지만 실제 움직임을 따라가는 반응 속도가 느림
}

EKF::~EKF() {}

void EKF::init(const Vector3d& p, const Vector3d& v, const Eigen::Quaterniond& q) {
    x_.setZero();
    x_.segment<3>(0) = p; // 상태 벡터 x_의 0~2 인덱스에 위치 p 저장
    x_.segment<3>(3) = v; // 상태 벡터 x_의 3~5 인덱스에 속도 v 저장
    // bias는 0으로 시작한다고 가정 (혹은 초기 캘리브레이션 값)
    q_ = q;
    q_.normalize(); // 중요, 계산 과정에서 발생할 수 있는 미세한 오차를 제거하여 항상 쿼터니언 크기를 1로 강제 조정
}

void EKF::predict(const Vector3d& acc_meas, const Vector3d& gyro_meas, double dt) {
    // GPS가 들어오기 전, 아주 짧은 dt동안 IMU 데이터를 믿고 다음 위치를 추측하는 시간 업데이트 단계
    // [DEBUG] dt가 튀는지, P 행렬이 터지는지 확인
    // std::cout을 쓰려면 맨 위에 #include <iostream> 확인
    if (dt > 1.0 || dt < 0.0) std::cout << "[Warning] Bad dt: " << dt << std::endl;
    // 1. 현재 상태 가져오기
    Vector3d pos = x_.segment<3>(0);
    Vector3d vel = x_.segment<3>(3);
    Vector3d ba  = x_.segment<3>(9);
    Vector3d bg  = x_.segment<3>(12);

    // 2. 입력 보정 (Measurement(meas) - Bias), 실제 센서값에는 일정 오차인 바이어스가 포함되너있음
    // 이를 빼주어야 순수한 물리량에 가까운 unbiased 값을 얻음
    Vector3d acc_unbiased = acc_meas - ba;
    Vector3d gyro_unbiased = gyro_meas - bg;

    // 3. Nominal State(명목 상태) 적분 (비선형 예측)
    // 비선형 동역학 수식을 사용하여 위치, 속도, 자세를 업데이트
    // 위치/속도 업데이트: 뉴턴 운동 법칙을 따름
    // Position Update
    pos += vel * dt + 0.5 * (q_ * acc_unbiased - Vector3d(0, 0, g_)) * dt * dt;
    // Velocity Update
    // Body 가속도를 World로 회전 후 중력 빼어 순수 가속도를 계산
    Vector3d acc_world = q_ * acc_unbiased - Vector3d(0, 0, g_);
    vel += acc_world * dt;

    // Attitude Update (Quaternion kinematics)
    // q_dot = 0.5 * q * omega
    // 작은 회전 가정 (소각 근사), 실제로는 dq.w() = sqrt(1 - |omega|^2)로 계산할 수도 있지만, 작은 dt에서는 1에 매우 가까움
    // 기존 자세에 회전량을 곱해 새로운 자세를 구한 뒤, 수치적 오차를 잡기 위해 정규화
    Eigen::Quaterniond dq;
    Vector3d omega = gyro_unbiased * dt * 0.5;
    dq.w() = 1.0; 
    dq.vec() = omega; // 소각 근사
    q_ = q_ * dq;
    q_.normalize();

    // 4. Jacobian F 행렬 계산 (15x15, EKF의 핵심인 선형화 단계)
    // 비선형적인 드론의 움직임을 선형 행렬 F로 근사화하여 오차(공분산(P))의 전파를 계산
    // F 행렬의 각 블록은 현재 상태가 변할 때 다음 상태가 얼마나 변하는지에 대한 미분 값(자코비안)
    // ex) 지금 속도가 불확실 -> F 행렬의 (0,3) 블록(dt) 때문에 다음 스탬에서는 위치도 불확실해짐을 EKF가 스스로 판단하도록 함
    // 사용중인 상태 벡터는 15차원(위치 3, 속도 3, 자세 오차 3, 가속도 바이어스 3, 자이로 바이어스 3)
    // Error State Kinematics의 선형화 행렬
    MatrixXd F = MatrixXd::Identity(15, 15);
    // 기본적인 15x15 단위 행렬로 시작, 아무 외부 입력이 없다면 다음 상태는 현재 상태와 같다는 관성 의미
    Matrix3d R = q_.toRotationMatrix();

    // 속도가 위치에 주는 영향, dp/dv = I * dt
    // 현재 속도가 빠르면, 다음 위치가 변한다
    F.block<3, 3>(0, 3) = Matrix3d::Identity() * dt; 
    // 0행3열 위치에 dt를 넣음으로써 속도의 변화가 위치에 dt만큼 영향을 준다는 것을 표현

    // 자세가 속도에 주는 영향, dv/dtheta (skew symmetric of acc, 속도와 자세의 관계, 매우 중요)
    // 기체가 기울어지면, 가속도 방향이 바뀌어 속도가 변한다
    // acc_skew는 외적을 행렬로 만든 것. 기체가 회전할 때 가속도 벡터가 어느 방향으로 휘어지는지를 계산하기 위해 사용
    // -인 이유: 오차 상태(Error State) 모델에서 자세 오차가 발생했을 때 가속도가 보정되는 수학적 방향
    // F_v_q = -R * [a_body]x * dt
    Matrix3d acc_skew;
    acc_skew << 0, -acc_unbiased.z(), acc_unbiased.y(),
                acc_unbiased.z(), 0, -acc_unbiased.x(),
                -acc_unbiased.y(), acc_unbiased.x(), 0;
    F.block<3, 3>(3, 6) = -R * acc_skew * dt; // 현재 자세(R)에서 가속도 오차가 발생했을 때 속도가 어떻게 변하는가

    // 가속도 바이어스가 속도에 주는 영향, dv/dba = -R * dt
    // 센서가 매 순간 일정한 오차(바이어스)를 가지고 있으면, 속도 계산이 틀려진다
    // 현재 기체의 자세(R)를 타고 월드 좌표계 속도에 영향을 주기 때문에 R이 곱해져 있음
    F.block<3, 3>(3, 9) = -R * dt;

    // 자세가 자세에 주는 영향, dq/dq (attitude error dynamics)
    // 지금 회전하고 있다면, 다음 순간의 자세는 현재 회전 속도에 따라 변한다
    // 쿼터니언 미분 방정식을 선형화한 결과
    // gyro_skew는 현재 각속도(w)를 이용해 회전하고 있는 상태에서 자세 오차가 어떻게 전이되는가를 나타냄
    // F_q_q = I - [omega]x * dt
    Matrix3d gyro_skew;
    gyro_skew << 0, -gyro_unbiased.z(), gyro_unbiased.y(),
                 gyro_unbiased.z(), 0, -gyro_unbiased.x(),
                 -gyro_unbiased.y(), gyro_unbiased.x(), 0;
    F.block<3, 3>(6, 6) = Matrix3d::Identity() - gyro_skew * dt; // 단순화된 형태

    // 자세와 자이로 바이어스의 관계, dq/dbg = -I * dt
    // 자이로 센서에 바이어스가 있으면, 기체가 가만히 있어도 자세가 계속 흐른다(Drift)
    // 자이로 바이어스는 각속도 측정값을 왜곡시키고, 이는 곧 자세 오차로 직결
    // 시간이 지날수록 바이어스만큼 오차가 누적되는 관계를 나타냄
    // 자이로 센서의 바이어스가있다면 자세(q)가 계속 틀어짐, 자이로 오차가 누적되어 자세 오차를 만든다는 관계
    F.block<3, 3>(6, 12) = -Matrix3d::Identity() * dt;

    // 5. 공분산 예측 수식(Time Update): P = F*P*F' + Q
    P_ = F * P_ * F.transpose() + Q_;

    // 6. 상태 벡터 업데이트 (Nominal State에 반영했으므로 x_ 자체는 다시 0 근처로 유지하거나 값 저장)
    // 여기서는 x_에 nominal state 값을 저장해두는 방식 사용
    x_.segment<3>(0) = pos;
    x_.segment<3>(3) = vel;
    // Attitude Error는 항상 0으로 리셋 (Error-State 방식의 특징)
    // 하지만 P 행렬에는 오차가 누적됨.

    // nan 디버그용
    if (std::isnan(x_(0))) {
        std::cout << "[EKF FAIL] State is NaN at dt=" << dt << std::endl;
        std::cout << "P diagonal: " << P_.diagonal().transpose() << std::endl; }
}

// 보정 단계, 예측 단계에서 이정도만큼 움직였을것이다라고 추측한 값과, GPS가 실제로 알려준 위치 값 사이 간극을 줄여나가는 단계
void EKF::update_gps(const Vector3d& meas_pos) {
    // 1. 측정 모델 행렬 H (GPS는 위치만 측정하므로 15x15 중 앞 3x3만 Identity) 정의
    // 사용중인 전체 상태 벡터 x는 15차원, GPS는 위치 3차원 정보만 줌, H 행렬은 15차원 상태에서 위치에 해당하는 앞부분 3개만 뽑아내는 필터 역할
    // z = Hx + v
    MatrixXd H = MatrixXd::Zero(3, 15);
    H.block<3, 3>(0, 0) = Matrix3d::Identity(); // 앞 3x3 블록만 Identity를 넣어 위치 값만 보겠다 선언

    // 2. 칼만 이득 계산, K = P * H' * (H * P * H' + R)^-1
    // 칼만 이득: 내가 예측한 값(P)를 더 믿을지, 아니면 GPS 측정값(R)을 더 믿을지 결정
    // 역할: GPS 상태가 좋으면(오차 R이 작으면) K 값이 커져서 GPS 정보를 더 많이 반영, GPS 상태가 나쁘면(R이 크면) K 값이 작아져서 기존 예측값을 더 유지
    // 여기서 S는 혁신 공분산(Innovation Covariance), 예측 오차와 측정 오차를 합친 전체 불확실성
    MatrixXd PHt = P_ * H.transpose();
    MatrixXd S = H * PHt + R_gps_; // Innovation Covariance
    MatrixXd K = PHt * S.inverse();

    // 3. 잔차(Residual/Innovation) 계산, y = z - Hx
    // 이론과 실제의 차이를 계산, GPS가 알려준 위치(meas_pos)와 내가 예측한 위치(pred_pos) 사이의 차이
    Vector3d pred_pos = x_.segment<3>(0); // x_.segment<3>(0)로 현재 상태 백터에서 위치 성분 3개 가져옴
    Vector3d y = meas_pos - pred_pos;

    // 4. 상태 업데이트, x = x + K * y
    // 계산된 오차(y)에 가중치(K)를 곱해서 실제 상태 값에 주입하는 과정
    VectorXd dx = K * y;
    // 명목 상태 업데이트 (Injection)
    x_.segment<3>(0) += dx.segment<3>(0); // Pos
    x_.segment<3>(3) += dx.segment<3>(3); // Vel
    x_.segment<3>(9) += dx.segment<3>(9); // Acc Bias
    x_.segment<3>(12) += dx.segment<3>(12); // Gyro Bias
    // Attitude Update(특이사항) (Error Quaternion Injection)
    // 자세 오차는 상태 벡터에 직접 포함되어 있지 않고, 별도의 쿼터니언(q_)으로 관리되고 있기 때문에, 자세 업데이트는 별도로 처리
    // 따라서 오차 각도(dθ)를 계산한 뒤, 이를 아주 작은 회전량인 dq로 변환해서 기존 쿼터니언 q_에 곱해주는 방식을 씀, 이를 통해 자세를 부드럽게 보정
    // dq = [1, 0.5*dtheta]
    Vector3d dtheta = dx.segment<3>(6); // 오차 추출, 칼만 게인과 잔차(y)를 곱해 계산된 보정값(dx)에서 자세 오차 부분(6~8 인덱스)을 dtheta로 사용
    Eigen::Quaterniond dq;
    // 오차 쿼터니언 생성, 소각 근사(Small Angle Approximation)을 이용하여 3차원 오차 벡터를 4차원 쿼터니언으로 변환
    dq.w() = 1.0;
    dq.vec() = 0.5 * dtheta; 
    q_ = q_ * dq; // 오차 주입, 현재의 명목 상태 자세(q)에 계산된 오차(dq)를 곱해서(Hamilton Product) 실제 자세로 보정
    // 오차는 3차원으로 계산하고, 실제 자세는 쿼터니언으로 유지한다는 것의 코드 부분
    q_.normalize();

    // 5. Covariance Update: P = (I - K*H) * P
    // 데이터를 확인했으니, 내 예측의 불확실성이 줄어들었다라고 기록
    // 한 번 보정을 거치고 나면 드론의 현재 위치에 대한 확신이 커지기 때문에 공분산 행렬 P의 값들을 작게 만들어줌
    // 다음 스탭에서는 더 정확한 예측을 시작할 수 있음
    MatrixXd I = MatrixXd::Identity(15, 15);
    P_ = (I - K * H) * P_;
}