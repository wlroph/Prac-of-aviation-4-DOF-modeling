#include <stdio.h>
#include <math.h>
#define N 1201      // 0~120초 (dt=0.1초)

/*--------------------------------------------------
  모터 RPM -> 추력, 토크 계산
--------------------------------------------------*/
void calc_Thr_Tor(double RPM1, double RPM2,
                  double RPM3, double RPM4,
                  double K_F, double K_tor,
                  double *F1, double *F2,
                  double *F3, double *F4,
                  double *tor1, double *tor2,
                  double *tor3, double *tor4)
{
    // 추력 계산
    *F1 = K_F * RPM1;
    *F2 = K_F * RPM2;
    *F3 = K_F * RPM3;
    *F4 = K_F * RPM4;

    // 토크 계산
    *tor1 = K_tor * RPM1;
    *tor2 = K_tor * RPM2;
    *tor3 = K_tor * RPM3;
    *tor4 = K_tor * RPM4;
}

/*--------------------------------------------------
  병진운동 계산
  u,v,w : x,y,z 방향 속도
--------------------------------------------------*/
void calc_uvw_dot(double *u_dot,
                  double *v_dot,
                  double *w_dot,
                  double m, double g,
                  double F1, double F2,
                  double F3, double F4,
                  double u, double v, double w,
                  double p, double q, double r,
                  double phi, double theta)
{
    double Fx, Fy, Fz;

    // 중력에 의한 힘
    Fx = -m * g * sin(theta);
    Fy = m * g * sin(phi) * cos(theta);

    // 총 추력 + 중력
    Fz = -(F1 + F2 + F3 + F4)
         + m * g * cos(phi) * cos(theta);

    // 운동방정식
    *u_dot = Fx / m - q * w + r * v;
    *v_dot = Fy / m - r * u + p * w;
    *w_dot = Fz / m - p * v + q * u;
}

/*--------------------------------------------------
  회전운동 계산
  p,q,r : Roll, Pitch, Yaw 각속도
--------------------------------------------------*/
void calc_pqr_dot(double *p_dot,
                  double *q_dot,
                  double *r_dot,
                  double Ixx,
                  double Iyy,
                  double Izz,
                  double l,
                  double F1, double F2, double F3, double F4,
                  double tor1, double tor2, double tor3, double tor4,
                  double p, double q, double r)
{
    double L, M, Nm;

    // Roll 모멘트
    L = l * (F1 + F3 - F2 - F4);

    // Pitch 모멘트
    M = l * (F1 + F2 - F3 - F4);

    // Yaw 모멘트
    Nm = tor1 - tor2 - tor3 + tor4;

    // 회전 운동방정식
    *p_dot = ((Izz - Iyy) * q * r + L) / Ixx;
    *q_dot = ((Izz - Ixx) * p * r + M) / Iyy;
    *r_dot = ((Ixx - Iyy) * p * q + Nm) / Izz;
}

/*--------------------------------------------------
  자세각 및 위치 계산
--------------------------------------------------*/
void calc_att_pos_dot(double *phi_dot,
                      double *theta_dot,
                      double *psi_dot,
                      double *P_N_dot,
                      double *P_E_dot,
                      double *P_D_dot,
                      double u, double v, double w,
                      double p, double q, double r,
                      double phi,
                      double theta,
                      double psi)
{
    // 오일러 각 변화율
    *phi_dot =
        p + tan(theta) *
        (q * sin(phi) + r * cos(phi));

    *theta_dot =
        q * cos(phi) - r * sin(phi);

    *psi_dot =
        (q * sin(phi) + r * cos(phi))
        / cos(theta);

    // 위치 변화율
    *P_N_dot =
        cos(theta) * cos(psi) * u +
        (sin(phi) * sin(theta) * cos(psi)
         - cos(phi) * sin(psi)) * v +
        (cos(phi) * sin(theta) * cos(psi)
         + sin(phi) * sin(psi)) * w;

    *P_E_dot =
        cos(theta) * sin(psi) * u +
        (sin(phi) * sin(theta) * sin(psi)
         + cos(phi) * cos(psi)) * v +
        (cos(phi) * sin(theta) * sin(psi)
         - sin(phi) * cos(psi)) * w;

    *P_D_dot =
        -sin(theta) * u +
        sin(phi) * cos(theta) * v +
        cos(phi) * cos(theta) * w;
}

/*==================================================
                    MAIN
==================================================*/
int main()
{
    /*---------- 시뮬레이션 설정 ----------*/

    double dt = 0.1;      // 시간 간격
    double g = 9.81;      // 중력가속도
    double m = 5.0;       // 드론 질량

    double l = 0.5;       // 모터 암 길이

    double Ixx = 0.15;
    double Iyy = 0.15;
    double Izz = 0.30;

    double K_F = 0.04;    // 추력 계수
    double K_tor = 0.005; // 토크 계수

    /*---------- 상태 변수 ----------*/

    static double u[N] = {0};
    static double v[N] = {0};
    static double w[N] = {0};

    static double p[N] = {0};
    static double q[N] = {0};
    static double r[N] = {0};

    static double phi[N] = {0};
    static double theta[N] = {0};
    static double psi[N] = {0};

    static double P_N[N] = {0};
    static double P_E[N] = {0};
    static double P_D[N] = {0};

    static double t[N] = {0};

    /*---------- PID 제어기 ----------*/

    double D_cmd = -10.0;     // 목표 고도

    double Kp_D = 2.0;
    double Ki_D = 0.2;
    double Kd_D = 1.0;

    double err_D = 0.0;
    double err_D_prev = 0.0;
    double err_D_dot = 0.0;
    double err_D_int = 0.0;

    int i;

    for (i = 0; i < N - 1; i++)
    {
        double RPM_hover;
        double RPM1, RPM2, RPM3, RPM4;

        double F1, F2, F3, F4;
        double tor1, tor2, tor3, tor4;

        double u_dot, v_dot, w_dot;
        double p_dot, q_dot, r_dot;

        double phi_dot, theta_dot, psi_dot;
        double P_N_dot, P_E_dot, P_D_dot;

        double u_pid;

        // 현재 시간
        t[i] = i * dt;

        /*=================================
               PID 제어 계산
        =================================*/

        // 현재 고도 오차
        err_D = D_cmd - P_D[i];

        // 미분항
        err_D_dot = (err_D - err_D_prev) / dt;

        // 적분항
        err_D_int = err_D_int + err_D * dt;

        // PID 출력
        u_pid =
            Kp_D * err_D +
            Ki_D * err_D_int +
            Kd_D * err_D_dot;

        /*=================================
              RPM 생성
        =================================*/

        // 호버링 RPM
        RPM_hover = (m * g / 4.0) / K_F;

        // PID 출력 반영
        RPM1 = RPM_hover + u_pid;
        RPM2 = RPM_hover + u_pid;
        RPM3 = RPM_hover + u_pid;
        RPM4 = RPM_hover + u_pid;

        /*=================================
            추력 및 토크 계산
        =================================*/
        calc_Thr_Tor(
            RPM1, RPM2, RPM3, RPM4,
            K_F, K_tor,
            &F1, &F2, &F3, &F4,
            &tor1, &tor2, &tor3, &tor4);

        /*=================================
             병진 운동 계산
        =================================*/
        calc_uvw_dot(
            &u_dot, &v_dot, &w_dot,
            m, g,
            F1, F2, F3, F4,
            u[i], v[i], w[i],
            p[i], q[i], r[i],
            phi[i], theta[i]);

        /*=================================
             회전 운동 계산
        =================================*/
        calc_pqr_dot(
            &p_dot, &q_dot, &r_dot,
            Ixx, Iyy, Izz,
            l,
            F1, F2, F3, F4,
            tor1, tor2, tor3, tor4,
            p[i], q[i], r[i]);

        /*=================================
           자세 및 위치 계산
        =================================*/
        calc_att_pos_dot(
            &phi_dot, &theta_dot, &psi_dot,
            &P_N_dot, &P_E_dot, &P_D_dot,
            u[i], v[i], w[i],
            p[i], q[i], r[i],
            phi[i], theta[i], psi[i]);

        /*=================================
              Euler 적분
        =================================*/

        u[i + 1] = u[i] + u_dot * dt;
        v[i + 1] = v[i] + v_dot * dt;
        w[i + 1] = w[i] + w_dot * dt;

        p[i + 1] = p[i] + p_dot * dt;
        q[i + 1] = q[i] + q_dot * dt;
        r[i + 1] = r[i] + r_dot * dt;

        phi[i + 1] = phi[i] + phi_dot * dt;
        theta[i + 1] = theta[i] + theta_dot * dt;
        psi[i + 1] = psi[i] + psi_dot * dt;

        P_N[i + 1] = P_N[i] + P_N_dot * dt;
        P_E[i + 1] = P_E[i] + P_E_dot * dt;
        P_D[i + 1] = P_D[i] + P_D_dot * dt;

        // 이전 오차 저장
        err_D_prev = err_D;

        /*=================================
                결과 출력
        =================================*/
        if (i % 10 == 0)
        {
            printf("Time = %.1f sec\n", t[i]);

            // NED 좌표계라 -를 붙여 실제 고도 표시
            printf("Altitude = %.3f m\n", -P_D[i]);

            printf("Position = %.3f %.3f %.3f\n",
                   P_N[i],
                   P_E[i],
                   P_D[i]);

            printf("Attitude = %.3f %.3f %.3f\n",
                   phi[i],
                   theta[i],
                   psi[i]);

            printf("---------------------\n");
        }
    }

    FILE *fp;

    fp = fopen("altitude.csv", "w");

    if (fp == NULL)
    {
        printf("File open error!\n");
        return 1;
    }

    // CSV 헤더
    fprintf(fp, "Time,Altitude\n");

    // 시간과 고도 저장
    for (i = 0; i < N; i++)
    {
        fprintf(fp,
                "%lf,%lf\n",
                i * dt,
                -P_D[i]);
    }

    fclose(fp);

    printf("\naltitude.csv file saved successfully.\n");

    return 0;
}
