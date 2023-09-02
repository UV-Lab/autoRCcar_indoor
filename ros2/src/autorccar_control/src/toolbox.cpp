#include "toolbox.h"

Eigen::Matrix<double, 3, 3> Quat2DCM(const Eigen::Quaterniond &quat)
{
    Eigen::Matrix<double, 3, 3> dcm = Eigen::Matrix3d::Identity();

    double a = quat.w();
    double b = quat.x();
    double c = quat.y();
    double d = quat.z();

    double dcm11 = a*a + b*b - c*c - d*d;
	double dcm12 = 2*(a*d + b*c);
	double dcm13 = 2*(b*d - a*c);
	double dcm21 = 2*(b*c - a*d);
	double dcm22 = a*a - b*b + c*c - d*d;
	double dcm23 = 2*(a*b + c*d);
	double dcm31 = 2*(a*c + b*d);
	double dcm32 = 2*(c*d - a*b);
	double dcm33 = a*a - b*b - c*c + d*d;

    dcm << dcm11, dcm12, dcm13,
           dcm21, dcm22, dcm23,
           dcm31, dcm32, dcm33;

    return dcm;
}

Eigen::Vector3d Quat2Euler(const Eigen::Quaterniond &quat)
{
    Eigen::Vector3d eulr = Eigen::Vector3d::Zero();

    double a = quat.w();
    double b = quat.x();
    double c = quat.y();
    double d = quat.z();

    double A = 2*(a*b + c*d);
    double B = a*a - b*b - c*c + d*d;
    double C = 2*(b*d - a*c);
    double D = 2*(a*d + b*c);
    double E = a*a + b*b - c*c - d*d;

    double phi = atan2(A,B);
    double the = asin(-C);
    double psi = atan2(D,E);

    eulr << phi, the, psi;

    return eulr;
}

Eigen::Quaterniond Euler2Quat(const Eigen::Vector3d &eulr)
{
    Eigen::Quaterniond quat;

    double phi = eulr(0); // [rad]
    double the = eulr(1);
    double psi = eulr(2);

    double a = cos(phi/2)*cos(the/2)*cos(psi/2) + sin(phi/2)*sin(the/2)*sin(psi/2);
	double b = sin(phi/2)*cos(the/2)*cos(psi/2) - cos(phi/2)*sin(the/2)*sin(psi/2);
	double c = cos(phi/2)*sin(the/2)*cos(psi/2) + sin(phi/2)*cos(the/2)*sin(psi/2);
	double d = cos(phi/2)*cos(the/2)*sin(psi/2) - sin(phi/2)*sin(the/2)*cos(psi/2);

    quat.w() = a;
    quat.vec() = Eigen::Vector3d(b, c, d);
    quat = quat.normalized();

    return quat;
}

Eigen::Quaterniond QuatProduct(const Eigen::Quaterniond &quat1, const Eigen::Quaterniond &quat2)
{
    Eigen::Quaterniond quat = Eigen::Quaterniond::Identity();

	double a = quat1.w()*quat2.w() - quat1.x()*quat2.x() - quat1.y()*quat2.y() - quat1.z()*quat2.z();
	double b = quat1.x()*quat2.w() + quat1.w()*quat2.x() - quat1.z()*quat2.y() + quat1.y()*quat2.z();
	double c = quat1.y()*quat2.w() + quat1.z()*quat2.x() + quat1.w()*quat2.y() - quat1.x()*quat2.z();
	double d = quat1.z()*quat2.w() - quat1.y()*quat2.x() + quat1.x()*quat2.y() + quat1.w()*quat2.z();

    quat.w() = a;
    quat.x() = b;
    quat.y() = c;
    quat.z() = d;
    quat = quat.normalized();

    return quat;
}

Eigen::Vector3d ECEF2LLH(const Eigen::Vector3d & pos_ecef)
{
    // In: ECEF x,y,z coordinate in meters.
    // Out: Latitude and Longitude in radians, Height above ellipsoid in meters.

    Eigen::Vector3d LLH = Eigen::Vector3d::Zero();

    const double pi = 3.14159265358979;

	double x = pos_ecef.x();
	double y = pos_ecef.y();
	double z = pos_ecef.z();

	double x2 = pow(x,2);
	double y2 = pow(y,2);
	double z2 = pow(z,2);

	double a = 6378137.0000;
	double b = 6356752.3142;
	double e = sqrt(1-pow((b/a),2));
	double b2 = b*b;
	double e2 = pow(e,2);
	double ep = e*(a/b);
	double r = sqrt(x2+y2);
	double r2 = r*r;
	double E2 = pow(a,2) - pow(b,2);
	double F = 54*b2*z2;
	double G = r2 + (1-e2)*z2 - e2*E2;
	double c = (e2*e2*F*r2)/(G*G*G);
	double s = pow(( 1 + c + sqrt(c*c + 2*c) ),(1/3));
	double P = F / (3 * pow((s+1/s+1),2) * G*G);
	double Q = sqrt(1+2*e2*e2*P);
	double ro = -(P*e2*r)/(1+Q) + sqrt((a*a/2)*(1+1/Q) - (P*(1-e2)*z2)/(Q*(1+Q)) - P*r2/2);
	double tmp = pow((r - e2*ro),2);
	double U = sqrt( tmp + z2 );
	double V = sqrt( tmp + (1-e2)*z2 );
	double zo = (b2*z)/(a*V);

	double height = U*( 1 - b2/(a*V) );
	double lat = atan( (z + ep*ep*zo)/r );
	double temp = atan(y/x);
    double lon;
	if (x >=0){
        lon = temp;
    }
	else if ((x < 0) && (y >= 0)){
        lon = pi + temp;
    }
	else{
		lon = temp - pi;
    }

    LLH << lat, lon, height;

    // double Rea = 6378.137*1000;  // Semi-major axis of Earth [m]
    // double e = 0.0818191908426;  // eccentricity of the Earth

    // double R1 = pos_ecef.x();
    // double R2 = pos_ecef.y();
    // double R3 = pos_ecef.z();

    // double lon = atan2(R2, R1); // [rad]
    // double lat = 0;
    // double height = 0;

    // double lat1 = atan(R3/sqrt(R1*R1+R2*R2));
    // double lat2 = 0;
    // double h1 = 0;
    // double h2 = 0;
    // double Rn = 0;
    // double eps = 1e-4; // tolerance

    // while(true)
    // {
    //     Rn = Rea / sqrt(1-pow(e,2)*pow(sin(lat1),2));    // prime vertical radius of curvature
    //     h2 = sqrt(R1*R1+R2*R2)/cos(lat1) - Rn;
    //     lat2 = atan(R3 / sqrt(R1*R1+R2*R2)*(Rn+h2)/(Rn*(1-pow(e,2))+h2));

    //     if ((abs(lat2-lat1)<eps) && (abs(h2-h1)<eps))
    //     {
    //         lat = lat2;
    //         height = h2;
    //         break;
    //     }
    //     h1 = h2;
    //     lat1 = lat2;
    // }

    // LLH << lat, lon, height;

    return LLH;
}
