#ifndef __BEAM_FUNC__
#define __BEAM_FUNC__

inline static double beamFunc(double *x, double *p) {
    double amplitude  = p[0];
    double mean_x     = p[1];
    double sigma_x    = p[2];
    double mean_y     = p[3];
    double sigma_y    = p[4];
    double rho        = p[5];
    double u = (x[0]-mean_x) / sigma_x ;
    double v = (x[1]-mean_y) / sigma_y ;
    double c = 1 - rho*rho ;
    double result = amplitude*exp (-(u * u - 2 * rho * u * v + v * v ) / (2 * c));

    //(1 / (2 * TMath::Pi() * sigma_x * sigma_y * sqrt(c)))

    return result;
}
#endif
