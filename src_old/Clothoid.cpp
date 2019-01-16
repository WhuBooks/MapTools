//
// Created by books on 2017/11/21.
//

#include "Clothoid.h"

namespace MapBase
{

    static const std::vector<double> fns = {0.49999988085884732562,
                                            1.3511177791210715095,
                                            1.3175407836168659241,
                                            1.1861149300293854992,
                                            0.7709627298888346769,
                                            0.4173874338787963957,
                                            0.19044202705272903923,
                                            0.06655998896627697537,
                                            0.022789258616785717418,
                                            0.0040116689358507943804,
                                            0.0012192036851249883877};

    static const std::vector<double> fds = {1.0,
                                            2.7022305772400260215,
                                            4.2059268151438492767,
                                            4.5221882840107715516,
                                            3.7240352281630359588,
                                            2.4589286254678152943,
                                            1.3125491629443702962,
                                            0.5997685720120932908,
                                            0.20907680750378849485,
                                            0.07159621634657901433,
                                            0.012602969513793714191,
                                            0.0038302423512931250065};

    static const std::vector<double> gns = {0.50000014392706344801,
                                            0.032346434925349128728,
                                            0.17619325157863254363,
                                            0.038606273170706486252,
                                            0.023693692309257725361,
                                            0.007092018516845033662,
                                            0.0012492123212412087428,
                                            0.00044023040894778468486,
                                            -8.80266827476172521e-6,
                                            -1.4033554916580018648e-8,
                                            2.3509221782155474353e-10};

    static const std::vector<double> gds = {1.0,
                                            2.0646987497019598937,
                                            2.9109311766948031235,
                                            2.6561936751333032911,
                                            2.0195563983177268073,
                                            1.1167891129189363902,
                                            0.57267874755973172715,
                                            0.19408481169593070798,
                                            0.07634808341431248904,
                                            0.011573247407207865977,
                                            0.0044099273693067311209,
                                            -0.00009070958410429993314};


    void FresnelCS(double &FresnelC, double &FresnelS, double y)
    {
        double t, twofn, fact, denterm, numterm, sum, ratio, term, sumn, sumd, f, g, U, SinU, CosU, oldterm, eps10, absterm;

        double x = abs(y);
        if (x < 1.0)
        {
            t = -((M_PI / 2) * x * x) * ((M_PI / 2) * x * x);
            //	% Cosine integral series
            twofn = 0.0;
            fact = 1.0;
            denterm = 1.0;
            numterm = 1.0;
            sum = 1.0;
            ratio = 10.0;

            while (ratio > eps1)
            {
                twofn = twofn + 2.0;
                fact = fact * twofn * (twofn - 1.0);
                denterm = denterm + 4.0;
                numterm = numterm * t;
                term = numterm / (fact * denterm);
                sum = sum + term;
                ratio = abs(term / sum);
            }
            FresnelC = x * sum;

            //% Sine integral series
            twofn = 1.0;
            fact = 1.0;
            denterm = 3.0;
            numterm = 1.0;
            sum = 1.0 / 3.0;
            ratio = 10.0;

            while (ratio > eps1)
            {
                twofn = twofn + 2.0;
                fact = fact * twofn * (twofn - 1.0);
                denterm = denterm + 4.0;
                numterm = numterm * t;
                term = numterm / (fact * denterm);
                sum = sum + term;
                ratio = abs(term / sum);
            }
            FresnelS = (M_PI / 2) * sum * x * x * x;
        }
        else if (x < 6.0)

            //	% Rational approximation for f
        {
            sumn = 0.0;
            sumd = fds[11];
            for (int k = 11; k >= 1; k--)
            {
                sumn = fns[k - 1] + x * sumn;
                sumd = fds[k - 1] + x * sumd;
            }

            f = sumn / sumd;
            //% Rational approximation for  g
            sumn = 0.0;
            sumd = gds[11];
            for (int k = 11; k >= 1; k--)
            {
                sumn = gns[k - 1] + x * sumn;
                sumd = gds[k - 1] + x * sumd;
            }

            g = sumn / sumd;
            U = (M_PI / 2) * x * x;
            SinU = sin(U);
            CosU = cos(U);
            FresnelC = 0.5 + f * SinU - g * CosU;
            FresnelS = 0.5 - f * CosU - g * SinU;
        }
        else
        {
            //	% x >= 6; asymptotic expansions for  f  and  g
            t = -1 / ((M_PI * x * x) * (M_PI * x * x));
            //	% Expansion for  f
            numterm = -1.0;
            term = 1.0;
            sum = 1.0;
            oldterm = 1.0;
            ratio = 10.0;
            eps10 = 0.1 * eps1;

            while (ratio > eps10)
            {
                numterm = numterm + 4.0;
                term = term * numterm * (numterm - 2.0) * t;
                sum = sum + term;
                absterm = abs(term);
                ratio = abs(term / sum);
                if (oldterm < absterm)
                {
                    //disp('\n\n !!In FresnelCS f not converged to eps');
                    ratio = eps10;
                }

                oldterm = absterm;
            }


            f = sum / (M_PI * x);
            //% Expansion for  g
            numterm = -1.0;
            term = 1.0;
            sum = 1.0;
            oldterm = 1.0;
            ratio = 10.0;
            eps10 = 0.1 * eps1;

            while (ratio > eps10)
            {
                numterm = numterm + 4.0;
                term = term * numterm * (numterm + 2.0) * t;
                sum = sum + term;
                absterm = abs(term);
                ratio = abs(term / sum);
                if (oldterm < absterm)
                {
                    //disp('\n\n!!In FresnelCS g not converged to eps');
                    ratio = eps10;
                }
                oldterm = absterm;

            }
            g = sum / ((M_PI * x) * (M_PI * x) * x);
            U = (M_PI / 2) * x * x;
            SinU = sin(U);
            CosU = cos(U);
            FresnelC = 0.5 + f * SinU - g * CosU;
            FresnelS = 0.5 - f * CosU - g * SinU;
        }


        if (y < 0)
        {
            FresnelC = -FresnelC;
            FresnelS = -FresnelS;
        }
    }

    void FresnelCSk(int nk, double t, std::vector<double> &FresnelC, std::vector<double> &FresnelS)
    {
        double C = 0;
        double S = 0;
        FresnelCS(C, S, t);

        FresnelC[0] = C;
        FresnelS[0] = S;
        if (nk > 1)
        {
            double tt = M_PI / 2 * t * t;
            double ss = sin(tt);
            double cc = cos(tt);
            FresnelC[1] = ss / M_PI;
            FresnelS[1] = (1 - cc) / M_PI;
            if (nk > 2)
            {
                FresnelC[2] = (t * ss - FresnelS[0]) / M_PI;
                FresnelS[2] = (FresnelC[0] - t * cc) / M_PI;
            }
        }
    }

    void Clothoid::evalXYaLarge(std::vector<double> &X, std::vector<double> &Y, int nk, double a, double b)
    {
        double s = 0;
        if (a > 0)s = 1;
        else if (a == 0)s = 0;
        else if (a < 0)s = -1;

        double z = sqrt(abs(a) / M_PI);
        double ell = s * b / sqrt(abs(a) * M_PI);
        double g = -0.5 * s * b * b / abs(a);

        std::vector<double> Cl(3, 0);
        std::vector<double> Sl(3, 0);


        std::vector<double> Cz(3, 0);
        std::vector<double> Sz(3, 0);

        FresnelCSk(nk, ell, Cl, Sl);
        FresnelCSk(nk, ell + z, Cz, Sz);

        std::vector<double> dC(5, 0);
        std::vector<double> dS(5, 0);

        dC[0] = Cz[0] - Cl[0];
        dS[0] = Sz[0] - Sl[0];
        double cg = cos(g) / z;
        double sg = sin(g) / z;
        X[0] = cg * dC[0] - s * sg * dS[0];
        Y[0] = sg * dC[0] + s * cg * dS[0];
        if (nk > 1)
        {
            dC[1] = Cz[1] - Cl[1];
            dC[2] = Cz[2] - Cl[2];
            dS[1] = Sz[1] - Sl[1];
            dS[2] = Sz[2] - Sl[2];
            cg = cg / z;
            sg = sg / z;
            double DC = dC[1] - ell * dC[0];
            double DS = dS[1] - ell * dS[0];
            X[1] = cg * DC - s * sg * DS;
            Y[1] = sg * DC + s * cg * DS;
            if (nk > 2)
            {
                cg = cg / z;
                sg = sg / z;
                DC = dC[2] + ell * (ell * dC[0] - 2 * dC[1]);
                DS = dS[2] + ell * (ell * dS[0] - 2 * dS[1]);
                X[2] = cg * DC - s * sg * DS;
                Y[2] = sg * DC + s * cg * DS;
            }
        }

    }

    void Clothoid::evalXYaSmall(std::vector<double> &X, std::vector<double> &Y, int nk, double a, double b, double p)
    {
        /// gauss-newton iteration
        std::function<double(double, double, double)> S = [](double mu, double nu, double b) -> double {
            double tmp = 1 / ((mu + nu + 1) * (mu - nu + 1));
            double res = tmp;
            for (int n = 1; n <= 100; n++)
            {
                tmp = tmp * (-b / (2 * n + mu - nu + 1)) * (b / (2 * n + mu + nu + 1));
                res = res + tmp;
                if (abs(tmp) < abs(res) * 1e-50)
                    break;

            }
            return res;
        };

        /// evalXYazero
        int num = nk + 4 * (int) p + 2;
        std::vector<double> X0(num, 0);
        std::vector<double> Y0(num, 0);
        double sb = sin(b);
        double cb = cos(b);
        double b2 = b * b;
        if (abs(b) < 1e-3)
        {
            X0[0] = 1 - b2 / 6 * (1 - b2 / 20 * (1 - b2 / 42));
            Y0[0] = (b / 2) * (1 - b2 / 12 * (1 - b2 / 30));
        }
        else
        {
            X0[0] = sb / b;
            Y0[0] = (1 - cb) / b;
        }
        double A = b * sb;
        double D = sb - b * cb;
        double B = b * D;
        double C = -b2 * sb;
        for (int k = 1; k < num; k++)
        {
            X0[k] = (k * A * S(k + 0.5, 1.5, b) + B * S(k + 1.5, 0.5, b) + cb) / (1 + k);
            Y0[k] = (C * S(k + 1.5, 1.5, b) + sb) / (2 + k) + D * S(k + 0.5, 0.5, b);
        }

        /// compute (X,Y) by (X0,Y0)
        for (int j = 0; j < nk; j++)
        {
            X[j] = X0[j] - a * Y0[j + 2] / 2;
            Y[j] = Y0[j] + a * X0[j + 2] / 2;
        }

        int t = 1;
        double aa = -(a / 4) * (a / 4);
        for (int n = 1; n <= p; n++)
        {
            t = t * aa / (n * (2 * n - 1));
            double bf = a / (4 * n + 2);
            for (int j = 0; j < nk; j++)
            {
                X[j] = X[j] + t * (X0[4 * n + j] - bf * Y0[4 * n + j + 2]);
                Y[j] = Y[j] + t * (Y0[4 * n + j] + bf * X0[4 * n + j + 2]);
            }
        }

    }

    void Clothoid::intXY(std::vector<double> &X, std::vector<double> &Y, int nk, double a, double b, double c)
    {
        double absa = abs(a);
        double epsi = 0.01;// % best thresold

        if (absa < epsi)//% case `a` small
        {
            evalXYaSmall(X, Y, nk, a, b, 5);
        }
        else
        {
            evalXYaLarge(X, Y, nk, a, b);
        }

        double cc = cos(c);
        double ss = sin(c);
        for (int k = 0; k < nk; k++)
        {
            double xx = X[k];
            double yy = Y[k];
            X[k] = xx * cc - yy * ss;
            Y[k] = xx * ss + yy * cc;
        }
    }

    PtXYAVec Clothoid::Build(const PointXYA &pt_s, const PointXYA &pt_e, int npts, double tol)
    {
        double dx = pt_e.x - pt_s.x;
        double dy = pt_e.y - pt_s.y;
        double r = std::sqrt(dx * dx + dy * dy);
        double phi = std::atan2(dy, dx);

        /// calculate angle between pt_s's direction and s_e_vector's direction
        double phi0 =Degree2Grad(pt_s.yaw) - phi;
//        if(phi0>M_PI)
//            phi0=2.0*M_PI-phi0;
        while(phi0>M_PI)
            phi0=phi0-2*M_PI;
        while(phi0<-M_PI)
            phi0=phi0+2*M_PI;

        /// calculate angle between pt_e's direction and s_e_vector's direction
        double phi1 =Degree2Grad(pt_e.yaw) - phi;
//        if(phi1>M_PI)
//            phi1=2.0*M_PI-phi1;
        while(phi1>M_PI)
            phi1=phi1-2*M_PI;
        while(phi1<-M_PI)
            phi1=phi1+2*M_PI;
        
        double delta = phi1 - phi0;

        // % initial point, guess A
        std::vector<double> CF = {2.989696028701907, 0.716228953608281, -0.458969738821509,
                                  -0.502821153340377, 0.261062141752652, -0.045854475238709};
        double X_guess = phi0 / M_PI;
        double Y_guess = phi1 / M_PI;
        double xy_guess = X_guess * Y_guess;
        double A_guess = (phi0 + phi1) * (CF[0] + xy_guess * (CF[1] + xy_guess * CF[2]) +
                                          (CF[3] + xy_guess * CF[4]) * (X_guess * X_guess + Y_guess * Y_guess) +
                                          CF[5] * (X_guess * X_guess * X_guess * X_guess +
                                                   Y_guess * Y_guess * Y_guess * Y_guess));

        /// Newton iteration, find A
        double A = A_guess;
        for (int iter = 1; iter <= 100; iter++)
        {
            std::vector<double> intC(3, 0.0);
            std::vector<double> intS(3, 0.0);
            intXY(intC, intS, 3, 2 * A, delta - A, phi0);
            double f = intS[0];
            double df = intC[2] - intC[1];
            A = A - f / df;
            if (std::abs(f) < tol)
                break;
        }

        /// final operation, calculate L
        std::vector<double> h(1, 0.0);
        std::vector<double> g(1, 0.0);
        intXY(h, g, 1, 2 * A, delta - A, phi0);
        if(h[0]==0)
            return PtXYAVec();
        double L = r / (h[0]+0.0000001);
        if (L <= 0)
            return PtXYAVec();

        /// calculate Kappa and DKappa
        double Kappa = (delta - A) / L;
        double DKappa = 2 * A / L / L;

        /// ensure the parameters
        if (std::abs(Kappa) > 0.5 && std::abs(Kappa + DKappa * L) > 0.5 && L > 1.5 * pt_s.Distance(pt_e)&&L<10*pt_s.Distance(pt_e))
            return PtXYAVec();

        /// interpolate
        PtXYAVec XY;
        for (double t = 0; t <= L;)
        {
            std::vector<double> C(1, 0.0);
            std::vector<double> S(1, 0.0);
            intXY(C, S, 1, DKappa * t * t, Kappa * t, Degree2Grad(pt_s.yaw));

            double tmp_x = pt_s.x + t * C[0];
            double tmp_y = pt_s.y + t * S[0];
            double tmp_yaw = pt_s.yaw + Grad2Degree(t * Kappa + 0.5 * t * t * DKappa);
            XY.push_back(PointXYA(tmp_x, tmp_y, tmp_yaw));

            t = t + L / (double) npts;
        }
        return XY;
    }

}