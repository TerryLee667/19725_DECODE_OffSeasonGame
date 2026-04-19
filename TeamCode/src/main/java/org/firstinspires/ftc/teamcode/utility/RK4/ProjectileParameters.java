package org.firstinspires.ftc.teamcode.utility.RK4;

public class ProjectileParameters {
    public double v0;
    public double k;
    public double n;
    public double m;
    public double g;
    public double deltaH;
    public double thetaMax;

    public ProjectileParameters() {
        this.v0 = 6.0;
        this.k = 0.015;
        this.n = 2.0;
        this.m = 0.1;
        this.g = 9.81;
        this.deltaH = 0.5;
        this.thetaMax = Math.PI / 4;
    }

    public ProjectileParameters(double v0, double k, double n, double m, double deltaH, double thetaMax) {
        this.v0 = v0;
        this.k = k;
        this.n = n;
        this.m = m;
        this.g = 9.81;
        this.deltaH = deltaH;
        this.thetaMax = thetaMax;
    }

    public ProjectileParameters withV0(double v0) {
        this.v0 = v0;
        return this;
    }

    public ProjectileParameters withK(double k) {
        this.k = k;
        return this;
    }

    public ProjectileParameters withN(double n) {
        this.n = n;
        return this;
    }

    public ProjectileParameters withM(double m) {
        this.m = m;
        return this;
    }

    public ProjectileParameters withDeltaH(double deltaH) {
        this.deltaH = deltaH;
        return this;
    }

    public ProjectileParameters withThetaMax(double thetaMax) {
        this.thetaMax = thetaMax;
        return this;
    }

    public ProjectileParameters copy() {
        return new ProjectileParameters(v0, k, n, m, deltaH, thetaMax);
    }
}