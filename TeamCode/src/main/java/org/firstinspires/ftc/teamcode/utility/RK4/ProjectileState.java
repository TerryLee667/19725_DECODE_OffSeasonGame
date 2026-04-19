package org.firstinspires.ftc.teamcode.utility.RK4;

public class ProjectileState {
    public double x;
    public double y;
    public double z;
    public double vx;
    public double vy;
    public double vz;
    public double time;

    public ProjectileState() {
        this.x = 0;
        this.y = 0;
        this.z = 0;
        this.vx = 0;
        this.vy = 0;
        this.vz = 0;
        this.time = 0;
    }

    public ProjectileState(double x, double y, double z, double vx, double vy, double vz, double time) {
        this.x = x;
        this.y = y;
        this.z = z;
        this.vx = vx;
        this.vy = vy;
        this.vz = vz;
        this.time = time;
    }

    public ProjectileState copy() {
        return new ProjectileState(x, y, z, vx, vy, vz, time);
    }

    public double getSpeed() {
        return Math.sqrt(vx * vx + vy * vy + vz * vz);
    }

    public double getHorizontalSpeed() {
        return Math.sqrt(vx * vx + vy * vy);
    }

    public double getHorizontalDistance() {
        return Math.sqrt(x * x + y * y);
    }
}