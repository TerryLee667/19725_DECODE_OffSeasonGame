package org.firstinspires.ftc.teamcode.utility.RK4;

public class RobotState {
    public double x;
    public double y;
    public double vx;
    public double vy;

    public RobotState() {
        this.x = 0;
        this.y = 0;
        this.vx = 0;
        this.vy = 0;
    }

    public RobotState(double x, double y, double vx, double vy) {
        this.x = x;
        this.y = y;
        this.vx = vx;
        this.vy = vy;
    }

    public RobotState copy() {
        return new RobotState(x, y, vx, vy);
    }

    public void update(double vx, double vy) {
        this.vx = vx;
        this.vy = vy;
    }
}