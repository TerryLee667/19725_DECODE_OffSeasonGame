package org.firstinspires.ftc.teamcode.utility.RK4;

public class TargetPredictor {

    public static TargetState predict(double x0, double y0, double vx, double vy, double t) {
        double x = x0 + vx * t;
        double y = y0 + vy * t;
        return new TargetState(x, y, vx, vy);
    }

    public static TargetState predict(TargetState initial, double t) {
        return predict(initial.x, initial.y, initial.vx, initial.vy, t);
    }

    public static class TargetState {
        public final double x;
        public final double y;
        public final double vx;
        public final double vy;

        public TargetState(double x, double y, double vx, double vy) {
            this.x = x;
            this.y = y;
            this.vx = vx;
            this.vy = vy;
        }

        public double getSpeed() {
            return Math.sqrt(vx * vx + vy * vy);
        }

        public double getDistanceTo(double otherX, double otherY) {
            return Math.sqrt(Math.pow(x - otherX, 2) + Math.pow(y - otherY, 2));
        }
    }
}