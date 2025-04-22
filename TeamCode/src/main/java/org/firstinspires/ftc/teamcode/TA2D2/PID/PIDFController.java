package org.firstinspires.ftc.teamcode.TA2D2.PID;

public class PIDFController extends PIDController {

    private double kF = 0.0;

    public PIDFController(double kp, double ki, double kd, double sp, double pv) {
        super(kp, ki, kd, sp, pv);
    }

    public PIDFController(double kp, double ki, double kd, double kf) {
        this(kp, ki, kd, 0, 0);
    }

    public double getkF() {
        return kF;
    }

    public void setkF(double kF) {
        this.kF = kF;
    }

    @Override
    public double calculate(double pv) {
        return super.calculate(pv) + kF;
    }

    public double calculate(double pv, double setPoint) {
        setSetPoint(getSetPoint());
        return calculate(pv, getSetPoint());
    }

    public double calculate() {
        return calculate(getMeasuredValue());
    }

    @Override
    public double[] getCoefficients() {
        return new double[]{getP(), getI(), getD(), getkF()};

    }

    public void setPIDF(double kp, double ki, double kd, double kf) {
        super.setPID(kp, ki, kd);
        this.kF = kf;
    }


}
