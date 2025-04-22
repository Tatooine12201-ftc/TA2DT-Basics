package org.firstinspires.ftc.teamcode.TA2D2.PID;

public class PIDFFController extends PIDController {

    private double kF = 0.0;

    public PIDFFController(double kp, double ki, double kd, double kf,double sp, double pv) {
        super(kp, ki, kd, sp, pv);
        setkF(kf);
    }

    public PIDFFController(double kp, double ki, double kd, double kf) {
        this(kp, ki, kd, kf, 0, 0);
    }

    public double getkF() {
        return kF;
    }

    public void setkF(double kF) {
        this.kF = kF;
    }

    @Override
    public double calculate(double pv) {
        return super.calculate(pv) + kF * pv;
    }

    public double calculate(double pv, double sp) {
        setSetPoint(sp);
        return calculate(pv, getSetPoint());
    }

    public double calculate() {
        return calculate(getMeasuredValue());
    }

    @Override
    public double[] getCoefficients() {
        return new double[]{getP(), getI(), getD(), getkF()};

    }

    public void setPIDFF(double kp, double ki, double kd, double kf) {
        super.setPID(kp, ki, kd);
        this.kF = kf;
    }


}
