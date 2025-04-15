package org.firstinspires.ftc.teamcode.TA2D2;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.TA2D2.mathUtil.MathUtil;

public class Encoder {

    public double TICK_PER_REV = 0;

    public double SPOOL_DIA = 0;

    public DcMotor encoder;

    public Encoder(DcMotor encoder) {
        this.encoder = encoder;
    }

    public double getTICK_PER_REV() {
        return TICK_PER_REV;
    }

    public void setTICK_PER_REV(double TICK_PER_REV) {
        this.TICK_PER_REV = TICK_PER_REV;

    }

    public double getSPOOL_DIA() {
        return SPOOL_DIA;
    }

    public void setSPOOL_DIA(double SPOOL_DIA) {
        this.SPOOL_DIA = SPOOL_DIA;
    }

    public void setDirection(DcMotorSimple.Direction direction) {
        encoder.setDirection(direction);
    }

    public double getPosition() {
        return encoder.getCurrentPosition();
    }

    public double getAngle() {
        return MathUtil.convertTicksToDegrees(TICK_PER_REV, getPosition());
    }

    public double getDistance() {
        return MathUtil.convertTicksToDistance(TICK_PER_REV, SPOOL_DIA, getPosition());
    }

    public void resetEncoder() {
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

}
