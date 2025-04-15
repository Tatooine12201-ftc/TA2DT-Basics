package org.firstinspires.ftc.teamcode.TA2D2;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.TA2D2.mathUtil.MathUtil;

public class DriveEncoder extends Encoder {

    public DriveEncoder(DcMotor encoder, DcMotorSimple.Direction direction, double TICK_PER_REV, double SPOOL_DIA) {
        super(encoder);
        setDirection(direction);
        setTICK_PER_REV(TICK_PER_REV);
        setSPOOL_DIA(SPOOL_DIA);
    }


}
