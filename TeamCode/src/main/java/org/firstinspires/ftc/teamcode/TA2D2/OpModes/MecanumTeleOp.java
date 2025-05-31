package org.firstinspires.ftc.teamcode.TA2D2.OpModes;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.TA2D2.Poses.Pose2d;
import org.firstinspires.ftc.teamcode.TA2D2.SubSystems.MecanumDriveNew;

@TeleOp(name = "MecanumTeleOp", group = "Testing")

public class MecanumTeleOp extends LinearOpMode {

    MecanumDriveNew mecanum;

    @Override
            public void runOpMode() {
        mecanum = new MecanumDriveNew(this, true);

        waitForStart();
        while (opModeIsActive()){
            mecanum.setPower(new Pose2d(gamepad1.left_stick_x , gamepad1.left_stick_y, gamepad1.right_stick_x));
        }
    }
}