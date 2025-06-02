package org.firstinspires.ftc.teamcode.TA2D2.OpModes;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.TA2D2.Poses.Pose2d;
import org.firstinspires.ftc.teamcode.TA2D2.SubSystems.MecanumDrive;

@TeleOp(name = "MecanumTeleOp", group = "Testing")

public class MecanumTeleOp extends LinearOpMode {

    MecanumDrive mecanum;

    @Override
            public void runOpMode() {
        mecanum = new MecanumDrive(this, false);

        waitForStart();
        while (opModeIsActive()){
//            if (gamepad2.cross){
//                mecanum.setPowerBackRight(1);
//            }
//            else{
//                mecanum.setPowerBackRight(0);
//            }
//            if (gamepad2.circle){
//                mecanum.setPowerFrontRight(1);
//            }
//            else {
//                mecanum.setPowerFrontRight(0);
//            }
//            if (gamepad2.triangle){
//                mecanum.setPowerFrontLeft(1);
//            }
//            else {
//                mecanum.setPowerFrontLeft(0);
//            }
//            if (gamepad2.square){
//                mecanum.setPowerBackLeft(1);
//            }
//            else {
//                mecanum.setPowerBackLeft(0);
//            }
//            mecanum.setPower(new Pose2d(gamepad2.left_stick_x , -gamepad2.left_stick_y, gamepad2.right_stick_x));
            mecanum.fieldDrive(new Pose2d(gamepad1.left_stick_x , -gamepad1.left_stick_y, gamepad1.right_stick_x));
            telemetry.update();
        }
    }
}