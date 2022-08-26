package org.firstinspires.ftc.teamcode.Basic;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Mecanum;


@TeleOp( name = "Teleop")
public class Teleop extends LinearOpMode {
    Mecanum mecanum = new Mecanum(hardwareMap);
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            mecanum.drive(
                            gamepad1.left_stick_x,
                            gamepad1.left_stick_y,
                            gamepad1.right_stick_x,
                            true);
        }
    }















}
