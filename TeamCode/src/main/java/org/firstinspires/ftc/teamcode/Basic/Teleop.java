package org.firstinspires.ftc.teamcode.Basic;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.mecanum;


@TeleOp( name = "Teleop")
public class Teleop extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        mecanum mecanum = new mecanum(hardwareMap);
        while (opModeIsActive()) {
            mecanum.drive(gamepad1.left_stick_x,gamepad1.left_stick_y,gamepad1.right_stick_x,true);
        }

    }















}
