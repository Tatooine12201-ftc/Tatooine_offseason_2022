package org.firstinspires.ftc.teamcode.Basic;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;



@TeleOp( name = "Teleop")


public class Teleop extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotorEx flm = null;
        DcMotorEx blm = null;
        DcMotorEx frm = null;
        DcMotorEx brm = null;



        frm.setDirection(DcMotorSimple.Direction.REVERSE);
        brm.setDirection(DcMotorSimple.Direction.REVERSE);
        flm.setDirection(DcMotorSimple.Direction.REVERSE);
        blm.setDirection(DcMotorSimple.Direction.REVERSE);


        waitForStart();

        while (opModeIsActive()) {

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            flm.setPower(frontLeftPower);
            blm.setPower(backLeftPower);
            frm.setPower(frontRightPower);
            brm.setPower(backRightPower);


        }

    }















}
