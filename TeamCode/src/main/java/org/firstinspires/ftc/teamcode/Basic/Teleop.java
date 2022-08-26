package org.firstinspires.ftc.teamcode.Basic;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.mecanum;


@TeleOp( name = "Teleop")
public class Teleop extends LinearOpMode {

    mecanum robot=  new mecanum(hardwareMap);
    private final double MAX_SPEED = 1;


    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);


        // defining motors
        DcMotorEx flm = null;
        DcMotorEx blm = null;
        DcMotorEx frm = null;
        DcMotorEx brm = null;

        flm.setDirection(DcMotorEx.Direction.REVERSE);
        blm.setDirection(DcMotorEx.Direction.REVERSE);
        frm.setDirection(DcMotorEx.Direction.REVERSE);
        brm.setDirection(DcMotorEx.Direction.REVERSE);

        mecanum mecanum = new mecanum(hardwareMap);

        mecanum = new mecanum(null);
        // creating an array for the motor speeds

        waitForStart();
        double[] motorSpeeds = new double[2];
        double max =1;



        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
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
