package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;


public class Mecanum {

    private static final double COUNTS_PER_MOTOR_REV = 28;    // eg: TETRIX Motor Encoder
    //private static final double COUNTS_PER_RADIAN = 6.283185307179586; //
    private static final double DRIVE_GEAR_REDUCTION = 19.2;     // This is < 1.0 if geared UP
    private static final double WHEEL_DIAMETER_MM = 4.0 * 25.4;     // For figuring circumference
    private static final double WHEEL_CIRCUMFERENCE = (WHEEL_DIAMETER_MM * Math.PI);
    private static final double COUNTS_PER_MM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / WHEEL_CIRCUMFERENCE;
    //private static final double COUNTS_PER_DE = (COUNTS_PER_RADIAN * 180/Math.PI) ;
    //DRIVE motors//
    private BNO055IMU imu = null;
    private DcMotorEx flm = null;
    private DcMotorEx blm = null;
    private DcMotorEx frm = null;
    private DcMotorEx brm = null;

    private double robotX = 0;
    private double robotY = 0;


    private double fieldX = 0;
    private double fieldY = 0;

    private double fvStartingPointR = 0;

    public Mecanum(HardwareMap hw) {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        // Parts in hardware map
        imu = hw.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        flm = hw.get(DcMotorEx.class, "FLM");//x
        blm = hw.get(DcMotorEx.class, "BLM");//y1
        frm = hw.get(DcMotorEx.class, "FRM");//y2
        brm = hw.get(DcMotorEx.class, "BRM");
        setZeroBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        stop();
    }

    public double getX() {
        update();
        return fieldX;
    }

    public void setStartingPointX(double fStartingPointX) {
        this.fieldX = fStartingPointX;
        robotX = fieldX * Math.cos(Math.toRadians(fvStartingPointR)) - fieldY * Math.sin(Math.toRadians(fvStartingPointR));
    }

    public double getY() {
        update();
        return fieldY;
    }

    public void setStartingPointY(double fStartingPointY) {
        this.fieldY = fStartingPointY;
        robotY = fieldX * Math.sin(Math.toRadians(fvStartingPointR)) + fieldY * Math.cos(Math.toRadians(fvStartingPointR));
    }

    public double getFvStartingPointR() {
        return fvStartingPointR;
    }

    public void setFvStartingPointR(double fvStartingPointR) {
        this.fvStartingPointR = fvStartingPointR;
    }

    public void setStartingPoint(double x, double y, double r) {
        setFvStartingPointR(r);
        setStartingPointX(x);
        setStartingPointY(y);
    }

    public void update() {
        double prvRobotX = robotX;
        double prvRobotY = robotY;

        robotX = ticksToMM(getXTicks());
        robotY = ticksToMM(getYTicks());

        double deltaY = robotY - prvRobotY;
        double deltaX = robotX - prvRobotX;
        double robotHeading = headingInRed();

        fieldX += (deltaX * Math.cos(robotHeading) - deltaY * Math.sin(robotHeading));
        fieldY += (deltaX * Math.sin(robotHeading) + deltaY * Math.cos(robotHeading));
    }

    public int getXTicks() {
        return flm.getCurrentPosition();
    }

    public int getYTicks() {
        return ((frm.getCurrentPosition() + blm.getCurrentPosition()) / 2);
    }

    private double ticksToMM(int ticks) {
        return ticks / COUNTS_PER_MM;
    }

    public void drive(double x, double y, double r, boolean squaredInputs) {
        // Read inverse IMU heading, as the IMU heading is CW positive
        double botHeading = headingInRed();
        double newX = squaredInputs ? x * x : x;// if (squaredInputs == true) {newX = X * X;} else {newX = X;}
        double newY = squaredInputs ? y * y : y;
        double newR = r;
        double rotX = newX * Math.cos(botHeading) - newY * Math.sin(botHeading);
        double rotY = newX * Math.sin(botHeading) + newY * Math.cos(botHeading);

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(newY) + Math.abs(newX) + Math.abs(newR), 1);
        double frontLeftPower = (rotY + rotX + newR) / denominator;
        double backLeftPower = (rotY - rotX + newR) / denominator;
        double frontRightPower = (rotY - rotX - newR) / denominator;
        double backRightPower = (rotY + rotX - newR) / denominator;

        flm.setPower(frontLeftPower);
        blm.setPower(backLeftPower);
        frm.setPower(frontRightPower);
        brm.setPower(backRightPower);
        update();

    }


    public double heading() {
        return -imu.getAngularOrientation().firstAngle + fvStartingPointR;
    }

    public double headingInRed() {
        return Math.toRadians(heading());
    }

    public void stop() {
        flm.setPower(0);
        blm.setPower(0);
        frm.setPower(0);
        brm.setPower(0);
    }

    public void setZeroBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        flm.setZeroPowerBehavior(zeroPowerBehavior);
        blm.setZeroPowerBehavior(zeroPowerBehavior);
        frm.setZeroPowerBehavior(zeroPowerBehavior);
        brm.setZeroPowerBehavior(zeroPowerBehavior);
    }


}













