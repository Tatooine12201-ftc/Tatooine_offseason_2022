package org.firstinspires.ftc.teamcode;

import android.telephony.CellSignalStrengthGsm;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Engagable;
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;


public class mecanum {

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
    private double RStartingPointX = 0;
    private double RStartingPointY = 0;
    private double RStartingPointr = 0;
    private double fStartingPointX = 0;
    private double fStartingPointY = 0;
    private double fvStartingPointr = 0;

    public mecanum(HardwareMap hw) {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
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
        setZeroBhavier(DcMotor.ZeroPowerBehavior.FLOAT);
        stop();
    }

    /**
     * @param x left - right
     * @param y front - beck
     * @param r rotation
     */

    public void drive(double x, double y, double r, boolean squaredInputs) {
        // Read inverse IMU heading, as the IMU heading is CW positive
        double botHeading = heading();
        double newX = squaredInputs ? x * x : x;
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

    }

    public void setStartingPoint(double x, double y, double r ) {
        RStartingPointX = x;
        RStartingPointY = y;
        RStartingPointr = r;

    }

    public int getXticks() {
        return flm.getCurrentPosition();
    }

    public int getYticks() {
        return ((frm.getCurrentPosition() + blm.getCurrentPosition()) / 2);
    }

    private double tickesToMM(int tickes) {
        return tickes / COUNTS_PER_MM;
    }

    public double heading() {
        return -imu.getAngularOrientation().firstAngle;
    }

    public void stop() {
        flm.setPower(0);
        blm.setPower(0);
        frm.setPower(0);
        brm.setPower(0);
    }

    public void setZeroBhavier(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        flm.setZeroPowerBehavior(zeroPowerBehavior);
        blm.setZeroPowerBehavior(zeroPowerBehavior);
        frm.setZeroPowerBehavior(zeroPowerBehavior);
        brm.setZeroPowerBehavior(zeroPowerBehavior);
    }

    private double radiansTodegrees(double heading) {
        return heading / COUNTS_PER_MM;
    }

    public void WsetStartingPoint(double x, double y, double r) {
        fStartingPointX = x;
        fStartingPointY = y;
        fvStartingPointr = r;


    }
}













