package org.firstinspires.ftc.teamcode.java.util;


import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

public class Hardwere {
    public HardwareMap hardwareMap;
    //DRIVE motors

    public DcMotor leftMotor = null;
    public DcMotor rightMotor = null;
    public BNO055IMU imu = null;

    public Servo in1 = null;
    public CRServo lift = null;
    public CRServo lift2 = null;

    /**
     * Sets up the HardwareMap
     *
     * @param hardwareMap is the hardware map
     */
    public void init(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        // Parts in hardware map
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        // config
        leftMotor = hardwareMap.get(DcMotor.class, "Left_Motor");
        rightMotor = hardwareMap.get(DcMotor.class, "Right_Motor");
        in1 = hardwareMap.get(Servo.class, "in1");
        lift = hardwareMap.get(CRServo.class, "lift");
        lift2 = hardwareMap.get(CRServo.class, "lift2");

        in1.setDirection(Servo.Direction.REVERSE);
        in1.setPosition(1);





        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setDirection(FORWARD);
        lift2.setDirection(REVERSE);

    }
}
