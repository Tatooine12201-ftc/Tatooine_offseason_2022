package org.firstinspires.ftc.teamcode.java.util;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.java.util.Hardwere;


public class Intake
{
    Hardwere robot;
    private final Servo in1;
    private final CRServo lift;
    private final CRServo lift2;
    private int degrees;


    private static final double lift_speed = -0.4;
    private static final double down_speed = 1;

    private static final double OPEN = 1;
    private static final double CLOSED = 0;


    /**
     * this function creates anew intake
     * @param robot the robot hardware
     */
    public Intake(Hardwere robot) {
        this.robot = robot;
        this.in1 = robot.in1;
        this.lift = robot.lift;
        this.lift2 = robot.lift2;
    }

    public Intake(Servo in1, CRServo lift, CRServo lift2) {
        this.in1 = in1;
        this.lift = lift;
        this.lift2 = lift2;
    }

    /**
     * this function intakes
     */
    public void intake() {
        in1.setPosition(1);
    }

    /**
     * this function outtakes
     */
    public void outtake() {
        in1.setPosition(0);
    }

    public void higher() {
        lift.setPower(1);
        lift2.setPower(1);
    }
    public void  lower(){
        lift.setPower(-1);
        lift2.setPower(-1);
    }


    /**
     * this function turns off the intake
     */
    public void stop() {
        lift.setPower(0);
        lift2.setPower(0);
    }
}