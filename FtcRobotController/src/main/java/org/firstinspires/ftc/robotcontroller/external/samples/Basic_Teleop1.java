package org.firstinspires.ftc.teamcode.java.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name="Basic:tleop", group="Linear Opmode")

    public abstract class Basic_Teleop1 extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private  DcMotorEx flm = null;
    private  DcMotorEx blm = null;
    private  DcMotorEx frm = null;
    private  DcMotorEx brm = null;

    }
