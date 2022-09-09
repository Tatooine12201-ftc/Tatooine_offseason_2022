package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "auto")
@Disabled
public class auto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Mecanum mecanum =new Mecanum(hardwareMap);
        while (opModeIsActive() && !isStopRequested()) {
        mecanum.driveTo(12,45,30);

    }
}
}

