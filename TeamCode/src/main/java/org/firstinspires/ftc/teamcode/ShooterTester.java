package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="ShooterTester", group="Memorial")

public class ShooterTester extends LinearOpMode {

    private DcMotor shooter;

    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();

        shooter = hardwareMap.get(DcMotor.class, "motor0");
        // speed is double from -1.0 to +1.0
        while (opModeIsActive()){
            activateMotorByDirection(shooter, -1.0);
        }

    }

    public void activateMotorByDirection(DcMotor motor, double dirSpeed){

        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setPower(dirSpeed);
    }


}
