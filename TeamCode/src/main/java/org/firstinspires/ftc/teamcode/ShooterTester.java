package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="ShooterTester", group="Memorial")

class ShooterTester extends LinearOpMode {

    private DcMotor shooter;

    @Override
    public void runOpMode() throws InterruptedException {

        shooter = hardwareMap.get(DcMotor.class, "motor0");


    }
}
