package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Armtester", group = "Memorial")


public class ArmTester extends LinearOpMode {

    private CRServo continous;

    @Override
    public void runOpMode() throws InterruptedException {

        continous = hardwareMap.get(CRServo.class, "servo1");

        waitForStart();

        while (opModeIsActive()) {

            continous.setPower(Range.clip(gamepad1.left_trigger-gamepad1.right_trigger, -1.0, 1.0));

        }

    }
}
