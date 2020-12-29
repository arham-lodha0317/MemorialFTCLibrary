package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="testOpmode", group="Memorial")

public class FIRSTOpMode extends LinearOpMode {
    private DcMotor motorTest;
    private ElapsedTime time = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        motorTest = hardwareMap.get(DcMotor.class, "motor2");
        waitForStart();
        DcMotorSimple.Direction dir = DcMotorSimple.Direction.FORWARD;
        motorTest.setDirection(dir);
        motorTest.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorTest.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorTest.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorTest.setPower(1);
        while (opModeIsActive()){
            if (time.milliseconds()>5000){
                time.reset();
                telemetry.addData("Direction Change", "Time of change: " + time.toString());
                motorTest.setPower((motorTest.getPower()*-1));
            }
            telemetry.addData("Status", "power: " + motorTest.getPower());
            telemetry.update();
        }
        motorTest.setPower(0);
    }
}
