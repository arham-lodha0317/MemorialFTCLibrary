package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Player", group="Memorial")


public class PlayerOpMode extends LinearOpMode {
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private DcMotor armMotor;

    private final double    ARM_REDUCTION = .15;
    private final double    ARM_COUNTS_PER_REVOLUTION = ARM_REDUCTION * COUNTS_PER_MOTOR_REV;

    private Servo handServo;

    private ElapsedTime time = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);                   // counts per revolution over circumference
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;
    static final double     REST_POSITION           = 0;
    static final double     HOLD_POSITION           = 1288;
    static final double     GRAB_POSITION           = 4288;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        leftMotor = hardwareMap.get(DcMotor.class, "motor1");
        rightMotor = hardwareMap.get(DcMotor.class, "motor2");
        armMotor = hardwareMap.get(DcMotor.class, "motor0");
//        handServo = hardwareMap.get(Servo.class, "servo0");

        waitForStart();
        time.reset();

        while (opModeIsActive()) {
            speedEncoder(-gamepad1.left_stick_y, gamepad1.right_stick_x);
            if(gamepad1.a){
                moveByRotation(TURN_SPEED*10, armMotor, .25, 10);
            }
            else if(gamepad1.b){
                moveByRotation(TURN_SPEED*10, armMotor, -.25, 10);
            }
        }



    }

    public void moveByRotation(double speed, DcMotor motor, double rotations, double timeoutS){
        int target;

        if (opModeIsActive()){
            target = motor.getCurrentPosition() + (int)(rotations * ARM_COUNTS_PER_REVOLUTION);
            motor.setTargetPosition(target);

            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            time.reset();
            motor.setPower(Math.abs(speed));

            telemetry.addData("armP1",  "Running to %7d", target);
            telemetry.addData("armP2",  "Running at %7d", motor.getCurrentPosition());
            telemetry.update();

//            while (opModeIsActive() &&
//                    (time.seconds() < timeoutS) &&
//                    (leftMotor.isBusy() && rightMotor.isBusy())) {
//
//                // Display it for the driver.
//
//            }

            motor.setPower(0);

            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void moveServo(Servo servo, double openClose){
        servo.setPosition(openClose);
    }

    public void moveToPosition(DcMotor motor, double speed, int toPosition, double timeoutS){
        if(opModeIsActive()){
            motor.setTargetPosition(toPosition);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            time.reset();
            motor.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (time.seconds() < timeoutS) &&
                    (leftMotor.isBusy() && rightMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("armP1",  "Running to %7d", toPosition);
                telemetry.addData("armP2",  "Running at %7d", motor.getCurrentPosition());
                telemetry.update();
            }

            motor.setPower(0);

            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }



    public void speedEncoder(double x, double y){

        double leftPower = Range.clip(y + x, -1.0, 1.0);
        double rightPower = Range.clip(y - x, -1.0, 1.0);

        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);

//        telemetry.addData("Status", "Run Time: " + time.toString());
//        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
//        telemetry.update();
    }

    public void arm(double power){
        armMotor.setPower(power);

        telemetry.addData("Status", "Run Time: " + time.toString());
        telemetry.addData("Motors", "arm (%.2f)", power);
        telemetry.update();
    }
}
