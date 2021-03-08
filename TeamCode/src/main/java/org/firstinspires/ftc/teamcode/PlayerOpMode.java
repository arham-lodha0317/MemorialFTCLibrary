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
    static final int     REST_POSITION           = 0;
    static final int     HOLD_POSITION           = 1600;
    static final int     TEMP_POSITION           = 5000;
    static final int     GRAB_POSITION           = 5800;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        leftMotor = hardwareMap.get(DcMotor.class, "motor1");
        rightMotor = hardwareMap.get(DcMotor.class, "motor2");
        armMotor = hardwareMap.get(DcMotor.class, "motor0");
        handServo = hardwareMap.get(Servo.class, "servo0");

        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
        time.reset();

        boolean open = false;

        while (opModeIsActive()) {
            speedEncoder(gamepad1.right_stick_x, -gamepad1.left_stick_y);

            if (gamepad1.dpad_left) {
                toRest();
            }
            if (gamepad1.dpad_up) {
                toHold();
            }
            if (gamepad1.dpad_right) {
                toTemp();
            }
            if (gamepad1.dpad_down){
                toGrab();
            }
            if (gamepad1.b) {
                grab(false);
            } else if (gamepad1.a) {
                grab(true);
            }
            if (gamepad1.left_bumper) {
                moveByRotation(.5, armMotor, -1);
            } else if (gamepad1.right_bumper) {
                moveByRotation(.5, armMotor, 1);
            }
            if (gamepad1.x) {
                armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
        }
    }

    public void moveByRotation(double speed, DcMotor motor, double rotations){
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
        telemetry.addData(servo.toString(), "running to " , openClose);
    }

    public void moveToPosition( double speed, DcMotor motor, int toPosition){
        if(opModeIsActive()){
            motor.setTargetPosition(toPosition);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            time.reset();
            motor.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (motor.isBusy()) &&
                    Math.abs(armMotor.getTargetPosition() - armMotor.getCurrentPosition()) > 20) {

                // Display it for the driver.
                telemetry.addData(motor.getDeviceName(),  "Running to %7d", toPosition);
                telemetry.addData(motor.getDeviceName(),  "Running at %7d", motor.getCurrentPosition());
                telemetry.update();
            }

            motor.setPower(0);

            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void toRest(){
        moveToPosition(.2, armMotor, REST_POSITION);
    }

    public void toHold(){
        moveToPosition(.2 , armMotor, HOLD_POSITION);
    }

    public void toGrab(){
        moveToPosition(.2, armMotor, GRAB_POSITION);
    }

    private void toTemp() {
        moveToPosition(.2, armMotor, TEMP_POSITION);
    }

    public void grab(boolean open){
        if(!open) moveServo(handServo, .19);
        else moveServo(handServo , .90);
    }


    public void speedEncoder(double x, double y){

        double leftPower = Range.clip(y + x, -1.0, 1.0);
        double rightPower = Range.clip(y - x, -1.0, 1.0);

        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);

        telemetry.addData("Status", "Run Time: " + time.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
        telemetry.update();
    }
}
