package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="AutonomousA", group="Memorial")

public class AutonomousOpModeA extends LinearOpMode {
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private DcMotor armMotor; // gear ratio is 6 : 1

    private final double    ARM_REDUCTION = .15;
    private final double    ARM_COUNTS_PER_REVOLUTION = ARM_REDUCTION * COUNTS_PER_MOTOR_REV;

    private Servo handServo;

    private ElapsedTime time = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1440;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0;     // For figuring circumference
    static final double     WHEEL_DIAMETER_FEET     = WHEEL_DIAMETER_INCHES/12.0;
    static final double     COUNTS_PER_FOOT         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_FEET * 3.1415);
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
        leftMotor = hardwareMap.get(DcMotor.class, "motor1");
        rightMotor = hardwareMap.get(DcMotor.class, "motor2");
        armMotor = hardwareMap.get(DcMotor.class, "motor0");
        handServo = hardwareMap.get(Servo.class, "servo0");

        //start of autonomous period
        waitForStart();


        toGrab();
        encoderDrive(.3, 1, 0, 10); //forward 1 right 0
        grab(true);
        toTemp();

        encoderDrive(.3, 5, 1, 10);
        toGrab();
        grab(false);
        toHold();
        grab(true);
        toRest();




    }

    public void toRest(){
        moveToPosition(.3, armMotor, REST_POSITION);
    }

    public void toHold(){
        moveToPosition(.3 , armMotor, HOLD_POSITION);
    }

    public void toGrab(){
        moveToPosition(.3, armMotor, GRAB_POSITION);
    }

    private void toTemp() {
        moveToPosition(.3, armMotor, TEMP_POSITION);
    }

    public void grab(boolean open){
        if(open) moveServo(handServo, .19);
        else moveServo(handServo , .90);
    }


    public void moveServo(Servo servo, double openClose){
        servo.setPosition(openClose);
    }

    public void moveByFeet(double speed, DcMotor motor1, DcMotor motor2, double feet){
        int target1, target2;

        if (opModeIsActive()){
            target1 = motor1.getCurrentPosition() + (int)(feet * COUNTS_PER_FOOT);
            motor1.setTargetPosition(target1);

            motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            target2 = motor2.getCurrentPosition() + (int)(feet * COUNTS_PER_FOOT);
            motor2.setTargetPosition(target2);

            motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            time.reset();
            motor1.setPower(Math.abs(speed));
            motor2.setPower(Math.abs(speed));

            telemetry.addData("wheel1",  "Running to %7d", target1);
            telemetry.addData("wheel2",  "Running to %7d", target2);
            telemetry.addData("wheel1",  "Running at %7d", motor1.getCurrentPosition());
            telemetry.addData("wheel2",  "Running at %7d", motor2.getCurrentPosition());
            telemetry.update();

//            while (opModeIsActive() &&
//                    (time.seconds() < timeoutS) &&
//                    (leftMotor.isBusy() && rightMotor.isBusy())) {
//
//                // Display it for the driver.
//
//            }

            motor1.setPower(0);
            motor2.setPower(0);
            motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


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

    public void encoderDrive(double speed,
                             double leftFeet, double rightFeet,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = leftMotor.getCurrentPosition() + (int)(leftFeet * COUNTS_PER_INCH * 12);
            newRightTarget = rightMotor.getCurrentPosition() + (int)(rightFeet * COUNTS_PER_INCH * 12);
            leftMotor.setTargetPosition(newLeftTarget);
            rightMotor.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            time.reset();
            leftMotor.setPower(Math.abs(speed));
            rightMotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (time.seconds() < timeoutS) &&
                    (leftMotor.isBusy() && rightMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        leftMotor.getCurrentPosition(),
                        rightMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            leftMotor.setPower(0);
            rightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
}