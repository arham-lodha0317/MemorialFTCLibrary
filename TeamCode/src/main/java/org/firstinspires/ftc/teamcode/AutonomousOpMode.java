package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Autonomous", group="Memorial")

public class AutonomousOpMode extends LinearOpMode {
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private DcMotor armMotor; // gear ratio is 6 : 1

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
        leftMotor = hardwareMap.get(DcMotor.class, "motor1");
        rightMotor = hardwareMap.get(DcMotor.class, "motor2");
        armMotor = hardwareMap.get(DcMotor.class, "motor0");
        handServo = hardwareMap.get(Servo.class, "servo0");

        //start of autonomous period
        waitForStart();
        // spin arm 1/2 way
        while (opModeIsActive()) {
//            moveByRotation(TURN_SPEED, armMotor, 1, 10.0);
            moveByRotation(TURN_SPEED, armMotor, -1, 10.0);
            break;
        }
        //encoderDrive();

    }

    public void moveServo(Servo servo, boolean openClose){
        //make code
    }

    public void moveByRotation(double speed, DcMotor motor, double rotations, double timeoutS){
        int target;

        if (opModeIsActive()){
            target = motor.getCurrentPosition() + (int)(rotations * ARM_COUNTS_PER_REVOLUTION);
            motor.setTargetPosition(target);

            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            time.reset();
            motor.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (time.seconds() < timeoutS) &&
                    (leftMotor.isBusy() && rightMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("armP1",  "Running to %7d", target);
                telemetry.addData("armP2",  "Running at %7d", motor.getCurrentPosition());
                telemetry.update();
            }

            motor.setPower(0);

            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = leftMotor.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = rightMotor.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
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