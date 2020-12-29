package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="testOpmode", group="Memorial")

public class FIRSTOpMode extends LinearOpMode {
    private DcMotor motor1;
    private DcMotor motor2;
    private ElapsedTime time = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;


    @Override
    public void runOpMode() throws InterruptedException {
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        waitForStart();
        DcMotorSimple.Direction dir = DcMotorSimple.Direction.FORWARD;
        motor1.setDirection(dir);
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor1.setPower(1);
        while (opModeIsActive()){
            if (time.milliseconds()>5000){
                time.reset();
                telemetry.addData("Direction Change", "Time of change: " + time.toString());
                motor1.setPower((motor1.getPower()*-1));
            }
            telemetry.addData("Status", "power: " + motor1.getPower());
            telemetry.update();
        }
        motor1.setPower(0);
    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = motor1.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = motor2.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            motor1.setTargetPosition(newLeftTarget);
            motor2.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            time.reset();
            motor1.setPower(Math.abs(speed));
            motor2.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (time.seconds() < timeoutS) &&
                    (motor1.isBusy() && motor2.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        motor1.getCurrentPosition(),
                        motor2.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            motor1.setPower(0);
            motor2.setPower(0);

            // Turn off RUN_TO_POSITION
            motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
}
