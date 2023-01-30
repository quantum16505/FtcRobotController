package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.HardwareProfile;

@Autonomous(name="WarehouseAutoBlue")
public class WarehouseAutoBlue extends LinearOpMode {
    HardwareProfile robot = new HardwareProfile();
    private ElapsedTime runtime = new ElapsedTime();

    static final double DRIVE_SPEED = 0.8;
    static final double TURN_SPEED = 0.5;



    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        DcMotor[] motors = {robot.FrontLeftDrive, robot.FrontRightDrive, robot.RearRightDrive, robot.RearLeftDrive};
        stopAndResetEncoder(motors);

        runWithEncoder(robot.RearLeftDrive);
        runWithEncoder(robot.FrontLeftDrive);
        runWithEncoder(robot.RearRightDrive);
        runWithEncoder(robot.FrontRightDrive);

        //46.5
        waitForStart();
        encoderDrive(DRIVE_SPEED, 53, 53, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
//        encoderDrive(DRIVE_SPEED, 200, -200, 18.0);
    }
    private void stopAndResetEncoder(DcMotor[] motors) {
       int index;
        for (index = 0; index < motors.length; index++) {
            DcMotor motor = motors[index];
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }
    private void runWithEncoder(DcMotor motor) {
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.RearLeftDrive.getCurrentPosition() + (int) (leftInches * robot.WHEEL_COUNTS_PER_INCH);
            newRightTarget = robot.RearRightDrive.getCurrentPosition() + (int) (rightInches * robot.WHEEL_COUNTS_PER_INCH);
            robot.RearLeftDrive.setTargetPosition(newLeftTarget);
            robot.RearRightDrive.setTargetPosition(newRightTarget);
            robot.FrontRightDrive.setTargetPosition(newRightTarget);
            robot.FrontLeftDrive.setTargetPosition(newLeftTarget);


            // Turn On RUN_TO_POSITION
            robot.RearLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.RearRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.FrontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.FrontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // reset the timeout time and start motion.
            runtime.reset();
            robot.RearLeftDrive.setPower(Math.abs(speed));
            robot.RearRightDrive.setPower(Math.abs(speed));
            robot.FrontRightDrive.setPower(Math.abs(speed));
            robot.FrontLeftDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.RearLeftDrive.isBusy() && robot.RearRightDrive.isBusy())) {
                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.RearLeftDrive.getCurrentPosition(),
                        robot.RearRightDrive.getCurrentPosition());
                telemetry.update();

            }

            // Stop all motion;
            robot.RearLeftDrive.setPower(0);
            robot.RearRightDrive.setPower(0);
            robot.FrontLeftDrive.setPower(0);
            robot.FrontRightDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.RearLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.RearRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.FrontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.FrontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //  sleep(250);   // optional pause after each move
        }
    }
}