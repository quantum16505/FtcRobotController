package org.firstinspires.ftc.teamcode.OpModes;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Hardware.HardwareProfile;


@Autonomous(name = "CarouselBlueParkAuto")
public class CarouselBlueParkAuto extends LinearOpMode {
    HardwareProfile robot2 = new HardwareProfile();
    private ElapsedTime runtime = new ElapsedTime();




    boolean Step1 = false;
    boolean Step2 = false;
    static final double DRIVE_SPEED = 0.55;
    static final double TURN_SPEED = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        robot2.init(hardwareMap);
        DcMotor[] motors = {robot2.FrontLeftDrive, robot2.FrontRightDrive, robot2.RearRightDrive, robot2.RearLeftDrive,robot2.Carousel};
        stopAndResetEncoder(motors);
        runWithEncoder(motors);

        waitForStart();

        encoderDrive(DRIVE_SPEED, 12, 12, 10 * 7 * 12338 * 2194);
        encoderDrive(DRIVE_SPEED, 17, -17 , 13333333);  // S1: Forward 47 Inches with 5 Sec timeout //was 37.5
        encoderDrive(DRIVE_SPEED, 20, 20 , 13333333);
        encoderDrive(DRIVE_SPEED, 13, -13 , 13333333);
        encoderDrive(DRIVE_SPEED, 15, 15, 10);

        Log.d("CurrentPos", String.valueOf(robot2.Carousel.getCurrentPosition()));

        int newCarouselTarget = robot2.Carousel.getCurrentPosition() + (int) (250 *  robot2.CAROUSEL_COUNTS_PER_INCH);
        Log.d("NewPos", String.valueOf(newCarouselTarget));
        robot2.Carousel.setTargetPosition(newCarouselTarget);
        robot2.Carousel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot2.Carousel.setPower(0.8);
        Log.d("CurrentPos", String.valueOf(robot2.Carousel.getCurrentPosition()));
        ;
        while (robot2.Carousel.isBusy()) {
        }

        encoderDrive(DRIVE_SPEED, -26, -26, 10);
        encoderDrive(DRIVE_SPEED, -12, 12, 10);
        encoderDrive(DRIVE_SPEED, 14.5, 14.5, 10);
//
//        encoderDrive(DRIVE_SPEED, -14.45, 14.45, 140); //turn 90 degrees right
//
//        encoderDrive(DRIVE_SPEED, 22.345, 22.345, 10); //drive forward







    }

    private void stopAndResetEncoder(DcMotor[] motors) {
        int index;
        for (index = 0; index < motors.length; index++) {
            DcMotor motor = motors[index];
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }
    private void runWithEncoder(DcMotor[] motors) {
        int index;
        for (index = 0; index < motors.length; index++){
            DcMotor motor = motors[index];
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

            Log.d("LeftCurrentTarg : ", String.valueOf(robot2.RearLeftDrive.getCurrentPosition()));
            Log.d("RightCurrentTarg : ", String.valueOf(robot2.RearRightDrive.getCurrentPosition()));
            // Determine new target position, and pass to motor controller
            newLeftTarget = robot2.RearLeftDrive.getCurrentPosition() + (int) (leftInches * robot2.WHEEL_COUNTS_PER_INCH);
            newRightTarget = robot2.RearRightDrive.getCurrentPosition() + (int) (rightInches * robot2.WHEEL_COUNTS_PER_INCH);
            Log.d("LeftTarg : ", String.valueOf (newLeftTarget));
            Log.d("RightTarg : ", String.valueOf(newRightTarget));
            robot2.RearLeftDrive.setTargetPosition(newLeftTarget);
            robot2.RearRightDrive.setTargetPosition(newRightTarget);
            robot2.FrontRightDrive.setTargetPosition(newRightTarget);
            robot2.FrontLeftDrive.setTargetPosition(newLeftTarget);


            // Turn On RUN_TO_POSITION
            robot2.RearLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot2.RearRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot2.FrontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot2.FrontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // reset the timeout time and start motion
            // .
            runtime.reset();
            robot2.RearLeftDrive.setPower(Math.abs(speed));
            robot2.RearRightDrive.setPower(Math.abs(speed));
            robot2.FrontRightDrive.setPower(Math.abs(speed));
            robot2.FrontLeftDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot2.RearLeftDrive.isBusy() && robot2.RearRightDrive.isBusy())) {
                // Display it for the driver.

            }

            // Stop all motion;
            robot2.RearLeftDrive.setPower(0);
            robot2.RearRightDrive.setPower(0);
            robot2.FrontLeftDrive.setPower(0);
            robot2.FrontRightDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            robot2.RearLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot2.RearRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot2.FrontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot2.FrontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //  sleep(250);   // optional pause after each move

        }
    }
}


//jimmy
//NAH
//YEAH
//It's Jong
//It's an army of Jimothys! Run for your life!!!!
