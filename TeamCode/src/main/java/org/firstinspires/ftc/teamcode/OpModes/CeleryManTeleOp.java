package org.firstinspires.ftc.teamcode.OpModes;
//

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Hardware.HardwareProfile;

@TeleOp(name = "Mechanum_TeleOp", group = "")
public class CeleryManTeleOp extends LinearOpMode {

    HardwareProfile robot = new HardwareProfile();   // Use a Pushbots hardware
    double rx;
    double y;
    double x;
    double speedFunction = .8;



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

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        robot.init(hardwareMap);


        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                DriveControls();

                CarouselControls();

                ArmControls();

                IntakeControls();

                telemetry.update();
            }

        }
    }
    private void IntakeControls() {
        double openPos = 0.86;
        double startPos = 1;
        double closePos = 1;
        if (gamepad1.left_bumper) {
            robot.IntakeLeft.setPosition(openPos);
            robot.IntakeRight.setPosition(openPos);
        }
        else if (gamepad1.right_bumper) {
            robot.IntakeLeft.setPosition(closePos);
            robot.IntakeRight.setPosition(closePos);
        }
        else {
            robot.IntakeLeft.setPosition(startPos);
            robot.IntakeRight.setPosition(startPos);
        }
    }
    private void ArmControls() {
        double armPos = robot.ArmServo.getPosition();

        double newPos = armPos;
        if(gamepad1.a) {
            newPos = armPos + 0.002;
        }
        if(gamepad1.y) {
            newPos = armPos - 0.002;

        }
        if(newPos > 0.82) {
            newPos = 0.82;
        }
        if(newPos < 0.2) {
            newPos = 0.2;
        }
        robot.ArmServo.setPosition(newPos);
    }

    private void CarouselControls() {
        // int position = robot.Carousel.getCurrentPosition();
        double liftPower = .5;
        telemetry.addData("Carousel: ", robot.Carousel.getCurrentPosition());
        // low junction position
        if (gamepad1.dpad_left) {
            // robot.Carousel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.Carousel.setTargetPosition(-1225);
            robot.Carousel.setPower(liftPower);
        }
        // mid junction position
        else if (gamepad1.dpad_up) {
            // robot.Carousel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.Carousel.setTargetPosition(-2150);
            robot.Carousel.setPower(liftPower);
        }
        // high junction position
        else if (gamepad1.dpad_right) {
            // robot.Carousel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.Carousel.setTargetPosition(-2925);
            robot.Carousel.setPower(liftPower);
        }
        // ground position
        else if (gamepad1.dpad_down) {
            // robot.Carousel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.Carousel.setTargetPosition(0);
            robot.Carousel.setPower(liftPower);
        }
        // top of stack position
        else if (gamepad1.y) {
            // robot.Carousel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.Carousel.setTargetPosition(-450);
            robot.Carousel.setPower(liftPower);
        }
        // second stack position
        else if (gamepad1.a) {
            // robot.Carousel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.Carousel.setTargetPosition(-100);
            robot.Carousel.setPower(liftPower);
        }
        // third stack position
        else if (gamepad1.x) {
            // robot.Carousel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.Carousel.setTargetPosition(-225);
            robot.Carousel.setPower(liftPower);
        }
        // fourth stack position
        else if (gamepad1.b) {
            // robot.Carousel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.Carousel.setTargetPosition(-325);
            robot.Carousel.setPower(liftPower);
        }
        // else {
        //    if (position >= -10) {
        //        robot.Carousel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //    }
        // }


    }

    private void DriveControls() {
        y = -gamepad1.left_stick_y;
        x = gamepad1.right_stick_x * 1.1;
        rx = gamepad1.left_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx),1);

        double FrontLeftPower = speedFunction * ((y + x + rx) / denominator);
        double RearLeftPower = speedFunction * ((y - rx + x) / denominator);
        double FrontRightPower = speedFunction * ((y - rx - x) / denominator);
        double RearRightPower = speedFunction * ((y + rx - x) / denominator);

        robot.FrontLeftDrive.setPower(FrontLeftPower);
        robot.RearLeftDrive.setPower(RearLeftPower);
        robot.FrontRightDrive.setPower(FrontRightPower);
        robot.RearRightDrive.setPower(RearRightPower);

        telemetry.addData("FrontLeftPower", FrontLeftPower);
        telemetry.addData("RearLeftPower", RearLeftPower);
        telemetry.addData("FrontRightPower", FrontRightPower);
        telemetry.addData("FrontLeftPower", RearRightPower);

    }
}
//No Jimothy
//Lee
//JIMOTHY