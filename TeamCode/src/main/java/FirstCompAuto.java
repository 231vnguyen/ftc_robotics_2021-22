package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@Autonomous(name = "First Competition Auto", group = "Competition")

public class FirstCompAuto extends LinearOpMode {

    //dynamic variables
    private int autoStart = 0;
    private int autoMode = 0;
    private int initialDelay = 0;

    private final double    wheelDiameter = 3.77953,
            wheelGearRatio = 1.0,
            wheelMotorTicks = 383.6,
            wheelRPM = 435,
            wheelMaxVelocity = wheelRPM * wheelMotorTicks / 60,
            botDiagonal = 1.4 * Math.sqrt(Math.pow(16.5, 2) + Math.pow(13.375, 2)),
            avgPower = .4;

    //create imu object
    private BNO055IMU imu;
    //
    private double  currentHeading = 0,
            targetHeading = 0;

    private final double servoLeftStart= .7;
    private final double servoLeftEnd= .1;
    private final double servoRightStart = .2;
    private final double servoRightEnd= .75;

    private boolean bumpersPressed = false;
    private int activeGear = 2;//values 1 - 3

    //inverse drive variables
    private boolean xPressed = false;
    private int inverse = 1;

    //create the gear array
    private final double[] gearValues = {.18, .3, .5, 1.0};


    //create general variables
    private double gamepad1LY;
    private double gamepad1LX;
    private double gamepad1RX;

    //create motor/servo objects
    private DcMotorEx flMotor;
    private DcMotorEx frMotor;
    private DcMotorEx blMotor;
    private DcMotorEx brMotor;
    private DcMotor spinny;
    private CRServo intakeServo;

    private Servo rotateIntakeServo;
    private Servo rightDropdown;
    private Servo leftDropdown;

    private double calculateTicks (double inches) {
        return (int)((inches / (Math.PI * wheelDiameter) * wheelMotorTicks) / wheelGearRatio);
    }




    private boolean nearPosition (double correction) {
        return (
                Math.abs(flMotor.getTargetPosition() - flMotor.getCurrentPosition()) < calculateTicks(correction) &&
                        Math.abs(frMotor.getTargetPosition() - frMotor.getCurrentPosition()) < calculateTicks(correction) &&
                        Math.abs(blMotor.getTargetPosition() - blMotor.getCurrentPosition()) < calculateTicks(correction) &&
                        Math.abs(brMotor.getTargetPosition() - brMotor.getCurrentPosition()) < calculateTicks(correction)
        );
    }

    public void spinnyspin(double seconds, double power) {
        spinny.setPower(power);
        sleep((long) seconds * 1000);
        spinny.setPower(0);
    }

    public void driveForward(double inches, double power) {

        //reset encoders
        flMotor.setMode(STOP_AND_RESET_ENCODER);
        frMotor.setMode(STOP_AND_RESET_ENCODER);
        blMotor.setMode(STOP_AND_RESET_ENCODER);
        brMotor.setMode(STOP_AND_RESET_ENCODER);

        double ticks = (int) inches / (Math.PI * 4);

        ticks *= wheelMotorTicks;

        //set target position
        flMotor.setTargetPosition((int) -ticks);
        frMotor.setTargetPosition((int) -ticks);
        blMotor.setTargetPosition((int) -ticks);
        brMotor.setTargetPosition((int) -ticks);

        //set power
        flMotor.setPower(power);
        frMotor.setPower(power);
        blMotor.setPower(power);
        brMotor.setPower(power);

        //set to RUN_TO_POSITION
        flMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        blMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        brMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        while (flMotor.isBusy()){

        }

        //Stop Motors
        flMotor.setPower(0);
        frMotor.setPower(0);
        blMotor.setPower(0);
        brMotor.setPower(0);

        //Reset Encoders
        flMotor.setMode(STOP_AND_RESET_ENCODER);
        frMotor.setMode(STOP_AND_RESET_ENCODER);
        blMotor.setMode(STOP_AND_RESET_ENCODER);
        brMotor.setMode(STOP_AND_RESET_ENCODER);


        flMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        blMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        brMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



    }

    public void strafe(double inches, double power) {

        //reset encoders
        flMotor.setMode(STOP_AND_RESET_ENCODER);
        frMotor.setMode(STOP_AND_RESET_ENCODER);
        blMotor.setMode(STOP_AND_RESET_ENCODER);
        brMotor.setMode(STOP_AND_RESET_ENCODER);

        double ticks = (int) inches / (Math.PI * 4);

        ticks *= wheelMotorTicks;

        //set target position
        flMotor.setTargetPosition((int)  -ticks);
        frMotor.setTargetPosition((int)  ticks);
        blMotor.setTargetPosition((int)  ticks);
        brMotor.setTargetPosition((int)  -ticks);

        //set power
        flMotor.setPower(power);
        frMotor.setPower(power);
        blMotor.setPower(power);
        brMotor.setPower(power);

        //set to RUN_TO_POSITION
        flMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        blMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        brMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        while (flMotor.isBusy()){

        }

        //Stop Motors
        flMotor.setPower(0);
        frMotor.setPower(0);
        blMotor.setPower(0);
        brMotor.setPower(0);

        //Reset Encoders
        flMotor.setMode(STOP_AND_RESET_ENCODER);
        frMotor.setMode(STOP_AND_RESET_ENCODER);
        blMotor.setMode(STOP_AND_RESET_ENCODER);
        brMotor.setMode(STOP_AND_RESET_ENCODER);


        flMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        blMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        brMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



    }

    public void sleepDelay(double seconds) {
        flMotor.setPower(0);
        flMotor.setPower(0);
        flMotor.setPower(0);
        flMotor.setPower(0);
        sleep((long) seconds);


    }
    public void rotate(double degrees, double power) {

        //reset encoders
        flMotor.setMode(STOP_AND_RESET_ENCODER);
        frMotor.setMode(STOP_AND_RESET_ENCODER);
        blMotor.setMode(STOP_AND_RESET_ENCODER);
        brMotor.setMode(STOP_AND_RESET_ENCODER);

        double ticks = (((int) degrees / (Math.PI * 4))/4.1);

        ticks *= wheelMotorTicks;

        //set target position
        flMotor.setTargetPosition((int) -ticks);
        frMotor.setTargetPosition((int)  ticks);
        blMotor.setTargetPosition((int) -ticks);
        brMotor.setTargetPosition((int)  ticks);

        //set power
        flMotor.setPower(power);
        frMotor.setPower(power);
        blMotor.setPower(power);
        brMotor.setPower(power);

        //set to RUN_TO_POSITION
        flMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        blMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        brMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        while (flMotor.isBusy()){

        }

        //Stop Motors
        flMotor.setPower(0);
        frMotor.setPower(0);
        blMotor.setPower(0);
        brMotor.setPower(0);

        //Reset Encoders
        flMotor.setMode(STOP_AND_RESET_ENCODER);
        frMotor.setMode(STOP_AND_RESET_ENCODER);
        blMotor.setMode(STOP_AND_RESET_ENCODER);
        brMotor.setMode(STOP_AND_RESET_ENCODER);


        flMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        blMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        brMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void intakePosition(double position) {
        rotateIntakeServo.setPosition(position);

    }

    public void intakePower(double power, double seconds) {
        intakeServo.setPower(power);
        sleep((long) (seconds * 1000));
    }

    @Override
    public void runOpMode() throws InterruptedException {


        //initialize imu parameters
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.loggingEnabled = false;
        //imu hardware map
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);



        //create variable to store visual information
        String[] display = {"Carousel-Blue", "Warehouse-Blue", "Carousel-Red",
                "Warehouse-Red", "Park", "Deliver Freight and Park"};
        //for loop to allow selection of autonomous situation
        for (boolean buttonPressed = false; !gamepad2.a && !opModeIsActive() && !isStopRequested();) {

            //change starting line
            if (gamepad2.dpad_left && autoStart > 0 && !buttonPressed) {
                autoStart--;
                buttonPressed = true;
            } else if (gamepad2.dpad_right && autoStart < 3 && !buttonPressed) {
                autoStart++;
                buttonPressed = true;
            //change automode
            }  /*else if (gamepad1.dpad_down && autoMode > 0 && !buttonPressed) {
                autoMode--;
                buttonPressed = true;
            } else if (gamepad1.dpad_up && autoMode < 1 && !buttonPressed) {
                autoMode++;
                buttonPressed = true;
            }*/
            //change initial delay
            else if (gamepad2.dpad_down && initialDelay > 0 && !buttonPressed) {
                initialDelay--;
                buttonPressed = true;
            } else if (gamepad2.dpad_up && initialDelay < 25 && !buttonPressed) {
                initialDelay++;
                buttonPressed = true;
            }
            //choose which autonomous mode
            else if (gamepad2.left_bumper && autoMode > 0 && !buttonPressed) {
                autoMode--;
                buttonPressed = true;
            } else if (gamepad2.right_bumper && autoMode < 1 && !buttonPressed) {
                autoMode++;
                buttonPressed = true;
            }

            //wait until buttons are not pressed
            else if (buttonPressed && !gamepad2.dpad_left && !gamepad2.dpad_right && !gamepad2.dpad_up &&
                    !gamepad2.dpad_down && !gamepad2.left_bumper && !gamepad2.right_bumper)
                buttonPressed = false;

            //output telemetry
            telemetry.addData("Auto Start (Right/Left Dpad)", display[autoStart]);
            telemetry.addData("Auto Mode (Bumpers)", display[autoMode + 4]);
            telemetry.addData("initialDelay (Up/Down Dpad)", initialDelay);

            //robot position notes
            if (autoStart == 0 && autoMode == 0) //CAROUSEL BLUE SPIN AND PARK
                telemetry.addLine("Spinny - Bottom Right");
            else if (autoStart == 1 && autoMode == 0) //WAREHOUSE BLUE PARK
                telemetry.addLine("Spinny - Bottom Left");
            else if (autoStart == 2 && autoMode == 0) //CAROUSEL RED SPIN AND PARK
                telemetry.addLine("Spinny - Bottom Left");
            else if (autoStart == 3 && autoMode == 0) //WAREHOUSE RED PARK
                telemetry.addLine("Spinny - Top Right");




            //
            telemetry.addLine("");
            //imu telemetry
            if (imu.isGyroCalibrated())
                telemetry.addLine("IMU calibrated");

            telemetry.update();

        }

        //setup driving motors
        flMotor = hardwareMap.get(DcMotorEx.class, "flMotor");
        frMotor = hardwareMap.get(DcMotorEx.class, "frMotor");
        blMotor = hardwareMap.get(DcMotorEx.class, "blMotor");
        brMotor = hardwareMap.get(DcMotorEx.class, "brMotor");
        spinny = hardwareMap.get(DcMotorEx.class, "spinny");
        intakeServo = hardwareMap.get(CRServo.class, "intakeServo");

        rotateIntakeServo = hardwareMap.get(Servo.class, "rotateIntakeServo");
        rightDropdown = hardwareMap.get(Servo.class, "rightDropdown");
        leftDropdown = hardwareMap.get(Servo.class, "leftDropdown");


        rightDropdown.setDirection(Servo.Direction.REVERSE);
        leftDropdown.setDirection(Servo.Direction.FORWARD);
        //flip right motors
        flMotor.setDirection(DcMotor.Direction.FORWARD);
        frMotor.setDirection(DcMotor.Direction.REVERSE);
        blMotor.setDirection(DcMotor.Direction.FORWARD);
        brMotor.setDirection(DcMotor.Direction.REVERSE);
        //reset wheel encoders
        flMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        blMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //set target position for driving motors to avoid program crash
        flMotor.setTargetPosition(0);
        frMotor.setTargetPosition(0);
        blMotor.setTargetPosition(0);
        brMotor.setTargetPosition(0);

        flMotor.setVelocity(0);
        frMotor.setVelocity(0);
        blMotor.setVelocity(0);
        brMotor.setVelocity(0);
        //set driving motors to run to position
        flMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        blMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        brMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        flMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        blMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        brMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        //intake Servo
        intakePosition(.4);
        leftDropdown.setPosition(.1);
        rightDropdown.setPosition(.1);


        //TODO, basic park / spin carousel

        if (autoStart == 0 && autoMode == 0) { //Carousel Blue, spin and park
            sleep(initialDelay * 1000);

            driveForward(10, .5);
            strafe(30, .5);
            strafe(3, .3);
            spinnyspin(4, .3);
            driveForward(23, .5);
            strafe(7, .5);
//            driveForward(-10, .5);

        }
        else if (autoStart == 1 && autoMode == 0) { //Warehouse Blue, spin and park
            sleep(initialDelay * 1000);
            driveForward(-30, .5);
            strafe(-25, .5);
//            sleep(initialDelay * 1000);
//            driveForward(19, .5);
//            rotate(90, .5);
//            driveForward(-45, .7);

        }
        else if (autoStart == 2 && autoMode == 0) { //Carousel Red, spin and park
            sleep(initialDelay * 1000);
            strafe(-5, .5);
            driveForward(-16, .5);
            driveForward(-7, .5);
            spinnyspin(4, -.3);
            strafe(-24, .5);
            driveForward(-7, .5);

        }
        else if (autoStart == 3 && autoMode == 0) { //Warehouse Red, spin and park
            sleep(initialDelay * 1000);
            driveForward(-45, .5);
            strafe(25, .5);

        }

        //TODO Deliver Freight and Park
          else if (autoStart == 0 && autoMode == 1) { //Carousel Blue Deliver
              //negative intakePower is intake
             //positive is outtake

              intakePosition(.2);
              sleep(500);
              intakePower(1,2);



        } else if (autoStart == 1 && autoMode == 1) { //Warehouse Blue Deliver

        } else if (autoStart == 2 && autoMode == 1) { //Carousel Red Deliver

        } else if (autoStart == 3 && autoMode == 1) { //Warehouse Red Deliver

        }








    }
}
