package Archive;

import java.sql.Time;
import java.util.*;
import java.util.concurrent.TimeUnit;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

@Disabled
@TeleOp(name = "Control", group = "Testing")

//@Disabled

public class Control extends LinearOpMode {

    //create general variables
    private double gamepad1LY;
    private double gamepad1LX;
    private double gamepad1RX;

    private final double wheelMotorTicks = 384.5;
    private final double wheelMotorRPM = 435;
    private final double wheelDiameter = 3.77953;
    private final double botRotationSpeed = .5; //always <= 1
    private final double wheelMaxVelocity = wheelMotorRPM * wheelMotorTicks / 60;

    private boolean bumpersPressed = false;
    private int activeGear = 1;                  //values 1 - 3
    //create the gear array
    private final double[] gearValues = {.18, .3, .5, 1.0};


    //shooter stuff
    //create shooter objects
    private DcMotorEx shooterMotor;
    private Servo shooterServo;

    public static double powershotTurnR = 2.5;
    public static double powershotTurnL = 2.9;
    private final double powerShotPower = .38;
    private final double shooterTicks = 28;
    private final double shooterRPM = 6000;   //measured in inches
    private final double shooterMaxVelocity = shooterRPM * shooterTicks / 60;
    //shooter timing
    private ElapsedTime shooterTime = new ElapsedTime();
    private int shooterDelayStart = 600;
    //changable shooter values
    private int shooterDelayAssignment = 400;
    private int shooterDelayServo = 300;

    //create motor objects
    private DcMotorEx flMotor;
    private DcMotorEx frMotor;
    private DcMotorEx blMotor;
    private DcMotorEx brMotor;


    //create wobble objects
    private DcMotorEx wobbleMotor;
    private Servo wobbleServo;
    //create starting and ending values for servo
    private final double shooterServoStart = .1;
    private final double shooterServoEnd = .25;
    //create wobble variables
    private final double wobbleTicks = 288;
    private final double wobbleGearRatio = 125 / 45.0; // number >= 1
    private final double wobbleMinimumPower = .15;
    private final double wobbleMaximumPower = .6;
    //wobble positions
    private int wobblePosition = 0;
    private final double[] wobbleValues = {
            0,
            70 / 360.0 * wobbleTicks * wobbleGearRatio,
            150 / 360.0 * wobbleTicks * wobbleGearRatio,
            175 / 360.0 * wobbleTicks * wobbleGearRatio
    };
    //wobble toggle booleans
    private boolean wobblePlayerControlled = false;
    private boolean wobbleMoving = false;
    private boolean wobbleClaw = false;
    private boolean wobbleSwitch = false;
    private boolean wobbleOpen = false;

    //create conveyor objects
    private DcMotorEx conveyorUpper;
    private DcMotorEx conveyorLower;
    //create conveyor variables
    private double conveyorTicks = 288;
    private double conveyorRPM = 125;
    private double conveyorMaxVelocity = conveyorRPM * conveyorTicks / 60;
    private boolean conveyorToggle = false;
    private boolean conveyorForward = false;
    private double conveyorDelay = 200;
    private ElapsedTime conveyorTime = new ElapsedTime();

    //create tape servo object
    private Servo tapeServo;

    private double[] shooterSpeed = {.465, .46};

    //PID
    //shooter pid
    FtcDashboard dashboard;
    ElapsedTime shooterPIDFTimer = new ElapsedTime();
    VoltageSensor batteryVoltageSensor; //to find PID.f
    double integral = 0;
    private static PIDFCoefficients shooterPIDF = new PIDFCoefficients(33, 0, .01, 13.3);
    private double TARGET_VELOCITY = .465;

    //wobble pid
    ElapsedTime wobblePIDFTimer = new ElapsedTime();
    private static PIDFCoefficients wobblePIDF = new PIDFCoefficients(40, 0, 0, 30);




    //method to shoot three rings
    private void shoot3Rings() {

        shooterTime.reset();
        //shooterTime.startTime();

        new Thread(
                new Runnable() {
                    @Override
                    public void run() {

                        //create variable to store time passed
                        double shooterTimeElapsed = shooterDelayStart;
                        shooterMotor.setVelocityPIDFCoefficients(shooterPIDF.p, shooterPIDF.i, shooterPIDF.d, shooterPIDF.f);


                        //for loop to create countable value for servo fires
                        for (int i = 0; i < 3 || shooterTime.milliseconds() - shooterTimeElapsed < 300 + shooterDelayServo && opModeIsActive();) {

                            //turn on PID loop
                            if (i < 3 || shooterTime.milliseconds() - shooterTimeElapsed < shooterDelayServo)
                                //shooterPIDF(TARGET_VELOCITY * shooterTicksPID);
                                shooterMotor.setVelocity(shooterSpeed[0] * shooterMaxVelocity);
                                //turn off PID loop
                            else if (i >= 3 && shooterTime.milliseconds() - shooterTimeElapsed > shooterDelayServo)
                                shooterMotor.setVelocity(0);

                            //start servo loop
                            if (shooterTime.milliseconds() > shooterTimeElapsed && i < 3) {
                                shooterServo.setPosition(shooterServoEnd);
                                shooterTimeElapsed += shooterDelayAssignment;
                                i++;
                            }
                            else if (shooterTime.milliseconds() > shooterTimeElapsed - shooterDelayAssignment + shooterDelayServo)
                                shooterServo.setPosition(shooterServoStart);
                        }

                        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    }
                }).start();
    }

    //method to shoot three rings
    private void shoot3RingsMiddle() {

        shooterTime.reset();
        //shooterTime.startTime();

        new Thread(
                new Runnable() {
                    @Override
                    public void run() {

                        //create variable to store time passed
                        double shooterTimeElapsed = shooterDelayStart;
                        shooterMotor.setVelocityPIDFCoefficients(shooterPIDF.p, shooterPIDF.i, shooterPIDF.d, shooterPIDF.f);


                        //for loop to create countable value for servo fires
                        for (int i = 0; i < 3 || shooterTime.milliseconds() - shooterTimeElapsed < 300 + shooterDelayServo && opModeIsActive();) {

                            //turn on PID loop
                            if (i < 3 || shooterTime.milliseconds() - shooterTimeElapsed < shooterDelayServo)
                                //shooterPIDF(TARGET_VELOCITY * shooterTicksPID);
                                shooterMotor.setVelocity(shooterSpeed[1] * shooterMaxVelocity);
                                //turn off PID loop
                            else if (i >= 3 && shooterTime.milliseconds() - shooterTimeElapsed > shooterDelayServo)
                                shooterMotor.setVelocity(0);

                            //start servo loop
                            if (shooterTime.milliseconds() > shooterTimeElapsed && i < 3) {
                                shooterServo.setPosition(shooterServoEnd);
                                shooterTimeElapsed += shooterDelayAssignment;
                                i++;
                            }
                            else if (shooterTime.milliseconds() > shooterTimeElapsed - shooterDelayAssignment + shooterDelayServo)
                                shooterServo.setPosition(shooterServoStart);
                        }

                        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    }
                }).start();
    }

    //method to shoot powershots
    private void powershot1Ring() {
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterMotor.setVelocityPIDFCoefficients(shooterPIDF.p, shooterPIDF.i, shooterPIDF.d, shooterPIDF.f);

        new Thread(
                new Runnable() {
                    @Override
                    public void run() {

                        shooterMotor.setVelocity(.43 * shooterMaxVelocity);//.44
                        sleep(900);

                        shooterServo.setPosition(shooterServoEnd);
                        sleep(200);

                        shooterServo.setPosition(shooterServoStart);
                        sleep(500);

                        shooterServo.setPosition(shooterServoStart);
                        shooterMotor.setVelocity(0);
                    }
                }).start();
    }

    //method to change gears
    private void changeGears () {

        //if statement to change active gear count
        if (gamepad1.right_trigger > 0 && !bumpersPressed && activeGear < gearValues.length - 1) {

            //set boolean variable to true to avoid multiple inputs
            bumpersPressed = true;
            //add 1 to active gear
            activeGear++;

        } else if (gamepad1.left_trigger > 0 && !bumpersPressed && activeGear > 0) {

            //set boolean variable to true to avoid multiple inputs
            bumpersPressed = true;
            //subtract 1 from active gear
            activeGear--;

        } else if (gamepad1.left_trigger == 0 && gamepad1.right_trigger == 0)
            bumpersPressed = false;

    }

    //method to move wobble arm
    private void moveWobble () {
        wobbleMotor.setVelocityPIDFCoefficients(wobblePIDF.p, wobblePIDF.i, wobblePIDF.d, wobblePIDF.f);
        //----------------------------------------------------------------------
        //move arm based on different mode
        if (wobblePlayerControlled) {

            //allow the player two gamepad to increase move wobble by changing tick value
            if (-gamepad2.left_stick_y > 0)
                wobbleMotor.setTargetPosition(wobbleMotor.getCurrentPosition() + 50);
            else if (-gamepad2.left_stick_y < 0)// && wobbleMotor.getCurrentPosition() > wobbleValues[0] + 50)
                wobbleMotor.setTargetPosition(wobbleMotor.getCurrentPosition() - 50);
            else if (gamepad2.left_stick_y == 0)
                wobbleMotor.setTargetPosition(wobbleMotor.getCurrentPosition());

        } else {
            //move wobble arm based on preset values
            wobbleMotor.setTargetPosition((int) wobbleValues[wobblePosition]);
            //wobblePIDF(wobbleValues[wobblePosition]);
        }
        //----------------------------------------------------------------------
        //move wobble claw
        if (gamepad2.y && !wobbleOpen && !wobbleClaw) {
            //open wobble
            wobbleServo.setPosition(1);
            wobbleClaw = true;
            wobbleOpen = true;
        } else if (gamepad2.y && wobbleOpen && !wobbleClaw) {
            //close wobble
            wobbleServo.setPosition(0);
            wobbleClaw = true;
            wobbleOpen = false;
        } else if (!gamepad2.y && wobbleClaw)
            wobbleClaw = false;

        //move claw based on position
        if (wobbleMotor.getCurrentPosition() < wobbleValues[1] * .85) {
            wobbleServo.setPosition(0);
            wobbleOpen = false;
        } else if (wobbleMotor.getCurrentPosition() < wobbleValues[1] * 1.2 && wobbleMotor.getCurrentPosition() > wobbleValues[1] * .85 && wobbleOpen) {
            wobbleServo.setPosition(.5);
            wobbleOpen = true;
        } else if (wobbleMotor.getCurrentPosition() > wobbleValues[1] && wobbleOpen)
            wobbleServo.setPosition(1);
        //----------------------------------------------------------------------
        //increase and decrease current position
        if (gamepad2.dpad_up && wobblePosition > 0 && !wobbleMoving) {
            //increase position
            wobblePosition--;
            wobbleMoving = true;
        } else if (gamepad2.dpad_down && wobblePosition < 3 && !wobbleMoving) {
            //decrease position
            wobblePosition++;
            wobbleMoving = true;
        } else if (!gamepad2.dpad_up && !gamepad2.dpad_down && wobbleMoving)
            wobbleMoving = false;

        //----------------------------------------------------------------------
        //toggle wobble control to and from player
        if (gamepad2.x && !wobbleSwitch && !wobblePlayerControlled) {
            //make wobble player controlled
            wobblePlayerControlled = true;
            wobbleSwitch = true;
        } else if (gamepad2.x && !wobbleSwitch && wobblePlayerControlled) {
            //remove player control from wobble
            wobblePlayerControlled = false;
            wobbleSwitch = true;
        } else if (wobbleSwitch && !gamepad2.x)
            wobbleSwitch = false;

        if (gamepad2.b) {

            wobbleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }

        //----------------------------------------------------------------------
        wobbleMotor.setVelocity(wobbleTicks * 1);

    }

    private void wobblePIDF(double targetPosition) {

        FtcDashboard dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();
        double error = wobbleMotor.getCurrentPosition();
        double lastError = 0;


        error = wobbleMotor.getCurrentPosition() - targetPosition;
        double changeInError = lastError - error;
        integral += changeInError * wobblePIDFTimer.time();
        double derivative = changeInError / wobblePIDFTimer.time();

        //P is the proportion that multiplies by the error to accelerate the velocity proportional to error
        double P = wobblePIDF.p * error;

        //I is the intregal that multiplies by the changeInError and the time to remove constant error
        double I = wobblePIDF.i * integral;

        //D is the derivative that multiplies by the changeInError/time to correct the minimize overshooting
        double D = wobblePIDF.d * derivative;

        //F is the feedforward controller that multiplies by the targetVelocity and then divided by the battery voltage to keep
        //the velocity on targetVelocity
        double F = (targetPosition * wobblePIDF.f) / batteryVoltageSensor.getVoltage();      //formula    voltage = targetvelocity * PID.f

        wobbleMotor.setPower(P+I+D+F);
        lastError = error;
        wobblePIDFTimer.reset();

        packet.put("CurrentPosition", wobbleMotor.getCurrentPosition());
        packet.put("TargetPosition", wobbleValues[wobblePosition]);
        dashboard.sendTelemetryPacket(packet);
    }
    private void shooterPIDF(double targetVelocity) {

        double error = shooterMotor.getVelocity();
        double lastError = 0;
        FtcDashboard dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();

        error = shooterMotor.getVelocity() - targetVelocity;
        double changeInError = lastError - error;
        integral += changeInError * shooterPIDFTimer.time();
        double derivative = changeInError / shooterPIDFTimer.time();

        //P is the proportion that multiplies by the error to accelerate the velocity proportional to error
        double P = shooterPIDF.p * error;

        //I is the intregal that multiplies by the changeInError and the time to remove constant error
        double I = shooterPIDF.i * integral;

        //D is the derivative that multiplies by the changeInError/time to correct the minimize overshooting
        double D = shooterPIDF.d * derivative;

        //F is the feedforward controller that multiplies by the targetVelocity and then divided by the battery voltage to keep
        //the velocity on targetVelocity
        double F = (targetVelocity * shooterPIDF.f) / batteryVoltageSensor.getVoltage();      //formula    voltage = targetvelocity * PID.f

        shooterMotor.setVelocity(P + I + D + F);
        lastError = error;
        shooterPIDFTimer.reset();

        packet.put("CurrentVelocity", shooterMotor.getVelocity());
        packet.put("TargetVelocity", TARGET_VELOCITY * shooterMaxVelocity);
        dashboard.sendTelemetryPacket(packet);


    }

    private void turn(double inches) {
        //variables
        double power = .2;
        double ticks = (int) inches / (Math.PI * 3.93701) * wheelMotorTicks;
        //reset encoders
        flMotor.setMode(STOP_AND_RESET_ENCODER);
        frMotor.setMode(STOP_AND_RESET_ENCODER);
        blMotor.setMode(STOP_AND_RESET_ENCODER);
        brMotor.setMode(STOP_AND_RESET_ENCODER);
        //set init position
        flMotor.setTargetPosition(0);
        frMotor.setTargetPosition(0);
        blMotor.setTargetPosition(0);
        brMotor.setTargetPosition(0);
        //set mode
        flMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        blMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        brMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //set target position
        flMotor.setTargetPosition((int) -ticks);
        frMotor.setTargetPosition((int)  ticks);
        blMotor.setTargetPosition((int) -ticks);
        brMotor.setTargetPosition((int)  ticks);
        //setting power
        flMotor.setVelocity(power * wheelMaxVelocity);
        frMotor.setVelocity(power * wheelMaxVelocity);
        blMotor.setVelocity(power * wheelMaxVelocity);
        brMotor.setVelocity(power * wheelMaxVelocity);

        //while loop to run to position
        while (flMotor.isBusy() && frMotor.isBusy() && blMotor.isBusy() && brMotor.isBusy()) {}
        //Stop Motors
        flMotor.setVelocity(0);
        frMotor.setVelocity(0);
        blMotor.setVelocity(0);
        brMotor.setVelocity(0);
        //setting RUN_WITHOUT_ENCODERS mode again, to continue running without encoders
        flMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        blMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        brMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void runOpMode() throws InterruptedException {


        //PID and Dashboard
        dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        //TODO organize
        //setup driving motors
        flMotor = hardwareMap.get(DcMotorEx.class, "flMotor");
        frMotor = hardwareMap.get(DcMotorEx.class, "frMotor");
        blMotor = hardwareMap.get(DcMotorEx.class, "blMotor");
        brMotor = hardwareMap.get(DcMotorEx.class, "brMotor");
        //flip right motors
        frMotor.setDirection(DcMotor.Direction.REVERSE);
        brMotor.setDirection(DcMotor.Direction.REVERSE);

        //shooter
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        shooterServo = hardwareMap.servo.get("shooterServo");

        //setup wobble motor
        wobbleMotor = hardwareMap.get(DcMotorEx.class, "wobbleMotor");
        wobbleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wobbleMotor.setTargetPosition(0);
        wobbleMotor.setPower(0);
        wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wobbleMotor.setDirection(DcMotor.Direction.REVERSE);
        //setup wobble servo
        wobbleServo = hardwareMap.get(Servo.class, "wobbleServo");
        wobbleServo.setDirection(Servo.Direction.REVERSE);

        //setup conveyor motors
        conveyorUpper = hardwareMap.get(DcMotorEx.class, "conveyorUpper");
        conveyorLower = hardwareMap.get(DcMotorEx.class, "conveyorLower");
        //reverse
        conveyorLower.setDirection(DcMotor.Direction.REVERSE);
        //set run mode
        conveyorUpper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        conveyorLower.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //set initial power
        conveyorUpper.setPower(0);
        conveyorLower.setPower(0);

        //setup tape servo
        tapeServo = hardwareMap.get(Servo.class, "tapeServo");

        waitForStart();


        //start time variables
        conveyorTime.startTime();
        shooterTime.startTime();



        //while loop
        while (opModeIsActive()) {

            //call shooter method to get rid of

            //update controller variables
            gamepad1LY = gamepad1.left_stick_y;
            gamepad1LX = gamepad1.left_stick_x;
            gamepad1RX = gamepad1.right_stick_x;

            //event methods
            changeGears();
            moveWobble();


            //TODO test setVelocity for drivetrain
            //if statement to ensure robot won't move unless control sticks are moved
            if (Math.abs(gamepad1LY) > .05 || Math.abs(gamepad1LX) > .05 || Math.abs(gamepad1RX) > .05) {
                //control logic: multiply by gear array for different speeds,
                //multiply right stick movement by rotation fraction to reduce uncontrolability
                flMotor.setVelocity((gearValues[activeGear] * (gamepad1LY - gamepad1LX) - botRotationSpeed * gamepad1RX) * wheelMaxVelocity);
                frMotor.setVelocity((gearValues[activeGear] * (gamepad1LY + gamepad1LX) + botRotationSpeed * gamepad1RX) * wheelMaxVelocity);
                blMotor.setVelocity((gearValues[activeGear] * (gamepad1LY + gamepad1LX) - botRotationSpeed * gamepad1RX) * wheelMaxVelocity);
                brMotor.setVelocity((gearValues[activeGear] * (gamepad1LY - gamepad1LX) + botRotationSpeed * gamepad1RX) * wheelMaxVelocity);
            } else if (gamepad1.left_bumper && !gamepad1.right_bumper && Math.abs(gamepad1LY) < .05 && Math.abs(gamepad1LX) < .05 && Math.abs(gamepad1RX) < .05) {
                flMotor.setVelocity( .2 * wheelMaxVelocity);
                frMotor.setVelocity(-.2 * wheelMaxVelocity);
                blMotor.setVelocity( .2 * wheelMaxVelocity);
                brMotor.setVelocity(-.2 * wheelMaxVelocity);
            } else if (gamepad1.right_bumper && !gamepad1.left_bumper && Math.abs(gamepad1LY) < .05 && Math.abs(gamepad1LX) < .05 && Math.abs(gamepad1RX) < .05) {
                flMotor.setVelocity(-.2 * wheelMaxVelocity);
                frMotor.setVelocity( .2 * wheelMaxVelocity);
                blMotor.setVelocity(-.2 * wheelMaxVelocity);
                brMotor.setVelocity( .2 * wheelMaxVelocity);
            } else {
                flMotor.setVelocity(0);
                frMotor.setVelocity(0);
                blMotor.setVelocity(0);
                brMotor.setVelocity(0);
            }

            //-------------------------------------------
            //shoot 3 rings
            if (gamepad1.y && shooterMotor.getVelocity() == 0)
                shoot3Rings();
                //powershots
            else if(gamepad1.b && shooterMotor.getVelocity() == 0)
                powershot1Ring();
                //middle goal
            else if (gamepad1.x && shooterMotor.getVelocity() == 0)
                shoot3RingsMiddle();

            //TODO Set dpad right and left to move between powershots with encoders and Tune
            if (gamepad1.dpad_right) {
                turn(powershotTurnR); //2.5
            } else if (gamepad1.dpad_left) {
                turn(-powershotTurnL); //3
            }


            //if statements to run conveyor
            if (gamepad2.a && !conveyorToggle) {
                //activate timer
                conveyorTime.reset();
                conveyorToggle = true;
            } else if (!gamepad2.a && conveyorToggle && conveyorTime.milliseconds() < conveyorDelay) {
                //toggle conveyor on
                if (!conveyorForward) {
                    conveyorUpper.setPower(1);
                    conveyorLower.setPower(1);
                    conveyorToggle = false;
                    conveyorForward = true;
                } else {
                    conveyorUpper.setPower(0);
                    conveyorLower.setPower(0);
                    conveyorToggle = false;
                    conveyorForward = false;
                }
            } else if (gamepad2.a && conveyorToggle && conveyorTime.milliseconds() >= conveyorDelay) {
                //toggle conveyor reverse
                conveyorUpper.setPower(-1);
                conveyorLower.setPower(-1);
            } else if (!gamepad2.a && conveyorToggle && conveyorTime.milliseconds() >= conveyorDelay) {
                //turn off conveyor
                conveyorUpper.setPower(0);
                conveyorLower.setPower(0);
                conveyorToggle = false;
                conveyorForward = false;
            }

            //move tape servo with gamepad two left stick y
            if (-gamepad2.right_stick_y > .1)
                tapeServo.setPosition(1);
            else if (-gamepad2.right_stick_y < -.1)
                tapeServo.setPosition(0);
            else
                tapeServo.setPosition(.5);

            //vary shooter with player two right stick
            //shootVary = .05 * -gamepad2.right_stick_y;


            //send what gear is active
            telemetry.addData("Gear", activeGear + 1);
            telemetry.addData("Wobble Position", wobblePosition + 1);
            telemetry.addData("Shooter Time", shooterTime.milliseconds());
            telemetry.addData("Servo moving", tapeServo.getPosition());
            telemetry.update();

        }

    }

}