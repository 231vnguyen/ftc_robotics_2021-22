package Archive;/*
package org.firstinspires.ftc.teamcode.Archive;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Archive.reliableOpenCV;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Arrays;

import static java.lang.Math.abs;
@Disabled
@Autonomous(name = "Competition Autonomous", group = "Competition")
public class CompetitionAutonomous extends LinearOpMode {

    //editable variables
    private int zone = 0;
    private int autoMode = 0;
    private int initialDelay = 0;
    private static int startingLine = 0;


    //create general drive train variables
    private DcMotorEx flMotor;
    private DcMotorEx frMotor;
    private DcMotorEx blMotor;
    private DcMotorEx brMotor;
    //
    private final double    wheelDiameter = 3.77953,
            wheelGearRatio = 1.0,
            wheelMotorTicks = 383.6,
            wheelRPM = 435,
            wheelMaxVelocity = wheelRPM * wheelMotorTicks / 60,
            botDiagonal = 1.4 * Math.sqrt(Math.pow(16.5, 2) + Math.pow(13.375, 2)),
            avgPower = .4;
    //
    private final double movingError = 2;
    private double correctDriving = movingError;


    private final double powershotPower = .1;
    private final double intakePower = .2;

    private final double powershotDistance = 7.5;
    private final double[] powershotTuning = {0.4, 6.6, 12.2};
    //------------------------------------------------------------------------------------------------------------
    //create shooter object
    private DcMotorEx shooterMotor;
    private final static PIDFCoefficients shooterPIDF = new PIDFCoefficients(33, 0, .01, 13.3);
    //
    private final double[] shooterVelocity = {.462,   .422};
    private final double    shooterTicks = 28,
            shooterRPM = 6000;
    private final double shooterMaxVelocity = shooterRPM * shooterTicks / 60;
    //create shooter servo object
    private Servo shooterServo;
    private VoltageSensor batteryVoltage;
    //
    private final double    shooterServoStart = .1,
            shooterServoEnd = .25;
    //
    private final double    voltagePower = .0048,
            voltageMax = 14.1;
    private double          voltageInitial = 0;
    //
    private final int shooterDelayServo = 300;
    //------------------------------------------------------------------------------------------------------------
    //create tape servo object
    private Servo tapeServo;
    //
    private boolean tapeMoving = false;
    //------------------------------------------------------------------------------------------------------------
    //create conveyor objects
    private DcMotor conveyorUpper;
    private DcMotor conveyorLower;
    //------------------------------------------------------------------------------------------------------------
    //create imu object
    private BNO055IMU imu;
    //
    private double  currentHeading = 0,
            targetHeading = 0;
    //------------------------------------------------------------------------------------------------------------
    //lists to store current running autonomous
    private ArrayList<Object[]> runningAutonomous = new ArrayList<>();
    private ArrayList<Integer> positionsX = new ArrayList<>();
    private ArrayList<Integer> positionsY = new ArrayList<>();
    private ArrayList<String[]> actions = new ArrayList<>();
    //autonomous positioning
    private Object[][][] autonomous = new Object[][][] {
            {//Blue-Left
                    //slow down
                    {0, 2 * 24 + 3, avgPower,
                            new String[]{"startShoot:" + shooterVelocity[0]}},
                    //shoot
                    {0, 2 * 24 + 9, avgPower / 3.0,
                            new String[]{"i-correctDriving:.5", "wobbleClose",
                                    "e-rotate:-24", "e-shoot", "e-rotate:0", "e-wobbleDown"}},
                    //wobble
                    {new Integer[]{-5, 0, 5}, new Integer[]{4 * 24 - 7, 4 * 24, 4 * 24 + 2}, avgPower,
                            new String[][]{
                                    {"e-wobbleOpen", "e-wait:150"},
                                    {"e-rotate:90", "e-wobbleOpen", "e-wait:150", "e-wobbleGround", "e-wait:200", "e-rotate:0"},
                                    {"e-rotate:230", "e-wobbleOpen", "e-wait:150", "e-wobbleGround", "e-wait:200", "e-rotate:0"}
                            }},
                    //park
                    {-6, 4 * 24, avgPower,
                            new String[]{"wait:400","wobbleClose", "wait:200", "wobbleReset", "tapeServo:5500"}}
            },
            {//Blue-Right
                    //slow down
                    {0, 2 * 24, avgPower,
                            new String[]{"startShoot:" + shooterVelocity[1]}},
                    //powershot
                    {0, 2 * 24 + 8, avgPower / 3.0,
                            new String[]{"i-correctDriving:.6", "wobbleClose", "powershotPosition",
                                    "e-rotate:-8.35", "e-shootPowershots", "e-wobbleDown", "e-wobbleDown"}},
                    //wobble
                    {new Integer[]{-25, -20, -26}, new Integer[]{3 * 24 + 2, 5 * 24 - 10, 5 * 24 - 4}, avgPower,
                            new String[][]{}},
                    //clear wobble
                    {new Integer[]{-12, 2, -12}, new Integer[]{3 * 24 + 6, 5 * 24 - 6, 5 * 24}, avgPower,
                            new String[][]{
                                    {"i-rotate:-90", "i-wobbleOpen", "i-wait:150", "i-wobbleGround", "i-wait:200", "i-rotate:0", "tapeServo:5500"},
                                    {"i-wobbleOpen", "wait:150", "wobbleUp", "tapeServo:5500"},
                                    {"i-rotate:-110", "i-wobbleOpen", "i-wait:150", "i-wobbleGround", "i-wait:200", "i-rotate:0", "tapeServo:5500"}
                            }},
                    {6, 4 * 24 + 4, avgPower,
                            new String[]{"wobbleClose", "wait:200", "wobbleReset"}}
            },
            {//Red-Left
                    //slow down
                    {0, 2 * 24, avgPower,
                            new String[]{"startShoot:" + shooterVelocity[1]}},
                    //powershot
                    {0, 2 * 24 + 8, avgPower / 3.0,
                            new String[]{"i-correctDriving:.6", "wobbleClose", "powershotPosition",
                                    "e-rotate:-13.5", "e-shootPowershots", "e-wobbleDown", "e-wobbleDown"}},
                    //wobble
                    {new Integer[]{24, 24, 30}, new Integer[]{3 * 24 + 6, 5 * 24 - 6, 5 * 24 - 4}, avgPower,
                            new String[][]{}},
                    //clear wobble
                    {new Integer[]{12, -2, 12}, new Integer[]{3 * 24 + 6, 5 * 24 - 6, 5 * 24}, avgPower,
                            new String[][]{
                                    {"i-rotate:90", "i-wobbleOpen", "i-wait:150", "i-wobbleGround", "i-wait:100", "i-rotate:0", "tapeServo:5500"},
                                    {"i-wobbleOpen", "wait:150", "wobbleUp", "tapeServo:5500"},
                                    {"i-rotate:110", "i-wobbleOpen", "i-wait:150", "i-wobbleGround", "i-wait:100", "i-rotate:0", "tapeServo:5500"}
                            }},
                    {-6, 4 * 24 + 4, avgPower,
                            new String[]{"wobbleClose", "wait:200", "wobbleReset"}}
            },
            {//Red-Right
                    //slow down
                    {0, 2 * 24 + 3, avgPower,
                            new String[]{"startShoot:" + shooterVelocity[0]}},
                    //shoot
                    {0, 2 * 24 + 9, avgPower,
                            new String[]{"i-correctDriving:.5", "wobbleClose",
                                    "e-rotate:3.5", "e-shoot", "e-rotate:0", "e-wobbleDown"}},
                    //wobble
                    {new Integer[]{5, 0, 0}, new Integer[]{4 * 24 - 7, 4 * 24 - 10, 4 * 24}, avgPower,
                            new String[][]{
                                    {"e-wobbleOpen", "e-wait:200"},
                                    {"e-rotate:-90", "e-wobbleOpen", "e-wait:150", "e-wobbleGround", "e-wait:100", "e-rotate:0"},
                                    {"e-rotate:-180", "e-wobbleOpen", "e-wait:150", "e-wobbleGround", "e-wait:100", "e-rotate:0"}
                            }},
                    //park
                    {3, 4 * 24, avgPower,
                            new String[]{"wait:400", "wobbleClose", "wait:200", "wobbleReset", "tapeServo:5500"}}
            },
            {//intake information
                    //move to intake
                    {
                            new Object[]{
                                    12, 2 * 24, avgPower,
                                    new String[]{}},
                            new Object[]{
                                    -8, 2 * 24, avgPower,
                                    new String[]{}},
                            new Object[]{
                                    8, 2 * 24, avgPower,
                                    new String[]{}},
                            new Object[]{
                                    -12, 2 * 24, avgPower,
                                    new String[]{}}
                    },
                    //start intaking and shooting
                    {
                            new Object[]{
                                    new Integer[]{}, new Integer[]{}, intakePower,
                                    new String[][]{
                                            {"i-intakeOn", ""},
                                            {"i-intakeOn"},
                                            {"i-intakeOn"}
                                    }}
                    }
            },
            {//shoot-park information
                    {2, 1, 1, 2},
                    //stuff for final
                    {0, 2 * 24, new String[]{"tapeServo:5500", "e-rotate:things"}}
            }
    };
    //
    private int arrayPosition = 0;
    private int powershotPosition = 0;
    private int index = 0;
    //------------------------------------------------------------------------------------------------------------
    //create wobble objects
    private DcMotorEx wobbleMotor;
    private final static PIDFCoefficients wobblePIDF = new PIDFCoefficients(40, 0, 0, 30);
    //
    private final double    wobbleTicks = 288,
            wobbleRPM = 125,
            wobbleGearRatio = 125 / 45.0,
            wobbleMaxVelocity = wobbleRPM * wobbleTicks / 60;
    //
    private final double[] wobbleValues = {
            0,
            70  / 360.0 * wobbleTicks * wobbleGearRatio,
            132 / 360.0 * wobbleTicks * wobbleGearRatio,
            140 / 360.0 * wobbleTicks * wobbleGearRatio
    };
    //create servo object
    private Servo wobbleServo;
    //------------------------------------------------------------------------------------------------------------
    //create webcam and pipeline objects
    OpenCvCamera webcam;
    UltimateGoalPipeline pipeline;
    //camera anchors
    private static final int[] xAnchor1 = {108, 10, 108, 10}; //blueLeft, blueRight, redLeft, redRight
    private static final int[] yAnchor1 = {250, 250, 250, 250};
    //
    private  static final int[] xAnchor2 = {108, 10, 108, 10}; //raising values moves it to the right
    private static final int[] yAnchor2 = {205, 200, 200, 200}; //raising value moves it down
    //
    private static final int[] xAnchor3 = {108, 10, 108, 10};
    private static final int[] yAnchor3 = {173, 170, 165, 170};
    //width and height for boxes
    private static final int[] region1Width = {90, 90, 90, 90};
    private static final int[] region1Height = {50, 50, 50, 50};
    //
    private static final int[] region2Width = {90, 90, 90, 90};
    private static final int[] region2Height = {20, 20, 20, 20};
    //
    private static final int[] region3Width = {90, 90, 90, 90};
    private static final int[] region3Height = {25, 25, 25, 25};
    //============================================================================================================
    //============================================================================================================
    //============================================================================================================
    //method to calculate ticks
    private double calculateTicks (double inches) {
        return (int)((inches / (Math.PI * wheelDiameter) * wheelMotorTicks) / wheelGearRatio);
    }

    //method to parse coordinates
    private void robotAction (String action) throws InterruptedException {

        //remove initial string from action
        action = action.replaceAll("i-","").replaceAll("e-", "");

        //set velocity and PID for wobble motor
        wobbleMotor.setVelocityPIDFCoefficients(wobblePIDF.p, wobblePIDF.i, wobblePIDF.d, wobblePIDF.f);
        wobbleMotor.setVelocity(.5 * wobbleMaxVelocity);

        //if statements to determine actions
        if (action.equals("shoot")) {

            //initialize PID loop
            shooterMotor.setVelocityPIDFCoefficients(shooterPIDF.p, shooterPIDF.i, shooterPIDF.d, shooterPIDF.f);
            //set velocity
            shooterMotor.setVelocity( (shooterVelocity[0] + voltagePower * (voltageMax - voltageInitial)) * shooterMaxVelocity);

            //delay before loop
            //Thread.sleep(shooterDelayStart);

            //for loop to create countable value for servo fires
            for (int i = 0; i < 3 && opModeIsActive(); i++) {

                //fire ring
                shooterServo.setPosition(shooterServoEnd);
                //delay
                Thread.sleep(shooterDelayServo);

                //close servo
                shooterServo.setPosition(shooterServoStart);
                //delay
                Thread.sleep(shooterDelayServo);

            }

            //turn off shooter
            shooterMotor.setVelocity(0);

        } else if (action.equals("shootPowershots")) {

            //initialize PID loop
            shooterMotor.setVelocityPIDFCoefficients(shooterPIDF.p, shooterPIDF.i, shooterPIDF.d, shooterPIDF.f);
            //set velocity
            shooterMotor.setVelocity((shooterVelocity[1] + voltagePower * (voltageMax - voltageInitial)) * shooterMaxVelocity);

            //create variables to store position
            double initX = 0, initY = 0, finalX = 0, finalY = 0;
            //initial
            if (runningAutonomous.get(powershotPosition)[0] instanceof Integer) {
                initY = calculateTicks((Integer) runningAutonomous.get(powershotPosition)[1]);
                initX = calculateTicks((Integer) runningAutonomous.get(powershotPosition)[0]);
            } else if (runningAutonomous.get(powershotPosition)[0] instanceof Integer[]) {
                initY = calculateTicks(((Integer[]) runningAutonomous.get(powershotPosition)[1])[zone]);
                initX = calculateTicks(((Integer[]) runningAutonomous.get(powershotPosition)[0])[zone]);
            }
            //final
            if (runningAutonomous.get(powershotPosition + 1)[0] instanceof Integer) {
                finalY = calculateTicks((Integer) runningAutonomous.get(powershotPosition + 1)[1]);
                finalX = calculateTicks((Integer) runningAutonomous.get(powershotPosition + 1)[0]);
            } else if (runningAutonomous.get(powershotPosition + 1)[0] instanceof Integer[]) {
                finalY = calculateTicks(((Integer[]) runningAutonomous.get(powershotPosition + 1)[1])[zone]);
                finalX = calculateTicks(((Integer[]) runningAutonomous.get(powershotPosition + 1)[0])[zone]);
            }

            //create variable to store direction polarity
            double direction = (finalX - initX) /
                    (double) Math.abs(finalX - initX);
            //create variable to store degree movement
            double ticksRotation = calculateTicks( (currentHeading / 360.0) * Math.PI * botDiagonal );

            telemetry.addData("Powershot position", arrayPosition);
            telemetry.update();

            //set velocity for wheel motors
            flMotor.setVelocity(powershotPower * wheelMaxVelocity);
            frMotor.setVelocity(powershotPower * wheelMaxVelocity);
            blMotor.setVelocity(powershotPower * wheelMaxVelocity);
            brMotor.setVelocity(powershotPower * wheelMaxVelocity);

            //

            //for loop to fire rings at intervals
            for (int i = 0; i < 3 && opModeIsActive();) {
                //set postition
                flMotor.setTargetPosition((int) (initY + initX - ticksRotation - direction * calculateTicks(((powershotTuning[i] + .1) / 360.0 ) * Math.PI * botDiagonal)) );
                frMotor.setTargetPosition((int) (initY - initX + ticksRotation + direction * calculateTicks(((powershotTuning[i] + .1) / 360.0 ) * Math.PI * botDiagonal)) );
                blMotor.setTargetPosition((int) (initY - initX - ticksRotation - direction * calculateTicks(((powershotTuning[i] + .1) / 360.0 ) * Math.PI * botDiagonal)) );
                brMotor.setTargetPosition((int) (initY + initX + ticksRotation + direction * calculateTicks(((powershotTuning[i] + .1) / 360.0 ) * Math.PI * botDiagonal)) );

                //if statement to ensure conditional is reached
                if (nearPosition(.46)) {
                    //open servo
                    shooterServo.setPosition(shooterServoEnd);
                    //delay
                    Thread.sleep(shooterDelayServo);
                    //close servo
                    shooterServo.setPosition(shooterServoStart);

                    //increment value
                    i++;
                }
                //delay loop
                Thread.sleep(1);
            }

            //undo rotation
            flMotor.setTargetPosition((int) (initY + initX));
            frMotor.setTargetPosition((int) (initY - initX));
            blMotor.setTargetPosition((int) (initY - initX));
            brMotor.setTargetPosition((int) (initY + initX));

            //wait for motors to reach position
            while (!nearPosition(3)) Thread.sleep(5);
            Thread.sleep(1000);

            //turn off shooter
            shooterMotor.setVelocity(0);

        }
        else if (action.equals("powershotPosition"))
            //set target position
            powershotPosition = arrayPosition;

        else if (action.startsWith("startShoot:"))
            shooterMotor.setVelocity(Double.parseDouble(action.substring(11)) * shooterMaxVelocity);

        else if (action.equals("wobbleDown"))
            //set target position
            wobbleMotor.setTargetPosition((int) wobbleValues[2]);

        else if (action.equals("wobbleUp"))
            //set target position
            wobbleMotor.setTargetPosition((int) wobbleValues[1]);

        else if (action.equals("wobbleReset"))
            //set target position
            wobbleMotor.setTargetPosition((int) wobbleValues[0]);

        else if (action.equals("wobbleGround"))
            //set target position
            wobbleMotor.setTargetPosition((int) wobbleValues[3]);

        else if (action.equals("wobbleOpen"))
            //open servo
            wobbleServo.setPosition(1);

        else if (action.equals("wobbleClose"))
            //close servo
            wobbleServo.setPosition(0);

        else if (action.equals("waitForWobble")) while (Math.abs(wobbleMotor.getCurrentPosition() - wobbleMotor.getTargetPosition()) > 0 && opModeIsActive()) Thread.sleep(5);

        else if (action.startsWith("wait:"))
            Thread.sleep(Integer.parseInt(action.substring(5)));

        else if (action.startsWith("correctDriving:"))
            correctDriving = Double.parseDouble(action.substring(15));

        else if (action.startsWith("rotate:")) {
            //create variables to store distance travelled and degrees
            double degrees = Double.parseDouble( action.substring(7) );
            double distance = ( (degrees - currentHeading) / 360 ) * (Math.PI * botDiagonal);

            //set velocity for wheels
            flMotor.setVelocity(.75 * avgPower * wheelMaxVelocity);
            frMotor.setVelocity(.75 * avgPower * wheelMaxVelocity);
            blMotor.setVelocity(.75 * avgPower * wheelMaxVelocity);
            brMotor.setVelocity(.75 * avgPower * wheelMaxVelocity);

            //set position for motors
            flMotor.setTargetPosition((int) (flMotor.getTargetPosition() - calculateTicks(distance)) );
            frMotor.setTargetPosition((int) (frMotor.getTargetPosition() + calculateTicks(distance)) );
            blMotor.setTargetPosition((int) (blMotor.getTargetPosition() - calculateTicks(distance)) );
            brMotor.setTargetPosition((int) (brMotor.getTargetPosition() + calculateTicks(distance)) );

            //while loop to delay until finished
            while (!nearPosition(2) && opModeIsActive()) Thread.sleep(5);
            //delay to finish movement
            Thread.sleep(400);

            //set new current heading
            currentHeading = Double.parseDouble( action.substring(7) );

        }

        else if (action.startsWith("tapeServo:")) {
            //create temporary variable to store time
            int time = Integer.parseInt(action.substring(10));
            //new thread to keep robot running
            new Thread(
                    new Runnable() {
                        @Override
                        public void run() {
                            //set servo to moving
                            tapeMoving = true;
                            //start tape measure
                            tapeServo.setPosition(1);
                            //delay custom time
                            try {
                                Thread.sleep(time);
                            } catch (InterruptedException ignored) {
                            }
                            //stop tape measure
                            tapeServo.setPosition(.5);
                            //set servo to stopped
                            tapeMoving = false;
                        }
                    }).start();
        }

        //additional intake actions
        else if (action.equals("intakeOn")) {
            //turn on conveyor motors
            conveyorUpper.setPower(1);
            conveyorLower.setPower(1);
        } else if (action.equals("intakeOff")) {
            //turn off conveyor motors
            conveyorUpper.setPower(0);
            conveyorLower.setPower(0);
        } else if (action.startsWith("shootOne:"));

    }
    //------------------------------------------------------------------------------------------------------------
    //method to calculate inches moved forward
    private void moveRobot (double inchesX, double inchesY, double power) {

        //create used variables
        double powerA = power * wheelMaxVelocity;
        double powerB = power * wheelMaxVelocity;
        int ticksX = (int) calculateTicks(inchesX);
        int ticksY = (int) calculateTicks(inchesY);

        //set distance
        flMotor.setTargetPosition(ticksY + ticksX);
        frMotor.setTargetPosition(ticksY - ticksX);
        blMotor.setTargetPosition(ticksY - ticksX);
        brMotor.setTargetPosition(ticksY + ticksX);

        //power variation
if (Math.abs(inchesY) > Math.abs(inchesX))
            powerA *= (1 - inchesX / inchesY);
        else if (Math.abs(inchesX) > Math.abs(inchesY))
            powerB *= 1 - inchesY / inchesX;


        //set velocity if horizontal movement is greater than vertical
        flMotor.setVelocity(powerA);
        frMotor.setVelocity(powerB);
        blMotor.setVelocity(powerB);
        brMotor.setVelocity(powerA);

    }

    //------------------------------------------------------------------------------------------------------------
    //method to return if zone is correct
    private boolean correctZone (String input) {
        return (input.endsWith("zone:A") && zone == 0) || (input.endsWith("zone:B") && zone == 1) || (input.endsWith("zone:C") && zone == 2);
    }

    //method to test if wheel motors are currently running
    private boolean wheelsRunning () {
        return flMotor.isBusy() || frMotor.isBusy() || blMotor.isBusy() || brMotor.isBusy();
    }
    //method to test if approximate position is reached
    private boolean nearPosition (double correction) {
        return (
                Math.abs(flMotor.getTargetPosition() - flMotor.getCurrentPosition()) < calculateTicks(correction) &&
                        Math.abs(frMotor.getTargetPosition() - frMotor.getCurrentPosition()) < calculateTicks(correction) &&
                        Math.abs(blMotor.getTargetPosition() - blMotor.getCurrentPosition()) < calculateTicks(correction) &&
                        Math.abs(brMotor.getTargetPosition() - brMotor.getCurrentPosition()) < calculateTicks(correction)
        );
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
        String[] display = {"A", "B", "C",
                "Blue-Left", "Blue-Right", "Red-Left", "Red-Right",
                "Normal", "Intake", "Shoot-Park"};
        //for loop to allow selection of autonomous situation
        for (boolean buttonPressed = false; !gamepad1.a && !opModeIsActive() && !isStopRequested();) {

            //change starting line
            if (gamepad1.dpad_left && startingLine > 0 && !buttonPressed) {
                startingLine--;
                buttonPressed = true;
            } else if (gamepad1.dpad_right && startingLine < 3 && !buttonPressed) {
                startingLine++;
                buttonPressed = true;
            }
            //change starting zone
            else if (gamepad1.dpad_down && zone > 0 && !buttonPressed) {
                zone--;
                buttonPressed = true;
            } else if (gamepad1.dpad_up && zone < 2 && !buttonPressed) {
                zone++;
                buttonPressed = true;
            }
            //change initial delay
            else if (gamepad1.left_bumper && initialDelay > 0 && !buttonPressed) {
                initialDelay--;
                buttonPressed = true;
            } else if (gamepad1.right_bumper && initialDelay < 25 && !buttonPressed) {
                initialDelay++;
                buttonPressed = true;
            }
            //choose which autonomous mode
            else if (gamepad1.left_trigger > 0 && autoMode > 0 && !buttonPressed) {
                autoMode--;
                buttonPressed = true;
            } else if (gamepad1.right_trigger > 0 && autoMode < 2 && !buttonPressed) {
                autoMode++;
                buttonPressed = true;
            }
            //wait until buttons are not pressed
            else if (buttonPressed && !gamepad1.dpad_left && !gamepad1.dpad_right && !gamepad1.dpad_up && !gamepad1.dpad_down &&
                    !gamepad1.left_bumper && !gamepad1.right_bumper && gamepad1.left_trigger == 0 && gamepad1.right_trigger == 0)
                buttonPressed = false;

            //output telemetry
            telemetry.addData("Starting Line", display[startingLine + 3]);
            telemetry.addData("Starting Zone", display[zone]);
            telemetry.addData("Starting Delay", initialDelay);
            telemetry.addData("Autonomous Mode", display[autoMode + 7]);
            //
            telemetry.addLine("");
            //imu telemetry
            if (imu.isGyroCalibrated())
                telemetry.addLine("IMU calibrated");

            telemetry.update();

        }


        //setup webcam
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.openCameraDevice();
        pipeline = new UltimateGoalPipeline();
        webcam.setPipeline(pipeline);
        //
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                //webcam.startStreaming(320, 240);
                webcam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);

            }

            @Override
            public void onError(int errorCode) {

            }
        });


        //show OpenCV information
        while (!opModeIsActive() && !isStopRequested()) {

            telemetry.addLine("");
            //
            telemetry.addData("Analysis1", pipeline.getAnalysis());
            telemetry.addData("Analysis2", pipeline.getAnalysis2());
            telemetry.addData("Analysis3", pipeline.getAnalysis3());
            telemetry.addData("Position", pipeline.position);
            telemetry.addData("Position2", pipeline.position2);

            telemetry.update();
        }


        //wait until play is pressed
        waitForStart();

        //set running zone
        if (pipeline.position == reliableOpenCV.UltimateGoalPipeline.RingPosition.NONE || pipeline.position2 == reliableOpenCV.UltimateGoalPipeline.RingPosition.NONE)
            zone = 0;
        if(pipeline.position == reliableOpenCV.UltimateGoalPipeline.RingPosition.FOUR || pipeline.position2 == reliableOpenCV.UltimateGoalPipeline.RingPosition.FOUR)
            zone = 2;
        if (pipeline.position == reliableOpenCV.UltimateGoalPipeline.RingPosition.ONE || pipeline.position2 == reliableOpenCV.UltimateGoalPipeline.RingPosition.ONE)
            zone = 1;

        //delay initial delay amount
        Thread.sleep(initialDelay * 1000);

        //sync components
        //setup driving motors
        flMotor = hardwareMap.get(DcMotorEx.class, "flMotor");
        frMotor = hardwareMap.get(DcMotorEx.class, "frMotor");
        blMotor = hardwareMap.get(DcMotorEx.class, "blMotor");
        brMotor = hardwareMap.get(DcMotorEx.class, "brMotor");
        //reset wheel encoders
        flMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        blMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //flip left motors
        flMotor.setDirection(DcMotor.Direction.REVERSE);
        blMotor.setDirection(DcMotor.Direction.REVERSE);

        //setup shooter
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooterServo = hardwareMap.get(Servo.class, "shooterServo");
        shooterServo.setPosition(0);
        batteryVoltage = hardwareMap.voltageSensor.iterator().next();
        //set initial battery voltage
        voltageInitial = batteryVoltage.getVoltage();

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

        //wobble motor
        wobbleMotor = hardwareMap.get(DcMotorEx.class, "wobbleMotor");
        wobbleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wobbleMotor.setTargetPosition(0);
        wobbleMotor.setPower(0);
        wobbleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wobbleMotor.setDirection(DcMotor.Direction.REVERSE);
        //wobble servo
        wobbleServo = hardwareMap.get(Servo.class, "wobbleServo");
        wobbleServo.setDirection(Servo.Direction.REVERSE);

        //tape servo
        tapeServo = hardwareMap.get(Servo.class, "tapeServo");

        //conveyor motors
        conveyorUpper = hardwareMap.get(DcMotor.class, "conveyorUpper");
        conveyorLower = hardwareMap.get(DcMotor.class, "conveyorLower");
        //reverse lower motor
        conveyorLower.setDirection(DcMotor.Direction.REVERSE);


        //set running array element
        runningAutonomous.addAll(Arrays.asList(autonomous[startingLine]));

        //reformat autonomous stuff
        //shoot-park
        if (autoMode == 2) {
            if (startingLine == 0 || startingLine == 3) {

            } else if (startingLine == 1 || startingLine == 2) {

            }
        }

        //for loop to run robot motion
        for (arrayPosition = 0; arrayPosition < runningAutonomous.size() && opModeIsActive(); arrayPosition++) {

            //clear temp array lists
            positionsX.clear();
            positionsY.clear();
            actions.clear();
            //store values
            if (runningAutonomous.get(arrayPosition)[0] instanceof Integer) {
                positionsX.add( (Integer) runningAutonomous.get(arrayPosition)[0] );
                positionsY.add( (Integer) runningAutonomous.get(arrayPosition)[1] );
                actions.add( (String[]) runningAutonomous.get(arrayPosition)[3] );
            } else if (runningAutonomous.get(arrayPosition)[0] instanceof Integer[]) {
                positionsX.addAll( Arrays.asList( (Integer[]) runningAutonomous.get(arrayPosition)[0] ) );
                positionsY.addAll( Arrays.asList( (Integer[]) runningAutonomous.get(arrayPosition)[1] ) );
                actions.addAll( Arrays.asList( (String[][]) runningAutonomous.get(arrayPosition)[3] ) );
            }

            //create index value
            index = 0;
            if (positionsX.size() == 3)
                index = zone;


            //for loop to run INITIAL specific actions
            for (int i = 0; actions.size() > 0 && i < actions.get(index).length && opModeIsActive(); i++)
                //determine if action needs to happen on index
                if ( actions.get(index)[i].startsWith("i-"))
                    robotAction( actions.get(index)[i] );

            //move robot
            moveRobot( positionsX.get(index), positionsY.get(index), (Double) runningAutonomous.get(arrayPosition)[2]);

            //for loop to run MIDDLE specific actions
            for (int i = 0; actions.size() > 0 && i < actions.get(index).length && opModeIsActive(); i++)
                //determine if action needs to happen on index
                if ( !actions.get(index)[i].startsWith("i-") && !actions.get(index)[i].startsWith("e-"))
                    robotAction( actions.get(index)[i] );

            //delay to prevent issues
            Thread.sleep(10);

            //while loop wait for position to be reached
            while (!nearPosition(correctDriving) && wheelsRunning() && opModeIsActive()) Thread.sleep(5);
            //reset correcting value
            correctDriving = movingError;

            //for loop to run FINAL specific actions
            for (int i = 0; actions.size() > 0 && i < actions.get(index).length && opModeIsActive(); i++)
                //determine if action needs to happen on index
                if ( actions.get(index)[i].startsWith("e-"))
                    robotAction( actions.get(index)[i] );

        }

        //make program wait if other objects are running
        while (wobbleMotor.isBusy() || tapeMoving) Thread.sleep(5);

        //keep program active a little while to prevent issues
        for (int i = 0; i < 20 && opModeIsActive(); i++)
            Thread.sleep(5);

    }

    //ring detector pipeline
    public static class UltimateGoalPipeline extends OpenCvPipeline
    {

        //An enum to define the ring position
        public enum RingPosition
        {
            FOUR,
            ONE,
            NONE
        }


        //Some color constants
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);

         * The core values which define the location and size of the sample regions




        //grey tile
        public static int x_anchor1 = xAnchor1[0];
        public static int y_anchor1 = yAnchor1[0];
        //bottom ring
        public static int x_anchor2 = xAnchor2[0];
        public static int y_anchor2 = yAnchor2[0];
        //top rings
        public static int x_anchor3 = xAnchor3[0];
        public static int y_anchor3 = yAnchor3[0];

        public static int x_anchor4 = xAnchor1[1];
        public static int y_anchor4 = yAnchor1[1];
        //bottom ring
        public static int x_anchor5 = xAnchor2[1];
        public static int y_anchor5 = yAnchor2[1];
        //top rings
        public static int x_anchor6 = xAnchor3[1];
        public static int y_anchor6 = yAnchor3[1];

        static Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(x_anchor1,y_anchor1);
        static Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(x_anchor2,y_anchor2);
        static Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(x_anchor3,y_anchor3);

        static Point REGION4_TOPLEFT_ANCHOR_POINT = new Point(x_anchor4,y_anchor4);
        static Point REGION5_TOPLEFT_ANCHOR_POINT = new Point(x_anchor5,y_anchor5);
        static Point REGION6_TOPLEFT_ANCHOR_POINT = new Point(x_anchor6,y_anchor6);


        public static  int REGION1_WIDTH = region1Width[0];
        public static  int REGION1_HEIGHT = region1Height[0];

        public static  int REGION2_WIDTH = region2Width[0];
        public static  int REGION2_HEIGHT = region2Height[0];

        public static  int REGION3_WIDTH = region3Width[0];
        public static  int REGION3_HEIGHT = region3Height[0];

        public static  int REGION4_WIDTH = region1Width[1];
        public static  int REGION4_HEIGHT = region1Height[1];

        public static  int REGION5_WIDTH = region2Width[1];
        public static  int REGION5_HEIGHT = region2Height[1];

        public static  int REGION6_WIDTH = region3Width[1];
        public static  int REGION6_HEIGHT = region3Height[1];



        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION1_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION1_HEIGHT);
        Point region2_pointA = new Point(
                REGION2_TOPLEFT_ANCHOR_POINT.x,
                REGION2_TOPLEFT_ANCHOR_POINT.y);
        Point region2_pointB = new Point(
                REGION2_TOPLEFT_ANCHOR_POINT.x + REGION2_WIDTH,
                REGION2_TOPLEFT_ANCHOR_POINT.y + REGION2_HEIGHT);
        Point region3_pointA = new Point(
                REGION3_TOPLEFT_ANCHOR_POINT.x,
                REGION3_TOPLEFT_ANCHOR_POINT.y);
        Point region3_pointB = new Point(
                REGION3_TOPLEFT_ANCHOR_POINT.x + REGION3_WIDTH,
                REGION3_TOPLEFT_ANCHOR_POINT.y + REGION3_HEIGHT);

        Point region4_pointA = new Point(
                REGION4_TOPLEFT_ANCHOR_POINT.x,
                REGION4_TOPLEFT_ANCHOR_POINT.y);
        Point region4_pointB = new Point(
                REGION4_TOPLEFT_ANCHOR_POINT.x + REGION4_WIDTH,
                REGION4_TOPLEFT_ANCHOR_POINT.y + REGION4_HEIGHT);
        Point region5_pointA = new Point(
                REGION5_TOPLEFT_ANCHOR_POINT.x,
                REGION5_TOPLEFT_ANCHOR_POINT.y);
        Point region5_pointB = new Point(
                REGION5_TOPLEFT_ANCHOR_POINT.x + REGION5_WIDTH,
                REGION5_TOPLEFT_ANCHOR_POINT.y + REGION5_HEIGHT);
        Point region6_pointA = new Point(
                REGION6_TOPLEFT_ANCHOR_POINT.x,
                REGION6_TOPLEFT_ANCHOR_POINT.y);
        Point region6_pointB = new Point(
                REGION6_TOPLEFT_ANCHOR_POINT.x + REGION6_WIDTH,
                REGION6_TOPLEFT_ANCHOR_POINT.y + REGION6_HEIGHT);
         * Working variables


        Mat region1_Cb;
        Mat region2_Cb;
        Mat region3_Cb;

        Mat region4_Cb;
        Mat region5_Cb;
        Mat region6_Cb;



        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1;
        int avg2;
        int avg3;
        int avg4;
        int avg5;
        int avg6;
        int threshold = 7;
        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile reliableOpenCV.UltimateGoalPipeline.RingPosition position = reliableOpenCV.UltimateGoalPipeline.RingPosition.FOUR;

        private volatile reliableOpenCV.UltimateGoalPipeline.RingPosition position2 = reliableOpenCV.UltimateGoalPipeline.RingPosition.FOUR;

public void setHeightAndWidth(int height1, int height2, int width1, int width2, int height3, int width3) {
            REGION1_WIDTH = width1;
            REGION2_WIDTH = width2;
            REGION3_WIDTH = width3;
            REGION1_HEIGHT = height1;
            REGION2_HEIGHT = height2;
            REGION3_HEIGHT = height3;
        }
        public void changeThreshold(int changeby){
            threshold = threshold+changeby;
        }
        public void setAnchorPoints(int x, int y) {
            x_anchor1 = x;
            y_anchor1 = y;
            REGION1_TOPLEFT_ANCHOR_POINT = new Point(x_anchor1, y_anchor1);
            region1_pointA = new Point(
                    REGION1_TOPLEFT_ANCHOR_POINT.x,
                    REGION1_TOPLEFT_ANCHOR_POINT.y);
            region1_pointB = new Point(
                    REGION1_TOPLEFT_ANCHOR_POINT.x + REGION1_WIDTH,
                    REGION1_TOPLEFT_ANCHOR_POINT.y + REGION1_HEIGHT);
            region2_pointA = new Point(
                    REGION2_TOPLEFT_ANCHOR_POINT.x,
                    REGION2_TOPLEFT_ANCHOR_POINT.y);
            region2_pointB = new Point(
                    REGION2_TOPLEFT_ANCHOR_POINT.x + REGION2_WIDTH,
                    REGION2_TOPLEFT_ANCHOR_POINT.y + REGION2_HEIGHT);
            region3_pointA = new Point(
                    REGION3_TOPLEFT_ANCHOR_POINT.x,
                    REGION3_TOPLEFT_ANCHOR_POINT.y);
            region3_pointB = new Point(
                    REGION3_TOPLEFT_ANCHOR_POINT.x + REGION3_WIDTH,
                    REGION3_TOPLEFT_ANCHOR_POINT.y + REGION3_HEIGHT);
        }
        public void setAnchorPoints2(int x1, int y1, int x2, int y2, int x3, int y3) {
            x_anchor1 = x1;
            y_anchor1 = y1;
            x_anchor2 = x2;
            y_anchor2 = y2;
            x_anchor3 = x3;
            y_anchor3 = y3;
            REGION1_TOPLEFT_ANCHOR_POINT = new Point(x_anchor1, y_anchor1);
            REGION2_TOPLEFT_ANCHOR_POINT = new Point(x_anchor2, y_anchor2);
            REGION3_TOPLEFT_ANCHOR_POINT = new Point(x_anchor3, y_anchor3);
            region1_pointA = new Point(
                    REGION1_TOPLEFT_ANCHOR_POINT.x,
                    REGION1_TOPLEFT_ANCHOR_POINT.y);
            region1_pointB = new Point(
                    REGION1_TOPLEFT_ANCHOR_POINT.x + REGION1_WIDTH,
                    REGION1_TOPLEFT_ANCHOR_POINT.y + REGION1_HEIGHT);
            region2_pointA = new Point(
                    REGION2_TOPLEFT_ANCHOR_POINT.x,
                    REGION2_TOPLEFT_ANCHOR_POINT.y);
            region2_pointB = new Point(
                    REGION2_TOPLEFT_ANCHOR_POINT.x + REGION2_WIDTH,
                    REGION2_TOPLEFT_ANCHOR_POINT.y + REGION2_HEIGHT);
            region3_pointA = new Point(
                    REGION3_TOPLEFT_ANCHOR_POINT.x,
                    REGION3_TOPLEFT_ANCHOR_POINT.y);
            region3_pointB = new Point(
                    REGION3_TOPLEFT_ANCHOR_POINT.x + REGION3_WIDTH,
                    REGION3_TOPLEFT_ANCHOR_POINT.y + REGION3_HEIGHT);
        }


         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable


        void inputToCb(Mat input)
        {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1);
        }

        @Override
        public void init(Mat firstFrame)
        {
            inputToCb(firstFrame);

            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
            region2_Cb = Cb.submat(new Rect(region2_pointA, region2_pointB));
            region3_Cb = Cb.submat(new Rect(region3_pointA, region3_pointB));

            region4_Cb = Cb.submat(new Rect(region4_pointA, region4_pointB));
            region5_Cb = Cb.submat(new Rect(region5_pointA, region5_pointB));
            region6_Cb = Cb.submat(new Rect(region6_pointA, region6_pointB));
        }
        public boolean isSimilarTo(int value1, int value2){
            if(abs(value1-value2) <= threshold){
                return true;
            }else{
                return false;
            }
        }
        @Override
        public Mat processFrame(Mat input)
        {

            inputToCb(input);

            //grey tile
            avg1 = (int) Core.mean(region1_Cb).val[0];
            //bottom ring
            avg2 = (int) Core.mean(region2_Cb).val[0];
            //top 3 rings
            avg3 = (int) Core.mean(region3_Cb).val[0];

            //grey tile
            avg4 = (int) Core.mean(region4_Cb).val[0];
            //bottom ring
            avg5 = (int) Core.mean(region5_Cb).val[0];
            //top 3 rings
            avg6 = (int) Core.mean(region6_Cb).val[0];
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region2_pointA, // First point which defines the rectangle
                    region2_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region3_pointA, // First point which defines the rectangle
                    region3_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region4_pointA, // First point which defines the rectangle
                    region4_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region5_pointA, // First point which defines the rectangle
                    region5_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region6_pointA, // First point which defines the rectangle
                    region6_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines


            position = reliableOpenCV.UltimateGoalPipeline.RingPosition.FOUR; // Record our analysis
            position2 = reliableOpenCV.UltimateGoalPipeline.RingPosition.FOUR; // Record our analysis

//

            if (!(isSimilarTo(avg1, avg3))) { // abs(tileAvg - topAvg) >= 7
                position = reliableOpenCV.UltimateGoalPipeline.RingPosition.FOUR;
            }
            else if((!isSimilarTo(avg1, avg2) && isSimilarTo(avg1, avg3))) { // abs(tileAvg - bottomAvg) >= 7
                position = reliableOpenCV.UltimateGoalPipeline.RingPosition.ONE;
            }
            else if(isSimilarTo(avg1, avg2)){ // abs(tileAvg - bottomAvg) <= 7
                position = reliableOpenCV.UltimateGoalPipeline.RingPosition.NONE;
            }

            if (!(isSimilarTo(avg4, avg6))) { // abs(tileAvg - topAvg) >= 7
                position2 = reliableOpenCV.UltimateGoalPipeline.RingPosition.FOUR;
            }
            else if((!isSimilarTo(avg4, avg5) && isSimilarTo(avg4, avg6))) { // abs(tileAvg - bottomAvg) >= 7
                position2 = reliableOpenCV.UltimateGoalPipeline.RingPosition.ONE;
            }
            else if(isSimilarTo(avg4, avg5)){ // abs(tileAvg - bottomAvg) <= 7
                position2 = reliableOpenCV.UltimateGoalPipeline.RingPosition.NONE;
            }


            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    2); // Negative thickness means solid fill
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region2_pointA, // First point which defines the rectangle
                    region2_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    2); // Negative thickness means solid fill
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region3_pointA, // First point which defines the rectangle
                    region3_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    2); // Negative thickness means solid fill

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region4_pointA, // First point which defines the rectangle
                    region4_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    2); // Negative thickness means solid fill
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region5_pointA, // First point which defines the rectangle
                    region5_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    2); // Negative thickness means solid fill
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region6_pointA, // First point which defines the rectangle
                    region6_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    2); // Negative thickness means solid fill


            return input;
        }

        public reliableOpenCV.UltimateGoalPipeline.RingPosition getPosition(){return position;}
        public reliableOpenCV.UltimateGoalPipeline.RingPosition getPosition2(){return position2;}
        public int getAnalysis()
        {
            return avg1;
        }
        public int getAnalysis2()
        {
            return avg2;
        }
        public int getAnalysis3()
        {
            return avg3;
        }
        public int getAnalysis4()
        {
            return avg4;
        }
        public int getAnalysis5()
        {
            return avg5;
        }
        public int getAnalysis6()
        {
            return avg6;
        }

    }

}
*/
