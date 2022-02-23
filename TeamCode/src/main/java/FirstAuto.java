

        package org.firstinspires.ftc.teamcode;


        import com.qualcomm.hardware.bosch.BNO055IMU;
        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
        import com.qualcomm.robotcore.eventloop.opmode.Disabled;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.DcMotorEx;
        import com.qualcomm.robotcore.hardware.PIDFCoefficients;
        import com.qualcomm.robotcore.hardware.Servo;
        import com.qualcomm.robotcore.hardware.VoltageSensor;
        import com.qualcomm.robotcore.util.ElapsedTime;

        import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
        import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
        import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
        import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
        import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
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
        import java.util.Collection;

        import static java.lang.Math.abs;

        @Autonomous(name = "Competition Autonomous", group = "Competition")

        @Disabled

        public class FirstAuto extends LinearOpMode {

            //editable variables
            private int zone = 0;


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

            //create imu object
            private BNO055IMU imu;
            //
            private double  currentHeading = 0,
                    targetHeading = 0;


            //method to calculate ticks
            private double calculateTicks (double inches) {
                return (int)((inches / (Math.PI * wheelDiameter) * wheelMotorTicks) / wheelGearRatio);
            }

            //method to parse coordinates
            private void robotAction (String action) throws InterruptedException {

                //remove initial string from action
                action = action.replaceAll("i-","").replaceAll("e-", "");



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


                    //create variable to store direction polarity
                    double direction = (finalX - initX) /
                            (double) Math.abs(finalX - initX);
                    //create variable to store degree movement
                    double ticksRotation = calculateTicks( (currentHeading / 360.0) * Math.PI * botDiagonal );


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
                /*if (Math.abs(inchesY) > Math.abs(inchesX))
                    powerA *= (1 - inchesX / inchesY);
                else if (Math.abs(inchesX) > Math.abs(inchesY))
                    powerB *= 1 - inchesY / inchesX;*/

                //set velocity if horizontal movement is greater than vertical
                flMotor.setVelocity(powerA);
                frMotor.setVelocity(powerB);
                blMotor.setVelocity(powerB);
                brMotor.setVelocity(powerA);

            }

            //------------------------------------------------------------------------------------------------------------


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



                    //
                    telemetry.addLine("");
                    //imu telemetry
                    if (imu.isGyroCalibrated())
                        telemetry.addLine("IMU calibrated");

                    telemetry.update();

                }



                //show OpenCV information
                while (!opModeIsActive() && !isStopRequested()) {

                    telemetry.addLine("");
                    //


                    telemetry.update();
                }


                //wait until play is pressed
                waitForStart();




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


                //tape servo
                tapeServo = hardwareMap.get(Servo.class, "tapeServo");



                //reformat autonomous stuff
                //shoot-park


                //keep program active a little while to prevent issues
                for (int i = 0; i < 20 && opModeIsActive(); i++)
                    Thread.sleep(5);

            }



        }
