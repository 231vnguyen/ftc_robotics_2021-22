import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

//@Disabled
@TeleOp(name = "TeleOpBlueBot", group = "Testing")



public class TeleOpModeBlue extends LinearOpMode {


    //static values
    private final double wheelMotorTicks = 384.5;
    private final double wheelMotorRPM = 435;
    private final double wheelDiameter = 3.77953;
    private final double botRotationSpeed = 1.1; //to match rotation with driving
    private final double wheelMaxVelocity = 1;
    private final double maxrightSlideTicks = 384.5 * 2;
    private final double maxSlideVelocity = wheelMotorRPM * wheelMotorTicks / 60;
    private final double spinnyTicks = 537.7;
    private final double maxSpinnyVelocity = 312 * spinnyTicks / 60;

    private final double intakeDropFreight = 0;
    private final double intakeDown = .5;
    private final double intakeUp = .3;
    private final double intakeDrop = .65;

    private final double down = .3;
    private final double forward = 8;




    //gear change variables
    private boolean bumpersPressed = false;
    private int activeGear = 2;//values 1 - 3


    //inverse drive variables
    private boolean inversePressed = false;
    private int inverse = 1;

    //create the gear array
    private final double[] gearValues = {.18, .3, .5, 1.0};

    private int horizontalPosition = 2;
    private final double[] horizontalValues = {
            -240,
            -120,
            0,

    };
    private boolean horizontalPlayerControlled = false;
    private boolean horizontalMoving = false;
    private boolean horizontalSwitch = false;



    //create general variables
    private double gamepad1LY;
    private double gamepad1LX;
    private double gamepad1RX;

    //timers
//    private ElapsedTime spinnyTime = new ElapsedTime();
    private ElapsedTime autoIntakeTime = new ElapsedTime();

    //create motor/servo objects
    private DcMotor flMotor;
    private DcMotor frMotor;
    private DcMotor blMotor;
    private DcMotor brMotor;

    
    private CRServo leftCarousel;
    private CRServo rightCarousel;

    private Servo stick;
    
    private DcMotorEx rightSlide;
    private DcMotorEx leftSlide;

    private DcMotor tubeys;

    private Servo rightDropdown;
    private Servo leftDropdown;
    

    private ColorSensor color;


    //create arm objects


    //create arm variables
    private final double maxSlideTicks = 360;

    //arm position values
    private int slidePosition = 0;
    private final double[] slideValues = {
            maxrightSlideTicks * 1, //top level
            maxrightSlideTicks * .55, //shared
            maxrightSlideTicks * .3, //low level
            maxrightSlideTicks * .6, //middle level

    };
    String[] slideLevel = {"Top level", "Lowest level", "Middle level", "shared shipping"};

    //intake servo position values
    private int intakePosition = 1;
    private final double[] intakeValues = {
            .1, .4, .8, 1
    };

    //dropdown servos position values
    private int dropdownPosition = 0;
    private final double[] dropdownValues = {
            0, .1, .2, .3, .4, .5, .6, .7, .85, .9, 1
    };

    private int stickPosition = 0;
    private final double [] stickValues = {
            0, .1, .2, .3, .4, .5, .6, .7, .85, .9, 1
    };

    private final double stickUp = .4;
    private final double stickDown = 0;

    //toggle booleans
    private boolean slideMoving = false;
    private boolean slidePlayerControlled = true;
    private boolean slideSwitch = false;
    private boolean autoIntakeActive = false;

    private boolean intakeMoving = false;
    private boolean dropdownMoving = false;
    private boolean stickMoving = false;


    public enum AutoIntakeState {
        DEFAULT_POSITION,
        INTAKE_ACTIVE,
        ACTIVE_INTAKE_NO_SENSOR,
        REVERSE_INTAKE,
        TOP_LEVEL,
        HORIZONTAL_ROTATION_RIGHT,
        HORIZONTAL_ROTATION_LEFT,
        HORIZONTAL_ROTATION_FORWARD,
        DROP_FREIGHT,
        SLIDE_DOWN,
        SHARED_RIGHT,
        SHARED_LEFT
    }

    AutoIntakeState autointakeState = AutoIntakeState.DEFAULT_POSITION;

    public enum LiftState {
        DEFAULT_POSITION,
        TOP_LEVEL, //DPAD up = top, down = shared, left = middle, right = bottom
        HORIZONTAL_ROTATION_RIGHT, //circle = rotate right, square = rotate left
        HORIZONTAL_ROTATION_LEFT,
        DROP_FREIGHT, //automatically after horizontal rotation
        SLIDE_DOWN, //automatically after dropping freight, rotate intake to default, LiftState liftState = LiftState.DEFAULT_POSITION;
        SHARED_RIGHT,
        SHARED_lEFT
    }

    LiftState liftState = LiftState.DEFAULT_POSITION;


    //method to change gears
    private void changeGears() {

        //if statement to change active gear count
        if (gamepad1.right_trigger > 0 && !bumpersPressed && activeGear < gearValues.length - 1) {

            //set boolean variable to true to avoid multiple inputs
            bumpersPressed = true;
            gamepad1.rumble(0.9, 0, 200);
            //add 1 to active gear
            activeGear++;

        } else if (gamepad1.left_trigger > 0 && !bumpersPressed && activeGear > 0) {

            //set boolean variable to true to avoid multiple inputs
            bumpersPressed = true;
            gamepad1.rumble(0.9, 0, 200);
            //subtract 1 from active gear
            activeGear--;

        } else if (gamepad1.left_trigger == 0 && gamepad1.right_trigger == 0)
            bumpersPressed = false;

    }




  /*  public void slideControl() {

        if (slidePlayerControlled) {
            if (gamepad2.right_stick_y > 0)
                rightSlide.setTargetPosition((int) (rightSlide.getCurrentPosition() + (maxSlideTicks * .2)));
            else if (gamepad2.right_stick_y < 0)
                rightSlide.setTargetPosition((int) (rightSlide.getCurrentPosition() - (maxSlideTicks * .2)));
            else if (gamepad2.right_stick_y == 0)
                rightSlide.setTargetPosition(rightSlide.getCurrentPosition());

        } else {
            rightSlide.setTargetPosition((int) slideValues[slidePosition]);
        }

        //increase and decrease current position
        if (gamepad2.dpad_down && slidePosition > 0 && !slideMoving) {
            //decrease
            slidePosition--;
            slideMoving = true;
        } else if (gamepad2.dpad_up && slidePosition < 3 && !slideMoving) {
            //increase
            slidePosition++;
            slideMoving = true;
        } else if (!gamepad2.dpad_up && !gamepad2.dpad_down && slideMoving)
            slideMoving = false;

        //toggle slide control to and from player
        if (gamepad2.x && !slideSwitch && !slidePlayerControlled) {
            //make slide player controlled
            slidePlayerControlled = true;
            slideSwitch = true;
        } else if (gamepad2.x && !slideSwitch && slidePlayerControlled) {
            //remove player control from slide
            slidePlayerControlled = false;
            slideSwitch = true;
        } else if (slideSwitch && !gamepad2.x)
            slideSwitch = false;




        rightSlide.setVelocity(wheelMaxVelocity);


    }*/

    public void dropdownControl() {
        if (gamepad2.dpad_up && dropdownPosition > 0 && !dropdownMoving) {
            //decrease position
            dropdownPosition--;
            dropdownMoving = true;
        } else if (gamepad2.dpad_down && dropdownPosition < 10 && !dropdownMoving) {
            //increase position
            dropdownPosition++;
            dropdownMoving = true;
        } else if (!gamepad2.dpad_up && !gamepad2.dpad_down && dropdownMoving)
            dropdownMoving = false;

        rightDropdown.setPosition(dropdownValues[dropdownPosition]);
        leftDropdown.setPosition(dropdownValues[dropdownPosition]);

    }

    public void stickTest() {
        if (gamepad2.dpad_right && stickPosition > 0 && !stickMoving) {
            //decrease position
            stickPosition--;
            stickMoving = true;
        } else if (gamepad2.dpad_left && stickPosition < 10 && !stickMoving) {
            //increase position
            stickPosition++;
            stickMoving = true;
        } else if (!gamepad2.dpad_right && !gamepad2.dpad_left && stickMoving)
            stickMoving = false;

        stick.setPosition(stickValues[stickPosition]);
    }

 

    


    @Override
    public void runOpMode() throws InterruptedException {


        //TODO organize
        //setup driving motors
        flMotor = hardwareMap.get(DcMotor.class, "flMotor");
        frMotor = hardwareMap.get(DcMotor.class, "frMotor");
        blMotor = hardwareMap.get(DcMotor.class, "blMotor");
        brMotor = hardwareMap.get(DcMotor.class, "brMotor");

        //setup other objects
        leftCarousel = hardwareMap.get(CRServo.class, "leftCarousel");
        rightCarousel = hardwareMap.get(CRServo.class, "rightCarousel");

        stick = hardwareMap.get(Servo.class, "stick");


        rightDropdown = hardwareMap.get(Servo.class, "rightDropdown");
        leftDropdown = hardwareMap.get(Servo.class, "leftDropdown");

        //servo direction
        rightDropdown.setDirection(Servo.Direction.REVERSE);
        leftDropdown.setDirection(Servo.Direction.FORWARD);



        //motor direction
        flMotor.setDirection(DcMotor.Direction.FORWARD);
        frMotor.setDirection(DcMotor.Direction.REVERSE);
        blMotor.setDirection(DcMotor.Direction.FORWARD);
        brMotor.setDirection(DcMotor.Direction.REVERSE);

        //zero power behavior
        flMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        blMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        brMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        spinny.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //armMotor encoders
        rightSlide = hardwareMap.get(DcMotorEx.class, "rightSlide");
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setTargetPosition(0);
        rightSlide.setVelocity(0);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftSlide = hardwareMap.get(DcMotorEx.class, "leftSlide");
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setTargetPosition(0);
        leftSlide.setVelocity(0);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        color = hardwareMap.colorSensor.get("color");

        tubeys = hardwareMap.get(DcMotor.class, "tubeys");


        waitForStart();


        rightSlide.setVelocity(maxSlideVelocity);
        leftSlide.setVelocity(maxSlideVelocity);

        //rotateIntakeServo.setPosition(.5);
        /*leftDropdown.setPosition(.1);
        rightDropdown.setPosition(.1);
        rotateIntakeServo.setPosition(.4);*/


        //while loop
        while (opModeIsActive() && !isStopRequested()) {


            //TODO Gamepad 1


            //update controller variables
            gamepad1LY = gamepad1.left_stick_y;
            gamepad1LX = gamepad1.left_stick_x;
            gamepad1RX = gamepad1.right_stick_x;

            //event methods
            changeGears();
            dropdownControl();



            //-----------------------------------------------------------------------------
            //Inverse drive with start
            if (gamepad1.options && !inversePressed)
                inverse *= -1;
            inversePressed = gamepad1.options;


            //-----------------------------------------------------------------------------
            //spinny with b and a

            if (gamepad1.circle) {
                //spinnyTime.reset();
                //spinny.setPower(-(Math.pow(10, (.5 * spinnyTime.seconds()) - .5)));
                leftCarousel.setPower(-1);
                rightCarousel.setPower(-1);

            } else if (gamepad1.cross) {
                //spinny.setPower((Math.pow(10, (.5 * spinnyTime.seconds()) - .5)));
                leftCarousel.setPower(1);
                rightCarousel.setPower(1);

            } /*else if (!gamepad1.cross || !gamepad1.circle) {
                *//*spinnyTime.reset();*//*
                carouselServo.setPower(0);
            } */else
                leftCarousel.setPower(0);
                rightCarousel.setPower(0);


            //---------------------------------------------------------------------
            //Mecanum Drivetrain Control
            //if statement to ensure robot won't move unless control sticks are moved
            if (Math.abs(gamepad1LY) > .05 || Math.abs(gamepad1LX) > .05 || Math.abs(gamepad1RX) > .05) {
                //control logic: multiply by gear array for different speeds,
                //multiply right stick movement by rotation fraction to reduce uncontrolability
                flMotor.setPower((gearValues[activeGear] * ((gamepad1LY - gamepad1LX)) - gearValues[activeGear] * botRotationSpeed * gamepad1RX) * wheelMaxVelocity);
                frMotor.setPower((gearValues[activeGear] * ((gamepad1LY + gamepad1LX)) + gearValues[activeGear] * botRotationSpeed * gamepad1RX) * wheelMaxVelocity);
                blMotor.setPower((gearValues[activeGear] * ((gamepad1LY + gamepad1LX)) - gearValues[activeGear] * botRotationSpeed * gamepad1RX) * wheelMaxVelocity);
                brMotor.setPower((gearValues[activeGear] * ((gamepad1LY - gamepad1LX)) + gearValues[activeGear] * botRotationSpeed * gamepad1RX) * wheelMaxVelocity);
            } else if (gamepad1.left_bumper && !gamepad1.right_bumper && Math.abs(gamepad1LY) < .05 && Math.abs(gamepad1LX) < .05 && Math.abs(gamepad1RX) < .05) {
                flMotor.setPower(.3 * wheelMaxVelocity);
                frMotor.setPower(-.3 * wheelMaxVelocity);
                blMotor.setPower(.3 * wheelMaxVelocity);
                brMotor.setPower(-.3 * wheelMaxVelocity);
            } else if (gamepad1.right_bumper && !gamepad1.left_bumper && Math.abs(gamepad1LY) < .05 && Math.abs(gamepad1LX) < .05 && Math.abs(gamepad1RX) < .05) {
                flMotor.setPower(-.3 * wheelMaxVelocity);
                frMotor.setPower(.3 * wheelMaxVelocity);
                blMotor.setPower(-.3 * wheelMaxVelocity);
                brMotor.setPower(.3 * wheelMaxVelocity);
            } else if (gamepad1.dpad_up) {
                flMotor.setPower((-.15 * wheelMaxVelocity));
                frMotor.setPower((-.15 * wheelMaxVelocity));
                blMotor.setPower((-.15 * wheelMaxVelocity));
                brMotor.setPower((-.15 * wheelMaxVelocity));

            } else if (gamepad1.dpad_down) {
                flMotor.setPower((.15 * wheelMaxVelocity));
                frMotor.setPower((.15 * wheelMaxVelocity));
                blMotor.setPower((.15 * wheelMaxVelocity));
                brMotor.setPower((.15 * wheelMaxVelocity));
            } else {
                flMotor.setPower(0);
                frMotor.setPower(0);
                blMotor.setPower(0);
                brMotor.setPower(0);
            }
            //-----------------------------------------------------------------------------------


            //TODO Gamepad 2





            switch (autointakeState) {
                case DEFAULT_POSITION:

                    rightSlide.setVelocity(maxSlideVelocity);
                    leftSlide.setVelocity(maxSlideVelocity);

                    //TODO
                    /*leftDropdown.setPosition(dropdownUp);
                    rightDropdown.setPosition(dropdownUp);*/

                    rightSlide.setTargetPosition(0);
                    leftSlide.setTargetPosition(0);
                    tubeys.setPower(0);



                    if (gamepad2.right_bumper)
                        autointakeState = AutoIntakeState.INTAKE_ACTIVE;
                    else if (gamepad2.right_trigger > 0)
                        autointakeState = AutoIntakeState.ACTIVE_INTAKE_NO_SENSOR;
                    else if (gamepad2.touchpad)
                        autointakeState = AutoIntakeState.REVERSE_INTAKE;
                    else if (gamepad2.triangle && autointakeState == AutoIntakeState.DEFAULT_POSITION) {
                        autoIntakeTime.reset();
                        autointakeState = AutoIntakeState.TOP_LEVEL;
                    }

                    break;




                case TOP_LEVEL:

//                    rightDropdown.setPosition(dropdownDown);
//                    leftDropdown.setPosition(dropdownDown);
                    if (autoIntakeTime.seconds() > .5 && autoIntakeTime.seconds() < 1)
                        rightSlide.setTargetPosition((int) slideValues[0]);
                        leftSlide.setTargetPosition((int) slideValues[0]);
                    if (autoIntakeTime.seconds() > 1) {

                        if (gamepad2.ps) {
                            autoIntakeTime.reset();
                            autointakeState = AutoIntakeState.SLIDE_DOWN;

                        } else  if (gamepad2.left_bumper) {
                            autoIntakeTime.reset();
                            autointakeState = AutoIntakeState.DROP_FREIGHT;
                        }
                    }

                    break;


                case DROP_FREIGHT:


                    if (autoIntakeTime.seconds() > 0 && autoIntakeTime.seconds() < .5) {

                    }
                    else if (autoIntakeTime.seconds() > .5 && autoIntakeTime.seconds() < 1) {


                    } else if (autoIntakeTime.seconds() > 1 && autoIntakeTime.seconds() < 1.5) {


                    } else if (autoIntakeTime.seconds() > 1.5) {
                        autoIntakeTime.reset();
                        autointakeState = AutoIntakeState.SLIDE_DOWN;

                        if (gamepad2.ps) {
                            autoIntakeTime.reset();
                            autointakeState = AutoIntakeState.SLIDE_DOWN;
                        }


                    }
                    break;

                case SLIDE_DOWN:
                    rightSlide.setVelocity(maxSlideVelocity * .5);


//                    leftDropdown.setPosition(dropdownDown);
//                    rightDropdown.setPosition(dropdownDown);
                    rightSlide.setTargetPosition(0);
                    leftSlide.setTargetPosition(0);
                    /*slideRotation.setPosition(middle);*/
                    if (autoIntakeTime.seconds() > 1.5)
                        autointakeState = AutoIntakeState.DEFAULT_POSITION;
                    break;
                case INTAKE_ACTIVE:

                    tubeys.setPower(.5);
                    stick.setPosition(stickUp);

                /*    leftDropdown.setPosition(dropdownDown);
                    rightDropdown.setPosition(dropdownDown);*/

                    autoIntakeTime.reset();

                    if (!gamepad2.right_bumper && autoIntakeTime.seconds() < .75) {


//                        leftDropdown.setPosition(dropdownDown);
//                        rightDropdown.setPosition(dropdownDown);

                        autointakeState = AutoIntakeState.DEFAULT_POSITION;
                    } else if (((DistanceSensor) color).getDistance(DistanceUnit.CM) < 3 && autoIntakeTime.seconds() < .5) {
                        gamepad1.rumbleBlips(1);
                        gamepad2.rumbleBlips(1);

                        stick.setPosition(stickDown);

//                        leftDropdown.setPosition(dropdownUp);
//                        rightDropdown.setPosition(dropdownUp);

                        autointakeState = AutoIntakeState.DEFAULT_POSITION;
                    }
                    break;
                case ACTIVE_INTAKE_NO_SENSOR:

//                    leftDropdown.setPosition(dropdownDown);
//                    rightDropdown.setPosition(dropdownDown);
                    autoIntakeTime.reset();
                    if (gamepad2.right_trigger == 0 && autoIntakeTime.seconds() < .75) {
                        tubeys.setPower(.5);

//                        leftDropdown.setPosition(dropdownDown);
//                        rightDropdown.setPosition(dropdownDown);

                        autointakeState = AutoIntakeState.DEFAULT_POSITION;
                    }
                    break;
                case REVERSE_INTAKE:
                    tubeys.setPower(-.5);

//                    leftDropdown.setPosition(dropdownDown);
//                    rightDropdown.setPosition(dropdownDown);
                    autoIntakeTime.reset();
                    if (!gamepad2.touchpad && autoIntakeTime.seconds() < .5) {
                        tubeys.setPower(-.5);

//                        leftDropdown.setPosition(dropdownUp);
//                        rightDropdown.setPosition(dropdownUp);

                        autointakeState = AutoIntakeState.DEFAULT_POSITION;
                    }
                    break;

            }



           /* switch (liftState) {
                case DEFAULT_POSITION:

                    if (gamepad2.triangle) {
                        liftState = LiftState.TOP_LEVEL;
                    }

                    break;
                case TOP_LEVEL:
                    rightSlide.setTargetPosition((int) slideValues[slidePosition]);
                    rightDropdown.setPosition(dropdownDown);
                    leftDropdown.setPosition(dropdownDown);
                    if (gamepad2.circle) {
                        slideRotation.setPosition(1);
                        rotateIntakeServo.setPosition(0);
                        liftState = LiftState.HORIZONTAL_ROTATION_LEFT;


                    } else if (gamepad2.square) {
                        slideRotation.setPosition(0);
                        rotateIntakeServo.setPosition(0);
                        liftState = LiftState.HORIZONTAL_ROTATION_RIGHT;

                    }
                    break;
                case HORIZONTAL_ROTATION_LEFT:

                    if (slideRotation.getPosition() == 1 && rotateIntakeServo.getPosition() == 0)
                    liftState = LiftState.DROP_FREIGHT;
                    break;
                case HORIZONTAL_ROTATION_RIGHT:

                    if (slideRotation.getPosition() == 0 && rotateIntakeServo.getPosition() == 0)
                    liftState = LiftState.DROP_FREIGHT;
                    break;
                case DROP_FREIGHT:

                        rotateIntakeServo.setPosition(intakeUp);
                        slideRotation.setPosition(.5);
                        liftState = LiftState.DEFAULT_POSITION;

                    }

            }*/


           // tubeys.setPower(gamepad2.right_stick_y * .5);


            telemetry.addLine()
                    .addData("Red", color.red())
                    .addData("Green", color.green())
                    .addData("Blue", color.blue());
            telemetry.addData("Alpha", color.alpha());

            if (color instanceof DistanceSensor) {
                telemetry.addData("Distance (cm)", "%.3f", ((DistanceSensor) color).getDistance(DistanceUnit.CM));
            }
            //send what gear is active
            telemetry.addData("Gear", activeGear + 1);
            telemetry.addData("Inverse Value", inverse);

//            telemetry.addData("SpinnyTime", spinnyTime.seconds());
            telemetry.addData("Slide Tick Position", rightSlide.getCurrentPosition());
            telemetry.addData("Intake Rotation", intakeValues[intakePosition]);
            telemetry.addData("Dropdown Position", rightDropdown.getPosition());
            telemetry.addData("Dropdown Array Value", dropdownValues[dropdownPosition]);
            telemetry.addData("Slide Target Position", rightSlide.getCurrentPosition());
            telemetry.addData("Slide Position", slideLevel[slidePosition]);
            telemetry.addData("gamepad2 state", autointakeState);
            telemetry.addData("dropdownposition", dropdownPosition);
            telemetry.addData("stickvalue", stickPosition);
            telemetry.addData("stickposition", stick.getPosition());


            telemetry.update();


        }

    }
}



