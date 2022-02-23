package TeleopArchive;

import static com.sun.tools.doclint.Entity.ge;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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

@Disabled
@TeleOp(name = "TeleOp", group = "Testing")



public class TeleOpMode extends LinearOpMode {


    //static values
    private final double wheelMotorTicks = 384.5;
    private final double wheelMotorRPM = 435;
    private final double wheelDiameter = 3.77953;
    private final double botRotationSpeed = 1.1; //to match rotation with driving
    private final double wheelMaxVelocity = wheelMotorRPM * wheelMotorTicks / 60;
    private final double maxVerticalSlideTicks = 384.5 * 3.64;
    private final double maxSlideVelocity = wheelMotorRPM * wheelMotorTicks / 60;
    private final double spinnyTicks = 537.7;
    private final double maxSpinnyVelocity = 312 * spinnyTicks / 60;

    private final double intakeDropFreight = 0;
    private final double intakeDown = .5;
    private final double intakeUp = .3;
    private final double intakeDrop = .65;

    private final double dropdownDown = .55;
    private final double dropdownUp = .1;
    private final double dropdownDownLeft = .3;
    private final double dropdownUpLeft = .055;

    private final double right = 1;
    private final double middle = .7;
    private final double left = .33;
    private final double forward = 0;


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
    private DcMotorEx flMotor;
    private DcMotorEx frMotor;
    private DcMotorEx blMotor;
    private DcMotorEx brMotor;
    private DcMotorEx spinny;
    private CRServo intakeServo;
    private CRServo carouselServo;

    private Servo rotateIntakeServo;
    private DcMotorEx verticalSlide;

    private Servo rightDropdown;
    private Servo leftDropdown;
    private Servo slideRotation;
    private Servo horizontalServo;

    private ColorSensor color;


    //create arm objects


    //create arm variables
    private final double maxSlideTicks = 360;

    //arm position values
    private int slidePosition = 0;
    private final double[] slideValues = {
            maxVerticalSlideTicks * 1, //top level
            maxVerticalSlideTicks * .55, //shared
            maxVerticalSlideTicks * .3, //low level
            maxVerticalSlideTicks * .6, //middle level

    };
    String[] slideLevel = {"Top level", "Lowest level", "Middle level", "shared shipping"};

    //intake servo position values
    private int intakePosition = 1;
    private final double[] intakeValues = {
            .1, .4, .8, 1
    };

    //dropdown servos position values
    private int dropdownPosition = 1;
    private final double[] dropdownValues = {
            .55, .1
    };


    //toggle booleans
    private boolean slideMoving = false;
    private boolean slidePlayerControlled = true;
    private boolean slideSwitch = false;
    private boolean autoIntakeActive = false;

    private boolean intakeMoving = false;
    private boolean dropdownMoving = false;


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

    public void changeSlidePosition() {
        if (gamepad2.dpad_up) {
            slidePosition = 0;
        } else if (gamepad2.dpad_right) {
            slidePosition = 2;
        } else if (gamepad2.dpad_left) {
            slidePosition = 1;
        } else if (gamepad2.dpad_down) {
            slidePosition = 3;
        }
    }


  /*  public void slideControl() {

        if (slidePlayerControlled) {
            if (gamepad2.right_stick_y > 0)
                verticalSlide.setTargetPosition((int) (verticalSlide.getCurrentPosition() + (maxSlideTicks * .2)));
            else if (gamepad2.right_stick_y < 0)
                verticalSlide.setTargetPosition((int) (verticalSlide.getCurrentPosition() - (maxSlideTicks * .2)));
            else if (gamepad2.right_stick_y == 0)
                verticalSlide.setTargetPosition(verticalSlide.getCurrentPosition());

        } else {
            verticalSlide.setTargetPosition((int) slideValues[slidePosition]);
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




        verticalSlide.setVelocity(wheelMaxVelocity);


    }*/

    public void intakeRotationControl() {
        if (gamepad1.square && intakePosition > 0 && !intakeMoving) {
            //decrease position
            intakePosition--;
            intakeMoving = true;
        } else if (gamepad1.triangle && intakePosition < 3 && !intakeMoving) {
            //increase position
            intakePosition++;
            intakeMoving = true;
        } else if (!gamepad1.square && !gamepad1.triangle && intakeMoving)
            intakeMoving = false;

        rotateIntakeServo.setPosition(intakeValues[intakePosition]);

    }

    public void dropdownControl() {
        if (gamepad2.a && dropdownPosition > 0 && !dropdownMoving) {
            //decrease position
            dropdownPosition--;
            dropdownMoving = true;
        } else if (gamepad2.y && dropdownPosition < 1 && !dropdownMoving) {
            //increase position
            dropdownPosition++;
            dropdownMoving = true;
        } else if (!gamepad2.a && !gamepad2.y && dropdownMoving)
            dropdownMoving = false;

        rightDropdown.setPosition(dropdownValues[dropdownPosition]);
        leftDropdown.setPosition(dropdownValues[dropdownPosition]);

    }

    public void rotateHorizontal() {
        if (horizontalPlayerControlled) {
            if (-gamepad2.left_stick_y > 0)
                spinny.setTargetPosition(spinny.getCurrentPosition() + 50);
            else if (-gamepad2.left_stick_y < 0)// && wobbleMotor.getCurrentPosition() > wobbleValues[0] + 50)
                spinny.setTargetPosition(spinny.getCurrentPosition() - 50);
            else if (gamepad2.left_stick_y == 0)
                spinny.setTargetPosition(spinny.getCurrentPosition());
        } else {
            spinny.setTargetPosition((int) horizontalValues[horizontalPosition]);
        }

        if (gamepad2.dpad_left && horizontalPosition > 0 && !horizontalMoving) {
            horizontalPosition--;
            horizontalMoving = true;
        } else if (gamepad2.dpad_right && horizontalPosition < 2 && !horizontalMoving) {
            horizontalPosition++;
            horizontalMoving = true;
        } else if (!gamepad2.dpad_left && !gamepad2.dpad_right && horizontalMoving) {
            horizontalMoving = false;
        }

        if (gamepad2.share && !horizontalSwitch && !horizontalPlayerControlled) {
            //make wobble player controlled
            horizontalPlayerControlled = true;
            horizontalSwitch = true;
        } else if (gamepad2.share && !horizontalSwitch && horizontalPlayerControlled) {
            //remove player control from wobble
            horizontalPlayerControlled = false;
            horizontalSwitch = true;
        } else if (horizontalSwitch && !gamepad2.share)
            horizontalSwitch = false;

        if (gamepad2.options) {

//            spinny.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            spinny.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        }

    }


    @Override
    public void runOpMode() throws InterruptedException {


        //TODO organize
        //setup driving motors
        flMotor = hardwareMap.get(DcMotorEx.class, "flMotor");
        frMotor = hardwareMap.get(DcMotorEx.class, "frMotor");
        blMotor = hardwareMap.get(DcMotorEx.class, "blMotor");
        brMotor = hardwareMap.get(DcMotorEx.class, "brMotor");

        //setup other objects
        spinny = hardwareMap.get(DcMotorEx.class, "spinny");
        intakeServo = hardwareMap.get(CRServo.class, "intakeServo");
        intakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
        carouselServo = hardwareMap.get(CRServo.class, "carouselServo");

        slideRotation = hardwareMap.get(Servo.class, "slideRotation");
        horizontalServo = hardwareMap.get(Servo.class, "horizontalServo");

        rotateIntakeServo = hardwareMap.get(Servo.class, "rotateIntakeServo");

        rightDropdown = hardwareMap.get(Servo.class, "rightDropdown");
        leftDropdown = hardwareMap.get(Servo.class, "leftDropdown");

        //servo direction
        rightDropdown.setDirection(Servo.Direction.REVERSE);
        leftDropdown.setDirection(Servo.Direction.FORWARD);

        rotateIntakeServo.setDirection(Servo.Direction.FORWARD);
        slideRotation.setDirection(Servo.Direction.FORWARD);

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
        verticalSlide = hardwareMap.get(DcMotorEx.class, "verticalSlide");
        verticalSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalSlide.setTargetPosition(0);
        verticalSlide.setVelocity(0);
        verticalSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        spinny = hardwareMap.get(DcMotorEx.class, "spinny");
        spinny.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spinny.setTargetPosition(0);
        spinny.setVelocity(0);
        spinny.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        color = hardwareMap.colorSensor.get("color");


        waitForStart();


        verticalSlide.setVelocity(maxSlideVelocity);
        spinny.setVelocity(maxSpinnyVelocity * .2);
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
                carouselServo.setPower(-1);

            } else if (gamepad1.cross) {
                //spinny.setPower((Math.pow(10, (.5 * spinnyTime.seconds()) - .5)));
                carouselServo.setPower(1);

            } /*else if (!gamepad1.cross || !gamepad1.circle) {
                *//*spinnyTime.reset();*//*
                carouselServo.setPower(0);
            } */else
                carouselServo.setPower(0);


            //---------------------------------------------------------------------
            //Mecanum Drivetrain Control
            //if statement to ensure robot won't move unless control sticks are moved
            if (Math.abs(gamepad1LY) > .05 || Math.abs(gamepad1LX) > .05 || Math.abs(gamepad1RX) > .05) {
                //control logic: multiply by gear array for different speeds,
                //multiply right stick movement by rotation fraction to reduce uncontrolability
                flMotor.setVelocity((gearValues[activeGear] * ((gamepad1LY - gamepad1LX)) - gearValues[activeGear] * botRotationSpeed * gamepad1RX) * wheelMaxVelocity);
                frMotor.setVelocity((gearValues[activeGear] * ((gamepad1LY + gamepad1LX)) + gearValues[activeGear] * botRotationSpeed * gamepad1RX) * wheelMaxVelocity);
                blMotor.setVelocity((gearValues[activeGear] * ((gamepad1LY + gamepad1LX)) - gearValues[activeGear] * botRotationSpeed * gamepad1RX) * wheelMaxVelocity);
                brMotor.setVelocity((gearValues[activeGear] * ((gamepad1LY - gamepad1LX)) + gearValues[activeGear] * botRotationSpeed * gamepad1RX) * wheelMaxVelocity);
            } else if (gamepad1.left_bumper && !gamepad1.right_bumper && Math.abs(gamepad1LY) < .05 && Math.abs(gamepad1LX) < .05 && Math.abs(gamepad1RX) < .05) {
                flMotor.setVelocity(.3 * wheelMaxVelocity);
                frMotor.setVelocity(-.3 * wheelMaxVelocity);
                blMotor.setVelocity(.3 * wheelMaxVelocity);
                brMotor.setVelocity(-.3 * wheelMaxVelocity);
            } else if (gamepad1.right_bumper && !gamepad1.left_bumper && Math.abs(gamepad1LY) < .05 && Math.abs(gamepad1LX) < .05 && Math.abs(gamepad1RX) < .05) {
                flMotor.setVelocity(-.3 * wheelMaxVelocity);
                frMotor.setVelocity(.3 * wheelMaxVelocity);
                blMotor.setVelocity(-.3 * wheelMaxVelocity);
                brMotor.setVelocity(.3 * wheelMaxVelocity);
            } else if (gamepad1.dpad_up) {
                flMotor.setVelocity((-.15 * wheelMaxVelocity));
                frMotor.setVelocity((-.15 * wheelMaxVelocity));
                blMotor.setVelocity((-.15 * wheelMaxVelocity));
                brMotor.setVelocity((-.15 * wheelMaxVelocity));

            } else if (gamepad1.dpad_down) {
                flMotor.setVelocity((.15 * wheelMaxVelocity));
                frMotor.setVelocity((.15 * wheelMaxVelocity));
                blMotor.setVelocity((.15 * wheelMaxVelocity));
                brMotor.setVelocity((.15 * wheelMaxVelocity));
            } else {
                flMotor.setVelocity(0);
                frMotor.setVelocity(0);
                blMotor.setVelocity(0);
                brMotor.setVelocity(0);
            }
            //-----------------------------------------------------------------------------------


            //TODO Gamepad 2

            rotateHorizontal();

            spinny.setPower(gamepad2.right_stick_y);



            switch (autointakeState) {
                case DEFAULT_POSITION:
                    verticalSlide.setVelocity(maxSlideVelocity);
                    intakeServo.setPower(0);
                    rotateIntakeServo.setPosition(intakeUp);
                    leftDropdown.setPosition(dropdownUp);
                    rightDropdown.setPosition(dropdownUp);
                    verticalSlide.setTargetPosition(0);
                    slideRotation.setPosition(middle);

                    if (gamepad2.right_bumper)
                        autointakeState = AutoIntakeState.INTAKE_ACTIVE;
                    else if (gamepad2.right_trigger > 0)
                        autointakeState = AutoIntakeState.ACTIVE_INTAKE_NO_SENSOR;
                    else if (gamepad2.touchpad)
                        autointakeState = AutoIntakeState.REVERSE_INTAKE;
                    else if (gamepad2.triangle && autointakeState == AutoIntakeState.DEFAULT_POSITION) {
                        autoIntakeTime.reset();
                        autointakeState = AutoIntakeState.TOP_LEVEL;
                    } else if (gamepad2.square) {
                        autoIntakeTime.reset();
                        autointakeState = AutoIntakeState.SHARED_LEFT;
                    } else if (gamepad2.circle) {
                        autoIntakeTime.reset();
                        autointakeState = AutoIntakeState.SHARED_RIGHT;
                    }

                    break;
                case SHARED_LEFT:
                    rightDropdown.setPosition(dropdownDown);
                    leftDropdown.setPosition(dropdownDown);
                    if (autoIntakeTime.seconds() > .5 && autoIntakeTime.seconds() < 1) {
                        verticalSlide.setTargetPosition((int) slideValues[1]);
                    }

                    if (autoIntakeTime.seconds() > 1) {
                        slideRotation.setPosition(left - .15);

                         if (gamepad2.left_bumper) {
                            autoIntakeTime.reset();
                            autointakeState = AutoIntakeState.DROP_FREIGHT;
                        } else if (gamepad2.ps) {
                            autoIntakeTime.reset();
                            autointakeState = AutoIntakeState.SLIDE_DOWN;

                        }
                    }


                    break;

                case SHARED_RIGHT:
                    rightDropdown.setPosition(dropdownDown);
                    leftDropdown.setPosition(dropdownDown);
                    if (autoIntakeTime.seconds() > .5 && autoIntakeTime.seconds() < 1) {
                        verticalSlide.setTargetPosition((int) slideValues[1]);
                    }

                    if (autoIntakeTime.seconds() > 1) {
                        slideRotation.setPosition(forward);

                        if (gamepad2.left_bumper) {
                            autoIntakeTime.reset();
                            autointakeState = AutoIntakeState.DROP_FREIGHT;
                        } else if (gamepad2.ps) {
                            autoIntakeTime.reset();
                            autointakeState = AutoIntakeState.SLIDE_DOWN;

                        }
                    }


                    break;

                case TOP_LEVEL:

                    rightDropdown.setPosition(dropdownDown);
                    leftDropdown.setPosition(dropdownDown);
                    if (autoIntakeTime.seconds() > .5 && autoIntakeTime.seconds() < 1)
                        verticalSlide.setTargetPosition((int) slideValues[0]);
                    if (autoIntakeTime.seconds() > 1) {
                        slideRotation.setPosition(forward);

                        if (gamepad2.dpad_right) {
                            slideRotation.setPosition(right);
                            autoIntakeTime.reset();
                            autointakeState = AutoIntakeState.HORIZONTAL_ROTATION_RIGHT;


                        } else if (gamepad2.dpad_left) {
                            slideRotation.setPosition(left);
                            autoIntakeTime.reset();
                            autointakeState = AutoIntakeState.HORIZONTAL_ROTATION_LEFT;


                        } else if (gamepad2.ps) {
                            autoIntakeTime.reset();
                            autointakeState = AutoIntakeState.SLIDE_DOWN;

                        } else  if (gamepad2.left_bumper) {
                            autoIntakeTime.reset();
                            autointakeState = AutoIntakeState.DROP_FREIGHT;
                        }
                    }

                    break;
                case HORIZONTAL_ROTATION_LEFT:
                    if (gamepad2.left_bumper) {
                        autoIntakeTime.reset();
                        autointakeState = AutoIntakeState.DROP_FREIGHT;
                    }
                    else if (gamepad2.dpad_right) {
                        slideRotation.setPosition(right);
                        autoIntakeTime.reset();
                        autointakeState = AutoIntakeState.HORIZONTAL_ROTATION_RIGHT;

                    } else if (gamepad2.dpad_up) {
                        slideRotation.setPosition(forward);
                        autoIntakeTime.reset();
                        autointakeState = AutoIntakeState.HORIZONTAL_ROTATION_FORWARD;

                    } else if (gamepad2.ps) {
                        autoIntakeTime.reset();
                        autointakeState = AutoIntakeState.SLIDE_DOWN;
                    }
                    break;

                case HORIZONTAL_ROTATION_RIGHT:

                    if (gamepad2.left_bumper) {
                        autoIntakeTime.reset();
                        autointakeState = AutoIntakeState.DROP_FREIGHT;
                    }
                    else if (gamepad2.dpad_left) {
                        slideRotation.setPosition(left);
                        autoIntakeTime.reset();
                        autointakeState = AutoIntakeState.HORIZONTAL_ROTATION_LEFT;

                    } else if (gamepad2.dpad_up) {
                        slideRotation.setPosition(forward);
                        autoIntakeTime.reset();
                        autointakeState = AutoIntakeState.HORIZONTAL_ROTATION_FORWARD;

                    } else if (gamepad2.ps) {
                        autoIntakeTime.reset();
                        autointakeState = AutoIntakeState.SLIDE_DOWN;
                    }
                    break;

                case HORIZONTAL_ROTATION_FORWARD:
                    if (gamepad2.left_bumper) {
                        autoIntakeTime.reset();
                        autointakeState = AutoIntakeState.DROP_FREIGHT;
                    }
                    else if (gamepad2.dpad_left) {
                        slideRotation.setPosition(left);
                        autoIntakeTime.reset();
                        autointakeState = AutoIntakeState.HORIZONTAL_ROTATION_LEFT;

                    } else if (gamepad2.dpad_right) {
                        slideRotation.setPosition(right);
                        autoIntakeTime.reset();
                        autointakeState = AutoIntakeState.HORIZONTAL_ROTATION_RIGHT;

                    } else if (gamepad2.ps) {
                        autoIntakeTime.reset();
                        autointakeState = AutoIntakeState.SLIDE_DOWN;
                    }
                    break;


                case DROP_FREIGHT:


                    if (autoIntakeTime.seconds() > 0 && autoIntakeTime.seconds() < .5) {
                        rotateIntakeServo.setPosition(intakeDrop);
                    }
                    else if (autoIntakeTime.seconds() > .5 && autoIntakeTime.seconds() < 1) {
                        rotateIntakeServo.setPosition(intakeUp);

                    } else if (autoIntakeTime.seconds() > 1 && autoIntakeTime.seconds() < 1.5) {
                        slideRotation.setPosition(middle);

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
                    verticalSlide.setVelocity(maxSlideVelocity * .5);

                    rotateIntakeServo.setPosition(intakeUp);
                    leftDropdown.setPosition(dropdownDown);
                    rightDropdown.setPosition(dropdownDown);
                    verticalSlide.setTargetPosition(0);
                    /*slideRotation.setPosition(middle);*/
                    if (autoIntakeTime.seconds() > 1.5)
                        autointakeState = AutoIntakeState.DEFAULT_POSITION;
                    break;
                case INTAKE_ACTIVE:
                    intakeServo.setPower(1);
                    rotateIntakeServo.setPosition(intakeDown);
                    leftDropdown.setPosition(dropdownDown);
                    rightDropdown.setPosition(dropdownDown);
                    autoIntakeTime.reset();

                    if (!gamepad2.right_bumper && autoIntakeTime.seconds() < .75) {
                        intakeServo.setPower(1);
                        rotateIntakeServo.setPosition(intakeUp);
                        leftDropdown.setPosition(dropdownDown);
                        rightDropdown.setPosition(dropdownDown);

                        autointakeState = AutoIntakeState.DEFAULT_POSITION;
                    } else if (((DistanceSensor) color).getDistance(DistanceUnit.CM) < 4.5 && autoIntakeTime.seconds() < .5) {
                        gamepad1.rumbleBlips(1);
                        gamepad2.rumbleBlips(1);
                        intakeServo.setPower(1);
                        rotateIntakeServo.setPosition(intakeUp);
                        leftDropdown.setPosition(dropdownUp);
                        rightDropdown.setPosition(dropdownUp);

                        autointakeState = AutoIntakeState.DEFAULT_POSITION;
                    }
                    break;
                case ACTIVE_INTAKE_NO_SENSOR:
                    intakeServo.setPower(1);
                    rotateIntakeServo.setPosition(intakeDown);
                    leftDropdown.setPosition(dropdownDown);
                    rightDropdown.setPosition(dropdownDown);
                    autoIntakeTime.reset();
                    if (gamepad2.right_trigger == 0 && autoIntakeTime.seconds() < .75) {
                        intakeServo.setPower(1);
                        rotateIntakeServo.setPosition(intakeUp);
                        leftDropdown.setPosition(dropdownDown);
                        rightDropdown.setPosition(dropdownDown);

                        autointakeState = AutoIntakeState.DEFAULT_POSITION;
                    }
                    break;
                case REVERSE_INTAKE:
                    intakeServo.setPower(-1);
                    rotateIntakeServo.setPosition(intakeDown);
                    leftDropdown.setPosition(dropdownDown);
                    rightDropdown.setPosition(dropdownDown);
                    autoIntakeTime.reset();
                    if (!gamepad2.touchpad && autoIntakeTime.seconds() < .5) {
                        intakeServo.setPower(-1);
                        rotateIntakeServo.setPosition(intakeDown);
                        leftDropdown.setPosition(dropdownUp);
                        rightDropdown.setPosition(dropdownUp);

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
                    verticalSlide.setTargetPosition((int) slideValues[slidePosition]);
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


            //intakeServo.setPower(gamepad2.right_stick_y);


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
            telemetry.addData("Servo Power", intakeServo.getPower());
            telemetry.addData("Spinny Power", spinny.getPower());
//            telemetry.addData("SpinnyTime", spinnyTime.seconds());
            telemetry.addData("Slide Tick Position", verticalSlide.getCurrentPosition());
            telemetry.addData("Intake Rotation", intakeValues[intakePosition]);
            telemetry.addData("Intake Rotation Servo", rotateIntakeServo.getPosition());
            telemetry.addData("Dropdown Position", rightDropdown.getPosition());
            telemetry.addData("Dropdown Array Value", dropdownValues[dropdownPosition]);
            telemetry.addData("Slide Target Position", verticalSlide.getCurrentPosition());
            telemetry.addData("Slide Position", slideLevel[slidePosition]);
            telemetry.addData("Slide Rotation Position", slideRotation.getPosition());
            telemetry.addData("gamepad2 state", autointakeState);
            telemetry.addData("spinnyTicks", spinny.getCurrentPosition());
            telemetry.addData("spinny", spinny.getTargetPosition());


            telemetry.update();


        }

    }
}



