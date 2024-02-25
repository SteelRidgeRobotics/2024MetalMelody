# Metal Melody

This is the repository for Steel Ridge Robotics' 2024 robot. We named it Metal Melody on purpose, unlike the last name. 

This year, there are 3 main elements of the robot, the swerve drive (our first competition robot with swerve!), the elevator (and climber), and the intake. We also did some work with autonomous routines and vision.

## Swerve Drive

The swerve drive consists of 8 motors: 4 for driving, 4 for direction. The Drive Motors use the control request VelocityVoltage to drive accurately at a consistent speed, and the Direction Motors use MotionMagicVoltage to finely control the full motion of the wheel, not just the end positions. This allows us to smoothly control our wanted direction. The direction motors also use CANcoders to align all wheels to ensure we can drive accurately, as without this the wheels could be offset at the start of the math, which is, shockingly, not good.

## Elevator

The elevator is controlled by two motors controlling two lifts using the MotionMagic feature of the TalonFX motors. The motors (and by extension the lift) move to specific positions when we press buttons on the controller. There are two positions, one for intaking notes, and one for shooting them downward into the amp.

## Intake

The intake consists of 2 motors, one to intake rings that we control via DutyCycleOut, and one to move the intake up and down that we control, similarly to the elevator, via MotionMagic. There are 3 positions on the pivot, one for intaking, one for stowing the intake (so it isn't on the ground while we drive), and one for shooting the note down into the amp. The reason for 2 different positions for intaking and shooting is because we discovered the intake position is too low to shoot into the amp. 

## Autonomous
beep boop
