#include "CDriver.h"
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <time.h>
#define PI 3.141592654


#ifdef REVERSE
#define TURNF 0
#define TURNB 1
#define GO 2
#endif
int revStartState;

/* Gear Changing Constants*/
const int gearUp[6] =
{
    5000,6000,6000,6500,7000,0
};
const int gearDown[6] =
{
    0,2500,3000,3000,3500,3500
};

/* Stuck constants*/
const int stuckTime = 25;
const float stuckAngle = .523598775; //PI/6

/* Accel and Brake Constants*/
const float maxSpeedDist = 50;
const float maxSpeed = 200;
const float sin5 = 0.08716;
const float cos5 = 0.99619;

/* Steering constants*/
const float steerLock = 0.785398;
const float wheelSensitivityCoeff = 1;

#ifndef REVERSE
const float steerSensitivityOffset = 120.0;
#endif // REVERSE

#ifdef REVERSE
const float steerSensitivityOffset = 30.0;
#endif // REVERSE


/* ABS Filter Constants */
const float wheelRadius[4] = { 0.3179,0.3179,0.3276,0.3276 };
const float absSlip = 2.0;
const float absRange = 3.0;
const float absMinSpeed = 3.0;

/* Clutch constants */
const float clutchMax = 0.5;
const float clutchDelta = 0.05;
const float clutchRange = 0.82;
const float clutchDeltaTime = 0.02;
const float clutchDeltaRaced = 10;
const float clutchDec = 0.01;
const float clutchMaxModifier = 1.3;
const float clutchMaxTime = 1.5;

int stuck;
float clutch;

const float sensorAngle[19] = { -PI / 2, -(PI / 12) * 5, -PI / 3, -PI / 4, -PI / 6, -PI / 9, -PI / 12, -PI / 18, -PI / 36, 0, PI / 36, PI / 18, PI / 12, PI / 9, PI / 6, PI / 4, PI / 3, (PI / 12) * 5, PI / 2 };

typedef float trackAngleArray[19];
trackAngleArray trackAngles;

int TurnState;
int TS;
int TurnPrep = 0;



struct
{
    int type;
    int direction;
    float angle;
} TurnData;

#define IN_TURN 1
#define IN_STR 0
#define WILL_TURN 2;
#define RIGHT 0;
#define LEFT 1;

#define ACC_MODE_CAREFUL 1
#define ACC_MODE_FAST 2

int accelMode;
int accelCareTimer;

//int TurnSate = IN_STR;

#define useFilter 1
#define simNoise 0

#define historyDataPoints 5

int enableScaredSteering;

int compare(const void* a, const void* b)
{
    return (*(float*)a - *(float*)b);
}



void inputProcessor(structCarState* cs)
{
    static float inputMatrix[19][historyDataPoints];
    static float sortedMatrix[19][historyDataPoints];
    static int idx = 0;


    for (int i = 0; i < 19; i++)
    {
        inputMatrix[i][idx] = cs->track[i];

#if simNoise
        if (rand() % 10 == 0)
        {
            printf("*\n");
            inputMatrix[i][idx] += (float)(rand() % 30) - 15.0f;
        }
#endif
    }


    for (int i = 0; i < 19; i++)
    {
        for (int j = 0; j < historyDataPoints; j++)
        {
            sortedMatrix[i][j] = inputMatrix[i][j];
        }
    }

#if useFilter
    for (int i = 0; i < 19; i++)
    {
        qsort(sortedMatrix[i], historyDataPoints, sizeof(float), compare);
    }
#else

    for (int i = 0; i < 19; i++)
	{
		sortedMatrix[i][8] = inputMatrix[i][idx];
	}
#endif

    for (int i = 0; i < 19; i++)
    {
        trackAngles[i] = sortedMatrix[i][historyDataPoints / 2];
    }

    idx++;
    if (idx == historyDataPoints)
        idx = 0;

    // reading of sensor at +5 degree w.r.t. car axis
    float rxSensor = cs->track[10];
    // reading of sensor parallel to car axis
    float cSensor = cs->track[9];
    // reading of sensor at -5 degree w.r.t. car axis
    float sxSensor = cs->track[8];

    // approaching a turn on right
    if (rxSensor > sxSensor)
    {
        // computing approximately the "angle" of turn
        float h = cSensor * sin5;
        float b = rxSensor - cSensor * cos5;
        float sinAngle = b * b / (h * h + b * b);
        TurnData.angle = asin(sinAngle);
        TurnData.direction = RIGHT;
    }
    // approaching a turn on left
    else
    {
        // computing approximately the "angle" of turn
        float h = cSensor * sin5;
        float b = sxSensor - cSensor * cos5;
        float sinAngle = b * b / (h * h + b * b);
        TurnData.angle = asin(sinAngle);
        TurnData.direction = LEFT;
    }
                                   
}

float indexRangeToAngle(const int idxStart, const int identCount)
{
	const float start = sensorAngle[idxStart];
	const float end = sensorAngle[idxStart + identCount - 1];

    //printf("%d[%f] .. %d[%f] = [%f]\n", idxStart, sensorAngle[idxStart], idxStart + identCount - 1, sensorAngle[idxStart + identCount - 1], (start + end) / 2.0f);

    return start;
}

float floatAbs(const float n)
{
    if (n < 0)
        return n * -1.0f;
    return n;
}
int cmpFloat(float a, float b, float tolerance)
{
	if (floatAbs(a - b) < tolerance)
	{
        return 1;
	}

    return 0;
}

float getLongestAngle()
{
    int maxIdx = 0; //start of range
    float maxDist = 0;
    int sameCtr = 0; //range 


    for (int i = 0; i < 19; i++) {
        if (trackAngles[i] > maxDist) {
            maxDist = trackAngles[i];
            maxIdx = i;
        }
    }

    for (int i = 0; i < 19; i++) {
        if (trackAngles[i] == maxDist) {
            sameCtr++;
        }
    }

    return indexRangeToAngle(maxIdx, sameCtr);
}


int getGear(structCarState* cs)
{

    int gear = cs->gear;
    int rpm = cs->rpm;

    // if gear is 0 (N) or -1 (R) just return 1 
    if (gear < 1)
        return 1;
    // check if the RPM value of car is greater than the one suggested 
    // to shift up the gear from the current one     
    if (gear < 6 && rpm >= gearUp[gear - 1])
        return gear + 1;
    else
        // check if the RPM value of car is lower than the one suggested 
        // to shift down the gear from the current one
        if (gear > 1 && rpm <= gearDown[gear - 1])
            return gear - 1;
        else // otherwhise keep current gear
            return gear;
}

float getSteer(structCarState* cs)
{
    float dirAngle;
    // steering angle is compute by correcting the actual car angle w.r.t. to track 
    // axis [cs->angle] and to adjust car position w.r.t to middle of track [cs->trackPos*0.5]
    //
    dirAngle = -cs->angle;

    if(cs->trackPos<0)
        dirAngle = cs->trackPos * cs->angle + (1 + cs->trackPos) * getLongestAngle();
    else
        dirAngle = -cs->trackPos * cs->angle + (1 - cs->trackPos) * getLongestAngle();

    float targetAngle = (-dirAngle - cs->trackPos * 0.5) *1.4;

    if (TurnState == IN_TURN || TurnPrep == 1)
    {
        targetAngle = -getLongestAngle() * 1.4;

       // printf("\r%d      ", accelCareTimer);

        if (targetAngle > 0.08)
        {
            int ignoreDanger = 0;
            if (targetAngle < 0 && cs->trackPos > 0)
                ignoreDanger = 1;

            if (targetAngle > 0 && cs->trackPos < 0)
                ignoreDanger = 1;

            if (!ignoreDanger)
            {
                accelCareTimer += 1;
                accelCareTimer += 50 * targetAngle;
              //  printf("DANGER!");
            }

            
        }
        else
        {
         //   printf("________");
        }
        
    }


   // printf("\rAt: %lf Turning: %lf", cs->trackPos, targetAngle);

    //return (targetAngle) / steerLock;
    // at high speed reduce the steering command to avoid loosing the control
    if (cs->speedX > steerSensitivityOffset)
        return targetAngle / (steerLock * (cs->speedX - steerSensitivityOffset) * wheelSensitivityCoeff);
    else
        return (targetAngle) / steerLock;
}

static float lerror = 0;



float getAccel(structCarState* cs)
{
    float vX = cs->speedX / 3.6;
    float speedDist = (vX*vX)/12;
    static int turnHistory[10];
    static int turnHistIdx = 0;
    //int maxDIdx = maxDistIdx(cs);
    // checks if car is out of track
    if (cs->trackPos < 1 && cs->trackPos > -1)
    {
        // reading of sensor at +5 degree w.r.t. car axis
        float rxSensor = cs->track[10];
        // reading of sensor parallel to car axis
        float cSensor = cs->track[9];
        // reading of sensor at -5 degree w.r.t. car axis
        float sxSensor = cs->track[8];

        float targetSpeed = maxSpeed;

        // track is straight and enough far from a turn so goes to max speed
        if (cSensor > speedDist || (cSensor >= rxSensor && cSensor >= sxSensor))
        {
            turnHistory[turnHistIdx] = IN_STR;
        }

        else
        {
            turnHistory[turnHistIdx] = IN_TURN;
        }

        if (rxSensor < cSensor && cSensor < sxSensor)
        {
            turnHistory[turnHistIdx] = IN_TURN;
        }

        if (rxSensor > cSensor && cSensor > sxSensor)
        {
            turnHistory[turnHistIdx] = IN_TURN;
        }


        turnHistIdx++;
        if (turnHistIdx == 10)
            turnHistIdx = 0;

        TurnState = IN_TURN;
        int lastTprep = TurnPrep;
        int strCount = 0;
        for (int i = 0; i < 10; i++)
        {
            if (turnHistory[i] == IN_STR)
                strCount++;
            if (strCount >= 5)
            {
                TurnState = IN_STR;
                TurnPrep = 0;
            }
        }

        accelCareTimer--;
        if (accelCareTimer == -1)
            accelCareTimer = 0;

        int spSoftLimit = 400;

        if (cSensor < 85)
        {
            spSoftLimit = 70;

            if (lastTprep == 0)
            {
                accelCareTimer = 285;
            }
                

            TurnPrep = 1;
        }

        //printf("\n\n%d\n\n", TurnState);
        if (TurnState == IN_TURN)
        {
            if (rxSensor > sxSensor)
            {
                //printf("\rR T %lf %lf %lf", rxSensor, cSensor, sxSensor);
                // computing approximately the "angle" of turn
                float h = cSensor * sin5;
                float b = rxSensor - cSensor * cos5;
                float sinAngle = b * b / (h * h + b * b);
                // estimate the target speed depending on turn and on how close it is
                //targetSpeed = maxSpeed * (cSensor * sinAngle / speedDist);
                //spSoftLimit = 25;
                if (!enableScaredSteering)
                targetSpeed = maxSpeed * (cSensor * sinAngle / speedDist);
            }
            // approaching a turn on left
            else
            {
                //printf("\rL T %lf %lf %lf", rxSensor, cSensor, sxSensor);
                // computing approximately the "angle" of turn
                float h = cSensor * sin5;
                float b = sxSensor - cSensor * cos5;
                float sinAngle = b * b / (h * h + b * b);
                // estimate the target speed depending on turn and on how close it is
                //targetSpeed = maxSpeed * (cSensor * sinAngle / speedDist);
                //spSoftLimit = 25;
                if (!enableScaredSteering)
                targetSpeed = maxSpeed * (cSensor * sinAngle / speedDist);
            }
        }
        else
        {
            //printf("\rSTR %lf %lf %lf", rxSensor, cSensor, sxSensor);
            targetSpeed = maxSpeed;
        }

        static float pgain = .093;
        static const float igain = .006;
        static float dgain = 0.64;


        if (enableScaredSteering)
            pgain = .093;
        else
            pgain = .041;




        static float setPoint = 0;
        static float ltgt = 0;
        const float setPointRampStep = 14;

        if (TurnState == IN_STR && TurnPrep == 0)
            accelCareTimer = 0;

        ltgt = targetSpeed;

        if (!enableScaredSteering)
        targetSpeed *= 1.25;

        if (floatAbs(setPoint - targetSpeed) > 5)
        if (targetSpeed > setPoint)
        {
            setPoint += setPointRampStep;
        }
        else if (targetSpeed < setPoint)
        {
            setPoint -= setPointRampStep;
        }
        else
            setPoint = targetSpeed;
        /*
        if (setPoint > 170)
            setPoint = 170;
            */
        if (enableScaredSteering)
	        if (accelCareTimer > 0)
		        if (setPoint > spSoftLimit)
		            setPoint = spSoftLimit;

        const float error = setPoint - cs->speedX;

        float dcontrol = ((error - lerror) * dgain);
        if (!enableScaredSteering)
        {
            if (dcontrol >= 2)
                dcontrol = 2;
            if (dcontrol <= -2)
                dcontrol = -2;
        }

        float control = 0;
        static float integ = 0;

        integ += error;

        control += error * pgain;
        if (integ > 100)
            integ = 100;
        if (integ < -100)
            integ = -100;
        control += integ * igain;
        control += dcontrol;

        //printf("  setP: %lf", setPoint);

        // accel/brake command is expontially scaled w.r.t. the difference between target speed and current one
        //return (2 / (1 + exp(cs->speedX - targetSpeed)) - 1);
        if (control > 1.0f)
            control = 1.0f;
        if (control < -1.0f)
            control = -1.0f;

        


        printf("\rctr: %lf err: %lf SP: %lf TS: %d TP: %d||| P[%lf] I[%lf] D[%lf]", control, error, setPoint, TurnState, TurnPrep, error * pgain, integ * igain, dcontrol);

        lerror = error;

        
        return control;
    }
    else
        return 0.3; // when out of track returns a moderate acceleration command

}


int getGearR(structCarState* cs)
{
    return -1;
}

float getSteerR(structCarState* cs)
{
    float angle;
    if (cs->angle < 0)
        angle = cs->angle + PI;
    else
        angle = cs->angle - PI;
    // steering angle is compute by correcting the actual car angle w.r.t. to track 
    // axis [cs->angle] and to adjust car position w.r.t to middle of track [cs->trackPos*0.5]
    float targetAngle = (angle - cs->trackPos * 0.5);
    // at high speed reduce the steering command to avoid loosing the control
    //if (-cs->speedX > steerSensitivityOffset)
    //    return targetAngle / (steerLock * (-cs->speedX - steerSensitivityOffset) * wheelSensitivityCoeff);
    //else
    return (targetAngle) / steerLock;

}

float getAccelR(structCarState* cs)
{
    // checks if car is out of track
    if (cs->trackPos < 1 && cs->trackPos > -1)
    {
        // reading of sensor at +5 degree w.r.t. car axis
        float rxSensor = cs->track[10];
        // reading of sensor parallel to car axis
        float cSensor = cs->track[9];
        // reading of sensor at -5 degree w.r.t. car axis
        float sxSensor = cs->track[8];

        float targetSpeed;

        // track is straight and enough far from a turn so goes to max speed
        if (cSensor > maxSpeedDist || (cSensor >= rxSensor && cSensor >= sxSensor))
            targetSpeed = maxSpeed;
        else
        {
            // approaching a turn on right
            if (rxSensor > sxSensor)
            {
                // computing approximately the "angle" of turn
                float h = cSensor * sin5;
                float b = rxSensor - cSensor * cos5;
                float sinAngle = b * b / (h * h + b * b);
                // estimate the target speed depending on turn and on how close it is
                targetSpeed = maxSpeed * (cSensor * sinAngle / maxSpeedDist);
            }
            // approaching a turn on left
            else
            {
                // computing approximately the "angle" of turn
                float h = cSensor * sin5;
                float b = sxSensor - cSensor * cos5;
                float sinAngle = b * b / (h * h + b * b);
                // estimate the target speed depending on turn and on how close it is
                targetSpeed = maxSpeed * (cSensor * sinAngle / maxSpeedDist);
            }

        }
        double xSpeed = -cs->speedX;
        // accel/brake command is expontially scaled w.r.t. the difference between target speed and current one
        return 2 / (1 + exp(xSpeed - targetSpeed)) - 1;
    }
    else
        return 0.3; // when out of track returns a moderate acceleration command

}

int a = 0;
structCarControl CDrive(structCarState cs)
{
    if (a == 0)
    {
        a = 1;
        srand(time(0));
    }

    inputProcessor(&cs);
    if (cs.stage != cs.prevStage)
    {
        cs.prevStage = cs.stage;
    }

#ifdef REVERSE
    if (revStartState != GO) {
        float steer, brk;
        int gear;
        if (revStartState == TURNF) {
            steer = -1;
            gear = 1;
            brk = 0.;
            if (cs.angle >= PI / 2 || cs.track[9] < 0.5) {
                revStartState = TURNB;
                brk = 1.;
            }

        }
        else { //TURNB
            steer = 1;
            gear = -1;
            brk = 0.;
            if (abs(cs.angle) > 9 * PI / 10) {
                revStartState = GO;
                brk = 1.;
            }
        }
        // Calculate clutching
        clutching(&cs, &clutch);

        structCarControl cc = { 0.1f,brk,gear,steer,clutch };
        return cc;
    }
#endif // REVERSE



#ifndef REVERSE
    // check if car is currently stuck
    if (fabs(cs.angle) > stuckAngle)
    {
        // update stuck counter
        stuck++;
    }
    else
    {
        // if not stuck reset stuck counter
        stuck = 0;
    }
   
    // after car is stuck for a while apply recovering policy
    if (stuck > stuckTime)
    {
        /* set gear and sterring command assuming car is
         * pointing in a direction out of track */
         // to bring car parallel to track axis
        float steer = cs.angle / steerLock;

        int gear = -1; // gear R

        // if car is pointing in the correct direction revert gear and steer  
        if (cs.angle * cs.trackPos > 0)
        {
            gear = 1;
            steer = -steer;
        }

        // Calculate clutching
        clutching(&cs, &clutch);

        // build a CarControl variable and return it
        structCarControl cc = { 0.25f,0.0f,gear,steer,clutch };
        return cc;
    }

    else // car is not stuck
#endif // !REVERSE
    {


#ifndef REVERSE
        // compute accel/brake command
        float accel_and_brake = getAccel(&cs);
        // compute gear 
        int gear = getGear(&cs);
        // compute steering
        float steer = getSteer(&cs);
#endif // !REVERSE

#ifdef REVERSE
        // compute accel/brake command
        float accel_and_brake = getAccelR(&cs);
        // compute gear 
        int gear = getGearR(&cs);
        // compute steering
        float steer = -1 * getSteerR(&cs);
#endif // !REVERSE

        // normalize steering
        if (steer < -1)
            steer = -1;
        if (steer > 1)
            steer = 1;

        // set accel and brake from the joint accel/brake command 
        float accel, brake;
        if (accel_and_brake > 0)
        {
            accel = accel_and_brake;
            brake = 0;
        }
        else
        {
            accel = 0;
            // apply ABS to brake
            brake = filterABS(&cs, -accel_and_brake);
        }

        // Calculate clutching
        clutching(&cs, &clutch);

        // build a CarControl variable and return it
        structCarControl cc = { accel,brake,gear,steer,clutch };
        return cc;
    }
}


float filterABS(structCarState* cs, float brake)
{
    // convert speed to m/s
    float speed = cs->speedX / 3.6;
    // when spedd lower than min speed for abs do nothing
    if (speed < absMinSpeed)
        return brake;

    // compute the speed of wheels in m/s
    float slip = 0.0f;
    for (int i = 0; i < 4; i++)
    {
        slip += cs->wheelSpinVel[i] * wheelRadius[i];
    }
    // slip is the difference between actual speed of car and average speed of wheels
    slip = speed - slip / 4.0f;
    // when slip too high applu ABS
    if (slip > absSlip)
    {
        brake = brake - (slip - absSlip) / absRange;
    }

    // check brake is not negative, otherwise set it to zero
    if (brake < 0)
        return 0;
    else
        return brake;
}

void ConShutdown()
{
    printf("Bye bye!");
}

void ConRestart()
{
    printf("Restarting the race!");
}

void clutching(structCarState* cs, float* clutch)
{
    float maxClutch = clutchMax;

    // Check if the current situation is the race start
    if (cs->curLapTime < clutchDeltaTime && cs->stage == RACE && cs->distRaced < clutchDeltaRaced)
        *clutch = maxClutch;

    // Adjust the current value of the clutch
    if (clutch > 0)
    {
        float delta = clutchDelta;
        if (cs->gear < 2)
        {
            // Apply a stronger clutch output when the gear is one and the race is just started
            delta /= 2;
            maxClutch *= clutchMaxModifier;
            if (cs->curLapTime < clutchMaxTime)
                *clutch = maxClutch;
        }

        // check clutch is not bigger than maximum values
        *clutch = fmin(maxClutch, *clutch);

        // if clutch is not at max value decrease it quite quickly
        if (*clutch != maxClutch)
        {
            *clutch -= delta;
            *clutch = fmax(0.0, *clutch);
        }
        // if clutch is at max value decrease it very slowly
        else
            *clutch -= clutchDec;
    }
}

//gives 19 angles for the distance sensors
void Cinit(float* angles)
{
    // set angles as {-90,-75,-60,-45,-30,-20,-15,-10,-5,0,5,10,15,20,30,45,60,75,90}
    enableScaredSteering = 1;
    for (int i = 0; i < 5; i++)
    {
        angles[i] = -90 + i * 15;
        angles[18 - i] = 90 - i * 15;
    }

    for (int i = 5; i < 9; i++)
    {
        angles[i] = -20 + (i - 5) * 5;
        angles[18 - i] = 20 - (i - 5) * 5;
    }
    angles[9] = 0;
}
