#pragma once
#include "AuxiliaryObjectiveFunction.h"
#include "ControlLib/Robot.h"
#include <OptimizationLib/BFGSFunctionMinimizer.h>
//#include <MathLib\Quaternion.h>
////////////////This version doesn't use any inverse Kinematics-----try that in another version----since it doesn't do any IK, there isn't any way to check for 
///////////////////collisions between fingers, or unknown collisions with the object it is manipulating 
///////////////////also, the projection step is thrown out, instead just using distance to the fingertip in lieu of that
////////////////////this version will only ever be able to do fingertip contacts with the object (only optimizing fingertip contacts on the object)
/////////////////////////we are still assuming that the object ot be manipulated is a capsule

#define NULLPOSITION V3D(INFINITY,INFINITY,INFINITY)  //null equivalents of position and orientation
#define NULLQUATERNION Quaternion(0,0,0,0)

class MordatchHandsObjective : public AuxiliaryObjectiveFunction {
protected:
	double calculateQuaternionDistance(Quaternion q1, Quaternion q2);
	Quaternion SLERPobjectOrientation(double time);
	Quaternion getQuaterionFromRPY(double r, double p, double y);
	void getRPYFromQuaternion(Quaternion q, double& r, double &p, double& y);
	V3D projectOntoObject(V3D localCoord); //////Assumes that the object is a capsule (important)
	V3D	convertFromLocalToWorld(V3D localPoint, double time);
	V3D	convertFromWorldToLocal(V3D worldPoint, double time);
	double getObjectPenetrationDepth(V3D worldCoord, double time);
	V3D multiplyMatrixByVector(Matrix3x3 mat, V3D v);
	void fillUpContactForceMap(double time);
public:
////TODO:::make a method sending all this info to the menu for easy parameter adjustment, and add on a better way for capturing objective positions and orientations
	
	///////TODO::::::::easy weight adjustment from gui, also make sure that you make all regularizer and weight parameters independent,,and normalize by number of samples--adjust in spaceship for testing
	double t_phys = .05;
	
	double physics_weight = 5;
	double kinematic_weight = 1.0;
	double task_weight = 20.0;  //task weight might not be high enough
	double ci_weight = 5.0;
//////////////////////////////TODO::::::there is a problem with the fingertips not sticking onto the object-----try setting the ci weight again, and making a 
	///////tradeoff parameter for the slippage and make it 0 to test only the projection of points onto the object

	//////TODO:::check the ci more rigorously
	
	
	double force_regularizer = .01;
	double angMomentumToLinMomentumTradeoff = .5;
	double penetrationPenalty = 1;  //set this to high since it should be a somewhat stiff constraint
	double contactInvariantPenalty = 1000; //this enforces the contact invariant weights c to remain between 0 and 1


	double objectMass;
	Matrix3x3 objectInertia; //this is the inertia in the object's local frame
	map<string, double> maxDistanceFromPalm; //needs the hand model object, if none is given then a default value is made up
	CapsuleCDP objectCapsule; //NOTE: assumes the object to be grasped is a capsule----later generalize to other geometries

	///these can be changed by the optimize function when loading a control policy as a seed
	KeyFrame initialFrame;
	V3D initialGravityContactForce;
	V3D	initialGravityContactPointLocal;
	map<string, V3D> initialFingertipPosition;
	vector<double> trajOptTimes; //does not include time 0---this replaces objectkeyframes

	//these are not modified by calls to computeValue
	//each of these task vectors needs to have the same length--to specify a task parameter as irrelevent, just designate with NULL
	vector<double> taskObjectiveTimes;
	vector<V3D> taskObjectivePositions;
	vector<Quaternion> taskObjectiveOrientations;

	//////these structures are all modified by each call to compute value
	map<string, GenericTrajectory<V3D>> fingertipPositions; //includes palm position too 
//	vector<Quaternion> palmOrientation; //time values are the same as the trajectories------not even used yet
	GenericTrajectory<V3D> objectPosition; // x_o,
	vector<Quaternion> objectOrientation; //x_o  (set with r,p,y angles and uses SLERP)
	map<string, contactForceMapObject> contactsMap; //weight(c_j) origin(local) force   f_j,r_j_local,c_j


	MordatchHandsObjective(vector<vector<double>> objectKeyFrames, Robot* handRobot, RigidBody* objectRobot, char* additionalCommands);

	virtual ~MordatchHandsObjective();

	///specify positions or orientations you don't care about with their NULL equivalents (above)
	void setObjectiveTimesAndPositions(vector<double> taskObjectiveTimes, vector<V3D> taskObjectivePositions, vector<Quaternion> taskObjectiveOrientations)
	{
		this->taskObjectiveTimes = taskObjectiveTimes;
		this->taskObjectivePositions = taskObjectivePositions;
		this->taskObjectiveOrientations = taskObjectiveOrientations;
	}

	//this should always return the current value of the objective function
	virtual double computeValue(const dVector& p);

	virtual GodModeControlPolicy* optimizePolicy();
	virtual GodModeControlPolicy* optimizePolicy(GodModeControlPolicy* seed);
};

