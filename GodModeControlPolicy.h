#pragma once
///Note this is not a controllerInterp object despite the similarities:::that is to reinforce the point that this is not 
//a real control policy--it is more like an animation for a trajectory motion plan

/////////////////////////This probably isnt the most generic way to do the contact invariant optimization-------it is mostly just suited to the Mordatch Hands objective function
/////////////////////////////Also, trajectories are assumed to have evenly spaced keyframes----change it later if you need to
////////////////////////At least for now it only adjusts the object--nothing else
#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <iterator>
#include <Utils/Utils.h>
#include <MathLib/Trajectory.h>
#include <RBSimLib/AbstractRBEngine.h>
#include "stdio.h"

using namespace std;

struct contactForceMapObject { 
	GenericTrajectory<double> contactWeights; //In mordatch: c_j  (the contact invariant weights for each contact)
	GenericTrajectory<V3D> contactOriginsLocal; //r_j (local coords of simpleObject)
	GenericTrajectory<V3D> contactForces; //f_j
};
///Note::remember: never use P3D with a generic trajectory---always just use V3D instead
class GodModeControlPolicy {
protected:
//	string description=""; //this is the description passed to it when it is created---only for the text file just so you know what its for
	void calculateTimeVariables(double time, int& lowerIndex, int& upperIndex, double& timeFraction);

public:
	vector<string> frozenRigidBodyNames; //the list of rigid bodies this freezes::TODO: add in joints later if you need to

	map<string, contactForceMapObject> contactsMap; //this is the collection of objects at contact index j---each index j has a string name (could just be "j")
	GenericTrajectory<V3D> objectPositions; //x_O
	vector<Quaternion> objectOrientations; ///x_O, we are using SLERP here, so it is treated like a trajectory--time values are the same as the knots in the trajectory
	map<string, GenericTrajectory<V3D>> handLandmarkLocations; //p_i and x_H---fingertips and palm (world coords): for animation purposes, this just draws them to give you hand pose
//	vector<Quaternion> palmOrientations; //x_H, again using slerp----------Mordatch Hands needs it, but I don't think control policy needs it

	/*x_dot for both the hand and the object and p_dot are left out since they apparently are only useful in calculating the cubic-hermite spline, but we are
	 using catmull-rom splines and SLERP, so we don't need derivatives, which should make things simpler hopefully---you may end up actually needing them*/

	//this constructor reads the god mode control policy from a file
	GodModeControlPolicy(){
		frozenRigidBodyNames = { "simpleObject" };//this is only good on the simple object for now
		contactsMap = {};
		objectPositions = GenericTrajectory<V3D>();
		objectOrientations = {};
		handLandmarkLocations = {};
	}
	GodModeControlPolicy(string filepath)
	{
		cout << filepath;
		readFromFile(filepath.c_str());
	}
	virtual ~GodModeControlPolicy() {
	}

	//read/write policy from file
	virtual void readFromFile(const char* filepath); 
	virtual void writeToFile(char* filepath);


	//sends commands to the ODERBEngine to affect the simulation (called by every iteration of the process loop)
	virtual void getControlCommands(AbstractRBEngine* rbEngine, double controlTime, vector<P3D> &pointsToDraw,
		vector<V3D> &forceVectorsOnObjectToDraw, vector<P3D> &forceContactsOnObjectToDraw,bool forceonly);

	//Note: the actual prepping is done in the sim (in GenericController command line)
	/////This just returns the objects to freeze, sim handles the rest----for now you can only freeze/unfreeze the simpleObject, not the hand or any part of it
	virtual vector<string> prepSim();
	virtual vector<string> revertSim();
};