#include "MordatchHandsObjective.h"

///From now on do stuff like this to rigorously test stuff from the bottom up
#define NORMAL_PRINT true
#define PRINT_FORCE_ERRORS false //true
#define DEBUG false
#define DEBUG2 false
#define DEBUG_ANGULAR_MOMENTUM false
#define DISABLE_GRAVITY true ///makes the object completely ignore any contact with the ground
#define INCLUDE_SLIPPAGE_TERM false //true (should be true in full objective function)

MordatchHandsObjective::MordatchHandsObjective(vector<vector<double>> objectKeyFrames, Robot* handRobot, RigidBody* object, char* additionalCommands)
{
	////NOTE:::::this makes heavy use of the assumption we have a capsule object, each fingertip is a capsule, and the palm is a box
	objectMass = object->rbProperties.mass;
	objectInertia = object->rbProperties.MOI_local;
	initialFrame.time = 0;
	initialFrame.p = P3D(objectKeyFrames[0][0], objectKeyFrames[0][1], objectKeyFrames[0][2]);
	initialFrame.q = Quaternion(objectKeyFrames[0][3], objectKeyFrames[0][4], objectKeyFrames[0][5], objectKeyFrames[0][6]);
	
	//this is the palm position marker point
	V3D ppos = V3D(handRobot->getRBByName("palm")->getWorldCoordinates(handRobot->getRBByName("palm")->rbProperties.bodyPointFeatures[0].coords));
	initialFingertipPosition.insert(pair<string, V3D>{"palm", ppos});
	
	//assume the robot passed in is the handCorrected.rbs file with the fingers sprawled out in the default stance----assumes palm is facing downward
	for (string fingertip : handRobot->getFingertips()) {
		//this gives the surface of the fingertip (at the center) and the surface of the palm at the center
		V3D fpos =V3D(handRobot->getRBByName(fingertip.c_str())->getWorldCoordinates(handRobot->getRBByName(fingertip.c_str())->rbProperties.bodyPointFeatures[0].coords));
		//these max distances are actually under-approximated, but good enough for now
		maxDistanceFromPalm.insert(pair<string, double>{fingertip,(fpos-ppos).norm()});
		initialFingertipPosition.insert(pair<string, V3D>{fingertip,fpos});

		if (DEBUG) {
			cout << "fingertip initial position " << fingertip << " " << fpos[0] << " " << fpos[1] << " " << fpos[2] << "\n";
			cout << "maxDistanceFromPalm " << fingertip << " " << maxDistanceFromPalm[fingertip]<< "\n";
		}
	}

	if (DEBUG)
	{
		cout << "Object Initial position " << initialFrame.p[0] << " " << initialFrame.p[1] << " " << initialFrame.p[2] << "\n";
		cout << "Object Initial orientation " << initialFrame.q[0] << " " << initialFrame.q[1] << " " << initialFrame.q[2] <<" " << initialFrame.q[3]<<"\n";
		cout<< "palm Initial position " << ppos[0] << " " << ppos[1] << " " << ppos[2] << "\n";
	}


/////////////////////////////TODO:::::::check that each of the points you have as possible contact points is correct and makes no assumptions about the object in simulation
//////////////////////////////////TODO:::::::this assumes the capsule is lying flat on the ground----i.e. the contact point is diecctly underneath the center of mass, and thhe object is horizontal 
////////////////////////////////on the table-------TODO::::::fix this so that this isn;t assumed--------instead change it so you look at marker points on the object defined in the rbs file
	
	CapsuleCDP* objCap=(CapsuleCDP*)object->cdps[0];
	objectCapsule = CapsuleCDP(objCap->p1, objCap->p2, objCap->r);

	
	initialGravityContactPointLocal = V3D(0, -objectCapsule.r, 0);
	initialGravityContactForce = V3D(0, -GRAVITY_ACCELLERATION*objectMass, 0); //need a negative here since the contact force opposes gravity
	
	if (DEBUG)
	{
		cout << "initialGravityContactPointLocal " << initialGravityContactPointLocal[0] << " " << initialGravityContactPointLocal[1] << " " << initialGravityContactPointLocal[2] << "\n";
		V3D p1 = projectOntoObject(V3D(.1,0,0));
		V3D p2 = projectOntoObject(V3D(-.75,0.01,0.01));
		V3D p3 = projectOntoObject(V3D(-3,0,0));
		V3D p4 = projectOntoObject(V3D(.3,2,3));
		cout << "p1 " << p1[0] << " " << p1[1] << " " << p1[2] << "\n";
		cout << "p2 " << p2[0] << " " << p2[1] << " " << p2[2] << "\n";
		cout << "p3 " << p3[0] << " " << p3[1] << " " << p3[2] << "\n";
		cout << "p4 " << p4[0] << " " << p4[1] << " " << p4[2] << "\n";
		double roll = .2; double pitch = .35; double yaw = .5;
		Quaternion q = getQuaterionFromRPY(roll, pitch, yaw);
		cout << "q " << q[0] << " " << q[1] << " " << q[2] <<" "<<q[3]<< "\n";
		roll = 0; pitch = 0; yaw = 0;
		getRPYFromQuaternion(q, roll, pitch, yaw);
		cout << "r "<< roll<< " p "<<pitch << " y "<< yaw<< "\n";
	}

	double t_keyframe = 1 / (objectKeyFrames.size() - 1.0);
	for (uint i = 1; i < objectKeyFrames.size(); i++)
		trajOptTimes.push_back(t_keyframe*i); //keyframes are evenly spaced
	

	fingertipPositions = {};
	objectPosition = GenericTrajectory<V3D>(); // x_o,
	objectOrientation = {}; //x_o  (set with r,p,y angles and uses SLERP)
	contactsMap = {}; //weight(c_j) origin(local) force   f_j,r_j_local,c_j

	//initialize fingertip positions map
	for (auto ent : initialFingertipPosition) {
		fingertipPositions.insert(pair<string, GenericTrajectory<V3D>>{ent.first, GenericTrajectory<V3D>()});
		contactForceMapObject obj;
		obj.contactWeights = GenericTrajectory<double>();
		obj.contactOriginsLocal = GenericTrajectory<V3D>();
		obj.contactForces = GenericTrajectory<V3D>();
		contactsMap.insert(pair<string, contactForceMapObject>{ent.first,obj});
	}
	//include both ground force points in the contact map
	for (string s : {"ground1", "ground2"})
	{
		contactForceMapObject obj;
		obj.contactWeights = GenericTrajectory<double>();
		obj.contactOriginsLocal = GenericTrajectory<V3D>();
		obj.contactForces = GenericTrajectory<V3D>();
		contactsMap.insert(pair<string, contactForceMapObject>{s, obj});
	}

	for (uint i = 0; i < objectKeyFrames.size(); i++)
	{
		double time = t_keyframe*i;
		///times here need to match up wth trajOptTimes (plus the initial position)
		objectPosition.addKnot(time,V3D(objectKeyFrames[i][0], objectKeyFrames[i][1], objectKeyFrames[i][2]));
		objectOrientation.push_back(Quaternion(objectKeyFrames[i][3], objectKeyFrames[i][4], objectKeyFrames[i][5], objectKeyFrames[i][6]));	
	
	///NOTE:::auto assumes const-----don't use it when modifying values in a map		
		for	(std::map<string, GenericTrajectory<V3D>>::iterator ent = fingertipPositions.begin(); ent != fingertipPositions.end(); ++ent)
		{
			V3D initLocal = convertFromWorldToLocal(initialFingertipPosition[ent->first], 0);
			ent->second.addKnot(time, convertFromLocalToWorld(initLocal,time));
		}

		fillUpContactForceMap(time);
	}


	if (DEBUG)
	{
		V3D worldPoint = V3D(1, 2, 3);
		V3D localPoint = convertFromWorldToLocal(worldPoint, 0);
		V3D worldPoint2 = convertFromLocalToWorld(localPoint, 0);
		cout << "worldPoint " << worldPoint[0] << " " << worldPoint[1] << " " << worldPoint[2] << "\n";
		cout << "localPoint " << localPoint[0] << " " << localPoint[1] << " " << localPoint[2] << "\n";
		cout << "worldPoint2 " << worldPoint2[0] << " " << worldPoint2[1] << " " << worldPoint2[2] << "\n";
		assert(worldPoint == worldPoint2);

		cout << "Fingertips map: \n";
		for (auto ent : fingertipPositions) {
				cout << "fingertip " << ent.first << "\n";
				cout << "num knots: " << ent.second.getKnotCount() << "\n";
				cout << "first knot: " << ent.second.getKnotValue(0)[0]<<" "<< ent.second.getKnotValue(0)[1]<<" "<< ent.second.getKnotValue(0)[2] << "\n";
		}
	}
	





	//taskObjectiveTimes, taskObjectivePositions, and taskObjectiveOrientations all still need to be specified by the input function (called after
	//initialization (make a thing in the GUI to do this))----for now this is here for testing
	taskObjectiveTimes = { 1.0 };
	taskObjectivePositions = { V3D(-1,.5,.5) };
	taskObjectiveOrientations = { NULLQUATERNION };

	if (DEBUG)
	{
		GodModeControlPolicy* policy = new GodModeControlPolicy();
	//	computeValue(params);
		policy->contactsMap = contactsMap;
		policy->objectPositions = objectPosition;
		policy->objectOrientations = objectOrientation;
		policy->handLandmarkLocations = fingertipPositions;
		cout << "writing starting policy to initial file\n";
		policy->writeToFile((char*)("..\\data\\controlPolicies\\" + string("initialPolicy") + ".godmode").c_str());
		delete policy;
	}



	cout << "Done initializing";
}

MordatchHandsObjective::~MordatchHandsObjective() {}

//this should always return the current value of the objective function
double MordatchHandsObjective::computeValue(const dVector& p) {
	
//	x_h, p_i,x_o,f_j,r_j_local,c_j
//	N_contacts=N_fingers+3 (1 for the palm, 2 for the ground----these contacts are kind of special)
//	indexes j   do orientations with roll,pitch,yaw

	if (DEBUG)
		cout << "Staring computeValue\n";

	//corresponds to how variables are blocked together in p---numfingertips+1 since it includes palm
	int entriesPerKeyframe = 6 + fingertipPositions.size() * 3 + 7 * contactsMap.size();
	
	//fill up the trajectories
	objectPosition = GenericTrajectory<V3D>();
	objectOrientation = {};
	objectPosition.addKnot(0, V3D(initialFrame.p));
	objectOrientation.push_back(initialFrame.q);
	//empty the map values and reinitialize them
	for (std::map<string, GenericTrajectory<V3D>>::iterator ent = fingertipPositions.begin(); ent != fingertipPositions.end(); ++ent) {
		ent->second = GenericTrajectory<V3D>();
		ent->second.addKnot(0, initialFingertipPosition[ent->first]);
	}
	/////we are assuming that we are not starting in a state where there is any contact except for a single contact point on the ground
	for (std::map<string, contactForceMapObject>::iterator ent = contactsMap.begin(); ent != contactsMap.end(); ++ent)
	{
		ent->second.contactWeights = GenericTrajectory<double>();
		ent->second.contactOriginsLocal = GenericTrajectory<V3D>();
		ent->second.contactForces = GenericTrajectory<V3D>();
	}
	fillUpContactForceMap(0);



	double L_contactInvariantBounds = 0; //this bounds the c_i (contact weights) to lie in [0,1]

	//fill up the components from the object keyframes
	for (uint i = 0; i < trajOptTimes.size(); i++)
	{
		int startIndex = entriesPerKeyframe*i;

		objectPosition.addKnot(trajOptTimes[i], V3D(p(startIndex), p(startIndex + 1), p(startIndex + 2)));
		objectOrientation.push_back(getQuaterionFromRPY(p(startIndex+3), p(startIndex+4), p(startIndex+5)));
		//an iterator over a map is garunteed to return the keys in ascending order
		int fingtipNum = 0;
		for (std::map<string, GenericTrajectory<V3D>>::iterator ent = fingertipPositions.begin(); ent != fingertipPositions.end(); ++ent) {
			ent->second.addKnot(trajOptTimes[i], V3D(p(startIndex+6+3*fingtipNum), p(startIndex + 7 + 3 * fingtipNum), p(startIndex + 8 + 3 * fingtipNum)));
			fingtipNum++;
		}
		int frcmapstartidx = startIndex+ 6 + fingertipPositions.size() * 3;
		int ct = 0;
		for (std::map<string, contactForceMapObject>::iterator ent = contactsMap.begin(); ent != contactsMap.end(); ++ent) {
			double ci_weight = p(frcmapstartidx + 7 * ct);
			//bound ci_weight between 0 and 1
			if (ci_weight < 0)
			{
				L_contactInvariantBounds += ci_weight*ci_weight*contactInvariantPenalty;
				ci_weight = 0;
			}
			else if (ci_weight > 1)
			{
				L_contactInvariantBounds += (ci_weight-1)*(ci_weight-1)*contactInvariantPenalty;
				ci_weight = 1;
			}
			if(DISABLE_GRAVITY&&(ent->first=="ground1"||ent->first=="ground2"))
				ci_weight = 0;

			ent->second.contactWeights.addKnot(trajOptTimes[i], ci_weight);
			ent->second.contactOriginsLocal.addKnot(trajOptTimes[i],V3D(p(frcmapstartidx + 7 * ct+1), p(frcmapstartidx + 7 * ct+2), p(frcmapstartidx + 7 * ct+3)));
			ent->second.contactForces.addKnot(trajOptTimes[i], V3D(p(frcmapstartidx + 7 * ct+4), p(frcmapstartidx + 7 * ct+5), p(frcmapstartidx + 7 * ct+6)));
			ct++;
		}
	}



	if (DEBUG2)
	{
		GodModeControlPolicy* policy = new GodModeControlPolicy();
		policy->contactsMap = contactsMap;
		policy->objectPositions = objectPosition;
		policy->objectOrientations = objectOrientation;
		policy->handLandmarkLocations = fingertipPositions;
		cout << "writing starting policy to initial file\n";
		policy->writeToFile((char*)("..\\data\\controlPolicies\\" + string("computeValuePolicy") + ".godmode").c_str());
		delete policy;
	}




	////////////////////////////L_ci (modified),L_physics,L_cone(omitted--can't do without inverse kinematics),L_kinematic(simplified),L_task (orientattion and position of object)
	double lastTime = trajOptTimes[trajOptTimes.size() - 1];
	double time = 0;
	//L_cone omitted
	double L_physics = 0;
	double L_kinematic = 0;
	double L_task = 0;
	double L_ci = 0;
	double L_forceReg = 0;
	double L_linMomentum = 0;
	double L_angMomentum = 0;
	double L_penetration = 0; //this is factored into the L_kinematic cost in the objective function
	while (time<lastTime+t_phys/2) {
		//L_ci (contact invariant)---simplification, no projection onto the body of contact, instead just minimizing distance to markerpoint of object (in this case just fingertips and palm)
		for (auto ent : contactsMap) {
			L_ci += ent.second.contactWeights.evaluate_piecewise_constant(time)*(
				//e_j,object
				(projectOntoObject(ent.second.contactOriginsLocal.evaluate_linear(time)) - ent.second.contactOriginsLocal.evaluate_linear(time)).squaredNorm());
				//e_j_dot,object
			if(INCLUDE_SLIPPAGE_TERM)
				L_ci += ent.second.contactWeights.evaluate_piecewise_constant(time)*(((projectOntoObject(ent.second.contactOriginsLocal.evaluate_linear(time)) -
					ent.second.contactOriginsLocal.evaluate_linear(time) -
				 (projectOntoObject(ent.second.contactOriginsLocal.evaluate_linear(time - t_phys)) - ent.second.contactOriginsLocal.evaluate_linear(time - t_phys))) / t_phys).squaredNorm());
			//now "projection" and slippage onto the fingertips, palm, and ground (foreign objects)
			if (ent.first == "ground1" || ent.first == "ground2")
			{
				//ground points---projection is just setting the y coordinate to 0, and slippage is pretty easy
				V3D cp=convertFromLocalToWorld(ent.second.contactOriginsLocal.evaluate_linear(time),time);
				V3D cpprev = convertFromLocalToWorld(ent.second.contactOriginsLocal.evaluate_linear(time-t_phys),(time-t_phys));
				
				L_ci += ent.second.contactWeights.evaluate_piecewise_constant(time)*(
					//e_j,ground
					(V3D(cp[0], 0, cp[2]) - cp).squaredNorm());
				if (INCLUDE_SLIPPAGE_TERM)
					//e_j_dot,ground
					L_ci += ent.second.contactWeights.evaluate_piecewise_constant(time)*((( (V3D(cp[0], 0, cp[2]) - cp) - (V3D(cpprev[0], 0, cpprev[2]) - cpprev)) / t_phys).squaredNorm());
			}
			else
			{
				//palm and fingertips are treated the same way
				//"projection" is just distance to the marker point (fingertip position) of the finger or palm marker and slip is just the change in this
				//note that this definition of projection can and will easily lead to premature collisions with the hand
				V3D cp = convertFromLocalToWorld(ent.second.contactOriginsLocal.evaluate_linear(time),time);
				V3D cpprev = convertFromLocalToWorld(ent.second.contactOriginsLocal.evaluate_linear(time - t_phys), (time - t_phys));
				
				
				L_ci += ent.second.contactWeights.evaluate_piecewise_constant(time)*(
					//e_j,fingertip/palm
					(fingertipPositions[ent.first].evaluate_catmull_rom(time) - cp).squaredNorm());
				if (INCLUDE_SLIPPAGE_TERM)
					//e_j_dot,fingertip/palm
					L_ci += ent.second.contactWeights.evaluate_piecewise_constant(time)*((((fingertipPositions[ent.first].evaluate_catmull_rom(time) - cp) -
						(V3D(fingertipPositions[ent.first].evaluate_catmull_rom(time-t_phys) - cpprev))   ) / t_phys).squaredNorm());
			}
		}

		////L_cone(omitted--can't do without inverse kinematics)----would have gone in above block iterating over all contact points

		/////////L_kinematic(simplified)----original Mordatch (with IK) has joint limits, distance constraints, and penalization of collisions
		///////////////we are ignoring the joint limits and the penalization of collisions since we have no way of knowing those without IK
		////////////////////so we are just stuck with fingertip distance constraints----by the way fingers are independent so all that matters is fingertip distance 
		///////////////////////to palm
		//////////////////////////////////////also, we are putting in a rather stiff penalty for penetrating simple object
		
		for (auto ent : fingertipPositions)
		{
			if (ent.first != "palm")
			{
				double violation = (ent.second.evaluate_catmull_rom(time) - fingertipPositions["palm"].evaluate_catmull_rom(time)).norm() - maxDistanceFromPalm[ent.first];
				if (violation > 0)
					L_kinematic += violation*violation;
			}
			L_penetration += (penetrationPenalty/ kinematic_weight)*getObjectPenetrationDepth(ent.second.evaluate_catmull_rom(time),time);
		}
		


		//calculate L_physics
		V3D f_total= V3D(0, GRAVITY_ACCELLERATION, 0)*objectMass; //total force
		V3D m_total = V3D(0,0,0); //total moment--------we're assuming there aren't any external moments (that is since there aren't any external contact forces)
		for (auto ent : contactsMap) {
			L_forceReg += force_regularizer*(ent.second.contactForces.evaluate_linear(time).squaredNorm());
			f_total += ent.second.contactForces.evaluate_linear(time)*ent.second.contactWeights.evaluate_piecewise_constant(time);
			//this is saying torque=-fXr
			m_total += -ent.second.contactForces.evaluate_linear(time).cross(convertFromLocalToWorld(ent.second.contactOriginsLocal.evaluate_linear(time),time) - 
				objectPosition.evaluate_catmull_rom(time))*ent.second.contactWeights.evaluate_piecewise_constant(time);
		}
		V3D linearMomentumChange = (objectPosition.evaluate_catmull_rom(time + t_phys) - objectPosition.evaluate_catmull_rom(time) * 2 +
			objectPosition.evaluate_catmull_rom(time - t_phys)) * (objectMass / (t_phys*t_phys)); //P_dot----change in linear momentum


	    //L_dot---this part is a little tricky

		//////you have this set as going from local to world for the orientations
		Quaternion q = SLERPobjectOrientation(time+t_phys)*SLERPobjectOrientation(time).getInverse(); 
		V3D axis; double angle;
		q.getAxisAngle(axis, angle); 
		V3D angularVelocity = axis*(angle/t_phys); //at current timestep
	
		Quaternion qprev = SLERPobjectOrientation(time)*SLERPobjectOrientation(time - t_phys).getInverse(); 
		V3D axisprev; double angleprev;
		qprev.getAxisAngle(axisprev, angleprev);
		V3D angularVelocityprev = axisprev*(angleprev / t_phys); //at previous timestep

		//the ineria matrix is in the object's local coordinate frame, we need the inertia matrix in the world frame
		Matrix3x3 objectInertiaWorld = SLERPobjectOrientation(time).getInverse().getRotationMatrix()*objectInertia*SLERPobjectOrientation(time).getRotationMatrix();
		V3D angularMomentumChange = angularVelocity.cross(multiplyMatrixByVector(objectInertiaWorld,angularVelocity)) + 
			multiplyMatrixByVector(objectInertiaWorld,((angularVelocity - angularVelocityprev) / t_phys));


		


		L_linMomentum += (f_total - linearMomentumChange).squaredNorm();
		if (DEBUG_ANGULAR_MOMENTUM)
		{
			cout << "m_total " << m_total[0] <<" "<< m_total[1]<<" " << m_total[2] << "\n";
			cout << "angularMomentumChange " << angularMomentumChange[0] << " "<<angularMomentumChange[1] <<" "<< angularMomentumChange[2] << "\n";
		}

		L_angMomentum += (m_total - angularMomentumChange).squaredNorm()*angMomentumToLinMomentumTradeoff;
		L_forceReg /= physics_weight; //this is to make the regularization parameter independent of physics_weight
		L_physics = L_linMomentum+ L_angMomentum + L_forceReg;
		

		if (PRINT_FORCE_ERRORS)
		{
			cout << "time " << time << "\n";
			cout << "lastTime " << lastTime << "\n";
			cout << "actual f_total: " << f_total[0] << " " << f_total[1] << " " << f_total[2] << "\n";
			cout << "linear momentum change: " << linearMomentumChange[0] << " " << linearMomentumChange[1] << " " << linearMomentumChange[2] << "\n";
			cout << "linear momentum error (non-normalized): " << L_linMomentum << "\n";
			cout << "actual m_total: " << m_total[0] << " " << m_total[1] << " " << m_total[2] << "\n";
			cout << "angular momentum change: " << angularMomentumChange[0] << " " << angularMomentumChange[1] << " " << angularMomentumChange[2] << "\n";
			cout << "angular momentum error (non-normalized): " << L_angMomentum << "\n";
			cout << "current pos: " << objectPosition.evaluate_catmull_rom(time)[0] << " " << objectPosition.evaluate_catmull_rom(time)[1] << " " << objectPosition.evaluate_catmull_rom(time)[2] << "\n";
			cout << "next pos: " << objectPosition.evaluate_catmull_rom(time + t_phys)[0] << " " << objectPosition.evaluate_catmull_rom(time + t_phys)[1] << " " << objectPosition.evaluate_catmull_rom(time + t_phys)[2] << "\n";
			cout << "prev pos: " << objectPosition.evaluate_catmull_rom(time - t_phys)[0] << " " << objectPosition.evaluate_catmull_rom(time - t_phys)[1] << " " << objectPosition.evaluate_catmull_rom(time - t_phys)[2] << "\n";
			cout << "current orientation: " << SLERPobjectOrientation(time)[0] << " " << SLERPobjectOrientation(time)[1] << " " << SLERPobjectOrientation(time)[2] << " " << SLERPobjectOrientation(time)[3] << "\n";
			cout << "L_penetration: " << L_penetration << "\n";
			cout << "\n";
		}
		
		time += t_phys;
	} //end of the loop summing objectives over time
	
	//calculate L_task:::either or both positions and orientations for each objective time----each list must have the same number of elements though ssince they corespond
	///to specify that something doesn't matter, it's entry can just be NULL
	
	for (uint i = 0; i < taskObjectiveTimes.size(); i++)
	{
		if(taskObjectivePositions[i]!=NULLPOSITION)
			L_task += (objectPosition.evaluate_catmull_rom(taskObjectiveTimes[i]) - taskObjectivePositions[i]).squaredNorm();
		if (taskObjectiveOrientations[i] != NULLQUATERNION)
			L_task += pow(calculateQuaternionDistance(taskObjectiveOrientations[i], SLERPobjectOrientation(taskObjectiveTimes[i])),2);
	}

	//normalize components of L_physics by number of samples---note none of these are part of the objective--just for analysis purposes
	L_linMomentum*= t_phys / lastTime;
	L_angMomentum *= t_phys / lastTime;
	L_forceReg *= t_phys / lastTime;

	L_kinematic += L_penetration;
	///add all of the L components here
	//L_cone omitted
	L_physics *= physics_weight*t_phys / lastTime; //normalize by number of samples
	L_kinematic *= kinematic_weight*t_phys / lastTime; //normalize by number of samples
	L_task *= task_weight/taskObjectiveTimes.size(); //normalize by number of tasks
	L_ci *= ci_weight*t_phys / lastTime; //normalize by number of samples
	if (NORMAL_PRINT) {
		std::cout << "L_linMomentum: " << L_linMomentum << "\n";
		std::cout << "L_angMomentum: " << L_angMomentum << "\n";
		std::cout << "L_forceReg: " << L_forceReg << "\n";
		std::cout << "L_physics: " << L_physics << "\n";
		std::cout << "L_kinematic: " << L_kinematic << "\n";
		std::cout << "L_penetration: " << L_penetration* kinematic_weight*t_phys / lastTime << "\n";
		std::cout << "L_ci: " << L_ci << "\n";
		std::cout << "L_contactInvariantBounds: " << L_contactInvariantBounds << "\n";
		std::cout << "L_task: " << L_task << "\n\n";
		
	}
	return L_physics + L_kinematic+ L_ci+ L_task+ L_contactInvariantBounds;
}

GodModeControlPolicy* MordatchHandsObjective::optimizePolicy() {
	if (DEBUG)
		cout << "Start policy optimization\n";


	//prepare the params vector (assuming the current parameters are set appropriately, including initial conditions and the things changed in the computeValue function)
	int entriesPerKeyframe = 6 + fingertipPositions.size() * 3 + 7 * contactsMap.size(); //corresponds to how variables are blocked together in params
	dVector params(entriesPerKeyframe*trajOptTimes.size());
	for (int i = 0; i < trajOptTimes.size(); i++)
	{
		int startIndex = entriesPerKeyframe*i;
		V3D v = objectPosition.getKnotValue(i + 1);
		params(startIndex) = v[0];
		params(startIndex + 1) = v[1];
		params(startIndex + 2) = v[2];
		Quaternion q = objectOrientation[i + 1];
		double r; double p; double y;
		getRPYFromQuaternion(q, r, p, y);
		params(startIndex + 3) = r;
		params(startIndex + 4) = p;
		params(startIndex + 5) = y;

		//an iterator over a map is garunteed to return the keys in ascending order
		int fingtipNum = 0;
		for (auto ent : fingertipPositions) {
			V3D v1 = ent.second.getKnotValue(i + 1);
			params(startIndex + 6 + 3 * fingtipNum) = v1[0];
			params(startIndex + 7 + 3 * fingtipNum) = v1[1];
			params(startIndex + 8 + 3 * fingtipNum) = v1[2];
			fingtipNum++;
		}

		int frcmapstartidx = startIndex + 6 + fingertipPositions.size() * 3;
		int ct = 0;
		for (auto ent : contactsMap) {
			params(frcmapstartidx + 7 * ct) = ent.second.contactWeights.getKnotValue(i + 1);
			V3D v1 = ent.second.contactOriginsLocal.getKnotValue(i + 1);
			V3D v2 = ent.second.contactForces.getKnotValue(i + 1);
			params(frcmapstartidx + 7 * ct + 1) = v1[0];
			params(frcmapstartidx + 7 * ct + 2) = v1[1];
			params(frcmapstartidx + 7 * ct + 3) = v1[2];
			params(frcmapstartidx + 7 * ct + 4) = v2[0];
			params(frcmapstartidx + 7 * ct + 5) = v2[1];
			params(frcmapstartidx + 7 * ct + 6) = v2[2];
			ct++;
		}
	}


	cout << "beginning minimization\n";
	int p_maxIterations = 150;//100,250
	double p_solveResidual = 0.01;
	int p_maxLineSearchIterations = 15;
	bool p_printOutput = true;
	BFGSFunctionMinimizer optimizer = BFGSFunctionMinimizer(p_maxIterations, p_solveResidual, p_maxLineSearchIterations, p_printOutput);
	double functionValue = 10000000000;
	//if (DEBUG2)
//	{
//		computeValue(params);
//	}
//	else
		optimizer.minimize((ObjectiveFunction *)this, params, functionValue);

	cout << "\noptimization function value: " << functionValue << "\n";

	//unwrap params into the godmodecontrolpolicy
	GodModeControlPolicy* policy = new GodModeControlPolicy();
	computeValue(params);
	policy->contactsMap = contactsMap;
	policy->objectPositions = objectPosition;
	policy->objectOrientations = objectOrientation;
	policy->handLandmarkLocations = fingertipPositions;

	return policy;
}

GodModeControlPolicy* MordatchHandsObjective::optimizePolicy(GodModeControlPolicy* seed) {
	//load up the seed into this function's parameters
	contactsMap= seed->contactsMap;
	objectPosition= seed->objectPositions;
	objectOrientation=seed->objectOrientations;
	fingertipPositions=seed->handLandmarkLocations;
	//set up the initial frame and initial values
	initialFrame.time = 0;
	initialFrame.p = P3D(objectPosition.getKnotValue(0));
	initialFrame.q = objectOrientation[0];

	for (std::map<string, GenericTrajectory<V3D>>::iterator ent = fingertipPositions.begin(); ent != fingertipPositions.end(); ++ent)
		initialFingertipPosition[ent->first] = ent->second.getKnotValue(0);

	//for the contacts map
	initialGravityContactPointLocal = contactsMap["ground1"].contactOriginsLocal.getKnotValue(0);
	initialGravityContactForce= contactsMap["ground1"].contactForces.getKnotValue(0);
	//////TODO:::might need 2 gravity contact points

	trajOptTimes = {};
	for (uint i = 1; i < objectPosition.getKnotCount(); i++)
		trajOptTimes.push_back(objectPosition.getKnotPosition(i));

	return optimizePolicy();
}




/////Auxiliary methods used above
/////////////////////////////TODO put in the way to specify objectives via command line in the gui (probably by just reading a keyframe file)--then figure out how to put in NULL equivalents
///////////////////////////////////also,, in the GUI make a freezehand function,,,and of course make the simpleCapsule object
double MordatchHandsObjective::calculateQuaternionDistance(Quaternion q1, Quaternion q2) {
	return acos(2*q1.dot(q2)*q1.dot(q2)-1);
}
Quaternion MordatchHandsObjective::SLERPobjectOrientation(double time) {
	///remember the times in the objectPosition trajectory correspond to the entries in the quaternions vector
	if (time <= objectPosition.getKnotPosition(0))
		return objectOrientation[0];
	if (time >= objectPosition.getKnotPosition(objectPosition.getKnotCount()-1))
		return objectOrientation[objectOrientation.size()-1];
	for (int i = 1; i<objectPosition.getKnotCount(); i++) {
		if (objectPosition.getKnotPosition(i) > time) {
			Quaternion lower = objectOrientation[i - 1];
			double lowerTime=objectPosition.getKnotPosition(i - 1);
			double upperTime= objectPosition.getKnotPosition(i);
			Quaternion upper = objectOrientation[i];
			return lower.sphericallyInterpolateWith(upper, (time - lowerTime) / (upperTime - lowerTime));
		}
	}
	//this should never happen
	return NULLQUATERNION;
}
Quaternion MordatchHandsObjective::getQuaterionFromRPY(double r, double p, double y)
{
	return getRotationQuaternion(y, V3D(0, 0, 1))*getRotationQuaternion(p, V3D(0, 1, 0))*getRotationQuaternion(r, V3D(1, 0, 0));
}
void MordatchHandsObjective::getRPYFromQuaternion(Quaternion q, double& r, double &p, double& y) {
	computeEulerAnglesFromQuaternion(q, V3D(1, 0, 0), V3D(0, 1, 0), V3D(0, 0, 1), r, p, y);
}
//////Assumes that the object is a capsule (important), also the point this returns is in the object's localFrame
////this doesn't handle the case where the point to be projected onto the object is on the axis of the capsule itself---but it doesn't need to worry about that because of the optimization
V3D MordatchHandsObjective::projectOntoObject(V3D localCoord)
{
	V3D axis = objectCapsule.p1 - objectCapsule.p2;
	V3D nearestPoint= objectCapsule.p1;
	if(objectCapsule.p1!= objectCapsule.p2)
		nearestPoint=axis*(axis.dot(localCoord) / axis.dot(axis)); //nearest point on the axis
	//test the edge cases where the point falls outside the object
	if ((nearestPoint - objectCapsule.p1).norm() > axis.norm()|| (nearestPoint - objectCapsule.p2).norm() > axis.norm())
	{
		if ((nearestPoint - objectCapsule.p1).norm() < (nearestPoint - objectCapsule.p2).norm())
			nearestPoint = V3D(objectCapsule.p1);
		else nearestPoint = V3D(objectCapsule.p2);
	}
	return nearestPoint+(localCoord-nearestPoint).toUnit()*objectCapsule.r;
}
////This also assumes the object is a capsule
double MordatchHandsObjective::getObjectPenetrationDepth(V3D worldCoord, double time)
{
	//return 0 if there is no penetration---penetration occurs when the point is within r of the line segment for the axis of the capsule
	V3D localCoord = convertFromWorldToLocal(worldCoord, time);
	
	//copied from the above method--find nearest point
	V3D axis = objectCapsule.p1 - objectCapsule.p2;
	V3D nearestPoint = objectCapsule.p1;
	if (objectCapsule.p1 != objectCapsule.p2)
		nearestPoint = axis*(axis.dot(localCoord) / axis.dot(axis)); //nearest point on the axis
	//test the edge cases where the point falls outside the object
	if ((nearestPoint - objectCapsule.p1).norm() > axis.norm() || (nearestPoint - objectCapsule.p2).norm() > axis.norm())
	{
		if ((nearestPoint - objectCapsule.p1).norm() < (nearestPoint - objectCapsule.p2).norm())
			nearestPoint = V3D(objectCapsule.p1);
		else nearestPoint = V3D(objectCapsule.p2);
	}

	//now penetration depth
	if ((localCoord - nearestPoint).norm() < objectCapsule.r)
		return objectCapsule.r - (localCoord - nearestPoint).norm();
	else return 0;
}



//this converts the objects from the object's local frame to the workd frame
V3D	MordatchHandsObjective::convertFromLocalToWorld(V3D localPoint, double time) {
	// the quaternions for object orientation rotates from the local coordinate frame to the world coordinate frame
	return objectPosition.evaluate_catmull_rom(time) + SLERPobjectOrientation(time).rotate(localPoint);
}

//this converts the objects from the world frame to the object's local frame
V3D	MordatchHandsObjective::convertFromWorldToLocal(V3D worldPoint, double time) {
	// the quaternions for object orientation rotates from the local coordinate frame to the world coordinate frame
	return  (SLERPobjectOrientation(time)).getInverse().rotate(worldPoint- objectPosition.evaluate_catmull_rom(time));
}



//compute mat*v
V3D MordatchHandsObjective::multiplyMatrixByVector(Matrix3x3 mat, V3D v) {
	V3D ret = V3D(0, 0, 0);
	for (int i = 0; i < mat.rows(); i++) {
		ret[i] = mat.coeff(i, 0)*v[0] + mat.coeff(i, 1)*v[1] + mat.coeff(i, 2)*v[2];
	}
	return ret;
}

void MordatchHandsObjective::fillUpContactForceMap(double time)
{
	////just copy the first time's value for each entry
	for (std::map<string, contactForceMapObject>::iterator ent = contactsMap.begin(); ent != contactsMap.end(); ++ent) {
		if (ent->first != "ground1"&&ent->first != "ground2") {
			double cweight = .5;
			if (time == 0)
				cweight = 0;
			ent->second.contactWeights.addKnot(time, cweight); //no contact in first frame, but initialize contact for all other frames
			ent->second.contactOriginsLocal.addKnot(time, convertFromWorldToLocal(fingertipPositions[ent->first].evaluate_catmull_rom(time), time)); //project onto the bodies you want contact points on
			ent->second.contactForces.addKnot(time, V3D(0, 0, 0)); //give some initial contact force to help the optimization hopefully
		}
		else if (ent->first == "ground1")
		{
			//single contact point with the ground
			double cweight = 0;
			if (time == 0 && !DISABLE_GRAVITY)
				cweight = 1;
			ent->second.contactWeights.addKnot(time, cweight);
			ent->second.contactOriginsLocal.addKnot(time, initialGravityContactPointLocal);
			ent->second.contactForces.addKnot(time, initialGravityContactForce); 
		}
		else {
			ent->second.contactWeights.addKnot(time, 0); //no contact in first frame
			ent->second.contactOriginsLocal.addKnot(time, initialGravityContactPointLocal); //project onto the bodies you want contact points on
			ent->second.contactForces.addKnot(time, V3D(0, 0, 0)); //no initial contact forces
		}
	}
}
