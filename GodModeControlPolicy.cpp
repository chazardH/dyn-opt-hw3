#include "GodModeControlPolicy.h"

vector<string> GodModeControlPolicy::prepSim() {
	//freese the objects
	return frozenRigidBodyNames;
}
vector<string> GodModeControlPolicy::revertSim() {
	//unfreeze the simple object
	return frozenRigidBodyNames;
}

//sends commands to the ODERBEngine to affect the simulation (called by every iteration of the process loop)
//NOTE:::this is written assuming that the ONLY thing we are controlling is the simpleObject and nothing else (we can also draw points on the gui, but that's it)
//points to draw are returned by the function, and drawn in the sim (external)
void GodModeControlPolicy::getControlCommands(AbstractRBEngine* rbEngine, double controlTime, vector<P3D> &pointsToDraw, 
	vector<V3D> &forceVectorsOnObjectToDraw, vector<P3D> &forceContactsOnObjectToDraw,bool forceonly) {
	//pointsToDraw, forceVectorsOnObjectToDraw and forceContactsOnObjectToDraw are in world coords
	forceContactsOnObjectToDraw.clear();
	forceVectorsOnObjectToDraw.clear();
	pointsToDraw.clear();

	RigidBody* simpleObj = rbEngine->getRBByName("simpleObject");
	double time = objectPositions.getMaxPosition()*controlTime; //we need to scale the time since controlTimes are the whole time scale compressed into [0,1]
	
	
	//set simpleObject state and orientation
	if(!forceonly)
		simpleObj->setCMPosition(P3D(objectPositions.evaluate_catmull_rom(time)));
	int lowerIndex;
	int upperIndex;
	double timeFraction;
	calculateTimeVariables(time, lowerIndex,upperIndex,timeFraction);
	simpleObj->state.orientation = objectOrientations[lowerIndex].sphericallyInterpolateWith(objectOrientations[upperIndex],timeFraction);
	
	//apply forces to simple object (for show, since it is frozen) and draw contact points
	for (auto &ent : contactsMap) {
		// ent.first is key, ent.second is value
		P3D p = P3D((ent.second.contactOriginsLocal).evaluate_linear(time)); //p should be in local coords
	//	V3D f = (ent.second.contactForces).evaluate_roundup_constant(time)*(ent.second.contactWeights).evaluate_piecewise_constant(time);
		V3D f = (ent.second.contactForces).evaluate_linear(time)*(ent.second.contactWeights).evaluate_piecewise_constant(time);
		if (f.norm() > .001) {
			if (controlTime <= 1.0) {
				rbEngine->applyForceTo(simpleObj, f, p);
	//			cout << "Applying force: " << f[0] << " " << f[1] << " " << f[2] << " time " << time << " controlTime " << controlTime << "\n";

				P3D q = simpleObj->getCMPosition();
				//			cout << "Actual object position: " << q[0] << " " << q[1] << " " << q[2] << " time " << time << " controlTime " << controlTime << "\n";
				P3D w = P3D(objectPositions.evaluate_catmull_rom(time));
				//			cout << "Desired object position: " << w[0] << " " << w[1] << " " << w[2] << " time " << time << " controlTime " << controlTime << "\n";
			}
			forceVectorsOnObjectToDraw.push_back(f);
			forceContactsOnObjectToDraw.push_back(simpleObj->getWorldCoordinates(p));
		}
	}

	//draw landmark points
	for (auto &ent : handLandmarkLocations) 
		pointsToDraw.push_back(P3D(ent.second.evaluate_catmull_rom(time)));
}

//used for interpolating items a GenericTrajectory can't handle---quaternions
void GodModeControlPolicy::calculateTimeVariables(double time,int& lowerIndex, int& upperIndex, double& timeFraction)
{
	int size = objectPositions.getKnotCount();
	if (time < objectPositions.getKnotPosition(0)) 
		throwError(("CalculateTimeVariables: You fucked up the control time: the first knot position always has to be at time 0: "+to_string(objectPositions.getKnotPosition(0))).c_str());
	else if (time >= objectPositions.getKnotPosition(size-1)) {
		lowerIndex = size - 1;
		upperIndex = lowerIndex;
		timeFraction = 1;
	}
	else
	{
		for (int i = 1; i<size; i++) {
			if (objectPositions.getKnotPosition(i) > time) {
				lowerIndex = i-1;
				upperIndex = i;
				timeFraction = (time - objectPositions.getKnotPosition(lowerIndex)) / (objectPositions.getKnotPosition(upperIndex) -objectPositions.getKnotPosition(lowerIndex));
				break;
			}
		}
	}
}





void GodModeControlPolicy::readFromFile(const char* filepath) {
	frozenRigidBodyNames = {};
	objectPositions = GenericTrajectory<V3D>();
	objectOrientations = {};
	handLandmarkLocations = {};
	contactsMap = {};
	cout << filepath << "\n";

	if (filepath == NULL)
		throwError("NULL file name provided.");
	FILE *f = fopen(filepath, "r");
	if (f == NULL)
		throwError("Could not open file: %s", filepath);
	//have a temporary buffer used to read the file line by line...
	char buffer[200];
	int lineNum = 1;

	vector<string> section = { "Object Positions","Object Orientations", "Hand Landmarks", "Contacts Map" };
	string currentSection="";

	while (!feof(f)) {
		//get a line from the file...
		readValidLine(buffer, f, 200);
		if (strlen(buffer) > 195)
			throwError("The input file contains a line that is longer than ~200 characters - not allowed");
		char *line = rTrim(lTrim(buffer));
		if (strlen(line) == 0) continue;

		if (lineNum == 1) { 
			if (string(line) != "GodModeControlPolicy")
				throwError("Error: trying to read a non-GodModeControlPolicy as a GodModeControlPolicy");
		}
		else if (lineNum == 2)
		{
			//this lists the frozen rigid bodies
			getCharSeparatedStringList(line, frozenRigidBodyNames, ' ');
		}
		//switching reading sections case
		else if (std::find(section.begin(), section.end(), string(line)) != section.end()) {
			currentSection = string(line);
		}
		else if(currentSection == "Object Positions")
		{ 
			vector<string> args;
			getCharSeparatedStringList(line, args, ' ');
			objectPositions.addKnot(stod(args[0]), V3D(stod(args[1]), stod(args[2]), stod(args[3])));			
		}
		else if (currentSection == "Object Orientations")
		{
			//objectPositions.getKnotCount() should equal objectOrientations.size() (they match up)
			vector<string> args;
			getCharSeparatedStringList(line, args, ' ');
			objectOrientations.push_back(Quaternion(stod(args[0]), stod(args[1]), stod(args[2]), stod(args[3])));
		}
		else if (currentSection == "Hand Landmarks")
		{
			///first line is always "Next Entry" under "Hand Landmarks"
			bool onNextEntry = true;
			string markName="";
			GenericTrajectory<V3D> locTraj = GenericTrajectory<V3D>();
			
			//read until we hit the end of the file or another section
			while (!feof(f)) {
				//get a line from the file...
				readValidLine(buffer, f, 200);
				if (strlen(buffer) > 195)
					throwError("The input file contains a line that is longer than ~200 characters - not allowed");
				char * linein = rTrim(lTrim(buffer));
				if (strlen(linein) == 0) continue;

				if (std::find(section.begin(), section.end(), string(linein)) != section.end()) {
					currentSection = string(linein);
					if(markName!="")
						handLandmarkLocations.insert(pair<string, GenericTrajectory<V3D>>{ markName,locTraj });
					break;
				}
				else if (string(linein) == "Next Entry") {
					if(markName!="")
						handLandmarkLocations.insert(pair<string, GenericTrajectory<V3D>>{ markName, locTraj });
					locTraj = GenericTrajectory<V3D>();
					onNextEntry = true;
				}
				else if (onNextEntry) {
					onNextEntry = false;
					markName = string(linein);
				}
				else {
					vector<string> args;
					getCharSeparatedStringList(linein, args, ' ');
					locTraj.addKnot(stod(args[0]), V3D(stod(args[1]), stod(args[2]), stod(args[3])));
				}
			}
		}
		else if (currentSection == "Contacts Map")
		{
			bool onNextEntry = true;
			string contactName = "";
			GenericTrajectory<double> cWeights = GenericTrajectory<double>();
			GenericTrajectory<V3D> cOriginsLocal = GenericTrajectory<V3D>();
			GenericTrajectory<V3D> cForces = GenericTrajectory<V3D>();
			vector<string> internalSections = { "Contact Weights","Contact Origins Local","Contact Forces" };
			string currentInternalSection = "";

			//read until we hit the end of the file or another section
			while (!feof(f)) {
				//get a line from the file...
				readValidLine(buffer, f, 200);
				if (strlen(buffer) > 195)
					throwError("The input file contains a line that is longer than ~200 characters - not allowed");
				char * linein = rTrim(lTrim(buffer));
				if (strlen(linein) == 0) continue;

				if (string(linein)== "End of Contacts Map") {
					if (contactName != "") {
						contactForceMapObject mapobj;
						mapobj.contactForces = cForces;
						mapobj.contactOriginsLocal = cOriginsLocal;
						mapobj.contactWeights = cWeights; 
						contactsMap.insert(pair<string, contactForceMapObject>{contactName, mapobj});
					}
					break;
				}
				else if (string(linein) == "Next Entry") {
					if (contactName != "") {
						contactForceMapObject mapobj;
						mapobj.contactForces = cForces;
						mapobj.contactOriginsLocal = cOriginsLocal;
						mapobj.contactWeights = cWeights;
						contactsMap.insert(pair<string, contactForceMapObject>{contactName, mapobj});
					}
					cWeights = GenericTrajectory<double>();
					cOriginsLocal = GenericTrajectory<V3D>();
					cForces = GenericTrajectory<V3D>();
					onNextEntry = true;
				}
				else if (onNextEntry) {
					onNextEntry = false;
					contactName = string(linein);
				}
				else if (std::find(internalSections.begin(), internalSections.end(), string(linein)) != internalSections.end())
				{
					currentInternalSection = string(linein);
				}
				else if (currentInternalSection== "Contact Weights"){
					vector<string> args;
					getCharSeparatedStringList(linein, args, ' ');
					cWeights.addKnot(stod(args[0]), stod(args[1]));
				}
				else if (currentInternalSection == "Contact Origins Local") {
					vector<string> args;
					getCharSeparatedStringList(linein, args, ' ');
					cOriginsLocal.addKnot(stod(args[0]), V3D(stod(args[1]), stod(args[2]), stod(args[3])));
				}
				else if (currentInternalSection == "Contact Forces") {
					vector<string> args;
					getCharSeparatedStringList(linein, args, ' ');
					cForces.addKnot(stod(args[0]), V3D(stod(args[1]), stod(args[2]), stod(args[3])));
				}
			}
		}	
		else 
		{
			throwError(("Read file in god mode: Line not understood: "+string(line)).c_str());
		}

		lineNum++;
	}
	fclose(f);
	std::cout << "Done reading from file \n";
}


void GodModeControlPolicy::writeToFile(char* filepath) {
	FILE * fp;
	fp = fopen(filepath, "w");
	if (fp == NULL) std::cout << ("\nError saving to file");
	else
	{
		fprintf(fp, "GodModeControlPolicy\n");
		for(string s:frozenRigidBodyNames)
			fprintf(fp, "%s ", s.c_str());
		fprintf(fp, "\nObject Positions\n");
		for (int i = 0; i < objectPositions.getKnotCount(); i++)
		{
			double t=objectPositions.getKnotPosition(i);
			V3D p = objectPositions.getKnotValue(i);
			fprintf(fp, "%f %f %f %f\n",t,p[0],p[1],p[2]);
		}
		fprintf(fp, "Object Orientations\n");
		//objectPositions.getKnotCount() should equal objectOrientations.size() (they match up)
		for (uint i = 0; i < objectOrientations.size(); i++)	
			fprintf(fp, "%f %f %f %f\n", objectOrientations[i].s, objectOrientations[i].v[0], objectOrientations[i].v[1], objectOrientations[i].v[2]);
		fprintf(fp, "Hand Landmarks\n");
		for (auto &ent : handLandmarkLocations) {
			fprintf(fp, "Next Entry\n");
			fprintf(fp, "%s\n",ent.first.c_str());
			for (int i = 0; i < ent.second.getKnotCount(); i++)
			{
				double t = ent.second.getKnotPosition(i);
				V3D p = ent.second.getKnotValue(i);
				fprintf(fp, "%f %f %f %f\n", t, p[0], p[1], p[2]);
			}
		}
		fprintf(fp, "Contacts Map\n");
		for (auto &ent : contactsMap)
		{
			fprintf(fp, "Next Entry\n");
			fprintf(fp, "%s\n", ent.first.c_str());
			fprintf(fp, "Contact Weights\n");
			for (int i = 0; i < ent.second.contactWeights.getKnotCount(); i++)
			{
				double t = ent.second.contactWeights.getKnotPosition(i);
				double w = ent.second.contactWeights.getKnotValue(i);
				fprintf(fp, "%f %f\n", t, w);
			}
			fprintf(fp, "Contact Origins Local\n");
			for (int i = 0; i < ent.second.contactOriginsLocal.getKnotCount(); i++)
			{
				double t = ent.second.contactOriginsLocal.getKnotPosition(i);
				V3D p = ent.second.contactOriginsLocal.getKnotValue(i);
				fprintf(fp, "%f %f %f %f\n", t, p[0], p[1], p[2]);
			}
			fprintf(fp, "Contact Forces\n");
			for (int i = 0; i < ent.second.contactForces.getKnotCount(); i++)
			{
				double t = ent.second.contactForces.getKnotPosition(i);
				V3D p = ent.second.contactForces.getKnotValue(i);
				fprintf(fp, "%f %f %f %f\n", t, p[0], p[1], p[2]);
			}
		}
		fprintf(fp, "End of Contacts Map\n"); //must have this here at the end of the contacts map
	}
	fclose(fp);
	std::cout << "Done saving to file \n";
}
