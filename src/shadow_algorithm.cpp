#include <ros/ros.h>
#include <iostream>
#include <turtlesim/Spawn.h>
#include <ilcplex/ilocplex.h>
#include <chrono>	//for endl
#include <math.h> 	//for hypot
#include <cmath> 	//for abs
#include "turtlesim/Pose.h"		
#include <limits.h>
#include "geometry_msgs/Twist.h"
#include <boost/assign/list_of.hpp>

using namespace std;
#define NTURTLE 7
#define OFFSET 9*M_PI/180
#define TOLERANCE 0*M_PI/180
// #define NTURTLE 7
// #define OFFSET 9*M_PI/180
// #define TOLERANCE 0*M_PI/180
#define isNear(X, A) ( ((X > (A - TOLERANCE)) && ( X < (A + TOLERANCE))) )

double constrainAngle(double x);
void rotate_world(turtlesim::Pose TPose[], double del_angle);
void evalcoeffs(double h[NTURTLE][NTURTLE], double k[NTURTLE][NTURTLE],  turtlesim::Pose TPose[]);
void spawn_my_turtles(ros::NodeHandle& nh, turtlesim::Spawn::Request req[], turtlesim::Spawn::Response resp[]);
void currPoseCallback(const turtlesim::Pose::ConstPtr& msg, int id, turtlesim::Pose TPose[]);
void optimizeme(IloModel model, IloNumVarArray var, IloRangeArray con, geometry_msgs::Twist cmdVel[], double Vinit[]);
static void populatebyrow (IloModel model, IloNumVarArray x, IloRangeArray c, turtlesim::Pose TPose[], double h[NTURTLE][NTURTLE], double k[NTURTLE][NTURTLE], double max_vel, double min_vel, double Vinit[]);
	   
int main(int argc, char **argv) {
	
	ros::init(argc, argv, "defaultnode");
	ros::NodeHandle nh;
	ros::Subscriber curr_pose_sub[NTURTLE];
	ros::Publisher turtle_vel_pub[NTURTLE];
	
	double max_vel,min_vel,Vinit[NTURTLE], pub_rate, h[NTURTLE][NTURTLE], k[NTURTLE][NTURTLE];
	
	geometry_msgs::Twist cmdVel[NTURTLE];
	turtlesim::Spawn::Request req[NTURTLE];
	turtlesim::Spawn::Response resp[NTURTLE];
	turtlesim::Pose TPose[NTURTLE];
	
	ros::param::get("pubrate",	pub_rate);
	ros::param::get("maxvel",	max_vel);
	ros::param::get("minvel",	min_vel);
	
	ros::Rate my_rate(pub_rate);
	
	spawn_my_turtles(nh, req, resp);	//spawn both the turtles and initiate their pose and check if the spawnings were successful
	
	for (int m=0; m<NTURTLE; ++m) {
		curr_pose_sub[m] 	= nh.subscribe<turtlesim::Pose>("/"+req[m].name+"/pose", 2, boost::bind(currPoseCallback, _1,m, TPose));	// subscriber objects to get current pose of each turtle
		turtle_vel_pub[m] 	= nh.advertise<geometry_msgs::Twist>("/"+req[m].name+"/cmd_vel", 2);	//publisher objects to publish the command velocity
		Vinit[m]		= max_vel;	//set the initial velocity of the robots to max
		cmdVel[m].linear.x	= Vinit[m];
		turtle_vel_pub[m].publish(cmdVel[m]); //publish the velocity of all turtles once
	}
	
	sleep(1);
	ros::spinOnce();
	
	IloEnv   env;				//create environment handle which also creates the implementation object internally
	IloModel model(env);		//create modelling object to define optimisation models with our enviroment env
	IloNumVarArray var(env);	//create modelling variables
	IloRangeArray con(env);		//create range objects for defining constraints
	
	evalcoeffs(h,k,TPose);				//calculates the coefficients of the constraint eqns etc
	populatebyrow (model, var, con, TPose,h,k, max_vel,min_vel,Vinit );	//populate the model
	cout<<"Pose before optimisation\n";
	cout<<"TPose.x\t= " << TPose[0].x;
	cout<<"\tTPose.y\t= " << TPose[0].y;
	cout<<"\tTPose.theta\t=" << TPose[0].theta*180/M_PI<<endl;
	optimizeme(model,var,con, cmdVel, Vinit);	//the real optimisation happens here
	
	//cout<<"going to Publish now: \t"<<cmdVel.linear.x<<endl; 
	//int i=1;
	//&& i<8
	while(ros::ok() && nh.ok() ){
		for (int a=0; a<NTURTLE; ++a) {
			turtle_vel_pub[a].publish(cmdVel[a]);
		}
		my_rate.sleep();
		ros::spinOnce();
		my_rate.sleep();
		//++i;
	}
	cout<<"done!"<<endl;
	sleep(1000);
   	return 0;
}	//END of main


static void populatebyrow (IloModel model, IloNumVarArray x, IloRangeArray c, turtlesim::Pose TPose[], double h[NTURTLE][NTURTLE], double k[NTURTLE][NTURTLE], double max_vel, double min_vel, double Vinit[])
{
	IloEnv env = model.getEnv();		//environment handle

	for(int m=0; m<NTURTLE; ++m) {
		//define upper and lower bounds for variables and the variable type (continous(float) or discrete (int/bool)
		x.add(IloNumVar(env, min_vel-Vinit[m], max_vel-Vinit[m], ILOFLOAT));	//qi
	}
	
	IloExpr expr(env);
	for(int j=0; j<NTURTLE; ++j) {
	  expr += - x[j] ;
	}
	model.add(IloMinimize(env,expr));
	//model.add(IloMinimize(env, - x[0] - x[1] - x[2] ));		//objective function for minimisation (Note: done in single step instead of two) 

	double theta1, theta2, h1, h2, k1, k2, v1, v2;
	for(int i=0; i<NTURTLE; ++i) {
		for(int j=0; j<NTURTLE; ++j) {
			if (i<j) {
				theta1=TPose[i].theta;
				theta2=TPose[j].theta;
				h1=h[i][j];
				h2=h[j][i];
				k1=k[i][j];
				k2=k[j][i];
				v1=Vinit[i];
				v2=Vinit[j];
				IloRange R1(-cos(theta1)*x[i] 	+ cos(theta2)*x[j] 	<=  v1*cos(theta1) 	- v2*cos(theta2));
				IloRange R2(h1*x[i] 			- h2*x[j] 	<= -v1*h1		+ v2*h2);
				IloRange R3(cos(theta1)*x[i] 	- cos(theta2)*x[j] 	<= -v1*cos(theta1) 	+ v2*cos(theta2));
				IloRange R4(-h1*x[i] 			+ h2*x[j] 	<=  v1*h1 		- v2*h2);
				IloRange R5(-cos(theta1)*x[i] 	+ cos(theta2)*x[j] 	<=  v1*cos(theta1) 	- v2*cos(theta2));
				IloRange R6(-k1*x[i] 			+ k2*x[j] 	<=  v1*k1		- v2*k2);
				IloRange R7(cos(theta1)*x[i] 	- cos(theta2)*x[j] 	<= -v1*cos(theta1) 	+ v2*cos(theta2));
				IloRange R8(k1*x[i] 			- k2*x[j] 	<= -v1*k1 		+ v2*k2);
				
				IloOr myOR(env);
				myOR.add(R1 && R2);
				myOR.add(R3 && R4);
				myOR.add(R5 && R6);
				myOR.add(R7 && R8);
				
				model.add(myOR);
			}
		}
	}

}  // END populatebyrow


void optimizeme(IloModel model, IloNumVarArray var, IloRangeArray con, geometry_msgs::Twist cmdVel[], double Vinit[]) {
	IloEnv env = model.getEnv();
    try {
		
      	IloCplex cplex(model);				//create cplex object for solving the problem (Note: two steps combined)
      	cplex.setParam( IloCplex::Param::Emphasis::MIP,1);
	cplex.setParam( IloCplex::Param::MIP::Tolerances::Integrality, 0);
	//cplex.setParam( IloCplex::Param::Simplex::Tolerances::Feasibility, 1e-9);
	cplex.setParam( IloCplex::Param::Barrier::ConvergeTol, 1e-12);
      	cplex.exportModel("/home/hari/my_iliad/src/shadow_algorithm/src/lpex1.lp");		//write the extracted model to lplex1.lp file (optional)
		cplex.setOut(env.getNullStream()); 	//discard the output from next step(solving) by dumping it to null stream rather than to the screen
		
		// Optimize the problem and obtain solution.
		if ( !cplex.solve() ) { //this returns an ILOBOOL value (true/false) depending on whether it obtained a feasible soln or not
		 env.error() << "Failed to optimize LP" << endl;
		 throw(-1);
		}
		
		IloNumArray vals(env);	//to store the solution values of all the variables
		IloNumArray vals2(env);	//to store the slack variable outputs
		env.out() << "Solution status = " << cplex.getStatus() << endl;		//to get more detailed status of the soln
		env.out() << "Cplex status = " << cplex.getCplexStatus() << endl;		//to get more detailed status of the soln
		env.out() << "Solution value  = " << cplex.getObjValue() << endl;	//query the objective function value that resulted from optimisation
		
		cout<<"Solved as MIP?\t: " <<cplex.isMIP()<<endl;
		cplex.getValues(vals, var);		//solution values for all variables will be stored to vals
		env.out() << "Values        = " << vals << endl;
		//cplex.getSlacks(vals2, con);
		//env.out() << "Slacks        = " << vals2 << endl;
      
		IloNum violation = cplex.getQuality(IloCplex::MaxPrimalInfeas);
		cout << "max violation\t: " << violation << endl;
		  		
		cout<< "q1\t:" << vals[0] << "\tq2\t: " << vals[1] << endl;
		for(int y=0; y< NTURTLE; ++y) {
			cmdVel[y].linear.x=Vinit[y]+vals[y];
		}
		
	}
   	catch (IloException& e) {
      	cerr << "Concert exception caught: " << e << endl;
  	}
   	catch (...) {
      	cerr << "Unknown exception caught" << endl;
   	}//end of try catch
}

void evalcoeffs(double h[NTURTLE][NTURTLE], double k[NTURTLE][NTURTLE], turtlesim::Pose TPose[]) {
	
	double safe_dia, alpha, A[NTURTLE][NTURTLE], w[NTURTLE][NTURTLE], l[NTURTLE][NTURTLE], r[NTURTLE][NTURTLE];
	
	ros::param::get("safedia",	safe_dia);
	
	for(int i=0; i<NTURTLE; ++i) {
		for(int j=0; j<NTURTLE; ++j) {
			if (i != j) {
				A[i][j]=hypot(TPose[j].x-TPose[i].x,TPose[j].y-TPose[i].y);
				w[i][j]=atan2(TPose[j].y-TPose[i].y,TPose[j].x-TPose[i].x);
				cout << endl<< "A[i][j]\t: " << A[i][j]  << "\tSafe_dia\t: " << safe_dia << "\tDifference\t: " << A[i][j] - safe_dia << endl;

				if(A[i][j] - safe_dia < 0) {
					cout << "Safety disc overlap identified!!!" <<endl;
					sleep(1500);
					exit (0);
				}
				else if(safe_dia/A[i][j] > 1) {
					cout << "It's too late to avoid collision...Sine Error...exiting" <<endl;
					sleep(1500);
					exit (0);
				}
				alpha=asin(safe_dia/A[i][j]);
				l[i][j]=constrainAngle(w[i][j]+alpha);	//convert to the range [-180,180) if it goes out of bound
				r[i][j]=constrainAngle(w[i][j]-alpha);
				cout << "alpha\t: " << alpha*180/M_PI <<"\tw["<<i<<"]["<<j<<"]\t: " << w[i][j]*180/M_PI << "\tl[i][j]\t: " << l[i][j]*180/M_PI << "\tr[i][j]\t: " << r[i][j]*180/M_PI << endl;
				
				if(isNear(w[i][j], M_PI/2)){
				  cout<<"if 1"<<endl;
				  double del_angle=TOLERANCE -abs(M_PI/2-w[i][j]);
				  del_angle=((w[i][j] > M_PI/2) ? del_angle : -del_angle);
				  rotate_world(TPose, del_angle);
				  memset(h, 0, NTURTLE*NTURTLE);	//set arrays to zero
				  memset(k, 0, NTURTLE*NTURTLE);
				  evalcoeffs(h,k,TPose);
				  return;
				}
				else if(isNear(l[i][j], M_PI/2)){
				  cout<<"if 2"<<endl;
				  double del_angle=TOLERANCE -abs(M_PI/2-l[i][j]);
				  del_angle=((l[i][j] > M_PI/2) ? del_angle : -del_angle);
				  rotate_world(TPose, del_angle);
				  memset(h, 0, NTURTLE*NTURTLE);	//set arrays to zero
				  memset(k, 0, NTURTLE*NTURTLE);
				  evalcoeffs(h,k,TPose);
				  return;
				}
				else if (isNear(r[i][j], M_PI/2)){
				  cout<<"if 3"<<endl;
				  double del_angle=TOLERANCE -abs(M_PI/2-r[i][j]);
				  del_angle=((r[i][j] > M_PI/2) ? del_angle : -del_angle);
				  rotate_world(TPose, del_angle);
				  memset(h, 0, NTURTLE*NTURTLE);	//set arrays to zero
				  memset(k, 0, NTURTLE*NTURTLE);
				  evalcoeffs(h,k,TPose);
				  return;
				}
				else if(isNear(w[i][j], -M_PI/2)){
				  cout<<"if 4"<<endl;
				  double del_angle=TOLERANCE -abs(-M_PI/2-w[i][j]);
				  del_angle=((w[i][j] > -M_PI/2) ? del_angle : -del_angle);
				  rotate_world(TPose, del_angle);
				  memset(h, 0, NTURTLE*NTURTLE);	//set arrays to zero
				  memset(k, 0, NTURTLE*NTURTLE);
				  evalcoeffs(h,k,TPose);
				  return;
				}  
				else if(isNear(l[i][j], -M_PI/2)){
				  cout<<"if 5"<<endl;
				  double del_angle=TOLERANCE -abs(-M_PI/2-l[i][j]);
				  del_angle=((l[i][j] > -M_PI/2) ? del_angle : -del_angle);
				  rotate_world(TPose, del_angle);
				  memset(h, 0, NTURTLE*NTURTLE);	//set arrays to zero
				  memset(k, 0, NTURTLE*NTURTLE);
				  evalcoeffs(h,k,TPose);
				  return;
				}  
				else if (isNear(r[i][j], -M_PI/2)){
				  cout<<"if 6"<<endl;
				  double del_angle=TOLERANCE -abs(-M_PI/2-r[i][j]);
				  del_angle=((r[i][j] > -M_PI/2) ? del_angle : -del_angle);
				  rotate_world(TPose, del_angle);
				  memset(h, 0, NTURTLE*NTURTLE);	//set arrays to zero
				  memset(k, 0, NTURTLE*NTURTLE);
				  evalcoeffs(h,k,TPose);
				  return;
				}
				else  { 
				  //v1=T1Pose.linear_velocity;
				  //v2=T2Pose.linear_velocity;

				  cout << "tan(l[i][j])\t: " << tan(l[i][j]) << "\ttan(r[i][j])\t: " << tan(r[i][j]) << endl;

				  h[i][j]=tan(l[i][j])*cos(TPose[i].theta) - sin(TPose[i].theta);
				  k[i][j]=tan(r[i][j])*cos(TPose[i].theta) - sin(TPose[i].theta);

				  cout << "h[i][j]\t: " << h[i][j] << "\tk[i][j]\t: " << k[i][j] << endl;
				}
			}
		}
	}

}

void rotate_world(turtlesim::Pose TPose[], double del_angle) {
  double tempx, tempy;
  del_angle=del_angle+0.01*del_angle;
  cout << "TPose is updated to: " <<endl;
  for(int j=0; j< NTURTLE; ++j) {
    tempx		= TPose[j].x * cos(del_angle) + TPose[j].y * sin(del_angle);
    tempy		=-TPose[j].x * sin(del_angle) + TPose[j].y * cos(del_angle);
    TPose[j].x		= tempx;
    TPose[j].y		= tempy;
    TPose[j].theta	= constrainAngle(TPose[j].theta - del_angle);
    	
	cout<<"TPose.x\t= " << TPose[j].x;
	cout<<"\tTPose.y\t= " << TPose[j].y;
	cout<<"\tTPose.theta\t=" << TPose[j].theta*180/M_PI<<endl;
  }
  //sleep(10);
}

void spawn_my_turtles(ros::NodeHandle& nh,turtlesim::Spawn::Request req[], turtlesim::Spawn::Response resp[]) {
	//spawn two turtles
	ros::service::waitForService("spawn");
	ros::ServiceClient spawnTurtle = nh.serviceClient<turtlesim::Spawn>("spawn");
	
	double x=5.5, y=5.5, p=0, r=5.5;

	for (int j=0; j<NTURTLE; ++j) {
		req[j].name="myturtle";
		req[j].name += std::to_string(j+1);	//append UID to the end of "myturtle" eg: myturtle3
		p=OFFSET+j*2*M_PI/NTURTLE;	
		//p=j*2*M_PI/NTURTLE;		//divide 360 equally into "NTURTLE parts
		req[j].x=x+r*cos(p);
		req[j].y=y+r*sin(p);
		req[j].theta=M_PI+p;
		
		bool success=spawnTurtle.call(req[j],resp[j]);
		if(success) { ROS_INFO_STREAM ("Spawned turtle\t:" << j ); }
		else { ROS_INFO_STREAM ("Error, unable to spawn turtle"); }
	}
}

void currPoseCallback(const turtlesim::Pose::ConstPtr& msg, int id, turtlesim::Pose TPose[]) {
	TPose[id].x 		= msg->x;
	TPose[id].y 		= msg->y;
	TPose[id].theta 	= constrainAngle(msg->theta);
// 	cout<<"TPose.x\t= " << TPose[id].x;
// 	cout<<"\tTPose.y\t= " << TPose[id].y;
// 	cout<<"\tTPose.theta\t=" << TPose[id].theta*180/M_PI<<endl;
	return;
}

double constrainAngle(double x){		//Normalize to [-180,180)
    x = fmod(x+M_PI,2*M_PI);
    if (x < 0)
        x += 2*M_PI;
    return x - M_PI;
}