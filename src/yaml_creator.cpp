#include <iostream>
#include <fstream>
using namespace std;

int main () {
	//ros::param::get("nTurtle",	nTurtle);
	
  ofstream myfile;
  myfile.open ("/home/hari/my_iliad/src/shadow_algorithm/params/params8.yaml");
  myfile << " this to a file.\n";
  myfile.close();
  return 0;
}