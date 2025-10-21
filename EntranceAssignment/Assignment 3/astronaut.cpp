#include "astronaut.h"

Astronaut::Astronaut(std::string n, std::string s, int missionCount)
: Person(n, s), numMissions(missionCount)
{}
void Astronaut::completeMission(){
    numMissions++;
}
//getter
int Astronaut::getNumMissions() const {return numMissions;}