#ifndef ASTRONAUT_H
#define ASTRONAUT_H

#include "person.h"
#include <iostream>

class Astronaut : public Person{
private:
    int numMissions;
public:
    Astronaut(std::string n = "", std::string s = "", int missionCount = 4);
    void completeMission();
    //getter
    int getNumMissions() const;
};

#endif