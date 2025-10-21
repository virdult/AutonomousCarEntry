#ifndef MISSION_H
#define MISSION_H

#include <iostream>
#include <string>
#include <vector>
#include <cstdlib>

#include "Passenger.h"
#include "Astronaut.h"

class Mission{
private:
    std::string name;
    int missionNumber;
    int cost;
    int faultProbability;
    bool completed;
    
    std::vector<Passenger> passengers;
    std::vector<Astronaut> crew;

    static int numMissions;

    bool isValidName(const std::string& n);
public:
    Mission(std::string n    = "AA-00", int c = 0, int fProb = 0);

    void addSpaceShipStaff(Passenger &p);
    void addSpaceShipStaff(Astronaut &a);

    bool executeMission();
    int calculateProfit(int ticketPrice);
    
    //getters
    int getMissionNumber() const;
    std::string getName() const;
    int getCost() const;
};

#endif