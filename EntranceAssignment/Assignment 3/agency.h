#ifndef AGENCY_H
#define AGENCY_H

#include <string>
#include <vector>
#include "mission.h"

class Agency{
private:
    std::string name;
    int cash;
    int ticketPrice;
    
    std::vector<Mission> completedMissions;
    std::vector<Mission> nextMissions;
public:
    Agency(std::string n = "", int c = 0, int tp = 0);

    void addMission(Mission m);
    void executeNextMission();
    void agencyInfo();
};

#endif