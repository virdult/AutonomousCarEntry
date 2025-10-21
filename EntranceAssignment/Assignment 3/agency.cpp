#include "agency.h"
#include <iostream>

Agency::Agency(std::string n, int c, int tp)
: name(n), cash(c), ticketPrice(tp){}

void Agency::addMission(Mission m){
    nextMissions.push_back(m);
}
void Agency::executeNextMission(){
    if(nextMissions.empty()){
        std::cout << "No missions! Returnin!\n";
        return;
    }
    Mission m = nextMissions.front();
    nextMissions.erase(nextMissions.begin());

    bool success = m.executeMission();

    int profit = m.calculateProfit(ticketPrice);
    cash += profit;

    if(success){
        completedMissions.push_back(m);
    }else{
        nextMissions.push_back(m);
    }
}
void Agency::agencyInfo(){
    std::cout << "Agency name: " << name << "\n";
    std::cout << "Agency's cash: " << cash << "\n";
    std::cout << "Ticket price for the next flight: " << ticketPrice << "\n";
    std::cout << "Next Missions: \n";
    for(const auto& m : nextMissions){
        std::cout << "Mission number: " << m.getMissionNumber()
        << " Mission name: " << m.getName()
        << " Mission cost: " << m.getCost() << "\n";
    }
    std::cout << "\nCompleted Missions: \n";
    for(const auto& m : completedMissions){
        std::cout << "Mission number: " << m.getMissionNumber()
        << " Mission name: " << m.getName()
        << " Mission cost: " << m.getCost() << "\n";
    }
}
