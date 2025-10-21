#include "mission.h"

//static variable definition
int Mission::numMissions = 0;

bool Mission::isValidName(const std::string& n){
    if(n.size() != 5) return false;

    if(n.find('-') != 2) return false;

    std::string letters = n.substr(0, 2);
    std::string digits  = n.substr(3, 2);
    
    if(!isupper(letters[0]) || !isupper(letters[1]))
        return false;

    if(!isdigit(digits[0]) || !isdigit(digits[1]))
        return false;

    return true;
}

Mission::Mission(std::string n, int c, int fProb)
    : cost(c), faultProbability(fProb), completed(false)
{
    if(isValidName(n)) name = n;
    else {
        std::cout << "Invalid mission name. Setting to AA-00." << std::endl;
        name = "AA-00";
    }

    if(faultProbability < 0) faultProbability = 0;
    if(faultProbability > 100) faultProbability = 100;

    numMissions++;
    missionNumber = numMissions;
}

void Mission::addSpaceShipStaff(Passenger &p){
    if(p.haveTicket()){
        passengers.push_back(p);
    } else {
        std::cout << "Passenger: " << p.getName()
        << " couldn't added because no ticket!" << std::endl;
    }
}

void Mission::addSpaceShipStaff(Astronaut &a){
    crew.push_back(a);
}

bool Mission::executeMission(){
    int randVal = rand() % 101;
    if(randVal > faultProbability){
        std::cout << "Mission: " << name << " success!" << std::endl;
        completed = true;
        for(auto& a : crew){
            a.completeMission();
            std::cout << a.getName() << " " << a.getSurname()
            << " now completed " << a.getNumMissions()
            << " missions!" << std::endl;
        }
    } else {
        std::cout << "Mission: " << name << " failed!" << std::endl;
        completed = false;
    }
    return completed;
}

int Mission::calculateProfit(int ticketPrice){
    int revenue = 0;
    if(completed){
        revenue = passengers.size() * ticketPrice;
    }
    int profit = revenue - cost;
    return profit;
}

//getters
int Mission::getMissionNumber() const {return missionNumber;}
std::string Mission::getName() const {return name;}
int Mission::getCost() const {return cost;}