#include <iostream>
#include "person.h"
#include "passenger.h"
#include "astronaut.h"
#include "mission.h"
#include "agency.h"

int main() {
    Passenger p1("John", "Doe", 2000);
    Passenger p2("Jane", "Smith", 500);

    Astronaut a1("Neil", "Armstrong", 5);
    Astronaut a2("Buzz", "Aldrin", 3);

    if(p1.buyTicket(1000)) {
        std::cout << p1.getName() << " bought a ticket!\n";
    }
    if(!p2.buyTicket(1000)) {
        std::cout << p2.getName() << " didn't have enough money!\n";
    }

    Mission m1("AB-12", 3000, 20); //name, cost, failure probability
    m1.addSpaceShipStaff(p1);
    m1.addSpaceShipStaff(p2);
    m1.addSpaceShipStaff(a1);
    m1.addSpaceShipStaff(a2);

    Agency nasa("NASA", 10000, 1000); //name, cash, ticket price
    nasa.addMission(m1);

    nasa.executeNextMission();

    nasa.agencyInfo();

    return 0;
}