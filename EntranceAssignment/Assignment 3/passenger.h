#ifndef PASSENGER_H
#define PASSENGER_H

#include "person.h"
#include <iostream>

class Passenger : public Person{
private:
    int cash;
    bool ticket;
public:
    Passenger(std::string n = "", std::string s = "", int c = 1250, bool t = false);
    bool buyTicket(int price);

    //getters
    int getCash() const;
    bool haveTicket() const;
    //I didn't put setters because we don't want the user to just change any of these's values
};

#endif