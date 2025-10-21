#include "passenger.h"

    Passenger::Passenger(std::string n, std::string s, int c, bool t) 
    : Person(n, s), cash(c), ticket(t)
    {
        if(cash < 0){
            std::cout << "Passenger cash can't be negative. It is set to 0." << std::endl;
            cash = 0;
        }
    }
    bool Passenger::buyTicket(int price){
        if(cash >= price){
            ticket = true;
            cash -= price;
            return true;
        }else{
            return false;
        }
    }
    //getters
    int Passenger::getCash() const {return cash;}
    bool Passenger::haveTicket() const {return ticket;}