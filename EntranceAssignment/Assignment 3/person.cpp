#include "person.h"

Person::Person(std::string n, std::string s) : name(n), surname(s){}

    //getters
    std::string Person::getName() const {return name;}
    std::string Person::getSurname() const {return surname;}

    //setters
    void Person::setName(std::string n){name = n;}
    void Person::setSurname(std::string s){surname = s;}