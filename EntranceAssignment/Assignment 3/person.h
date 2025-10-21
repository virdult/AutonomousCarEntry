#ifndef PERSON_H
#define PERSON_H

#include "person.h"
#include <iostream>

class Person{
private:
    std::string name;
    std::string surname;

public:
    Person(std::string n = "", std::string s = "");

    //getters
    std::string getName() const;
    std::string getSurname() const;

    //setters
    void setName(std::string n);
    void setSurname(std::string s);
};

#endif