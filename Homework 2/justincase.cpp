#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>

class MenuItem{
public:
    std::string foodName;
    float foodPrice;
    float tasteBalance;
};

class Starter : public MenuItem{

};

class Salad : public MenuItem{

};

class MainCourse : public MenuItem{

};

class Drink : public MenuItem{

};

class Appetizer : public MenuItem{

};

class Dessert : public MenuItem{

};

class Menu{
public:
    std::vector<MenuItem> menuItem;
    float tasteBalance;
    float cost;
};

class User{
    std::string firtName;
    std::string lastName;
    bool gender;
public:
    Menu* choosenMenu;
};
